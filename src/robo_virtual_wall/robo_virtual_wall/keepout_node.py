"""Accumulate confirmed, calibrated tape observations as a Nav2 keepout mask."""
from collections import deque
import copy
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.msg import CostmapFilterInfo
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from .wall_map import WallTracker, rasterize, save_walls, load_walls
from .visible_retirement import VisibleRetirement


def rotation_matrix(q):
    values = np.array([q.x,q.y,q.z,q.w],dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x,y,z,w = values/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class TapeKeepout(Node):
    def __init__(self):
        super().__init__('tape_keepout')
        defaults = dict(map_topic='/map', detections_topic='/virtual_wall/detections',
                        mask_topic='/virtual_wall/keepout_mask',
                        filter_info_topic='/virtual_wall/filter_info',
                        # FITA_ROBUSTA_20261121: 5 consecutive observations (~0.35 s
                        # at 14 Hz) filter transient look-alikes such as cables;
                        # 10 cm merge absorbs SLAM pose jumps seen while mapping.
                        confirmations=5, match_tolerance_m=.08, merge_tolerance_m=.10,
                        wall_half_width_m=.05, confirmation_tolerance_m=.025, min_segment_length_m=.08,
                        max_observation_age_s=.5,
                        max_walls=2000, max_map_cells=4000000,
                        remove_visible_missing=False, removal_timeout_s=2., removal_min_frames=6,
                        odom_topic='/odom',
                        # PAREDES_SALVAS_20261121: walls only valid in the map they were
                        # recorded in (fixed arena map). off | load (read at start,
                        # never written) | record (start empty, save on every change).
                        walls_file='', walls_mode='off')
        for key,value in defaults.items():
            self.declare_parameter(key,value)
        self.p = lambda key: self.get_parameter(key).value
        if (self.p('confirmations') < 2 or self.p('match_tolerance_m') <= 0 or
                self.p('wall_half_width_m') <= 0 or self.p('max_observation_age_s') <= 0 or
                self.p('max_walls') < 1 or self.p('max_map_cells') < 1 or
                self.p('confirmation_tolerance_m') <= 0 or self.p('min_segment_length_m') <= 0 or
                not 0 < self.p('merge_tolerance_m') <= .2):
            raise ValueError('Invalid keepout parameters')
        self.tracker = WallTracker(self.p('confirmations'),self.p('match_tolerance_m'),
                                   self.p('max_walls'),self.p('confirmation_tolerance_m'),
                                   self.p('min_segment_length_m'), self.p('merge_tolerance_m'))
        self.retirement = VisibleRetirement(self.p('removal_timeout_s'), self.p('removal_min_frames'),
                                           self.p('max_observation_age_s'))
        self.odom_stamp = None
        self.stationary_since = None
        self.stationary_pose = None
        self.map = None
        self.last_stamp = None
        self.waiting_tf = deque()
        self.last_clock = None
        self.dirty = True
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.mask_pub = self.create_publisher(OccupancyGrid,self.p('mask_topic'),qos)
        self.info_pub = self.create_publisher(CostmapFilterInfo,self.p('filter_info_topic'),qos)
        self.create_subscription(OccupancyGrid,self.p('map_topic'),self.on_map,qos)
        self.create_subscription(String,self.p('detections_topic'),self.on_detections,10)
        self.create_subscription(Odometry,self.p('odom_topic'),self.on_odom,qos_profile_sensor_data)
        self.create_service(Empty,'~/clear',self.clear)
        if self.p('walls_mode') not in ('off', 'load', 'record'):
            raise ValueError('walls_mode must be off, load or record')
        if self.p('walls_mode') != 'off' and not self.p('walls_file'):
            raise ValueError('walls_mode %s requires walls_file' % self.p('walls_mode'))
        self.walls_ready = self.p('walls_mode') == 'off'
        self.create_service(Trigger,'~/save',self.save_service)
        self.create_timer(.2,self.publish)
        self.create_timer(.02,self.retry_waiting_tf)
        self.get_logger().info('Keepout waiting for /map and validated base_link observations. '
                               'Visible missing tape retirement: %s.' % self.p('remove_visible_missing'))

    def on_odom(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        twist = message.twist.twist
        values = np.array([twist.linear.x, twist.linear.y, twist.linear.z,
                           twist.angular.x, twist.angular.y, twist.angular.z])
        stopped = np.isfinite(values).all() and np.linalg.norm(values[:3]) < .01 and np.linalg.norm(values[3:]) < .02
        if not stopped or (self.odom_stamp is not None and (stamp <= self.odom_stamp or stamp-self.odom_stamp > .5)):
            self.stationary_since = None
            self.stationary_pose = None
            self.retirement.reset()
        if stopped:
            self.stationary_since = self.stationary_since if self.stationary_since is not None else stamp
        self.odom_stamp = stamp

    def reset_absence(self):
        self.retirement.reset()
        self.stationary_pose = None

    def save_walls_file(self):
        if not self.p('walls_file') or self.map is None:
            return 0
        count = save_walls(Path(self.p('walls_file')).expanduser(), self.tracker.walls,
                           self.map.header.frame_id, 'tape_keepout walls_mode=%s' % self.p('walls_mode'))
        self.get_logger().info('PAREDES_SALVAS: %d paredes gravadas em %s' % (count, self.p('walls_file')),
                               throttle_duration_sec=2.)
        return count

    def save_service(self, request, response):
        try:
            count = self.save_walls_file()
            response.success = bool(self.p('walls_file')) and self.map is not None
            response.message = ('%d paredes gravadas em %s' % (count, self.p('walls_file'))
                                if response.success else 'sem walls_file ou sem /map ainda')
        except OSError as error:
            response.success, response.message = False, str(error)
        return response

    def prepare_walls(self):
        """On the first valid map: load (load) or start a fresh file (record)."""
        if self.walls_ready:
            return
        self.walls_ready = True
        path = Path(self.p('walls_file')).expanduser()
        frame = self.map.header.frame_id
        if self.p('walls_mode') == 'load':
            if not path.exists():
                self.get_logger().warning('PAREDES_SALVAS: arquivo %s nao existe; seguindo so com '
                                          'as paredes vistas nesta subida' % path)
                return
            try:
                walls = load_walls(path, frame)
            except (ValueError, OSError) as error:
                self.get_logger().error('PAREDES_SALVAS: arquivo %s recusado (%s); nenhuma '
                                        'parede carregada' % (path, error))
                return
            self.tracker.walls.extend(walls)
            self.dirty = True
            self.get_logger().info('PAREDES_SALVAS: %d paredes carregadas de %s (nao sera '
                                   'alterado nesta subida)' % (len(walls), path))
        else:
            if path.exists():
                backup = path.with_name(path.name + '.bak_' + time.strftime('%Y%m%d_%H%M%S'))
                path.replace(backup)
                self.get_logger().info('PAREDES_SALVAS: arquivo anterior guardado em %s' % backup)
            self.save_walls_file()
            self.get_logger().info('PAREDES_SALVAS: GRAVANDO fitas em %s, comecando vazio' % path)

    def clear(self, request, response):
        self.tracker.walls.clear()
        self.tracker.pending.clear()
        self.waiting_tf.clear()
        self.reset_absence()
        self.dirty = True
        self.get_logger().info('Tape walls cleared')
        if self.p('walls_mode') == 'record':
            self.save_walls_file()
        self.publish()
        return response

    def on_map(self, message):
        info = message.info
        if (not message.header.frame_id or info.width*info.height > self.p('max_map_cells') or
                min(info.width,info.height,info.resolution) <= 0 or
                not math.isfinite(info.resolution)):
            self.get_logger().warning('Invalid or oversized map ignored',throttle_duration_sec=5.)
            return
        try:
            rotation_matrix(info.origin.orientation)
        except ValueError:
            return
        if not all(math.isfinite(v) for v in [info.origin.position.x,info.origin.position.y]):
            return
        if self.map and self.map.header.frame_id != message.header.frame_id:
            self.tracker.walls.clear()
            self.tracker.pending.clear()
            self.reset_absence()
        changed = (self.map is None or self.map.info != info or
                   self.map.header.frame_id != message.header.frame_id)
        self.map = message
        self.dirty |= changed
        self.prepare_walls()

    def on_detections(self, message):
        # Retry at the exact image timestamp after TF callbacks have run.
        # Never substitute latest TF or block the single-thread executor.
        if self.waiting_tf:
            if len(self.waiting_tf) >= 16:
                self.waiting_tf.popleft()
                self.tracker.pending.clear()
                self.reset_absence()
            self.waiting_tf.append(message)
        elif not self.process_detection(message):
            self.waiting_tf.append(message)

    def retry_waiting_tf(self):
        while self.waiting_tf:
            if not self.process_detection(self.waiting_tf[0]):
                break
            self.waiting_tf.popleft()

    def process_detection(self, message):
        if self.map is None:
            return True
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.tracker.pending.clear()
            self.last_stamp = None
            self.reset_absence()
        self.last_clock = now
        try:
            data = json.loads(message.data)
            if data.get('metric_frame') != 'base_link' or data.get('perception_valid') is False:
                self.tracker.pending.clear()
                self.reset_absence()
                self.get_logger().warning('No validated base_link calibration; tape mask remains '
                                           'unchanged', throttle_duration_sec=10.)
                return True
            sec,nanosec = data['stamp']['sec'],data['stamp']['nanosec']
            if type(sec) is not int or type(nanosec) is not int or not 0 <= nanosec < 1000000000:
                raise ValueError('Invalid image timestamp')
            stamp = sec+nanosec*1e-9
            if not 0 <= now-stamp <= self.p('max_observation_age_s'):
                self.tracker.pending.clear()
                self.reset_absence()
                return True
            if self.last_stamp is not None:
                if stamp <= self.last_stamp:
                    return True
                if stamp-self.last_stamp > self.p('max_observation_age_s'):
                    self.tracker.pending.clear()
                    self.reset_absence()
            segments = []
            for entry in data['detections']:
                if 'base_link_m' not in entry:
                    continue
                points = np.asarray(entry['base_link_m'],dtype=float)
                if points.shape != (2,2) or not np.isfinite(points).all():
                    raise ValueError('Invalid metric tape segment')
                segments.append(points)
            view = data.get('observed_ground_base_link_m') if self.p('remove_visible_missing') else None
            if view is not None:
                view = np.asarray(view, dtype=float)
                if (view.ndim != 2 or view.shape[1] != 2 or not 3 <= len(view) <= 64
                        or not np.isfinite(view).all()):
                    raise ValueError('Invalid observed ground polygon')
            stationary = (self.stationary_since is not None and self.odom_stamp is not None
                          and 0 <= now-self.odom_stamp <= .5 and now-self.stationary_since >= 1.)
            if segments or view is not None:
                transform = self.tf_buffer.lookup_transform(self.map.header.frame_id,'base_link',
                                    Time(seconds=sec,nanoseconds=nanosec))
                rotation = rotation_matrix(transform.transform.rotation)
                t = transform.transform.translation
                translation = np.array([t.x,t.y,t.z])
                segments = [(np.column_stack((points,np.zeros(2))) @ rotation.T +
                             translation)[:,:2] for points in segments]
                if view is not None:
                    view = (np.column_stack((view,np.zeros(len(view)))) @ rotation.T + translation)[:,:2]
                if stationary:
                    pose = np.array([t.x, t.y, math.atan2(rotation[1,0], rotation[0,0])])
                    if self.stationary_pose is not None:
                        delta_yaw = math.atan2(math.sin(pose[2]-self.stationary_pose[2]),
                                              math.cos(pose[2]-self.stationary_pose[2]))
                        if np.linalg.norm(pose[:2]-self.stationary_pose[:2]) > .02 or abs(delta_yaw) > math.radians(1):
                            self.retirement.reset()
                            self.stationary_pose = None
                    self.stationary_pose = pose if self.stationary_pose is None else self.stationary_pose
                else:
                    self.reset_absence()
            self.last_stamp = stamp
            merged, copies = self.tracker.merged, len(self.tracker.inconsistent)
            added = self.tracker.update(segments)
            if self.tracker.merged > merged:
                self.get_logger().info('LINHA_FITA: parede estendida com trecho colinear (total %d paredes)'
                                       % len(self.tracker.walls))
            for item in self.tracker.inconsistent[copies:]:
                self.get_logger().warning('PAREDE_DUPLA: nova parede parece copia deslocada de outra '
                    '(%.1f graus, %.1f cm); provavel mudanca da localizacao entre passadas. Mantidas as duas.'
                    % (item['angle_deg'], 100 * item['offset_m']))
            removed = self.retirement.update(self.tracker.walls, segments, view, stamp, stationary)
            self.dirty |= added or removed > 0
            if (added or removed) and self.p('walls_mode') == 'record':
                self.save_walls_file()
            if added:
                self.get_logger().info('Tape walls confirmed in map: %d' % len(self.tracker.walls),
                                       throttle_duration_sec=1.)
            if removed:
                self.get_logger().info('Tape walls removed after stationary visible absence: %d; remaining: %d' %
                                       (removed, len(self.tracker.walls)))
            if len(self.tracker.walls) >= self.p('max_walls'):
                self.get_logger().warning('Tape wall capacity reached; clear/review mask',
                                           throttle_duration_sec=10.)
        except TransformException as error:
            self.get_logger().warning('Waiting for image-time TF: '+str(error),
                                       throttle_duration_sec=5.)
            return False
        except (ValueError,TypeError,KeyError) as error:
            self.tracker.pending.clear()
            self.reset_absence()
            self.get_logger().warning('Tape observation rejected: '+str(error),
                                       throttle_duration_sec=5.)

        return True

    def publish(self):
        if self.map is None or not self.dirty:
            return
        mask = OccupancyGrid()
        mask.header.frame_id = self.map.header.frame_id
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info = copy.deepcopy(self.map.info)
        o = mask.info.origin
        q = o.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        grid = rasterize(self.tracker.walls,mask.info.width,mask.info.height,
                         mask.info.resolution,(o.position.x,o.position.y,yaw),
                         self.p('wall_half_width_m'))
        mask.data = grid.ravel().tolist()
        info = CostmapFilterInfo()
        info.header = mask.header
        info.type,info.base,info.multiplier = 0,0.,1.
        info.filter_mask_topic = self.p('mask_topic')
        self.info_pub.publish(info)
        self.mask_pub.publish(mask)
        self.dirty = False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TapeKeepout()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
