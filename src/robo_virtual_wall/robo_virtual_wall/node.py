"""Observation-only ROS adapter for the existing striped-tape detector."""
import json
from pathlib import Path
import time
from datetime import datetime
import shutil
import numpy as np

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from . import detector
from .projection import load_calibration, project_observed_segment, preview_observation, resize_preview
from .point_probe import inspect_point, measured_check
from .stabilization import SegmentSmoother, PreviewDistanceFilter
from .self_mask import SelfMask
from .dynamic_self_mask import DynamicSelfMask
from .visible_retirement import observed_ground
from .camera import LatestFrameCamera
from .mount_check import MountCheck


class VirtualWall(Node):
    def __init__(self):
        super().__init__('virtual_wall')
        defaults = dict(device='/dev/video2', width=640, height=480, fps=30.0,
                        processing_hz=15.0, fourcc='MJPG', show_windows=False,
                        calibrate=False,
                        image_topic='', camera_frame='camera_optical_frame',
                        hsv_file=str(Path.home() / 'Downloads/hsv_fita.json'),
                        homography_file='', preview_homography_file='', preview_allow_resize=False, max_range_m=3.0,
                        roi_top=0.10, roi_bottom=0.85, self_mask_file='', self_mask_mode='static',
                        # CAMERA_RECONEXAO_20261121
                        v4l2_controls='', reconnect_after_s=2.0,
                        use_driver_timestamp=True, max_driver_age_s=0.5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        # GARRA_MASCARA_20261113: exclude only measured robot silhouette.
        if self.p('self_mask_mode') not in ('static', 'dynamic'):
            raise ValueError('self_mask_mode must be static or dynamic')
        if self.p('self_mask_mode') == 'dynamic' and not self.p('self_mask_file'):
            raise ValueError('Dynamic gripper requires a measured reference configuration')
        mask_class = DynamicSelfMask if self.p('self_mask_mode') == 'dynamic' else SelfMask
        self.self_mask = (mask_class(Path(self.p('self_mask_file')).expanduser())
                          if self.p('self_mask_file') else None)
        if self.p('calibrate') and not self.p('show_windows'):
            raise ValueError('calibrate requires show_windows=true')
        if not 0 <= self.p('roi_top') < self.p('roi_bottom') <= 1:
            raise ValueError('Require 0 <= roi_top < roi_bottom <= 1')
        if min(self.p('fps'), self.p('processing_hz'), self.p('max_range_m')) <= 0:
            raise ValueError('Rates and max_range_m must be positive')
        with open(Path(self.p('hsv_file')).expanduser(), encoding='utf-8') as stream:
            self.hsv = json.load(stream)
        for key, limit in [('h_min', 179), ('h_max', 179), ('s_min', 255),
                           ('s_max', 255), ('v_min', 255), ('v_max', 255)]:
            value = self.hsv[key]
            if type(value) is not int or not 0 <= value <= limit:
                raise ValueError('Invalid HSV value: ' + key)
        if self.hsv['s_min'] > self.hsv['s_max'] or self.hsv['v_min'] > self.hsv['v_max']:
            raise ValueError('Inverted saturation/value limits')
        detector.ROI_TOPO = self.p('roi_top')
        if self.p('calibrate'):
            cv2.namedWindow('ajuste HSV')
            for key in self.hsv:
                cv2.createTrackbar(key, 'ajuste HSV', self.hsv[key],
                                  179 if key.startswith('h_') else 255, lambda _: None)
            self.get_logger().info('Ajuste HSV ao vivo: s salva com backup; q encerra.')
        self.calibration = None
        self.preview = None
        self.preview_size = None
        self.probe_calibration = None
        self.probe_selection = None
        self.probe_message = ''
        self.probe_input = None
        self.probe_checks = []
        self.probe_checks_path = None
        self.fitting_pixels = []
        if self.p('preview_homography_file'):
            preview_path = Path(self.p('preview_homography_file')).expanduser()
            self.preview = load_calibration(preview_path, preview=True)
            self.fitting_pixels = json.loads(preview_path.read_text()).get('pixels', [])
        self.mount_check = None
        self.mount_inconclusive_since = None
        self.last_mount_check = 0.
        if self.p('homography_file'):
            homography_path = Path(self.p('homography_file')).expanduser()
            self.calibration = load_calibration(homography_path)
            # CAMERA_MEXEU_20261121: calibrations made with the ChArUco tool
            # carry two robot-body patches; a moved camera suspends metres.
            reference = json.loads(homography_path.read_text()).get('mount_reference')
            if reference:
                image = cv2.imread(str(homography_path.parent / reference['image']), cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise ValueError('Mount reference image missing: ' + reference['image'])
                self.mount_check = MountCheck(image, reference['rects'])
                self.get_logger().info('CAMERA_MEXEU: verificacao da montagem ativa')
        if self.p('show_windows'):
            cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('camera', self.on_camera_click)
            self.get_logger().info('Clique: medir ponto. M: informar trena. Botao direito: limpar.')
        self.bridge = CvBridge()
        self.annotated = self.create_publisher(Image, '~/image', qos_profile_sensor_data)
        self.mask_pub = self.create_publisher(Image, '~/mask', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '~/detections', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '~/segments', 10)
        self.camera = None
        self.last_sequence = 0
        self.driver_ages = []
        self.process_times = []
        self.last_latency_log = time.monotonic()
        self.last_processed = 0.0
        self.segment_smoother = SegmentSmoother()
        self.preview_filter = PreviewDistanceFilter()
        self.create_timer(0.5, self.expire_markers)
        if self.p('image_topic'):
            self.create_subscription(Image, self.p('image_topic'), self.on_image,
                                     qos_profile_sensor_data)
        else:
            # CAMERA_RECONEXAO_20261121: reader thread keeps only the newest
            # frame and reopens the device after loss, without restarting
            # tape_keepout (its confirmed walls live only in memory).
            logger = self.get_logger()
            log = dict(info=logger.info, warn=logger.warning, error=logger.error)
            self.camera = LatestFrameCamera(self.p('device'), self.p('width'), self.p('height'),
                self.p('fps'), self.p('fourcc'), self.p('v4l2_controls'),
                self.p('reconnect_after_s'), log=log)
            self.create_timer(1.0 / self.p('processing_hz'), self.capture)
        self.get_logger().info('Observation only; metric projection: %s' % bool(self.calibration))

    def on_camera_click(self, event, x, y, flags, userdata):
        if event == cv2.EVENT_RBUTTONDOWN:
            self.probe_selection = None
            self.probe_input = None
            self.probe_message = ''
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.probe_input = None
            self.probe_selection = None
            if self.probe_calibration is None:
                self.probe_message = 'Clique sem calibracao de previa disponivel'
                return
            sx = self.probe_calibration[1][0]/self.preview[1][0]
            sy = self.probe_calibration[1][1]/self.preview[1][1]
            fitting = np.asarray(self.fitting_pixels)*[sx,sy] if self.fitting_pixels else []
            self.probe_selection = inspect_point((x,y), self.probe_calibration, fitting)
            self.probe_message = ('Ponto fora da regiao calibrada' if self.probe_selection is None else '')
            if self.probe_selection:
                self.get_logger().info('Ponto clicado: '+json.dumps(self.probe_selection))

    def save_probe_check(self):
        try:
            check = measured_check(self.probe_selection, self.probe_input)
            checks = self.probe_checks + [check]
            directory = Path.home() / 'Robo-WORK' / 'camera_captures'
            directory.mkdir(parents=True, exist_ok=True)
            if self.probe_checks_path is None:
                self.probe_checks_path = directory / ('checks-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.json')
            data = dict(pixels=[c['pixel'] for c in checks],metres=[c['metres'] for c in checks],
                        checks=checks, image_size=list(self.probe_calibration[1]),
                        frame_id='virtual_wall_front', source_preview=self.p('preview_homography_file'),
                        validated=False)
            temporary = self.probe_checks_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(data,indent=2)+'\n', encoding='utf-8')
            temporary.replace(self.probe_checks_path)
            self.probe_checks = checks
            self.probe_message = 'Ponto salvo: erro %.1f cm; total %d' % (100*check['error_m'],len(checks))
            self.get_logger().info(self.probe_message+'; arquivo: '+str(self.probe_checks_path))
        except (ValueError, OSError) as error:
            self.probe_message = str(error)
            self.get_logger().warning(self.probe_message)
        self.probe_input = None

    def expire_markers(self):
        # Explicit removal also works when RViz does not advance ROS time.
        if time.monotonic() - self.last_processed > 1.0:
            clear = Marker()
            clear.action = Marker.DELETEALL
            self.markers_pub.publish(MarkerArray(markers=[clear]))

    def capture(self):
        item = self.camera.latest(self.last_sequence)
        if item is None:
            return
        frame, read_time, driver_age, self.last_sequence = item
        # Stamp at acquisition, not at processing. The V4L2 buffer time is
        # used only when plausible (driver on CLOCK_MONOTONIC, small age).
        delay = time.monotonic() - read_time
        if self.p('use_driver_timestamp') and driver_age is not None:
            if 0. <= driver_age <= self.p('max_driver_age_s'):
                delay += driver_age
                self.driver_ages.append(driver_age)
            else:
                self.get_logger().warning('Timestamp do driver implausivel (%.3f s); usando hora '
                                          'da leitura' % driver_age, throttle_duration_sec=30.)
        header = Image().header
        header.stamp = (self.get_clock().now() - Duration(nanoseconds=int(delay * 1e9))).to_msg()
        header.frame_id = self.p('camera_frame')
        started = time.monotonic()
        self.process(frame, header)
        self.process_times.append(time.monotonic() - started)
        if started - self.last_latency_log >= 10.:
            ages = np.asarray(self.driver_ages or [np.nan]) * 1000
            times = np.asarray(self.process_times or [np.nan]) * 1000
            self.get_logger().info('CAMERA_LATENCIA: idade no driver mediana %.0f ms p90 %.0f ms; '
                'processamento mediana %.0f ms p90 %.0f ms; %d quadros em %.0f s' % (
                np.median(ages), np.percentile(ages, 90), np.median(times),
                np.percentile(times, 90), len(times), started - self.last_latency_log))
            self.driver_ages, self.process_times = [], []
            self.last_latency_log = started

    def on_image(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not -0.1 <= now - stamp <= 0.5:
            return
        if time.monotonic() - self.last_processed < 1.0 / self.p('processing_hz'):
            return
        self.process(self.bridge.imgmsg_to_cv2(message, 'bgr8'), message.header)

    def process(self, frame, header):
        if time.monotonic() - self.last_processed > 0.5:
            self.segment_smoother.update([])
            self.preview_filter.update(None)
        self.last_processed = time.monotonic()
        height, width = frame.shape[:2]
        if self.p('calibrate'):
            for key in self.hsv:
                self.hsv[key] = cv2.getTrackbarPos(key, 'ajuste HSV')
        perception_valid = True
        if self.mount_check is not None:
            now = time.monotonic()
            if now - self.last_mount_check >= 2.:
                self.last_mount_check = now
                was_moved = self.mount_check.moved
                moved = self.mount_check.update(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                if moved != was_moved:
                    if moved:
                        self.get_logger().error('CAMERA_MEXEU: camera deslocada desde a calibracao %s; '
                            'medidas em metros SUSPENSAS ate recalibrar ou voltar a posicao'
                            % self.mount_check.last)
                    else:
                        self.get_logger().info('CAMERA_MEXEU: montagem de volta a referencia; medidas retomadas')
                if self.mount_check.last['state'] == 'inconclusive':
                    if self.mount_inconclusive_since is None:
                        self.mount_inconclusive_since = now
                    elif now - self.mount_inconclusive_since > 60.:
                        self.get_logger().warning('CAMERA_MEXEU: verificacao inconclusiva ha mais de 60 s '
                            '(cantos inferiores cobertos?) %s' % self.mount_check.last,
                            throttle_duration_sec=60.)
                else:
                    self.mount_inconclusive_since = None
            if self.mount_check.moved:
                perception_valid = False
        if isinstance(self.self_mask, DynamicSelfMask):
            previously_valid = self.self_mask.valid
            gripper_valid = self.self_mask.update(frame)
            # Both must hold: a tracked gripper does not cancel a moved camera.
            perception_valid = perception_valid and gripper_valid
            if gripper_valid != previously_valid:
                if gripper_valid:
                    self.get_logger().info('GARRA_DINAMICA: partes reconhecidas; observacoes habilitadas')
                else:
                    self.get_logger().warning('GARRA_DINAMICA: rastreio perdido; observacoes suspensas')
        detection_frame = self.self_mask.erase(frame) if self.self_mask else frame
        mask, hsv = detector.mascara_fita(detection_frame, self.hsv)
        if self.self_mask:
            mask, hsv = self.self_mask.erase(mask), self.self_mask.erase(hsv)
        mask[int(height * self.p('roi_bottom')):, :] = 0
        united = detector.unir_tracejado(mask)
        # Closing must not reintroduce excluded rows.
        united[:int(height * self.p('roi_top')), :] = 0
        united[int(height * self.p('roi_bottom')):, :] = 0
        if self.self_mask:
            united = self.self_mask.erase(united)
        annotated = frame.copy()
        walls = detector.detectar_paredes(united, mask, hsv, annotated)
        if not perception_valid:
            walls = []
            self.segment_smoother.update([])
        if self.self_mask:
            walls = self.self_mask.visible_segments(walls)
            self.self_mask.draw(annotated)
        # Smooth only the display. Published measurements retain current pixels.
        display_walls = self.segment_smoother.update(walls)
        for p1, p2, _ in display_walls:
            cv2.line(annotated, p1, p2, (0, 255, 0), 3)
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = MarkerArray(markers=[clear])
        output = []
        for index, (p1, p2, angle) in enumerate(walls):
            entry = dict(pixels=[[int(v) for v in p1], [int(v) for v in p2]],
                         angle_deg=float(angle))
            if self.calibration:
                try:
                    points = project_observed_segment((p1, p2), self.calibration,
                                             (width, height), self.p('max_range_m'))
                except ValueError as error:
                    points = None
                    self.get_logger().warning(str(error), throttle_duration_sec=5.0)
                if points is not None:
                    entry['base_link_m'] = points.tolist()
                    marker = Marker()
                    marker.header.stamp = header.stamp
                    marker.header.frame_id = 'base_link'
                    marker.ns, marker.id = 'virtual_wall_observation', index
                    marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.03
                    marker.color.r, marker.color.a = 1.0, 1.0
                    marker.lifetime.sec = 1
                    marker.points = [Point(x=float(x), y=float(y), z=0.01) for x, y in points]
                    markers.markers.append(marker)
            output.append(entry)
        self.markers_pub.publish(markers)
        ground_view = None
        if self.calibration and perception_valid and self.self_mask is None:
            support = observed_ground(frame, self.calibration, self.p('roi_top'),
                                      self.p('roi_bottom'), self.p('max_range_m'))
            if support is not None:
                ground_view = support.tolist()
        status = String()
        status.data = json.dumps(dict(observation_only=True,
            stamp=dict(sec=header.stamp.sec, nanosec=header.stamp.nanosec),
            image_size=[width, height], metric_frame='base_link' if self.calibration and perception_valid else '',
            perception_valid=perception_valid,
            mount_check=(self.mount_check.last if self.mount_check is not None else None),
            observed_ground_base_link_m=ground_view,
            self_mask=(self.self_mask.status if isinstance(self.self_mask, DynamicSelfMask)
                       else dict(mode='static' if self.self_mask else 'none', valid=True)),
            detections=output))
        self.status_pub.publish(status)
        active_preview = self.preview
        scaled_preview = False
        preview_error = None
        if self.preview_size != (width, height):
            self.preview_filter.update(None)
            self.probe_selection = None
            self.probe_input = None
            self.probe_checks = []
            self.probe_checks_path = None
            self.preview_size = (width, height)
        if self.preview is not None and (width, height) != self.preview[1]:
            active_preview = None
            if self.p('preview_allow_resize'):
                try:
                    active_preview = resize_preview(self.preview, (width, height))
                    scaled_preview = True
                except ValueError as error:
                    preview_error = str(error)
            else:
                preview_error = 'Resolucao mudou: recalibrar ou habilitar previa reescalada'
        self.probe_calibration = active_preview
        if preview_error:
            self.preview_filter.update(None)
            self.get_logger().warning(preview_error, throttle_duration_sec=5.)
            cv2.putText(annotated, 'Distancia indisponivel: resolucao sem calibracao',
                        (5, 50), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,255), 2)
        if active_preview is not None:
            matrix, _, polygon = active_preview
            cv2.polylines(annotated, [polygon.astype(np.int32)], True, (255, 200, 0), 1)
            cv2.putText(annotated, ('PREVIA REESCALADA: conferir enquadramento e trena' if scaled_preview else
                         'PREVIA: origem na borda frontal; conferir com trena'),
                        (5, height-12), cv2.FONT_HERSHEY_SIMPLEX, .4, (0,255,255), 1)
            observations = [preview_observation((p1, p2), active_preview,
                            (width, height), self.p('max_range_m'))
                            for p1, p2, _ in walls]
            crossings = [o for o in observations if o['kind'] == 'crossing']
            value = min((o['forward_m'] for o in crossings), default=None)
            filtered = self.preview_filter.update(value)
            for observation in crossings:
                cv2.circle(annotated, observation['pixel'], 6, (0,255,255), -1)
            longitudinal = [o for o in observations if o['kind'] == 'longitudinal']
            if filtered is not None:
                label = 'Eixo na linha central (filtrado): %.1f cm' % (100*filtered)
            elif longitudinal:
                offset = min(longitudinal, key=lambda o: abs(o['lateral_m']))['lateral_m']
                label = 'Fita longitudinal: lateral %.1f cm (%s)' % (
                    100*abs(offset), 'esquerda' if offset >= 0 else 'direita')
            elif observations and all(o['kind'] == 'outside' for o in observations):
                label = 'Fita detectada FORA da regiao calibrada'
            elif observations:
                label = 'Fita detectada sem cruzamento transversal valido'
            else:
                label = 'Nenhuma fita detectada neste quadro'
            cv2.putText(annotated, label, (5, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0,255,255), 2)
        if self.probe_selection is not None:
            x, y = self.probe_selection['pixel']
            forward, lateral = self.probe_selection['predicted_m']
            cv2.drawMarker(annotated, (int(x),int(y)), (255,0,255), cv2.MARKER_CROSS, 18, 2)
            side = 'esquerda' if lateral >= 0 else 'direita'
            text = 'CLIQUE: frente %.1f cm | %s %.1f cm' % (100*forward,side,100*abs(lateral))
            cv2.putText(annotated,text,(5,75),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,0,255),1)
            if self.probe_selection['fitting_reference']:
                cv2.putText(annotated,'Cruz usada no ajuste: nao valida precisao',
                            (5,96),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
        if self.probe_input is not None:
            text = 'Trena cm: FRENTE LATERAL (+esq/-dir): '+self.probe_input+'_'
        else:
            text = self.probe_message or 'Clique: medir | M: trena | R: foto | botao direito: limpar'
        cv2.putText(annotated,text,(5,height-32),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,255),1)
        if self.mount_check is not None and self.mount_check.moved:
            cv2.putText(annotated, 'CAMERA MEXEU: metros suspensos - recalibrar', (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
        cv2.putText(annotated, 'Fitas aceitas: %d' % len(walls), (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        for image, encoding, publisher in [(annotated, 'bgr8', self.annotated),
                                          (united, 'mono8', self.mask_pub)]:
            msg = self.bridge.cv2_to_imgmsg(image, encoding)
            msg.header = header
            publisher.publish(msg)
        if self.p('show_windows'):
            cv2.imshow('camera', annotated)
            cv2.imshow('mascara vermelha', mask)
            cv2.imshow('mascara unida', united)
            key = cv2.waitKey(1) & 0xff
            if ord('A') <= key <= ord('Z'):
                key += ord('a')-ord('A')
            if self.probe_input is not None:
                if key in (10,13):
                    self.save_probe_check()
                elif key == 27:
                    self.probe_input = None
                elif key in (8,127):
                    self.probe_input = self.probe_input[:-1]
                elif key in b'0123456789.,-+ ' and len(self.probe_input) < 32:
                    self.probe_input += chr(key)
                return
            if key == ord('m'):
                if self.probe_selection is not None:
                    self.probe_input = ''
                else:
                    self.probe_message = 'Clique primeiro em um ponto dentro da regiao azul'
            if key == ord('r'):
                directory = Path.home() / 'Robo-WORK' / 'camera_captures'
                directory.mkdir(parents=True, exist_ok=True)
                path = directory / ('raw-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.png')
                if cv2.imwrite(str(path), frame):
                    path.with_suffix('.json').write_text(json.dumps(dict(
                        hsv=self.hsv, clicked_point=self.probe_selection, image_size=[width, height],
                        self_mask_mode=self.p('self_mask_mode'),
                        self_mask_file=self.p('self_mask_file'),
                        tracking=(self.self_mask.status if isinstance(self.self_mask, DynamicSelfMask) else None),
                        fourcc=self.p('fourcc'), roi_top=self.p('roi_top'),
                        roi_bottom=self.p('roi_bottom')), indent=2), encoding='utf-8')
                    self.get_logger().info('Imagem original salva: %s' % path)
                else:
                    self.get_logger().error('Falha ao salvar imagem original: %s' % path)
            if key == ord('s') and self.p('calibrate'):
                if self.hsv['s_min'] > self.hsv['s_max'] or self.hsv['v_min'] > self.hsv['v_max']:
                    self.get_logger().warning('Limites S/V invertidos; ajuste antes de salvar.')
                else:
                    try:
                        path = Path(self.p('hsv_file')).expanduser()
                        backup = path.with_name(path.name + '.backup-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
                        with backup.open('xb') as output, path.open('rb') as source:
                            shutil.copyfileobj(source, output)
                        temporary = path.with_name(path.name + '.tmp-' + backup.name)
                        with temporary.open('x', encoding='utf-8') as stream:
                            json.dump(self.hsv, stream, indent=2)
                        temporary.replace(path)
                        self.get_logger().info('HSV salvo; backup: %s' % backup)
                    except OSError as error:
                        self.get_logger().error('Falha ao salvar HSV: %s' % error)
            if key == ord('q'):
                rclpy.try_shutdown()

    def destroy_node(self):
        if self.camera is not None:
            self.camera.close()
        if self.p('show_windows'):
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VirtualWall()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
