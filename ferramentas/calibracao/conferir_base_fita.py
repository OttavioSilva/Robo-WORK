#!/usr/bin/env python3
"""Read-only preflight, no velocity publishers or authority requests."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('preflight_fita_sem_movimento')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    seen = {}
    def receive(key):
        def callback(msg):
            if key == 'map' and (not msg.info.width or not msg.info.height):
                return
            if key != 'map':
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                age = node.get_clock().now().nanoseconds / 1e9 - stamp
                if not -.1 <= age <= 1.:
                    return
            seen[key] = time.monotonic()
        return callback
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [node.create_subscription(OccupancyGrid, '/map', receive('map'), map_qos),
                     node.create_subscription(Odometry, '/odom', receive('odom'), qos_profile_sensor_data),
                     node.create_subscription(LaserScan, '/scan_front', receive('scan'), qos_profile_sensor_data)]
    forbidden = {'controller_server', 'planner_server', 'bt_navigator',
                 'alinhador_passagem', 'arbitro_nav_principal', 'tape_keepout'}
    started = time.monotonic()
    stable_since = None
    reason = 'aguardando descoberta e dados'
    accepted = False
    try:
        while time.monotonic() - started < 20:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            duplicates = forbidden.intersection(node.get_node_names())
            if duplicates:
                reason = 'navegação já ativa: ' + ', '.join(sorted(duplicates))
                break
            fresh = ('map' in seen and all(k in seen and now-seen[k] < 1. for k in ('odom', 'scan')))
            tf_ok = False
            try:
                transform = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
                tf_ok = -.1 <= node.get_clock().now().nanoseconds / 1e9 - stamp <= 1.
            except Exception:
                pass
            if now-started >= 3 and fresh and tf_ok:
                stable_since = stable_since or now
                if now-stable_since >= 2:
                    accepted = True
                    break
            else:
                stable_since = None
            reason = f'mapa={"map" in seen}, odom/scan recentes={fresh}, TF map/base recente={tf_ok}'
        if not accepted:
            raise RuntimeError('RECUSADO: '+reason+'. Não iniciar outra base ou Nav2 automaticamente.')
        print('Base/TF disponíveis e descoberta persistente sem Nav2 anterior. Nenhuma velocidade publicada.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
