#!/usr/bin/env python3
"""Somente observador: odom -> Path, sem qualquer publicador de velocidade."""
from collections import deque


def main():
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped
    class Trajetoria(Node):
        def __init__(self):
            super().__init__('trajetoria_teste')
            self.poses = deque(maxlen=20000)
            self.ultimo = 0
            self.frame = None
            self.pub = self.create_publisher(Path, '/teste/trajetoria_estimada', 1)
            self.create_subscription(Odometry, '/odom', self.ao_odom, 20)
        def ao_odom(self, msg):
            t = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            if t-self.ultimo < 200000000 and t >= self.ultimo:
                return
            if self.frame != msg.header.frame_id or t < self.ultimo:
                self.poses.clear()
            self.frame, self.ultimo = msg.header.frame_id, t
            p = PoseStamped()
            p.header, p.pose = msg.header, msg.pose.pose
            self.poses.append(p)
            path = Path()
            path.header, path.poses = msg.header, list(self.poses)
            self.pub.publish(path)
    rclpy.init()
    n = Trajetoria()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
