#!/usr/bin/env python3
"""SLAM_SEM_TRAS (25/11/2026): /scan_slam_giro -> /scan_slam com o setor TRASEIRO em NaN.

Mapeando com o M2M2 + slam_toolbox, quem acompanha o robo por tras entrava no mapa.
So o SLAM le /scan_slam: alinhador, Collision Monitor, costmaps e tracker seguem no /scan_front
(continuam vendo atras). Usado so no base_m2m2_slam.launch.py.

- Angulo no frame do LiDAR (laser_raw, 0 = frente). Corta |angulo| >= corte_graus.
- Parametro corte_graus (padrao 120 = cone traseiro de 120 graus). 180 = nao corta nada.
  Muda na hora: ros2 param set /scan_slam_sem_tras corte_graus 135.0
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SemTras(Node):
    def __init__(self):
        super().__init__('scan_slam_sem_tras')
        self.declare_parameter('corte_graus', 120.0)
        self.pub = self.create_publisher(LaserScan, '/scan_slam', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_slam_giro', self.ao_scan, qos_profile_sensor_data)
        self.n = 0
        self.create_timer(30.0, lambda: self.get_logger().info(
            'SLAM_SEM_TRAS: %d scans, corte |ang| >= %.0f graus' % (self.n, self.corte())))
        self.get_logger().info('SLAM_SEM_TRAS: /scan_slam_giro -> /scan_slam, traseira |ang| >= %.0f graus em NaN'
                               % self.corte())

    def corte(self):
        return float(self.get_parameter('corte_graus').value)

    def ao_scan(self, m):
        lim = math.radians(self.corte())
        r = list(m.ranges)
        for i in range(len(r)):
            a = math.atan2(math.sin(m.angle_min + i * m.angle_increment),
                           math.cos(m.angle_min + i * m.angle_increment))
            if abs(a) >= lim:
                r[i] = float('nan')
        m.ranges = r
        if m.intensities:
            m.intensities = list(m.intensities)
        self.n += 1
        self.pub.publish(m)


def main():
    rclpy.init()
    no = SemTras()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == '__main__':
    main()
