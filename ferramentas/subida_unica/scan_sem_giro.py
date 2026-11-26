#!/usr/bin/env python3
"""SEM_MAPA_NO_GIRO (23/11/2026): /scan_front -> /scan_slam so com o robo sem girar.

Replay do bag mapeamento_orientacao_20261123 (18:41-18:44): o SLAM errava 2-15 graus em cada giro de 90
(IMU e giroscopio concordavam em ~3%). Sem os scans do giro (+ SLAM_IMU_FORTE no slam_mapping.yaml)
o erro por giro caiu para ~2 graus. Enquanto gira, o rumo vem da odometria (IMU).

- Descarta scan com |giroscopio z| > --limite rad/s, e ate --espera s depois do ultimo giro.
- IMU ausente ha mais de 0,5 s: deixa TODOS os scans passarem (nunca cega o SLAM).
- So o SLAM le /scan_slam; alinhador, Collision Monitor e costmaps continuam no /scan_front.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--limite', type=float, default=0.05, help='rad/s de giro que conta como girando')
    ap.add_argument('--espera', type=float, default=0.5, help='s sem mapear depois do giro')
    ap.add_argument('--saida', default='/scan_slam', help='topico de saida')
    a, _ = ap.parse_known_args()
    rclpy.init()
    no = Node('scan_sem_giro')
    pub = no.create_publisher(LaserScan, a.saida, qos_profile_sensor_data)
    st = {'giro': -1e9, 'imu': None, 'passou': 0, 'cortou': 0, 'sem_imu': 0}

    def ao_imu(m):
        st['imu'] = time.monotonic()
        if abs(m.angular_velocity.z) > a.limite:
            st['giro'] = time.monotonic()

    def ao_scan(m):
        agora = time.monotonic()
        if st['imu'] is None or agora - st['imu'] > 0.5:
            st['sem_imu'] += 1
            pub.publish(m)
        elif agora - st['giro'] > a.espera:
            st['passou'] += 1
            pub.publish(m)
        else:
            st['cortou'] += 1

    def resumo():
        no.get_logger().info('SEM_MAPA_NO_GIRO: %d scans para o SLAM, %d cortados no giro, %d sem IMU (passaram)'
                             % (st['passou'], st['cortou'], st['sem_imu']))

    no.create_subscription(Imu, '/imu/data', ao_imu, qos_profile_sensor_data)
    no.create_subscription(LaserScan, '/scan_front', ao_scan, qos_profile_sensor_data)
    no.create_timer(30.0, resumo)
    no.get_logger().info('SEM_MAPA_NO_GIRO: /scan_front -> /scan_slam sem giro (|wz| > %.2f rad/s, espera %.1f s)'
                         % (a.limite, a.espera))
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == '__main__':
    main()
