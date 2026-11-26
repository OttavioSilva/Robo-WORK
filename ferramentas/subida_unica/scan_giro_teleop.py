#!/usr/bin/env python3
"""GIRO_SO_TELEOP (26/11/2026): /scan_front -> /scan_slam_giro, cortando o giro SO no teleop.

Medido 26/11 00:04:25-00:04:55 (goal do Nav2, M2M2 + slam_toolbox): o scan_sem_giro (corte com
|giro z| > 0,05 rad/s, feito para o Delta em 23/11) passou 8 scans e cortou 226: as correcoes de
rumo do Nav2 e o balanco do robo passam de 0,05 rad/s e o SLAM ficou cego no trajeto.
Pedido do Marcos: o filtro de giro so vale quando o robo esta no TELEOP.

- Teleop = LB/L1 (homem-morto) apertado no controle (/dev/input/js*, botao --botao, Xbox 4).
  Le o controle em paralelo ao teleop (os dois recebem os eventos).
- No teleop: corta scan com |giro z da IMU| > --limite rad/s e ate --espera s depois (como antes).
- Fora do teleop (Nav2, alinhador): passa TODOS os scans.
- Sem IMU ha > 0,5 s: passa tudo (nunca cega o SLAM).
Saida /scan_slam_giro (o scan_slam_sem_tras faz /scan_slam_giro -> /scan_slam).
"""
import argparse
import glob
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

JS_FMT = 'IhBB'
JS_SIZE = struct.calcsize(JS_FMT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--limite', type=float, default=0.05)
    ap.add_argument('--espera', type=float, default=0.5)
    ap.add_argument('--botao', type=int, default=4, help='homem-morto do teleop (Xbox xpad: LB = 4)')
    a, _ = ap.parse_known_args()
    rclpy.init()
    no = Node('scan_giro_teleop')
    pub = no.create_publisher(LaserScan, '/scan_slam_giro', qos_profile_sensor_data)
    st = {'giro': -1e9, 'imu': None, 'lb': False, 'passou': 0, 'cortou': 0, 'nav': 0}

    def controle():
        while rclpy.ok():
            devs = sorted(glob.glob('/dev/input/js*'))
            if not devs:
                st['lb'] = False
                time.sleep(1.0)
                continue
            try:
                fd = os.open(devs[0], os.O_RDONLY)
            except OSError:
                time.sleep(1.0)
                continue
            try:
                while rclpy.ok():
                    d = os.read(fd, JS_SIZE)
                    if len(d) < JS_SIZE:
                        break
                    _, valor, tipo, num = struct.unpack(JS_FMT, d)
                    if (tipo & 0x7F) == 0x01 and num == a.botao:   # botao (inclui o estado inicial)
                        st['lb'] = bool(valor)
            except OSError:
                pass
            finally:
                os.close(fd)
                st['lb'] = False
            time.sleep(1.0)

    threading.Thread(target=controle, daemon=True).start()

    def ao_imu(m):
        st['imu'] = time.monotonic()
        if abs(m.angular_velocity.z) > a.limite:
            st['giro'] = time.monotonic()

    def ao_scan(m):
        agora = time.monotonic()
        if not st['lb']:
            st['nav'] += 1
            pub.publish(m)
        elif st['imu'] is None or agora - st['imu'] > 0.5 or agora - st['giro'] > a.espera:
            st['passou'] += 1
            pub.publish(m)
        else:
            st['cortou'] += 1

    no.create_subscription(Imu, '/imu/data', ao_imu, qos_profile_sensor_data)
    no.create_subscription(LaserScan, '/scan_front', ao_scan, qos_profile_sensor_data)
    no.create_timer(30.0, lambda: no.get_logger().info(
        'GIRO_SO_TELEOP: fora do teleop %d (todos passam) | teleop: %d passaram, %d cortados no giro'
        % (st['nav'], st['passou'], st['cortou'])))
    no.get_logger().info('GIRO_SO_TELEOP: corte de giro (|wz| > %.2f) so com o botao %d do controle apertado'
                         % (a.limite, a.botao))
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == '__main__':
    main()
