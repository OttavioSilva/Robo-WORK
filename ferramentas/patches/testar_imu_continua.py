#!/usr/bin/env python3
"""Testa o IMU_CONTINUA: odometria de verdade no dominio ROS 87, IMU falsa. Nada chega na ponte.
Uso: python3 -u -P testar_imu_continua.py <command_imu_odometry_node.py> <pasta de logs>"""
import math
import os
import subprocess
import sys
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '87'
import rclpy  # noqa: E402
from rclpy.qos import qos_profile_sensor_data as Q  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

rclpy.init()
n = rclpy.create_node('falsa_imu')
pub = n.create_publisher(Imu, '/imu/data', Q)
yaw_odom = [None]


def ao_odom(m):
    q = m.pose.pose.orientation
    yaw_odom[0] = math.degrees(2 * math.atan2(q.z, q.w))


n.create_subscription(Odometry, '/odom', ao_odom, 20)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
pr = subprocess.Popen(['python3', '-P', sys.argv[1]], env=dict(os.environ, ROS_LOG_DIR=sys.argv[2]),
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def manda(graus, seg):
    fim = time.monotonic() + seg
    while time.monotonic() < fim:
        i = Imu()
        i.header.stamp = n.get_clock().now().to_msg()
        i.orientation.z, i.orientation.w = math.sin(math.radians(graus) / 2), math.cos(math.radians(graus) / 2)
        pub.publish(i)
        time.sleep(0.04)
    time.sleep(0.1)
    return yaw_odom[0]


ok = True
try:
    time.sleep(3.0)
    manda(0.0, 1.0)
    for nome, passos, esperado in (
        ('gira ate 30,8', [(30.8, 1.0)], 30.8),
        ('falha de 2,6 s e volta zerada', [(None, 2.6), (0.0, 1.0)], 30.8),
        ('gira 10 passada a janela de 3 s', [(0.0, 3.2), (10.0, 1.0)], 40.8),
        ('falha curta (0,4 s) sem zerar: nao mexe', [(None, 0.4), (10.0, 0.5)], 40.8),
        ('falha de 2 s com zero atrasado', [(None, 2.0), (10.0, 0.5), (-20.0, 1.0)], 40.8),
    ):
        y = None
        for g, s in passos:
            if g is None:
                time.sleep(s)
            else:
                y = manda(g, s)
        bom = y is not None and abs(y - esperado) < 0.5
        ok = ok and bom
        print('%s %s: odom %.1f (esperado %.1f)' % ('OK    ' if bom else 'FALHOU', nome, y if y is not None else 999, esperado))
finally:
    pr.terminate()
print('RESULTADO: %s' % ('tudo OK' if ok else 'HA FALHAS'))
os._exit(0 if ok else 1)
