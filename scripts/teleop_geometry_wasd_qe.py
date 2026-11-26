#!/usr/bin/env python3
"""Teleop manual Geometry (ROS 2 Jazzy): TwistStamped em /cmd_vel_nav.
Collision Monitor permanece no pipeline. Nao usar junto de goal Nav2 ativo.
"""
import argparse
import os
import select
import subprocess
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


def parse_args():
    p = argparse.ArgumentParser(description='Teleop manual mecanum Geometry W/S/A/D/Q/E')
    p.add_argument('--linear', type=float, default=0.07, help='m/s para W/S, padrao 0.07')
    p.add_argument('--lateral', type=float, default=0.06, help='m/s para Q/E, padrao 0.06')
    p.add_argument('--angular', type=float, default=0.18, help='rad/s para A/D, padrao 0.18')
    p.add_argument('--timeout', type=float, default=0.35, help='segundos sem tecla ate parar')
    p.add_argument('--topic', default='/cmd_vel_nav')
    a = p.parse_args()
    if not (0 < a.linear <= 0.08 and 0 < a.lateral <= 0.08 and 0 < a.angular <= 0.30):
        p.error('Limites deste teleop: 0 < linear,lateral <= 0.08 m/s; 0 < angular <= 0.30 rad/s')
    if not 0.15 <= a.timeout <= 1.0:
        p.error('timeout deve estar entre 0.15 e 1.0 s')
    return a


def main():
    a = parse_args()
    if not sys.stdin.isatty():
        raise SystemExit('Execute em um terminal interativo.')
    rclpy.init(args=None)
    node = Node('teleop_geometry_wasd_qe')
    pub = node.create_publisher(TwistStamped, a.topic, 10)
    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    speeds = {'w': (a.linear, 0, 0), 's': (-a.linear, 0, 0),
              'q': (0, a.lateral, 0), 'e': (0, -a.lateral, 0),
              'a': (0, 0, a.angular), 'd': (0, 0, -a.angular)}
    velocity = (0., 0., 0.)
    last_key = 0.

    def publish(v):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.angular.z = float(v[2])
        pub.publish(msg)

    try:
        tty.setcbreak(fd)
        print('GEOMETRY — TELEOP MANUAL W/S/A/D/Q/E')
        print('W frente | S re | A girar esquerda | D girar direita')
        print('Q lateral esquerda | E lateral direita | T salvar mesa a frente como WS | ESPACO parar | X sair')
        print(f'Vx={a.linear:.3f} m/s Vy={a.lateral:.3f} m/s Wz={a.angular:.3f} rad/s')
        print(f'Publicacao: {a.topic} (TwistStamped). Collision Monitor permanece ativo.')
        print('IMPORTANTE: Sem objetivo autonomo ativo. Segure tecla (repeticao); soltar para em ate %.2fs.' % a.timeout)
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                key = sys.stdin.read(1).lower()
                if key == 'x' or key == '\x03':
                    break
                if key == ' ':
                    velocity = (0., 0., 0.)
                    last_key = 0.
                elif key == 't':   # TECLA_T (26/11): salva a mesa a frente como WS
                    velocity = (0., 0., 0.)
                    last_key = 0.
                    for _ in range(3):
                        publish(velocity)
                        time.sleep(0.04)
                    print('\nT: robo parado, medindo a mesa a frente (10 cm)...', flush=True)
                    try:
                        r = subprocess.run([sys.executable, os.path.expanduser(
                            '~/robo_jazzy_ws/src/robo_navigation/scripts/marcar_mesa.py'), 'WS', '10'], timeout=30)
                        print('T: WS salva.' if r.returncode == 0 else 'T: NADA salvo (veja o motivo acima).', flush=True)
                    except subprocess.TimeoutExpired:
                        print('T: NADA salvo (medicao passou de 30 s).', flush=True)
                    termios.tcflush(fd, termios.TCIFLUSH)   # teclas apertadas durante a medicao nao andam
                elif key in speeds:
                    velocity = speeds[key]
                    last_key = time.monotonic()
            if last_key and time.monotonic() - last_key > a.timeout:
                velocity = (0., 0., 0.)
                last_key = 0.
            publish(velocity)
    except KeyboardInterrupt:
        pass
    finally:
        # Varios zeros para garantir ultima parada sem depender do ultimo pacote.
        for _ in range(5):
            publish((0., 0., 0.))
            time.sleep(0.04)
        termios.tcsetattr(fd, termios.TCSADRAIN, original)
        node.destroy_node()
        rclpy.shutdown()
        print('\nTeleop encerrado; comando zero publicado.')


if __name__ == '__main__':
    main()
