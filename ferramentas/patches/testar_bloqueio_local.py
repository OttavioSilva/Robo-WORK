#!/usr/bin/env python3
"""Testa o BLOQUEIO_LOCAL sem o robo: roda o destravar_node.py de verdade num dominio ROS isolado
(ROS_DOMAIN_ID 87), com planner, costmaps, scan e odom falsos. Nada chega na ponte, que esta em outro
dominio, entao o robo nao se mexe.

Uso: python3 -P testar_bloqueio_local.py [caminho do destravar_node.py]
"""
import math
import os
import subprocess
import sys
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '87'

import rclpy  # noqa: E402
from rclpy.action import ActionClient, ActionServer, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from nav2_msgs.action import ComputePathToPose, DriveOnHeading  # noqa: E402
from nav2_msgs.srv import ClearEntireCostmap  # noqa: E402
from sensor_msgs.msg import LaserScan, PointCloud2  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

NODE = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.expanduser(
    '~/robo_jazzy_ws/src/robo_navigation/scripts/destravar_node.py')


class Mundo(Node):
    def __init__(self, respostas, frente_livre=False):
        super().__init__('mundo_teste')
        g = ReentrantCallbackGroup()
        self.respostas = list(respostas)   # True: planner acha caminho; False: sem caminho
        self.frente_livre = frente_livre
        self.eventos = []
        self.x = self.y = self.a = 0.0
        self.v = (0.0, 0.0, 0.0)
        self.pub_scan = self.create_publisher(LaserScan, '/scan_nav', qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_plano = self.create_publisher(Path, '/plan', 5)
        self.create_subscription(PointCloud2, '/bloqueio_virtual', self.ao_bloq, 5, callback_group=g)
        self.create_subscription(TwistStamped, '/cmd_vel_nav', self.ao_cmd, 10, callback_group=g)
        self.create_service(ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap',
                            lambda q, r: self.limpar('global', r), callback_group=g)
        self.create_service(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap',
                            lambda q, r: self.limpar('local', r), callback_group=g)
        ActionServer(self, ComputePathToPose, 'compute_path_to_pose', self.planejar, callback_group=g,
                     goal_callback=lambda _: GoalResponse.ACCEPT)
        self.cli = ActionClient(self, DriveOnHeading, 'destravar', callback_group=g)
        self.tfb = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'base_link'
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.transform.rotation.w = 1.0
        self.tfb.sendTransform(tf)
        self.create_timer(0.05, self.passo, callback_group=g)
        self.create_timer(0.1, self.scan, callback_group=g)

    def ev(self, nome):
        self.eventos.append((time.monotonic(), nome))

    def limpar(self, qual, r):
        self.ev('limpa_' + qual)
        return r

    def ao_bloq(self, m):
        self.ev('nuvem_cheia' if m.width > 0 else 'nuvem_vazia')

    def ao_cmd(self, m):
        self.v = (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)

    def planejar(self, gh):
        ok = self.respostas.pop(0) if self.respostas else True
        self.ev('plano_ok' if ok else 'plano_sem_caminho')
        res = ComputePathToPose.Result()
        if ok:
            res.path = self.caminho()
            gh.succeed()
        else:
            gh.abort()
        return res

    def caminho(self):
        p = Path()
        p.header.frame_id = 'map'
        for i in range(30):
            s = PoseStamped()
            s.header.frame_id = 'map'
            s.pose.position.x = -0.05 * i
            s.pose.position.y = 0.05 * i
            s.pose.orientation.w = 1.0
            p.poses.append(s)
        return p

    def passo(self):
        dt = 0.05
        c, s = math.cos(self.a), math.sin(self.a)
        self.x += (c * self.v[0] - s * self.v[1]) * dt
        self.y += (s * self.v[0] + c * self.v[1]) * dt
        self.a += self.v[2] * dt
        o = Odometry()
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.a / 2), math.cos(self.a / 2)
        self.pub_odom.publish(o)

    def scan(self):
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser_raw'
        n = 180
        m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / n
        m.range_min, m.range_max = 0.02, 6.0
        m.ranges = []
        for i in range(n):
            a = m.angle_min + i * m.angle_increment
            perto = abs(a) < 0.4 and not self.frente_livre
            m.ranges.append(0.145 if perto else 2.5)
        self.pub_scan.publish(m)

    def chamar(self, timeout=25.0):
        self.ev('chamada')
        assert self.cli.wait_for_server(timeout_sec=10.0), 'destravar nao subiu'
        g = DriveOnHeading.Goal()
        g.target.x = 0.15
        g.speed = 0.03
        g.time_allowance.sec = 8
        f = self.cli.send_goal_async(g)
        t1 = time.monotonic() + timeout
        while not f.done() and time.monotonic() < t1:
            time.sleep(0.05)
        r = f.result().get_result_async()
        while not r.done() and time.monotonic() < t1:
            time.sleep(0.05)
        self.ev('fim_chamada')
        return r.done()


def cenario(nome, respostas, frente_livre, verificar, depois=0.0, chamadas=1):
    print('\n--- %s' % nome)
    log = open(os.devnull, 'w')
    proc = subprocess.Popen([sys.executable, '-P', NODE], stdout=log, stderr=log, env=dict(os.environ))
    no = Mundo(respostas, frente_livre)
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(no)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        time.sleep(2.0)
        no.pub_plano.publish(no.caminho())
        time.sleep(1.0)
        for _ in range(chamadas):
            assert no.chamar(), 'o destravar nao terminou'
        t_fim = time.monotonic() + depois
        while time.monotonic() < t_fim:
            time.sleep(0.1)
        ev = list(no.eventos)
        t0 = next(t for t, n in ev if n == 'chamada')
        vistos, ant = [], None
        for t, n in ev:
            if t >= t0 and n != ant:
                vistos.append((round(t - t0, 2), n))
                ant = n
        print('   ', vistos)
        verificar([(t - t0, n) for t, n in ev if t >= t0])
        print('    OK')
        return True
    except AssertionError as e:
        print('    FALHOU: %s' % e)
        return False
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        ex.shutdown()
        no.destroy_node()


def primeiro(ev, nome, depois=-1.0):
    return next((t for t, n in ev if n == nome and t > depois), None)


def v_sem_caminho(ev):
    cheia = primeiro(ev, 'nuvem_cheia')
    assert cheia is not None, 'nao marcou o bloqueio'
    p1 = primeiro(ev, 'plano_sem_caminho')
    assert p1 is not None, 'nao perguntou ao planner'
    vazia = primeiro(ev, 'nuvem_vazia', p1)
    lg, ll = primeiro(ev, 'limpa_global', p1), primeiro(ev, 'limpa_local', p1)
    assert vazia is not None and lg is not None and ll is not None, 'faltou nuvem vazia ou limpeza'
    assert vazia + 0.3 < lg, 'limpou antes de a nuvem vazia assentar (%.2f x %.2f)' % (vazia, lg)
    p2 = primeiro(ev, 'plano_ok', lg)
    assert p2 is not None, 'nao planejou de novo depois de limpar'
    assert p2 > ll, 'planejou antes de limpar o local'
    assert primeiro(ev, 'nuvem_cheia', vazia + 0.2) is None, 'o bloqueio voltou depois de apagado'


def v_com_caminho(ev):
    cheia = primeiro(ev, 'nuvem_cheia')
    assert cheia is not None, 'nao marcou o bloqueio'
    assert primeiro(ev, 'limpa_global') is None, 'limpou o global sem precisar'
    vazia = primeiro(ev, 'nuvem_vazia', cheia)
    assert vazia is not None, 'o bloqueio nao venceu'
    assert 9.0 < vazia - cheia < 11.0, 'validade errada: %.1f s' % (vazia - cheia)
    ll = primeiro(ev, 'limpa_local', vazia)
    assert ll is not None and 0.2 < ll - vazia < 1.2, 'nao limpou o local ao vencer'


def v_frente_livre(ev):
    assert primeiro(ev, 'nuvem_cheia') is None, 'marcou com a frente livre'
    p1 = primeiro(ev, 'plano_sem_caminho')
    assert p1 is not None, 'nao perguntou ao planner'
    assert primeiro(ev, 'limpa_global', p1) is not None, 'nao limpou o global'
    assert primeiro(ev, 'plano_ok', p1) is not None, 'nao planejou de novo'


def main():
    rclpy.init()
    print('Testando %s no dominio %s' % (NODE, os.environ['ROS_DOMAIN_ID']))
    r = [cenario('frente perto, bloqueio vivo, planner sem caminho na 2a chamada', [True, False, True], False, v_sem_caminho, chamadas=2),
         cenario('frente perto, planner acha caminho', [True, True], False, v_com_caminho, depois=11.5),
         cenario('frente livre, planner sem caminho na 1a', [False, True], True, v_frente_livre)]
    rclpy.shutdown()
    print('\n%d de %d cenarios OK' % (sum(r), len(r)))
    return 0 if all(r) else 1


if __name__ == '__main__':
    sys.exit(main())
