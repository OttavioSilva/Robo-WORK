#!/usr/bin/env python3
"""Testa o H1 sem o robo: alinhador e odometria de verdade no dominio ROS 87, com mundo e Nav2 falsos.
Nada chega na ponte, que esta em outro dominio.

Uso: python3 -P testar_h1.py <pasta src com os patches> [saida dos logs]
"""
import math
import os
import subprocess
import sys
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '87'
import rclpy  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from nav2_msgs.action import ComputePathToPose, NavigateToPose  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from sensor_msgs.msg import Imu, LaserScan  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402


class Mundo(Node):
    def __init__(self):
        super().__init__('mundo_h1')
        g = ReentrantCallbackGroup()
        self.x = self.y = self.th = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = 0.0
        self.corta_scan = self.corta_imu = False
        self.t_scan = time.monotonic()
        self.movendo_velho = []   # comando > 0 com scan velho
        self.logs = []
        self.nav_goals = self.nav_cancel = 0
        self.nav_dura = 25.0
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan_front', qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose_alinhado', 10)
        self.pub_mapa = self.create_publisher(OccupancyGrid, '/map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self)
        self.stf = StaticTransformBroadcaster(self)
        self.create_subscription(TwistStamped, '/cmd_vel_nav', self.ao_cmd, 20, callback_group=g)
        self.create_subscription(Log, '/rosout', self.ao_log, 100, callback_group=g)
        self.create_timer(0.02, self.tick, callback_group=g)
        self.create_timer(0.17, self.scan, callback_group=g)
        self.create_timer(0.04, self.imu, callback_group=g)
        ActionServer(self, ComputePathToPose, 'compute_path_to_pose', self.plano, callback_group=g)
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self.nav, callback_group=g,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base_link', 'laser_raw'
        t.transform.translation.x = 0.33
        t.transform.rotation.w = 1.0
        self.stf.sendTransform(t)
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution = 0.05
        m.info.width = m.info.height = 200
        m.info.origin.position.x = m.info.origin.position.y = -5.0
        m.info.origin.orientation.w = 1.0
        m.data = [0] * 40000
        self.pub_mapa.publish(m)

    def ao_log(self, m):
        if m.name == 'alinhador_passagem':
            self.logs.append((time.monotonic(), m.msg))

    def ao_cmd(self, m):
        v = m.twist
        self.cmd = (v.linear.x, v.linear.y, v.angular.z)
        self.t_cmd = time.monotonic()
        if any(abs(c) > 0.001 for c in self.cmd) and time.monotonic() - self.t_scan > 0.65:
            self.movendo_velho.append(self.cmd)
        s = TwistStamped()
        s.header.stamp = self.get_clock().now().to_msg()
        s.twist = v
        self.pub_cmd.publish(s)

    def tick(self):
        vx, vy, wz = self.cmd if time.monotonic() - self.t_cmd < 0.25 else (0.0, 0.0, 0.0)
        c, s = math.cos(self.th), math.sin(self.th)
        k = 4.0 if abs(vx) + abs(vy) < 0.05 else 1.0   # comando pequeno anda ~0,1 m/s
        self.x += (c * vx - s * vy) * k * 0.02
        self.y += (s * vx + c * vy) * k * 0.02
        self.th += wz * 0.02
        ag = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp = ag
        o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.pub_odom.publish(o)
        a = TransformStamped()
        a.header.stamp = ag
        a.header.frame_id, a.child_frame_id = 'map', 'odom'
        a.transform.rotation.w = 1.0
        b = TransformStamped()
        b.header.stamp = ag
        b.header.frame_id, b.child_frame_id = 'odom', 'base_link'
        b.transform.translation.x, b.transform.translation.y = self.x, self.y
        b.transform.rotation = o.pose.pose.orientation
        self.tf.sendTransform([a, b])

    def scan(self):
        if self.corta_scan:
            return
        self.t_scan = time.monotonic()
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser_raw'
        m.angle_min, m.angle_increment, m.range_min, m.range_max = -math.pi, math.radians(2.0), 0.08, 6.0
        m.ranges = [4.0] * 180
        self.pub_scan.publish(m)

    def imu(self):
        if not self.corta_imu:
            i = Imu()
            i.header.stamp = self.get_clock().now().to_msg()
            self.pub_imu.publish(i)

    def plano(self, gh):
        from nav_msgs.msg import Path
        r = ComputePathToPose.Result()
        p = Path()
        p.header.frame_id = 'map'
        g = gh.request.goal.pose.position
        for i in range(21):
            q = PoseStamped()
            q.header.frame_id = 'map'
            q.pose.position.x = self.x + (g.x - self.x) * i / 20.0
            q.pose.position.y = self.y + (g.y - self.y) * i / 20.0
            q.pose.orientation.w = 1.0
            p.poses.append(q)
        r.path = p
        gh.succeed()
        return r

    def nav(self, gh):
        self.nav_goals += 1
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.nav_dura:
            if gh.is_cancel_requested:
                self.nav_cancel += 1
                gh.canceled()
                return NavigateToPose.Result()
            s = TwistStamped()
            s.header.stamp = self.get_clock().now().to_msg()
            s.twist.linear.x = 0.1
            self.pub_cmd.publish(s)   # o Nav2 de verdade passa pelo CM; aqui so o alinhador e cobrado
            self.cmd = (0.1, 0.0, 0.0)
            self.t_cmd = time.monotonic()
            time.sleep(0.05)
        self.cmd = (0.0, 0.0, 0.0)
        gh.succeed()
        return NavigateToPose.Result()

    def goal(self, x, y):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'map'
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.orientation.w = 1.0
        self.pub_goal.publish(g)

    def espera_log(self, trecho, t_max, desde=0.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < t_max:
            if any(t >= desde and trecho in m for t, m in self.logs):
                return True
            time.sleep(0.05)
        return False


def checa(nome, ok, extra=''):
    print('%s %s %s' % ('OK    ' if ok else 'FALHOU', nome, extra))
    return ok


def maquinas(src):
    """retomar(): nada de "paredes acabaram" nem tempo gasto por causa da pausa."""
    import importlib.util
    sp = importlib.util.spec_from_file_location('alin', os.path.join(src, 'robo_navigation/scripts/alinhador_passagem.py'))
    a = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(a)
    logs = []
    tl = a.TravessiaLateral(1, log=logs.append, tempo_max=60.0) if 'log' in a.TravessiaLateral.__init__.__code__.co_varnames else a.TravessiaLateral(1)
    tl.log = logs.append
    pose = (0.0, 0.0, 0.0)
    tl.passo(100.0, pose, 0.5)
    tl._mudar('deslizar', 100.0, pose)
    t = 123.7
    tl.retomar(23.7, t, pose)
    r = tl.passo(t + 0.05, pose, 0.5)
    ok = checa('lateral volta medindo, parada', tl.estado == 'assentar' and r == (0.0, 0.0, 0.0), tl.estado)
    ok &= checa('lateral: pausa nao conta no tempo maximo', abs(t - tl.t0 - 0.0) < 1e-6, 't0=%.1f' % tl.t0)
    ok &= checa('lateral: sem "paredes acabaram"', not any('Paredes acabaram' in m for m in logs))
    e = a.Encosto(log=logs.append)
    e.passo(200.0, pose)
    e._mudar('pulso', 201.0, pose)
    e.retomar(30.0, 231.0, pose)
    ok &= checa('encosto volta medindo', e.estado == 'assentar' and e.t0 == 230.0)
    v = a.Travessia(log=logs.append)
    v.passo(300.0, pose, 1.0)
    ta = v.t_estado
    v.retomar(5.0, 305.0, pose)
    ok &= checa('travessia aproximando continua', v.estado == 'aproximar' and v.t_estado == ta + 5.0)
    return ok


def main():
    src = sys.argv[1]
    saida = sys.argv[2] if len(sys.argv) > 2 else '/tmp'
    ok = maquinas(src)
    rclpy.init()
    w = Mundo()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(w)
    threading.Thread(target=ex.spin, daemon=True).start()
    env = dict(os.environ, ROS_LOG_DIR=saida, ROS_DOMAIN_ID='87')
    al = subprocess.Popen(['python3', '-P', os.path.join(src, 'robo_navigation/scripts/alinhador_passagem.py')],
                          env=env, stdout=open(os.path.join(saida, 'alinhador.out'), 'w'), stderr=subprocess.STDOUT)
    try:
        time.sleep(6.0)
        # 1) so o LiDAR some durante um trajeto curto sem Nav2
        t = time.monotonic()
        w.goal(0.0, 0.30)
        ok &= checa('trajeto curto comecou', w.espera_log('TRAJETO CURTO', 8.0, t))
        w.corta_scan = True
        ok &= checa('avisou LiDAR parado', w.espera_log('LiDAR parado ha', 3.0, t))
        p0 = (w.x, w.y)
        time.sleep(1.0)
        p1 = (w.x, w.y)
        time.sleep(3.0)
        d = math.hypot(w.x - p1[0], w.y - p1[1])
        ok &= checa('parado sem LiDAR', d < 0.005, '%.1f cm em 3 s' % (100 * d))
        w.corta_scan = False
        ok &= checa('voltou e mediu', w.espera_log('sensores voltaram', 3.0, t))
        ok &= checa('terminou o trajeto', w.espera_log('TRAJETO CURTO: SUCESSO', 25.0, t))
        # 2) LiDAR some com o Nav2 andando: cancela e manda de novo
        t = time.monotonic()
        g0, c0 = w.nav_goals, w.nav_cancel
        w.goal(3.0, 0.0)
        t0 = time.monotonic()
        while w.nav_goals == g0 and time.monotonic() - t0 < 25.0:
            time.sleep(0.05)
        ok &= checa('Nav2 recebeu o goal', w.nav_goals == g0 + 1)
        time.sleep(1.0)
        w.corta_scan = True
        time.sleep(4.0)
        ok &= checa('cancelou o Nav2 sem LiDAR', w.nav_cancel == c0 + 1, 'cancelados %d' % (w.nav_cancel - c0))
        ok &= checa('nao mandou outro goal sem LiDAR', w.nav_goals == g0 + 1)
        w.corta_scan = False
        t0 = time.monotonic()
        while w.nav_goals == g0 + 1 and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        ok &= checa('mandou o goal de novo ao voltar', w.nav_goals == g0 + 2)
        # 3) IMU e LiDAR somem (reconexao): espera a ponte
        t = time.monotonic()
        w.corta_scan = w.corta_imu = True
        time.sleep(3.4)
        w.corta_scan = w.corta_imu = False
        t_volta = time.monotonic()
        n = w.nav_goals
        ok &= checa('esperou a ponte', w.espera_log('espero a ponte', 3.0, t))
        while w.nav_goals == n and time.monotonic() - t_volta < 9.0:
            time.sleep(0.05)
        dt = time.monotonic() - t_volta
        ok &= checa('so mandou goal apos 6 s da volta', w.nav_goals == n + 1 and dt >= 5.8, '%.1f s' % dt)
        ok &= checa('nenhum comando com scan velho', not w.movendo_velho, '%d comandos' % len(w.movendo_velho))
        w.nav_dura = 0.0
    finally:
        al.terminate()
        try:
            al.wait(5.0)
        except subprocess.TimeoutExpired:
            al.kill()
    print('RESULTADO: %s' % ('tudo OK' if ok else 'HA FALHAS'))
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    main()
