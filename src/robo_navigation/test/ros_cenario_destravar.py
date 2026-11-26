#!/usr/bin/env python3
"""Cenario ROS ISOLADO para o destravar v4 (so roda com ROS_DOMAIN_ID != 0; o robo usa o 0).

Publica /scan_nav, /odom, TF map->odom->base_link sinteticos (do sim_recuperacao), oferece
compute_path_to_pose, is_path_valid e collision_monitor/get_state falsos, chama a acao 'destravar' e
imprime um JSON com o resultado e os comandos que apareceram em /cmd_vel_nav.
Uso: ros_cenario_destravar.py <cm_ativo|cm_inativo|cancelar|estrangeiro>
"""
import json
import math
import os
import sys
import threading
import time

if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
    sys.exit('RECUSADO: rode num dominio isolado (ROS_DOMAIN_ID diferente do robo)')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy  # noqa: E402
from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from geometry_msgs.msg import TransformStamped, TwistStamped, PoseStamped  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from nav2_msgs.action import ComputePathToPose, DriveOnHeading  # noqa: E402
from nav2_msgs.srv import IsPathValid  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402
from sim_recuperacao import SimIO  # noqa: E402

SALA = [(-2.1, 4.1, 1.5, 1.6), (-2.1, 4.1, -1.6, -1.5), (4.0, 4.1, -1.6, 1.6), (-2.1, -2.0, -1.6, 1.6)]
modo = sys.argv[1] if len(sys.argv) > 1 else 'cm_ativo'


class Cenario(Node):
    def __init__(self):
        super().__init__('cenario_destravar')
        self.sim = SimIO(SALA, semente=3, inf=0.3)
        self.cmds = []
        self.pub_scan = self.create_publisher(LaserScan, '/scan_nav', 5)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_plan = self.create_publisher(Path, '/plan', 1)
        self.pub_nav = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(TwistStamped, '/cmd_vel_nav', lambda m: self.cmds.append(
            (time.monotonic(), m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)), 50)
        self.create_timer(1 / 5.9, self.scan)
        self.create_timer(0.05, self.odom)
        if modo == 'estrangeiro':
            self.create_timer(0.1, self.estrangeiro)
        ActionServer(self, ComputePathToPose, 'compute_path_to_pose', self.planejar)
        self.create_service(IsPathValid, 'is_path_valid', self.valido)
        if modo != 'cm_inativo':
            self.create_service(GetState, 'collision_monitor/get_state', self.cm)
        self.cli = ActionClient(self, DriveOnHeading, 'destravar')
        self.pub_ctrl = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', 10)

    def oscilar(self, seg):
        t1, k = time.monotonic() + seg, 0
        while time.monotonic() < t1:
            t = TwistStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.twist.linear.y = 0.05 if (k // 3) % 2 else -0.05
            self.pub_ctrl.publish(t)
            k += 1
            time.sleep(0.1)
        self.pub_ctrl.publish(TwistStamped())

    def caminho(self):
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        for i in range(30):
            q = PoseStamped()
            q.header.frame_id = 'map'
            q.pose.position.x = 0.1 * i
            q.pose.orientation.w = 1.0
            p.poses.append(q)
        return p

    def scan(self):
        m = LaserScan()
        m.header.frame_id = 'laser'
        m.header.stamp = self.get_clock().now().to_msg()
        m.angle_min, m.angle_increment = -math.pi, math.radians(1)
        m.angle_max = m.angle_min + 359 * m.angle_increment
        m.range_min, m.range_max = 0.146, 12.0
        self.sim._gerar_scan()
        m.ranges = [float(r) for r in self.sim.scan().ranges]
        self.pub_scan.publish(m)
        self.pub_plan.publish(self.caminho())

    def odom(self):
        agora = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = agora, 'odom', 'base_link'
        o.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(o)
        for pai, filho in (('map', 'odom'), ('odom', 'base_link')):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = agora, pai, filho
            t.transform.rotation.w = 1.0
            self.tf.sendTransform(t)

    def estrangeiro(self):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.twist.linear.x = 0.05
        self.pub_nav.publish(t)

    def planejar(self, gh):
        gh.succeed()
        r = ComputePathToPose.Result()
        r.path = self.caminho()
        return r

    def valido(self, req, res):
        res.is_valid = False
        return res

    def cm(self, req, res):
        res.current_state.id = 3
        res.current_state.label = 'active'
        return res


def main():
    rclpy.init()
    no = Cenario()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(no)
    threading.Thread(target=ex.spin, daemon=True).start()
    saida = {'modo': modo}
    try:
        if not no.cli.wait_for_server(timeout_sec=20.0):
            saida['erro'] = 'destravar nao apareceu'
            return
        time.sleep(2.0)                      # scans/odom/TF chegarem
        if modo == 'cancelar':
            no.oscilar(2.5)                  # o DWB "ciscando" antes de travar
            time.sleep(1.0)
        g = DriveOnHeading.Goal()
        g.target.x, g.speed = 0.40, 0.03
        g.time_allowance.sec = 30
        f = no.cli.send_goal_async(g)
        while not f.done():
            time.sleep(0.05)
        h = f.result()
        saida['aceito'] = h.accepted
        t_goal = time.monotonic()
        if modo == 'cancelar':
            time.sleep(4.5)                  # ja dentro da manobra (diagnostico leva ~2 s)
            h.cancel_goal_async()
        fr = h.get_result_async()
        t1 = time.monotonic() + 45
        while not fr.done() and time.monotonic() < t1:
            time.sleep(0.05)
        r = fr.result()
        saida['status'] = r.status
        saida['error_code'] = r.result.error_code
        saida['error_msg'] = getattr(r.result, 'error_msg', '')
        saida['duracao'] = time.monotonic() - t_goal
        meus = [c for c in no.cmds if c[0] >= t_goal and not (modo == 'estrangeiro' and abs(c[1] - 0.05) < 1e-9)]
        saida['cmds_nao_nulos'] = sum(1 for c in meus if any(abs(v) > 1e-6 for v in c[1:]))
        saida['ultimo_cmd'] = meus[-1][1:] if meus else None
    finally:
        print('RESULTADO_JSON=' + json.dumps(saida))
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
