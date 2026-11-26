#!/usr/bin/env python3
"""POSE_ESTIMATE_MAPA (26/11): faz o "2D Pose Estimate" do RViz funcionar com o slam_toolbox em mapping.

A cada /initialpose: zera a odometria e recarrega o mapa salvo (deserialize_map, START_AT_GIVEN_POSE)
com o robo na pose clicada; limpa os costmaps. Nao move o robo.
Uso: python3 pose_estimate_mapa.py [~/mapas/NOME]   (padrao arena_26-11_0431). Ctrl+C sai.
"""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph
from std_srvs.srv import Trigger


def main():
    base = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/mapas/arena_26-11_0431')
    if not os.path.exists(base + '.posegraph'):
        sys.exit('nao achei %s.posegraph' % base)
    rclpy.init()
    no = Node('pose_estimate_mapa')
    fila = []
    no.create_subscription(PoseWithCovarianceStamped, '/initialpose', fila.append, 10)
    odom = no.create_client(Trigger, '/reset_odometry')
    carregar = no.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
    limpar = [no.create_client(ClearEntireCostmap, '/%s_costmap/clear_entirely_%s_costmap' % (c, c))
              for c in ('global', 'local')]

    def chamar(cli, req, t):
        if not cli.wait_for_service(timeout_sec=3.0):
            return None
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(no, f, timeout_sec=t)
        return f.result()

    print('Mapa: %s\nClique em "2D Pose Estimate" no RViz: no lugar do robo, arraste para a frente dele.' % base, flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(no, timeout_sec=0.2)
            if not fila:
                continue
            p = fila[-1].pose.pose
            fila.clear()
            yaw = math.atan2(2 * p.orientation.w * p.orientation.z, 1 - 2 * p.orientation.z ** 2)
            print('Pose clicada x=%.2f y=%.2f angulo=%.0f: recarregando...' % (p.position.x, p.position.y,
                                                                          math.degrees(yaw)), flush=True)
            r = chamar(odom, Trigger.Request(), 5.0)
            if r is None or not r.success:
                print('FALHOU: odometria nao zerada; nada mudou.', flush=True)
                continue
            time.sleep(0.5)
            req = DeserializePoseGraph.Request()
            req.filename, req.match_type = base, DeserializePoseGraph.Request.START_AT_GIVEN_POSE
            req.initial_pose.x, req.initial_pose.y, req.initial_pose.theta = p.position.x, p.position.y, yaw
            if chamar(carregar, req, 30.0) is None:
                print('FALHOU: o SLAM nao respondeu.', flush=True)
                continue
            fim = time.time() + 2.0
            while time.time() < fim:
                rclpy.spin_once(no, timeout_sec=0.1)
            for c in limpar:
                chamar(c, ClearEntireCostmap.Request(), 5.0)
            print('Pronto. Confira no RViz se o scan bate com as paredes; se nao, clique de novo.', flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
