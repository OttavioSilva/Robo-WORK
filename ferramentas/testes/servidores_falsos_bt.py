#!/usr/bin/env python3
"""So para conferir_bt_navigator_isolado.sh: anuncia (sem executar nada) as acoes/servicos que a arvore espera,
para o bt_navigator conseguir construir a arvore num dominio isolado. Qualquer goal recebido e REJEITADO."""
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose, ComputePathThroughPoses, SmoothPath, FollowPath, DriveOnHeading, Spin, BackUp, Wait
from nav2_msgs.srv import ClearEntireCostmap, IsPathValid

def main():
    rclpy.init()
    n = Node('servidores_falsos_bt')
    rej = lambda g: GoalResponse.REJECT  # noqa: E731
    acoes = [(ComputePathToPose, "compute_path_to_pose"), (ComputePathThroughPoses, "compute_path_through_poses"), (SmoothPath, 'smooth_path'), (FollowPath, 'follow_path'),
             (DriveOnHeading, 'destravar'), (DriveOnHeading, 'drive_on_heading'), (Spin, 'spin'),
             (Spin, 'smart_spin'), (BackUp, 'backup'), (Wait, 'wait')]
    s = [ActionServer(n, t, nome, execute_callback=lambda h: None, goal_callback=rej) for t, nome in acoes]
    s += [n.create_service(ClearEntireCostmap, nome, lambda rq, rs: rs) for nome in
          ('global_costmap/clear_entirely_global_costmap', 'local_costmap/clear_entirely_local_costmap')]
    s += [n.create_service(IsPathValid, 'is_path_valid', lambda rq, rs: rs)]
    rclpy.spin(n)

if __name__ == '__main__':
    main()
