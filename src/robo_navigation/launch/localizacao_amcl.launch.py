"""MAPA_MANUAL (24/11): map_server (mapa desenhado) + AMCL, no lugar do slam_toolbox."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SCRIPTS = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts')


def generate_launch_description():
    cfg = str(Path(get_package_share_directory('robo_navigation')) / 'config/amcl_mapa_manual.yaml')
    mapa, params = LaunchConfiguration('mapa'), LaunchConfiguration('params')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')
    return LaunchDescription([
        DeclareLaunchArgument('mapa'),
        DeclareLaunchArgument('params', default_value=cfg),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': mapa, 'use_sim_time': False}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'initial_pose.x': ParameterValue(x, value_type=float),
                                 'initial_pose.y': ParameterValue(y, value_type=float),
                                 'initial_pose.yaw': ParameterValue(yaw, value_type=float)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localizacao',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        # VIGIA_LOCALIZACAO + BLOQUEIO_ROTA (24/11): rodam direto do src, como os outros scripts
        ExecuteProcess(cmd=['python3', os.path.join(SCRIPTS, 'vigia_localizacao.py')], output='screen'),
        ExecuteProcess(cmd=['python3', os.path.join(SCRIPTS, 'bloqueios_rota.py')], output='screen'),
    ])
