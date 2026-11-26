import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("robo_lidar"))
    # LIDAR_M2M2 (24/11): ROBO_LIDAR_CONFIG=lidar_m2m2.yaml troca a config (padrao: Delta-2D)
    parameters = package_share / "config" / os.environ.get("ROBO_LIDAR_CONFIG", "lidar.yaml")

    with open(parameters) as fp:
        temporal_parameters = yaml.safe_load(fp)['scan_filter']['ros__parameters']
    temporal_parameters = dict(temporal_parameters, input_topic='/scan_raw_temporal',
                               output_topic='/scan_temporal_front')

    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")

    body_mount = {  # CORPO_REAL_M35: mascara e TF sempre usam a mesma montagem.
        'body_laser_x': ParameterValue(lidar_x, value_type=float),
        'body_laser_y': ParameterValue(lidar_y, value_type=float),
        'body_laser_yaw': ParameterValue(lidar_yaw, value_type=float),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_x",
                default_value="0.330",
                description="Posicao X do feixe do LiDAR em relacao ao base_link, em metros.",
            ),
            DeclareLaunchArgument(
                "lidar_y",
                default_value="0.005",
                description="Posicao Y do feixe do LiDAR em relacao ao base_link, em metros.",
            ),
            DeclareLaunchArgument(
                "lidar_z",
                default_value="0.000",
                description="Posicao Z do feixe do LiDAR em relacao ao base_link, em metros.",
            ),
            DeclareLaunchArgument(
                "lidar_yaw",
                default_value="0.000",
                description="Rotacao do LiDAR em torno de Z, em radianos.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                output="screen",
                arguments=[
                    "--x", lidar_x,
                    "--y", lidar_y,
                    "--z", lidar_z,
                    "--yaw", lidar_yaw,
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "base_link",
                    "--child-frame-id", "laser_raw",
                ],
            ),
            Node(package='robo_lidar', executable='scan_filter_node',
                 name='scan_filter_temporal', output='screen', parameters=[temporal_parameters, body_mount]),
            Node(
                package="robo_lidar",
                executable="scan_filter_node",
                name="scan_filter",
                output="screen",
                parameters=[str(parameters), body_mount],
            ),
        ]
    )
