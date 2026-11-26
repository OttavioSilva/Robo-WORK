from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robo_hardware"))
    parameters = package_share / "config" / "hardware.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument('temporal_enabled', default_value='false'),
            DeclareLaunchArgument('scan_topic', default_value='scan_raw'),  # LIDAR_M2M2: scan_raw_delta com M2M2
            DeclareLaunchArgument('command_authority', default_value=''),
            DeclareLaunchArgument('heading_hold_enabled', default_value='true'),
            DeclareLaunchArgument('device_clock_drift_bound_ppm', default_value='-1.0'),
            DeclareLaunchArgument('lidar_capture_lag_bound_s', default_value='-1.0'),
            DeclareLaunchArgument('imu_capture_lag_bound_s', default_value='-1.0'),
            Node(
                package="robo_hardware",
                executable="hardware_bridge_node",
                name="robo_hardware",
                output="screen",
                parameters=[str(parameters), {
                    'temporal_enabled': ParameterValue(LaunchConfiguration('temporal_enabled'), value_type=bool),
                    'scan_topic': ParameterValue(LaunchConfiguration('scan_topic'), value_type=str),
                    'command_authority': ParameterValue(LaunchConfiguration('command_authority'), value_type=str),
                    'heading_hold_enabled': ParameterValue(LaunchConfiguration('heading_hold_enabled'), value_type=bool),
                    **{k: ParameterValue(LaunchConfiguration(k), value_type=float) for k in
                       ('device_clock_drift_bound_ppm', 'lidar_capture_lag_bound_s', 'imu_capture_lag_bound_s')}
                }],
            )
        ]
    )
