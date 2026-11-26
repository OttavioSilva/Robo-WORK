"""Inicia somente os componentes Nav2 usados pelo robo durante o SLAM."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    navigation_share = get_package_share_directory("robo_navigation")
    default_params_file = os.path.join(
        navigation_share, "config", "nav2_params.yaml"
    )

    default_nav_to_pose_bt_xml = os.path.join(
        navigation_share,
        "behavior_trees",
        "navigate_to_pose_smooth.xml",
    )

    # O docking_server nao e iniciado: o robo nao possui rotina de docagem.
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
    ]

    common_parameters = [
        params_file,
        {"use_sim_time": use_sim_time},
    ]
    common_remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]
    motion_output_remappings = common_remappings + [
        ("cmd_vel", "cmd_vel_nav"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Usar o relogio de simulacao",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Ativar automaticamente os nos de ciclo de vida",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Arquivo de parametros do Nav2",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=common_parameters,
                remappings=motion_output_remappings,
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=common_parameters,
                remappings=common_remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=common_parameters,
                remappings=common_remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=common_parameters,
                remappings=motion_output_remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=common_parameters + [
                    {
                        "default_nav_to_pose_bt_xml":
                            default_nav_to_pose_bt_xml,
                    }
                ],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=common_parameters,
                remappings=common_remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=common_parameters,
                # Entrada: /cmd_vel_nav; saida preservada em /cmd_vel_smoothed.
                remappings=common_remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=common_parameters,
                # Parametros fazem /cmd_vel_smoothed -> /cmd_vel.
                remappings=common_remappings,
            ),
            # Recovery customizado para base mecanum.
            # Nao e Lifecycle Node: fica disponivel como action server
            # /mecanum_escape durante toda a navegacao.
            Node(
                package="robo_recovery",
                executable="mecanum_escape_node",
                name="mecanum_escape_node",
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ]
    )
