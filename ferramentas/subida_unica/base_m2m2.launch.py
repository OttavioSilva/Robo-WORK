# LIDAR_M2M2 (24/11): mesma base do base.launch.py, mas o SLAM e o do Mapper M2M2.
#  - robo_hardware: scan do Delta-2D desviado para /scan_raw_delta (nao usado)
#  - m2m2_bridge (~/m2m2_ws): /scan_raw (laser_raw), /map, TF map->odom, /m2m2/pose
#  - sem slam_toolbox (mapping.launch.py) e sem scan_sem_giro
# Argumentos: m2m2_limpar (true = mapa do zero), m2m2_mapeando (false = so localiza),
#             m2m2_mapa (.stcm) + m2m2_carregar (true = carrega no inicio).
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    home = Path.home()
    share = lambda p, f: str(Path(get_package_share_directory(p)) / "launch" / f)
    arg = LaunchConfiguration
    actions = [
        DeclareLaunchArgument("m2m2_host", default_value="192.168.11.1"),
        DeclareLaunchArgument("m2m2_limpar", default_value="true"),
        DeclareLaunchArgument("m2m2_mapeando", default_value="true"),
        DeclareLaunchArgument("m2m2_mapa", default_value=str(home / "mapas/m2m2/arena.stcm")),
        DeclareLaunchArgument("m2m2_carregar", default_value="false"),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share("robo_hardware", "hardware.launch.py")),
            launch_arguments={"scan_topic": "scan_raw_delta"}.items())]),
        # filtro sem inversao de angulo e sem cortes de chao do Delta-2D
        SetEnvironmentVariable("ROBO_LIDAR_CONFIG", "lidar_m2m2.yaml"),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share("robo_lidar", "lidar_filter.launch.py")))]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share("robo_localization", "localization.launch.py")))]),
        Node(package="m2m2_bridge", executable="m2m2_bridge", name="m2m2_bridge", output="screen",
             respawn=True, respawn_delay=2.0,
             parameters=[{
                 "host": arg("m2m2_host"),
                 "limpar_ao_iniciar": ParameterValue(arg("m2m2_limpar"), value_type=bool),
                 "mapeando": ParameterValue(arg("m2m2_mapeando"), value_type=bool),
                 "arquivo_mapa": arg("m2m2_mapa"),
                 "carregar_ao_iniciar": ParameterValue(arg("m2m2_carregar"), value_type=bool),
             }]),
        # MAPA_LIMPO (21/11): /map -> /map_nav para o costmap global.
        ExecuteProcess(name="mapa_limpo", output="screen", respawn=True, respawn_delay=2.0,
                       cmd=["python3", "-u", str(home / "robo_jazzy_ws/ferramentas/subida_unica/mapa_limpo.py")]),
    ]
    return LaunchDescription(actions)
