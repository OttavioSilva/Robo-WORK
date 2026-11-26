# M2M2_SLAM_TOOLBOX (25/11): M2M2 so como SENSOR (/scan_raw) + slam_toolbox faz o mapa.
# Motivo (medido 25/11): o mapa interno do M2M2 deixa as paredes da arena em 120-127, igual ao
# chao (127) -> /map dele sai quase todo livre com lixo solto. O scan do M2M2 e bom.
#  - m2m2_bridge: publish_tf=false (map->odom vem do slam_toolbox), publish_map=false, sem mapear
#  - scan_filter (lidar_m2m2.yaml, sem cortes de chao) -> /scan_front -> scan_sem_giro -> /scan_slam_giro
#    -> scan_slam_sem_tras (traseira |ang| >= 120 em NaN) -> /scan_slam
#  - slam_toolbox (mapping.launch.py / slam_mapping.yaml) -> /map e map->odom
#  - mapa_limpo: /map -> /map_nav (costmap global)
# Mapa comeca do zero onde o robo estiver. Salvar com salvar_mapa.sh (LIDAR=delta, caminho slam_toolbox).
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    home = Path.home()
    share = lambda p, f: str(Path(get_package_share_directory(p)) / "launch" / f)
    sub = home / "robo_jazzy_ws/ferramentas/subida_unica"
    inc = lambda p, f, **kw: GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(p, f)), launch_arguments=kw.items())])
    return LaunchDescription([
        DeclareLaunchArgument("m2m2_host", default_value="192.168.11.1"),
        inc("robo_hardware", "hardware.launch.py", scan_topic="scan_raw_delta"),
        SetEnvironmentVariable("ROBO_LIDAR_CONFIG", "lidar_m2m2.yaml"),
        # LIDAR_POS_M2M2 (25/11, medido pelo Marcos): centro do LiDAR a 2,5 cm da frente e
        # centralizado (30 cm de cada lado) -> x = 0,375 - 0,025 = 0,350; y = 0. Era 0,330 / 0,005 (Delta).
        # O mesmo valor vai para a mascara do corpo (body_mount) dentro do lidar_filter.launch.py.
        inc("robo_lidar", "lidar_filter.launch.py", lidar_x="0.350", lidar_y="0.000"),
        # M2M2_ODOM (25/11): odometria por comando so publica /odom (sem TF); o TF odom->base_link
        # vem da pose do M2M2 (m2m2_odom.py), que tambem atende /reset_odometry (teleop Y 2x).
        Node(package="robo_localization", executable="command_imu_odometry_node",
             name="command_imu_odometry", output="screen",
             parameters=[str(Path(get_package_share_directory("robo_localization")) / "config" / "localization.yaml"),
                         {"publish_tf": False}],
             remappings=[("/reset_odometry", "/reset_odometry_comando")]),
        ExecuteProcess(name="m2m2_odom", output="screen", respawn=True, respawn_delay=1.0,
                       cmd=["python3", "-u", str(sub / "m2m2_odom.py")]),
        inc("robo_navigation", "mapping.launch.py"),
        Node(package="m2m2_bridge", executable="m2m2_bridge", name="m2m2_bridge", output="screen",
             respawn=True, respawn_delay=2.0,
             parameters=[{
                 "host": LaunchConfiguration("m2m2_host"),
                 "publish_tf": False,
                 # M2M2_ESPELHO (25/11): M2M2 espelhado em relacao ao robo (giro com sinal trocado);
                 # o m2m2_odom espelha a pose (parametro espelhar, padrao true)
                 "angle_sign": -1.0,
                 # Referencia de ~79 cm em 26/11: mediana a 0° de 97,8 cm antes e 78,0 cm com o desconto.
                 # O desvio variou entre sessoes; conferir com a trena depois de religar o M2M2.
                 "range_offset": 0.20,
                 "range_scale": 1.0,
                 # RANGE_MAX_5 (26/11, pedido do Marcos): era 40 m; pontos alem de 5 m (fora da arena) saem
                 "range_max": 5.0,
                 "publish_map": False,
                 "limpar_ao_iniciar": False,
                 "mapeando": False,
                 "carregar_ao_iniciar": False,
             }]),
        # GIRO_SO_TELEOP (26/11): era scan_sem_giro.py (cortava o giro sempre; no Nav2 cortou 226 de
        # 234 scans e o SLAM ficou cego). Agora so corta com o LB do teleop apertado.
        ExecuteProcess(name="scan_giro_teleop", output="screen", respawn=True, respawn_delay=1.0,
                       cmd=["python3", "-u", str(sub / "scan_giro_teleop.py")]),
        # SLAM_SEM_TRAS (25/11): so o SLAM perde a traseira (quem segue o robo nao entra no mapa)
        ExecuteProcess(name="scan_slam_sem_tras", output="screen", respawn=True, respawn_delay=1.0,
                       cmd=["python3", "-u", str(sub / "scan_slam_sem_tras.py")]),
        # BLOQUEIOS_NA_BASE (25/11): o global costmap tem a StaticLayer bloqueio_layer (/bloqueios_mapa);
        # sem quem publique, o costmap nunca fica "current" e o planner recusa todo goal
        # ("Costmap timed out waiting for update"). So o localizacao_amcl.launch.py subia este no.
        ExecuteProcess(name="bloqueios_rota", output="screen", respawn=True, respawn_delay=2.0,
                       cmd=["python3", "-u", str(home / "robo_jazzy_ws/src/robo_navigation/scripts/bloqueios_rota.py")]),
        ExecuteProcess(name="mapa_limpo", output="screen", respawn=True, respawn_delay=2.0,
                       cmd=["python3", "-u", str(sub / "mapa_limpo.py")]),
    ])
