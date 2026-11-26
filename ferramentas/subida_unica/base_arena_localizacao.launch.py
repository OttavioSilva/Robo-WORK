# LOCALIZACAO_FIXA (19/11): arena_11-11_1938 em mode localization (mapa fixo), carregada no START com match_type 3.
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# MAPA_ARENA (21/11): mapa da arena escolhido por ARENA_MAPA; padrao = mapa remapeado em 21/11.
MAPA_ARENA = os.environ.get("ARENA_MAPA", "arena_21-11_1727")


def generate_launch_description():
    home = Path.home()
    actions = []
    for package, filename in [
        ("robo_hardware", "hardware.launch.py"),
        ("robo_lidar", "lidar_filter.launch.py"),
        ("robo_localization", "localization.launch.py"),
        ("robo_navigation", "localizacao_slam.launch.py"),  # LOCALIZACAO_FIXA 19/11
    ]:
        launch_file = Path(get_package_share_directory(package)) / "launch" / filename
        args = {"slam_params_file": str(Path(__file__).resolve().parent / f"slam_localizacao_{MAPA_ARENA}.yaml")} if "localizacao" in filename else {}
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)), launch_arguments=args.items()
        )]))

    map_base = home / "mapas" / MAPA_ARENA  # MAPA_ARENA (21/11): era "mapas/arena_11-11_1938"
    points = yaml.safe_load(Path(f"{map_base}_pontos.yaml").read_text())
    start = points["pontos"]["START"]
    request = {"filename": str(map_base), "match_type": 3,  # LOCALIZE_AT_POSE
               "initial_pose": {"x": start["x"], "y": start["y"], "theta": start["yaw"]}}
    workstation_script = str(home / "robo_jazzy_ws/src/robo_navigation/scripts/marcar_mesa.py")
    # SEM_PAREDE_VIRTUAL (23/11): parede virtual (camera/fitas gravadas) removida.
    for name, command in [
        ("mesas", ["python3", "-u", workstation_script, "--mostrar"]),
        ("cadastro", ["python3", "-u", workstation_script, "--listar"]),
        # MAPA_LIMPO (21/11): /map -> /map_nav com as zonas de <mapa>_limpar.yaml apagadas.
        ("mapa_limpo", ["python3", "-u", "/home/marcos/robo_jazzy_ws/ferramentas/subida_unica/mapa_limpo.py", "--zonas", f"{map_base}_limpar.yaml"]),
        ("mapa", ["ros2", "service", "call", "/slam_toolbox/deserialize_map",
                  "slam_toolbox/srv/DeserializePoseGraph", yaml.safe_dump(request)]),
    ]:
        extra = dict(respawn=True, respawn_delay=2.0) if name == "mapa_limpo" else {}  # MAPA_LIMPO
        actions.append(ExecuteProcess(name=name, cmd=command, output="screen", **extra))
    return LaunchDescription(actions)
