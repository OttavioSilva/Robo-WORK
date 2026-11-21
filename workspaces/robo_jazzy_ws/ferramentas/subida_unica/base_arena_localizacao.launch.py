# LOCALIZACAO_FIXA (19/11): arena_11-11_1938 em mode localization (mapa fixo), carregada no START com match_type 3.
import os  # PAREDES_SALVAS (21/11)
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
        args = {"slam_params_file": str(Path(__file__).resolve().parent / "slam_localizacao_arena_11-11_1938.yaml")} if "localizacao" in filename else {}
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)), launch_arguments=args.items()
        )]))

    map_base = home / "mapas/arena_11-11_1938"
    points = yaml.safe_load(Path(f"{map_base}_pontos.yaml").read_text())
    start = points["pontos"]["START"]
    request = {"filename": str(map_base), "match_type": 3,  # LOCALIZE_AT_POSE
               "initial_pose": {"x": start["x"], "y": start["y"], "theta": start["yaw"]}}
    workstation_script = str(home / "robo_jazzy_ws/src/robo_navigation/scripts/marcar_mesa.py")
    device = "/dev/v4l/by-id/usb-XIFT_Web_Camera_20250904.1549-video-index0"
    wall = str(home / "logs/reabertura_20261116_100105/parede.sh")
    camera_command = ('if [[ ! -e "$1" ]]; then echo "Aguardando a câmera frontal XIFT"; fi; '
                      'while [[ ! -e "$1" ]]; do sleep 2; done; exec bash "$2"')
    for name, command in [
        ("parede_virtual", ["bash", "-c", camera_command, "camera", device, wall]),
        ("mesas", ["python3", "-u", workstation_script, "--mostrar"]),
        ("cadastro", ["python3", "-u", workstation_script, "--listar"]),
        ("mapa", ["ros2", "service", "call", "/slam_toolbox/deserialize_map",
                  "slam_toolbox/srv/DeserializePoseGraph", yaml.safe_dump(request)]),
    ]:
        # PAREDES_SALVAS (21/11): fitas salvas ao lado do mapa; modo vem do abrir_arena.sh.
        extra = {}
        if name == "parede_virtual":
            extra = dict(additional_env={"PAREDES_FITA_ARQUIVO": f"{map_base}_fitas.json",
                                         "PAREDES_FITA_MODO": os.environ.get("PAREDES_FITA_MODO", "load")})
        actions.append(ExecuteProcess(name=name, cmd=command, output="screen", **extra))
    return LaunchDescription(actions)
