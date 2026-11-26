"""SUPERVISOR_LOCALIZATION_20261111: perfil opt-in, sem hardware ou Nav2."""
import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def iniciar(context):
    params = LaunchConfiguration('slam_params_file').perform(context)
    cfg = yaml.safe_load(Path(params).read_text())['slam_toolbox']['ros__parameters']
    if cfg['mode'] != 'localization':
        raise RuntimeError('perfil nao e localization')
    nome = cfg.get('map_file_name', '')
    if not nome:
        raise RuntimeError('map_file_name obrigatorio')
    for ext in ('.posegraph', '.data'):
        p = Path(nome+ext)
        if not p.is_file() or p.stat().st_size < 1000 or b'serialization::archive' not in p.open('rb').read(64):
            raise RuntimeError('mapa serializado ausente/invalido: '+str(p))
    pose = cfg.get('map_start_pose', [])
    if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
        raise RuntimeError('map_start_pose invalida')
    loc = str(Path(get_package_share_directory('slam_toolbox'))/'launch/localization_launch.py')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(loc), launch_arguments={
        'slam_params_file':params, 'use_sim_time':LaunchConfiguration('use_sim_time').perform(context),
        'autostart':'true'}.items())]


def generate_launch_description():
    params = str(Path(get_package_share_directory('robo_navigation'))/'config/slam_localizacao.yaml')
    return LaunchDescription([DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('slam_params_file',default_value=params),OpaqueFunction(function=iniciar)])
