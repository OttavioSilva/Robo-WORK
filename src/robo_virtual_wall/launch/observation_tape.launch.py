"""Reload only tape perception/keepout, leaving the current Nav2 and base alive."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    from robo_virtual_wall.projection import load_calibration
    value = lambda name: LaunchConfiguration(name).perform(context)
    calibration = Path(value('homography_file')).expanduser()
    load_calibration(calibration)
    # CAMERA_RECONEXAO_20261121: last-resort respawn of the vision node only;
    # tape_keepout (and its confirmed walls) keeps running.
    return [Node(package='robo_virtual_wall', executable='virtual_wall', name='virtual_wall',
        output='screen', respawn=True, respawn_delay=3.0, parameters=[dict(homography_file=str(calibration),
            preview_homography_file=value('preview_homography_file'), width=640, height=480,
            roi_top=0., roi_bottom=.85, self_mask_file='',
            show_windows=ParameterValue(LaunchConfiguration('show_windows'),value_type=bool),
            device=value('device'), v4l2_controls=value('v4l2_controls'))]),
        Node(package='robo_virtual_wall', executable='tape_keepout', name='tape_keepout',
            output='screen', parameters=[dict(remove_visible_missing=ParameterValue(
                LaunchConfiguration('remove_visible_missing'), value_type=bool),
                walls_file=value('walls_file'), walls_mode=value('walls_mode'))])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('homography_file'),
        DeclareLaunchArgument('preview_homography_file',default_value=''),
        DeclareLaunchArgument('show_windows',default_value='true'),
        DeclareLaunchArgument('device',default_value='/dev/video2'),
        # CAMERA_RECONEXAO_20261121: reapplied on every (re)open of the camera.
        DeclareLaunchArgument('v4l2_controls',default_value='focus_automatic_continuous=0,focus_absolute=150'),
        # SEM_RETIRADA_20261121: confirmed tape stays a wall. Enable only in the
        # dedicated tape-removal test; occlusion cannot be told from removal.
        DeclareLaunchArgument('remove_visible_missing',default_value='false',choices=['true','false']),
        # PAREDES_SALVAS_20261121: only with a FIXED map (arena in localization).
        DeclareLaunchArgument('walls_file',default_value=''),
        DeclareLaunchArgument('walls_mode',default_value='off',choices=['off','load','record']),
        OpaqueFunction(function=start)])
