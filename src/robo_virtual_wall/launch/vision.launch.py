from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = dict(width='640', height='480', fps='30.0', fourcc='MJPG',
                    preview_allow_resize='false', use_sim_time='false', device='/dev/video2', image_topic='', show_windows='false', calibrate='false',
                    homography_file='', preview_homography_file='', roi_top='0.10', roi_bottom='0.85', self_mask_file='', self_mask_mode='static')
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    params = {k: LaunchConfiguration(k) for k in defaults}
    for key in ('width', 'height'):
        params[key] = ParameterValue(LaunchConfiguration(key), value_type=int)
    params['fps'] = ParameterValue(LaunchConfiguration('fps'), value_type=float)
    params['preview_allow_resize'] = ParameterValue(LaunchConfiguration('preview_allow_resize'), value_type=bool)
    params['use_sim_time'] = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    params['show_windows'] = ParameterValue(LaunchConfiguration('show_windows'), value_type=bool)
    params['calibrate'] = ParameterValue(LaunchConfiguration('calibrate'), value_type=bool)
    params['roi_top'] = ParameterValue(LaunchConfiguration('roi_top'), value_type=float)
    params['roi_bottom'] = ParameterValue(LaunchConfiguration('roi_bottom'), value_type=float)
    return LaunchDescription(args + [Node(package='robo_virtual_wall', executable='virtual_wall',
                                         name='virtual_wall', parameters=[params], output='screen')])
