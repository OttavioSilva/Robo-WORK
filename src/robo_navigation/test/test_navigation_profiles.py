"""Resolve launch arguments in memory; never execute nodes or send goals."""
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

NAV=Path(__file__).resolve().parents[1]


def resolve(profile=None, test_launch=False):
    spec=importlib.util.spec_from_file_location('navigation_profiles', NAV/'launch'/('navigation_test.launch.py' if profile or test_launch else 'navigation.launch.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.get_package_share_directory=lambda _:str(NAV)
    description=module.generate_launch_description()
    context=LaunchContext()
    if profile:context.launch_configurations['nav_profile']=profile
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    nodes=[a for a in description.entities if isinstance(a,Node)]
    controller=next(n for n in nodes if n.node_executable=='controller_server')
    navigator=next(n for n in nodes if n.node_executable=='bt_navigator')
    return context,evaluate_parameters(context,controller._Node__parameters),evaluate_parameters(context,navigator._Node__parameters)


def test_default_still_uses_current_configuration():
    context,controller,navigator=resolve()
    assert Path(controller[0]).name=='nav2_params.yaml'
    assert Path(navigator[-1]['default_nav_to_pose_bt_xml']).name=='navigate_to_pose_smooth.xml'


def test_test_profiles_select_dwb_and_matching_tree():
    for profile in ('dwb_test','early_test'):
        context,controller,navigator=resolve(profile)
        config=yaml.safe_load(Path(controller[0]).read_text())
        assert config['controller_server']['ros__parameters']['FollowPath']['plugin']=='dwb_core::DWBLocalPlanner'
        assert Path(controller[0]).name==f'nav2_params_{profile}.yaml'
        assert Path(navigator[-1]['default_nav_to_pose_bt_xml']).name=='navigate_to_pose_early_test.xml'


def test_dwb_stage_preserves_parameters_and_all_existing_protections():
    # NAV_INTEGRADA (10/11): o principal ja e DWB direto; o perfil dwb_test ficou igual a ele.
    original=yaml.safe_load((NAV/'config/nav2_params.yaml').read_text())
    follow=original['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin']=='dwb_core::DWBLocalPlanner'
    for key in ['primary_controller','angular_dist_threshold','angular_disengage_threshold','forward_sampling_distance','rotate_to_heading_angular_vel','max_angular_accel','simulate_ahead_time','closed_loop','rotate_to_heading_once']:assert key not in follow
    # CAMINHO_ORIGINAL (11/11): o principal carrega o plugin robo_bt_plugins (so a arvore principal o usa);
    # os perfis de teste usam a arvore reduzida e nao precisam dele
    original['bt_navigator']['ros__parameters'].pop('plugin_lib_names',None)
    stage=yaml.safe_load((NAV/'config/nav2_params_dwb_test.yaml').read_text())
    assert stage==original
    early=yaml.safe_load((NAV/'config/nav2_params_early_test.yaml').read_text())
    follow['GoalAlign.scale']=0.0;follow['sim_time']=1.6
    assert early==original


def test_test_tree_fails_without_automatic_motion_recovery():
    tree=ET.parse(NAV/'behavior_trees/navigate_to_pose_early_test.xml')
    for name in ['RecoveryNode','RoundRobin','Spin','BackUp','DriveOnHeading','ClearEntireCostmap']:
        assert tree.find('.//'+name) is None
    assert tree.find('.//RateController').get('hz')=='2.0'
    assert tree.find('.//IsPathValid').get('path')=='{remaining_path}'
    assert tree.find('.//SmoothPath').get('check_for_collisions')=='true'


def test_explicit_test_launch_defaults_to_early_profile():
    context,controller,navigator=resolve(test_launch=True)
    assert context.launch_configurations['nav_profile']=='early_test'
    assert Path(controller[0]).name=='nav2_params_early_test.yaml'
    assert Path(navigator[-1]['default_nav_to_pose_bt_xml']).name=='navigate_to_pose_early_test.xml'
