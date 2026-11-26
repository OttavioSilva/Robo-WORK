"""NAV_INTEGRADA: configuracao integrada (YAML/launch/arvore) e plugins reais do Nav2 em memoria.

Sondas C++ (DWB real, Navfn real, costmaps reais, BT.CPP real) rodam no dominio 166 sem descoberta;
compiladas por ferramentas/testes/validar_nav_integrada.sh (variavel SONDAS_NAV). Sem elas, pula.
Nada aqui publica no robo nem envia goal.
"""
import importlib.util
import json
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

NAV = Path(__file__).resolve().parents[1]
YAML = NAV / 'config' / 'nav2_params.yaml'
XML = NAV / 'behavior_trees' / 'navigate_to_pose_smooth.xml'
SONDAS = Path(os.environ.get('SONDAS_NAV', NAV.parents[1] / 'build' / 'sondas_nav_integrada'))
ENV = dict(os.environ, ROS_DOMAIN_ID='166', ROS_AUTOMATIC_DISCOVERY_RANGE='OFF')


def cfg():
    return yaml.safe_load(YAML.read_text())


def fp():
    return cfg()['controller_server']['ros__parameters']['FollowPath']


# ------------------------------------------------------------------ DWB direto
def test_dwb_direto_sem_parametros_do_shim():
    f = fp()
    assert f['plugin'] == 'dwb_core::DWBLocalPlanner'
    for k in ('primary_controller', 'angular_dist_threshold', 'angular_disengage_threshold',
              'forward_sampling_distance', 'rotate_to_heading_angular_vel', 'max_angular_accel',
              'simulate_ahead_time', 'closed_loop', 'rotate_to_heading_once'):
        assert k not in f
    assert 'RotationShim' not in YAML.read_text().split('collision_monitor:')[0].replace(
        'era RotationShim', '')


def test_velocidades_aceleracoes_e_criticos_preservados():
    f = fp()
    esperado = {'min_vel_x': 0.0, 'max_vel_x': 0.20, 'min_vel_y': -0.08, 'max_vel_y': 0.08,
                'max_vel_theta': 0.16, 'min_speed_xy': 0.05, 'max_speed_xy': 0.20, 'min_speed_theta': 0.15,
                'acc_lim_x': 0.2, 'acc_lim_y': 0.2, 'acc_lim_theta': 0.4, 'decel_lim_x': -0.5,
                'decel_lim_y': -0.5, 'decel_lim_theta': -0.2, 'sim_time': 0.8, 'vx_samples': 7,
                'vy_samples': 7, 'vtheta_samples': 7, 'GoalAlign.scale': 18.0, 'PathAlign.scale': 24.0,
                'PathDist.scale': 24.0, 'GoalDist.scale': 18.0, 'RotateToGoal.scale': 24.0,
                'Twirling.scale': 5.0, 'ObstacleFootprint.scale': 0.1}
    for k, v in esperado.items():
        assert f[k] == pytest.approx(v), k
    assert f['critics'] == ['RotateToGoal', 'Oscillation', 'ObstacleFootprint', 'GoalAlign', 'PathAlign',
                            'PathDist', 'GoalDist', 'Twirling']


def test_checkers_footprint_e_collision_monitor_preservados():
    c = cfg()
    cs = c['controller_server']['ros__parameters']
    assert cs['goal_checker']['yaw_goal_tolerance'] == pytest.approx(0.17)     # orientacao final obrigatoria
    assert cs['goal_checker']['xy_goal_tolerance'] == pytest.approx(0.06)
    assert cs['progress_checker']['movement_time_allowance'] == pytest.approx(6.0)
    for k in ('local_costmap', 'global_costmap'):
        assert c[k][k]['ros__parameters']['footprint'] == \
            '[[0.365, 0.315], [0.365, -0.315], [-0.205, -0.315], [-0.205, 0.315]]'
    cm = c['collision_monitor']['ros__parameters']
    assert cm['cmd_vel_in_topic'] == 'cmd_vel_smoothed' and cm['cmd_vel_out_topic'] == 'cmd_vel'
    assert cm['VelocityPolygonStop']['enabled'] is True
    assert cm['SweptTurnStop']['enabled'] is False                   # giro frontal experimental desligado
    assert cm['source_timeout'] == pytest.approx(0.5)


def test_global_costmap_consolidado():
    txt = YAML.read_text()
    g = txt[txt.index('\nglobal_costmap:'):txt.index('\nplanner_server:')]
    assert g.count('      obstacle_layer:') == 1
    gp = cfg()['global_costmap']['global_costmap']['ros__parameters']
    assert gp['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
    ob = gp['obstacle_layer']
    assert ob['observation_sources'] == 'scan'
    assert ob['scan']['topic'] == '/scan_nav' and ob['scan']['marking'] and ob['scan']['clearing']
    assert ob['scan']['obstacle_max_range'] == pytest.approx(3.5)
    assert ob['scan']['raytrace_max_range'] == pytest.approx(4.0)


# ------------------------------------------------------------------ arvore
def test_arvore_integrada_tem_todas_as_pecas():
    t = ET.parse(XML)
    rc = t.find('.//RateController')
    assert rc.get('hz') == '2.0'
    assert t.find('.//PathExpiringTimer').get('seconds') == '15'
    assert t.find('.//TruncatePathLocal').get('distance_backward') == '0.0'
    assert [e.get('path') for e in t.iter('IsPathValid')][0] == '{remaining_path}'
    assert list(t.iter('SmoothPath')) == []  # SEM_SUAVIZADOR
    sb = list(t.iter('SetBlackboard'))
    assert len(sb) == 2 and all(e.get('output_key') == 'smoothed_path' and e.get('value') == '{path}' for e in sb)
    assert len(list(t.iter('IsPathValid'))) == 1               # ORIGINAL_SEM_GATE: so o trecho restante
    drv = list(t.iter('DriveOnHeading'))
    assert len(drv) == 1 and drv[0].get('server_name') == 'destravar'
    assert t.find('.//RoundRobin') is None and t.find('.//BackUp') is None
    assert t.find('.//Spin') is None                     # o giro e decidido pelo destravar (via smart_spin)
    assert t.find('.//GoalUpdated') is not None
    # GOAL_TF_ATUAL (11/11): o FollowPath recebe a copia sem carimbo; o smoothed_path continua sendo o
    # caminho conferido (TruncatePathLocal/IsPathValid) e a entrada do CaminhoSemCarimbo
    csc = t.find('.//CaminhoSemCarimbo')
    assert csc is not None and csc.get('input_path') == '{smoothed_path}'
    assert t.find('.//FollowPath').get('path') == csc.get('output_path') == '{caminho_controlador}'
    pipe = t.find('.//PipelineSequence')
    filhos = [c.tag for c in pipe]
    assert filhos[-2:] == ['CaminhoSemCarimbo', 'FollowPath']
    assert t.find('.//TruncatePathLocal').get('input_path') == '{smoothed_path}'
    # CAMINHO_DO_GOAL: zera no inicio do goal e em cada replanejamento; exigido para manter caminho
    assert t.find('.//Sequence[@name="GoalNovo"]/Script').get('code') == 'caminho_do_goal := false'
    assert t.find('.//ReactiveSequence/ScriptCondition').get('code') == 'caminho_do_goal'
    for seq in [s for s in t.iter('Sequence') if s.find('ComputePathToPose') is not None]:
        # CAMINHO_ORIGINAL (11/11): os Scripts de caminho_bloqueado ficam de fora desta ordem
        cs = [c.get('code') for c in seq.iter('Script') if 'caminho_do_goal' in c.get('code')]
        assert cs[0] == 'caminho_do_goal := false' and cs[-1] == 'caminho_do_goal := true'
    # CAMINHO_ORIGINAL (11/11): o original so vai ao FollowPath depois da CaminhoOriginalSeguro
    originais = [seq for seq in t.iter('Sequence') if seq.find('CaminhoOriginalSeguro') is not None]
    assert len(originais) == 2
    for seq in originais:
        assert [c.tag for c in seq] == ['CaminhoOriginalSeguro', 'SetBlackboard']
        assert seq[0].get('modo') == 'registrar'


def _launch(nome):
    spec = importlib.util.spec_from_file_location('lnav', NAV / 'launch' / nome)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.get_package_share_directory = lambda _: str(NAV)
    return mod.generate_launch_description()


def test_launch_principal_sobe_os_servidores_da_arvore():
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    d = _launch('navigation.launch.py')
    nos = [a for a in d.entities if isinstance(a, Node)]
    procs = [a for a in d.entities if isinstance(a, ExecuteProcess)]
    exe = {n.node_executable for n in nos}
    assert {'controller_server', 'bt_navigator', 'collision_monitor', 'velocity_smoother',
            'progressive_spin_node', 'behavior_server', 'planner_server', 'smoother_server'} <= exe
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    ctx = LaunchContext()
    cmds = ' '.join(perform_substitutions(ctx, parte) for p in procs for parte in p.cmd
                    if all(type(x).__name__ == 'TextSubstitution' for x in parte))
    for s in ('destravar_node.py', 'suavizador_nav.py', 'filtro_corpo_scan.py', 'alinhador_passagem.py'):
        assert s in cmds
    assert 'giro_frontal.py' not in cmds


def test_destravar_v4_compila_e_le_a_decisao_do_modulo():
    import py_compile
    py_compile.compile(str(NAV / 'scripts' / 'destravar_node.py'), doraise=True)
    txt = (NAV / 'scripts' / 'destravar_node.py').read_text()
    assert "ActionServer(self, DriveOnHeading, 'destravar'" in txt
    # RE_LIBERADA (12/11): padrao do no passou a True a pedido do usuario (era False)
    assert "'permitir_re', True" in txt and 'RE_LIBERADA' in txt
    assert 'ra.Recuperador(' in txt and "'smart_spin'" in txt
    assert "'/cmd_vel_nav'" in txt and 'cmd_vel_hardware' not in txt and "'/cmd_vel'," not in txt


# ------------------------------------------------------------------ sondas com plugins reais
def _sonda(*partes):
    p = SONDAS.joinpath(*partes)
    if not p.exists():
        pytest.skip('sonda %s nao compilada (rode ferramentas/testes/validar_nav_integrada.sh)' % p)
    return str(p)


# CAMINHO_ORIGINAL (11/11): a arvore usa o plugin robo_bt_plugins (CaminhoOriginalSeguro)
PLUGIN_ROBO = NAV.parents[1] / 'install' / 'robo_bt_plugins' / 'lib' / 'librobo_caminho_original_seguro_bt_node.so'
PLUGIN_SEM_CARIMBO = NAV.parents[1] / 'install' / 'robo_bt_plugins' / 'lib' / 'librobo_caminho_sem_carimbo_bt_node.so'


def test_bt_valida_com_os_plugins_reais_do_nav2():
    env = dict(ENV)
    if 'CaminhoOriginalSeguro' in XML.read_text():
        if not PLUGIN_ROBO.exists():
            pytest.fail('arvore usa CaminhoOriginalSeguro mas %s nao existe: colcon build --packages-select '
                        'robo_bt_plugins' % PLUGIN_ROBO)
        env['ROBO_BT_PLUGINS'] = str(PLUGIN_ROBO)
    if 'CaminhoSemCarimbo' in XML.read_text():   # GOAL_TF_ATUAL (11/11)
        assert PLUGIN_SEM_CARIMBO.exists(), 'colcon build --packages-select robo_bt_plugins'
        env['ROBO_BT_PLUGINS'] = ':'.join(p for p in (env.get('ROBO_BT_PLUGINS'), str(PLUGIN_SEM_CARIMBO)) if p)
    r = subprocess.run([_sonda('dwb_probe', 'bt_probe'), str(XML)], capture_output=True, text=True,
                       env=env, timeout=60)
    assert r.returncode == 0 and 'registered_and_parsed' in r.stdout, r.stderr


def test_plugin_lib_names_bate_com_a_arvore():
    cfg = yaml.safe_load(YAML.read_text())['bt_navigator']['ros__parameters']
    usa = 'CaminhoOriginalSeguro' in XML.read_text()
    assert usa == ('robo_caminho_original_seguro_bt_node' in cfg.get('plugin_lib_names', []))
    usa2 = 'CaminhoSemCarimbo' in XML.read_text()   # GOAL_TF_ATUAL (11/11)
    assert usa2 == ('robo_caminho_sem_carimbo_bt_node' in cfg.get('plugin_lib_names', []))


def test_setblackboard_copia_caminho_tipado():
    r = subprocess.run([_sonda('setblackboard', 'sonda')], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0 and 'consumido_poses=7' in r.stdout, r.stdout + r.stderr
    assert 'caminho_do_goal=OK' in r.stdout                   # BT.CPP real: sem caminho do goal anterior


def test_dwb_real_carrega_o_yaml_e_mantem_lateral_e_diagonal():
    r = subprocess.run([_sonda('dwb_probe', 'dwb_probe'), '--ros-args', '--params-file', str(YAML)],
                       capture_output=True, text=True, env=ENV, timeout=120)
    linha = [ln for ln in (r.stdout + r.stderr).splitlines() if ln.startswith('samples=')]
    assert linha, r.stdout[-2000:] + r.stderr[-2000:]
    v = dict(kv.split('=') for kv in linha[-1].split())
    assert int(v['pure_left']) > 0 and int(v['pure_right']) > 0 and int(v['diagonal']) > 0
    assert int(v['rear']) == 0                                  # DWB_SEM_RE preservado
    assert float(v['max_y']) == pytest.approx(0.08)


def _cenario(nome, tmp_path, replan=False):
    out = tmp_path / ('%s.json' % nome)
    cmd = [_sonda('native_navigation', 'native_navigation'), '--ros-args', '--params-file', str(YAML),
           '-p', 'experiment.scenario:=%s' % nome, '-p', 'experiment.output:=%s' % out,
           '-p', 'experiment.replanning:=%s' % ('true' if replan else 'false')]
    subprocess.run(cmd, capture_output=True, text=True, env=ENV, timeout=240)
    assert out.exists(), 'sonda nao gerou %s' % out
    return json.loads(out.read_text())


def test_mecanum_lateral_sem_girar(tmp_path):
    r = _cenario('lateral', tmp_path)
    assert r['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert r['complete'] and r['intermediate_rotation_rad'] == pytest.approx(0.0, abs=0.05)
    assert r['collisions'] == 0


def test_orientacao_final_obrigatoria(tmp_path):
    r = _cenario('final_yaw', tmp_path)
    assert r['complete']                                       # goal_checker exige yaw (0,17 rad)
    assert abs(r['yaw'] - 1.5708) <= 0.17 + 0.02


@pytest.mark.parametrize('nome', ['open', 'curve'])
def test_caminho_livre_e_curva(nome, tmp_path):
    r = _cenario(nome, tmp_path)
    assert r['complete'] and r['collisions'] == 0


@pytest.mark.parametrize('nome', ['obstacle', 'dynamic', 'corner'])  # near: 1 intersecao ja no Shim (sem CM no laco)
def test_sem_intersecao_com_caixa_no_laco_ideal(nome, tmp_path):
    r = _cenario(nome, tmp_path, replan=True)
    assert r['collisions'] == 0, r                             # concluir o contorno NAO e exigido aqui


def test_scan_novo_altera_o_planejamento_global():
    r = subprocess.run([_sonda('native_navigation', 'scan_replan_probe'), '--ros-args', '--params-file', str(YAML)],
                       capture_output=True, text=True, env=ENV, timeout=180)
    txt = r.stdout + r.stderr
    linha = [ln for ln in txt.splitlines() if 'configured_source=' in ln]
    assert linha, txt[-2000:]
    v = dict(kv.split('=') for kv in linha[-1].split() if '=' in kv)
    assert v['configured_source'] == '/scan_nav'
    assert float(v['after_max_y']) > float(v['before_max_y']) + 0.2   # o caminho mudou de passagem
    assert int(v['unsafe_plan_cells']) == 0


def test_truncate_path_local_entrega_so_o_restante():
    r = subprocess.run([_sonda('truncate_probe', 'truncate_probe')], capture_output=True, text=True,
                       env=ENV, timeout=60)
    assert 'remaining_points=' in r.stdout, r.stdout + r.stderr
