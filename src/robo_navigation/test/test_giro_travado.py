"""GIRO_TRAVADO (11/11/2026): reproducao offline do giro sem fim no goal (bag 2nav 12:37:10-42) com o codigo REAL
do Nav2 1.3.12 (sondas em test/sondas_nav2/giro_travado, compiladas por ferramentas/testes/validar_giro_travado.sh).
Fixture: map->odom, pose odom->base_link e fim do caminho por ciclo do controller, extraidos do bag.
Sem ROS de rede (dominio isolado), sem movimento."""
import os
import re
import subprocess
from pathlib import Path

import pytest

NAV = Path(__file__).resolve().parents[1]
YAML = NAV / 'config' / 'nav2_params.yaml'
FIX = Path(__file__).resolve().parent / 'dados' / 'giro_travado'
SONDAS = Path(os.environ.get('SONDAS_NAV', NAV.parents[1] / 'build' / 'sondas_nav_integrada')) / 'giro_travado'
ENV = dict(os.environ, ROS_DOMAIN_ID='166', ROS_AUTOMATIC_DISCOVERY_RANGE='OFF')


def _sonda(nome):
    p = SONDAS / nome
    if not p.exists():
        pytest.skip('sonda %s nao compilada (bash ferramentas/testes/validar_giro_travado.sh)' % p)
    return str(p)


def _janela(wz, *extra):
    r = subprocess.run([_sonda('janela_dwb'), str(wz), '--ros-args', '--params-file', str(YAML), *extra],
                       capture_output=True, text=True, env=ENV, timeout=60)
    m = re.search(r'amostras_validas=(\d+) giro_parado=\[ (.*)\]', r.stdout)
    assert m, r.stdout + r.stderr
    return int(m.group(1)), [float(x) for x in m.group(2).split()]


def test_goal_checker_atual_nunca_encerra_e_com_carimbo_zero_encerra_as_12_37_29():
    r = subprocess.run([_sonda('goal_checker_replay'), str(FIX), '--ros-args', '--params-file', str(YAML)],
                       capture_output=True, text=True, env=ENV, timeout=120)
    linhas = dict(re.findall(r'modo=(\w+) (chegou=\w+ t=[-\d.]+)', r.stdout))
    assert linhas['atual'].startswith('chegou=NAO'), r.stdout       # o que aconteceu no robo
    t = float(linhas['carimbo0'].split('t=')[1])
    assert linhas['carimbo0'].startswith('chegou=SIM') and 19.0 <= t <= 19.6   # 12:37:10 + 19,3 s = 12:37:29,3


LIB_SEM_CARIMBO = NAV.parents[1] / 'install' / 'robo_bt_plugins' / 'lib' / 'librobo_caminho_sem_carimbo_bt_node.so'


def test_replay_com_o_no_real_da_arvore_aplicada_encerra_as_12_37_29():
    """GOAL_TF_ATUAL (11/11): o fim do caminho passa pelo CaminhoSemCarimbo instalado antes do goal checker."""
    import xml.etree.ElementTree as ET
    arv = ET.parse(NAV / 'behavior_trees' / 'navigate_to_pose_smooth.xml')
    csc = arv.find('.//CaminhoSemCarimbo')
    assert csc is not None and arv.find('.//FollowPath').get('path') == csc.get('output_path')
    assert LIB_SEM_CARIMBO.exists(), 'colcon build --packages-select robo_bt_plugins'
    env = dict(ENV, ROBO_SEM_CARIMBO_LIB=str(LIB_SEM_CARIMBO))
    r = subprocess.run([_sonda('goal_checker_replay'), str(FIX), '--ros-args', '--params-file', str(YAML)],
                       capture_output=True, text=True, env=env, timeout=120)
    linhas = dict(re.findall(r'modo=(\w+) (chegou=\w+ t=[-\d.]+)', r.stdout))
    assert linhas['atual'].startswith('chegou=NAO'), r.stdout
    assert linhas['arvore'] == linhas['carimbo0'], r.stdout
    assert 19.0 <= float(linhas['arvore'].split('t=')[1]) <= 19.6


def test_dwb_girando_anti_horario_so_pode_continuar_girando():
    n, parado = _janela(0.16)
    assert n == 392                      # o mesmo total do /evaluation de 12:37:28-38
    assert parado == [0.16]              # nem frear, nem inverter: so +0,16
    assert _janela(0.23)[1] == [0.16]    # a odom media ~0,23 rad/s (vale o mesmo, a velocidade e limitada a 0,16)


def test_dwb_parado_ou_horario_pode_escolher_os_dois_sentidos():
    assert _janela(0.0)[1] == [-0.16, 0.16]
    assert _janela(-0.16)[1] == [-0.16, 0.16]   # assimetria: acc_lim aumenta o valor com sinal, decel_lim diminui


def test_so_decel_lim_theta_menos_0_4_destravaria_a_inversao():
    # comparacao de diagnostico, NAO aplicada: com -0,4 o giro anti-horario pode inverter
    assert _janela(0.16, '-p', 'FollowPath.decel_lim_theta:=-0.4')[1] == [-0.16, 0.16]


def test_bag_1148_modelo_fiel_e_goal_tf_atual_sem_regressao():
    """Os 5 'Reached the goal!' reais do bag nav_integrada 11:48: o modo atual reproduz os 5 no mesmo ciclo (o modelo
    e fiel, incluindo o fallback do tf_help para caminho com mais de 10 s) e a arvore com GOAL_TF_ATUAL tambem chega."""
    import json
    fix = Path(__file__).resolve().parent / 'dados' / 'giro_travado_bag_114847'
    reais = json.loads((fix / 'reached_reais.json').read_text())
    assert len(reais) == 5
    env = dict(ENV, ROBO_SEM_CARIMBO_LIB=str(LIB_SEM_CARIMBO), POR_CICLO='1')
    r = subprocess.run([_sonda('goal_checker_replay'), str(fix), '--ros-args', '--params-file', str(YAML)],
                       capture_output=True, text=True, env=env, timeout=300)
    cic = {}
    for m in re.finditer(r'ciclo modo=(\w+) t=([\d.]+) chegou=(\d)', r.stdout):
        cic.setdefault(m.group(1), []).append((float(m.group(2)), m.group(3) == '1'))
    for modo in ('atual', 'arvore'):
        for t in reais:
            assert any(c for (tc, c) in cic[modo] if abs(tc - t) <= 0.15), (modo, t)
