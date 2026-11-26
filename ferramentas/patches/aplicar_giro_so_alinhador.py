#!/usr/bin/env python3
"""GIRO_SO_ALINHADOR (25/11/2026): o Nav2 so leva a POSICAO; o giro final e sempre do alinhador (giro seguro).

Causa dos "No valid trajectories" (24/11, 9 casos): a <= 6 cm do goal a critica RotateToGoal do DWB so aceita
giro parado (para acertar o yaw do goal, tolerancia 0,17); parado, os unicos giros validos sao +-0,16 rad/s
(min_speed_theta 0,15) e o giro no lugar varre ~0,5 m de raio: perto de mesa/parede bate -> 100% recusadas
("1.00: RotateToGoal/Nonrotation command near goal") -> controle falha -> goal cai.
Todos os fluxos do alinhador ja giram ele mesmo depois (GIRO_FINAL nas mesas, ENTRADA_TRANSLADA no START/FINISH,
preparo/refino, CHEGADA_PERTO); goal sem area passa a ir pelo mesmo caminho com giro_final_comum.
  nav2_params.yaml: goal_checker.yaw_goal_tolerance 0.17 -> 3.14 (posicao 6 cm igual)
  navigation.launch.py: alinhador giro_final_comum:=true
Uso: aplicar_giro_so_alinhador.py | --desfazer   (os 2 arquivos)
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PARAMS = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
LAUNCH = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/launch/navigation.launch.py')
T_PARAMS = [("""      yaw_goal_tolerance: 0.17  # CHEGADA_10G (05/11): era 0.035; o encosto acerta o angulo
""", """      yaw_goal_tolerance: 3.14  # GIRO_SO_ALINHADOR (25/11): era 0.17 (CHEGADA_10G); o giro final e do alinhador
""")]
T_LAUNCH = [("""                     "-p", "rota_mapa_obstaculo:=recusar",  # ROTA_SEGURA_LAUNCH (11/11)
""", """                     "-p", "rota_mapa_obstaculo:=recusar",  # ROTA_SEGURA_LAUNCH (11/11)
                     "-p", "giro_final_comum:=true",  # GIRO_SO_ALINHADOR (25/11): goal sem area tambem: Nav2 so posicao
""")]


def c_params(v, n):
    a, b = yaml.safe_load(v), yaml.safe_load(n)
    gc = b['controller_server']['ros__parameters']['goal_checker']
    assert gc['yaw_goal_tolerance'] == 3.14
    gc['yaw_goal_tolerance'] = 0.17
    assert a == b, 'mudou outra chave'


def c_launch(v, n):
    assert n.count('GIRO_SO_ALINHADOR') == 1
    compile(n, 'launch', 'exec')


if __name__ == '__main__':
    desf = '--desfazer' in sys.argv
    sys.argv = [sys.argv[0]] + (['--desfazer'] if desf else [])
    rodar('giro_so_alinhador', 'GIRO_SO_ALINHADOR', PARAMS, T_PARAMS, c_params, doc=__doc__)
    rodar('giro_so_alinhador', 'GIRO_SO_ALINHADOR', LAUNCH, T_LAUNCH, c_launch, doc=__doc__)
