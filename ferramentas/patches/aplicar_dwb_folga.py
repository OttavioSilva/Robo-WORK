#!/usr/bin/env python3
"""DWB_FOLGA (25/11/2026): o controle local passa a pesar a proximidade de parede/quina, nao so o caminho.

Sim 25/11 02:49: com INFLACAO_SUAVE o caminho global foi para o meio, mas o DWB (ObstacleFootprint 0,1 contra
PathDist/PathAlign 24) seguia colado: s06 WS04 parou no vao leste junto da ponta da parede (5,49; 5,27) com
o Collision Monitor parando 11x, anti-ciscado 2x e FALHOU; s04 FINISH encostou (folga 0,0). Agora:
ObstacleFootprint.scale 0,1 -> 0,6 e o local com cost_scaling 5,0 -> 3,0 (raio 0,55 mantido): trajetorias
que raspam ficam mais caras, sem deixar de existir. Uso: aplicar_dwb_folga.py | --desfazer | --arquivo <copia>
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
TROCAS = [
    ("      ObstacleFootprint.scale: 0.1\n      PathAlign.scale: 24.0\n",
     "      ObstacleFootprint.scale: 0.6  # DWB_FOLGA (25/11): era 0.1\n      PathAlign.scale: 24.0\n"),
    ("""        inflation_radius: 0.55
        cost_scaling_factor: 5.0
      always_send_full_costmap: true
global_costmap:""",
     """        inflation_radius: 0.55
        cost_scaling_factor: 3.0  # DWB_FOLGA (25/11): era 5.0
      always_send_full_costmap: true
global_costmap:"""),
]


def conferir(velho, novo):
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    fa = a['controller_server']['ros__parameters']['FollowPath']
    fb = b['controller_server']['ros__parameters']['FollowPath']
    la = a['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']
    lb = b['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']
    assert fb['ObstacleFootprint.scale'] == 0.6 and lb['cost_scaling_factor'] == 3.0
    fb['ObstacleFootprint.scale'] = fa['ObstacleFootprint.scale']
    lb['cost_scaling_factor'] = la['cost_scaling_factor']
    assert a == b, 'mudou mais do que as 2 chaves'


if __name__ == '__main__':
    rodar('dwb_folga', 'DWB_FOLGA', PADRAO, TROCAS, conferir, doc=__doc__)
