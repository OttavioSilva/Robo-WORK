#!/usr/bin/env python3
"""QUINA_FOLGA (24/11/2026): planner global mais longe das quinas.

Simulacao da arena manual (24/11): 5 de 6 colisoes em quina (ponta da mesa central/parede do meio,
ilha WS09/WS10 com parede, shelf a 45 graus, toco do START); folga minima 1-3 cm nas passagens.
O NavFn so ve o raio inscrito e o custo da inflacao; com cost_scaling_factor 5 o custo cai rapido
e o caminho passa rente. So no GLOBAL: cost_scaling_factor 5 -> 3 (custo alto mais longe da quina,
o caminho se afasta onde ha espaco). inflation_radius 0,55 e o local (DWB) nao mudam; o vao de
80 cm continua passavel (o letal e so o inscrito). O footprint ja e o real: 0,57x0,63 + padding 0,01.
Uso: aplicar_quina_folga.py | --desfazer | --arquivo <copia>   (depois: colcon build robo_navigation)
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')

TROCAS = [
    ("""        inflation_radius: 0.55
        cost_scaling_factor: 5.0
      always_send_full_costmap: true
planner_server:
""",
     """        inflation_radius: 0.55
        cost_scaling_factor: 3.0  # QUINA_FOLGA (24/11): era 5.0; caminho global mais longe das quinas
      always_send_full_costmap: true
planner_server:
"""),
]


def conferir(velho, novo):
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    ib = b['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
    assert ib['cost_scaling_factor'] == 3.0
    ib['cost_scaling_factor'] = 5.0
    assert a == b, 'mudou outra chave'


if __name__ == '__main__':
    rodar('quina_folga', 'QUINA_FOLGA', PADRAO, TROCAS, conferir, doc=__doc__)
