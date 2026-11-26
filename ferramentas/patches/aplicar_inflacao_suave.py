#!/usr/bin/env python3
"""INFLACAO_SUAVE (25/11/2026): o caminho global passa longe das quinas em vez de raspar nelas.

Sim sem barreiras 25/11 02:29 (mundo corrigido, M2M2): s05 SH04 COLIDIU (folga 0,0 cm) contornando a ponta do
toco junto da SH03 em (4,37; 8,09): o NavFn traca rente porque com inflation_radius 0,55 e cost_scaling 5 o
custo cai a quase zero poucos cm alem do raio circunscrito (~0,48). Agora, so no GLOBAL: raio 0,80 e
cost_scaling 2,5: o custo desce devagar e o caminho mais barato fica no meio dos vaos (centraliza; barreira
fina inflada vira faixa larga). O local (DWB) fica como estava.
Uso: aplicar_inflacao_suave.py | --desfazer | --arquivo <copia>
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
planner_server:""",
     """        inflation_radius: 0.80  # INFLACAO_SUAVE (25/11): era 0.55
        cost_scaling_factor: 2.5  # INFLACAO_SUAVE: era 5.0
      always_send_full_costmap: true
planner_server:"""),
]


def conferir(velho, novo):
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    ga = a['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
    gb = b['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
    assert (gb['inflation_radius'], gb['cost_scaling_factor']) == (0.80, 2.5), gb
    b['global_costmap']['global_costmap']['ros__parameters']['inflation_layer'] = ga
    assert a == b, 'mudou mais do que o inflation_layer do global'


if __name__ == '__main__':
    rodar('inflacao_suave', 'INFLACAO_SUAVE', PADRAO, TROCAS, conferir, doc=__doc__)
