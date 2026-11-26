#!/usr/bin/env python3
"""SLAM_RAPIDO (24/11/2026): pontos aparecem mais rapido no mapa, ainda mais rigido que antes de 23/11.

Pedido do usuario no mapeamento 24/11: o mapa demorava a mostrar os pontos.
  occupancy_threshold 0.25 -> 0.18  (antes de 23/11: 0.10; CONFIRMA 23/11 subiu para 0.25)
  map_update_interval 2.0 -> 1.0 s  (so a frequencia de publicar o /map; nao muda o casamento)
min_pass_through 3 (CONFIRMA) e SLAM_IMU_FORTE nao mudam. Vale ao reiniciar o SLAM (yaml do src via link).
Uso: aplicar_slam_rapido.py | --desfazer | --arquivo <copia>
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/slam_mapping.yaml')

TROCAS = [
    ("""    occupancy_threshold: 0.25  # CONFIRMA 23/11: era 0.1 (SLAM_OCC01)
""",
     """    occupancy_threshold: 0.18  # SLAM_RAPIDO (24/11): era 0.25 (CONFIRMA 23/11; antes 0.1)
"""),
    ("""    map_update_interval: 2.0
""",
     """    map_update_interval: 1.0  # SLAM_RAPIDO (24/11): era 2.0
"""),
]


def conferir(velho, novo):
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    pb = b['slam_toolbox']['ros__parameters']
    assert pb['occupancy_threshold'] == 0.18 and pb['map_update_interval'] == 1.0
    pb['occupancy_threshold'], pb['map_update_interval'] = 0.25, 2.0
    assert a == b, 'mudou outra chave'


if __name__ == '__main__':
    rodar('slam_rapido', 'SLAM_RAPIDO', PADRAO, TROCAS, conferir, doc=__doc__)
