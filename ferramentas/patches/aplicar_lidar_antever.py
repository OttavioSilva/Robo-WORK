#!/usr/bin/env python3
"""LIDAR_ANTEVER (25/11/2026): o Nav2 ve a barreira de longe e replaneja sozinho antes de chegar nela.

A arvore (NAV_INTEGRADA) ja confere o trecho restante do caminho a 2 Hz (IsPathValid) e replaneja se ele
fica invalido; faltava o global MARCAR a barreira: obstacle_max_range 3,5 m (raytrace 4,0). Sim de barreiras
25/11: o robo so "descobria" o vao fechado ja dentro dele. Agora 5,5 / 6,0 m (LiDAR sim 6 m; o M2M2 alcanca
mais). Os cortes de chao falso (CHAO_TRASEIRO/LADOS/ESQUERDA) continuam no scan_filter, antes do /scan_nav.
Uso: aplicar_lidar_antever.py | --desfazer | --arquivo <copia>
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
TROCAS = [
    ("          obstacle_max_range: 3.5\n",
     "          obstacle_max_range: 5.5  # LIDAR_ANTEVER (25/11): era 3.5\n"),
    ("          raytrace_max_range: 4.0\n",
     "          raytrace_max_range: 6.0  # LIDAR_ANTEVER (25/11): era 4.0\n"),
]


def conferir(velho, novo):
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    sa = a['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']
    sb = b['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']
    assert (sb['obstacle_max_range'], sb['raytrace_max_range']) == (5.5, 6.0), sb
    sb = dict(sb, obstacle_max_range=sa['obstacle_max_range'], raytrace_max_range=sa['raytrace_max_range'])
    b['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan'] = sb
    assert a == b, 'mudou mais do que as 2 chaves'


if __name__ == '__main__':
    rodar('lidar_antever', 'LIDAR_ANTEVER', PADRAO, TROCAS, conferir, doc=__doc__)
