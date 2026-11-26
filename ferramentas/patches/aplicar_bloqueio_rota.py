#!/usr/bin/env python3
"""BLOQUEIO_ROTA (24/11/2026): global costmap le /bloqueios_mapa (scripts/bloqueios_rota.py).

Camada StaticLayer extra so no global, depois do mapa: use_maximum (nunca apaga parede do mapa),
sem updates. Bloqueio marcado pelo piloto (MODO_PROVA nivel 2/3) fica 120 s e o NavFn procura
outra rota pela arena inteira; sem o no bloqueios_rota a camada fica vazia (igual a antes).
Uso: aplicar_bloqueio_rota.py | --desfazer | --arquivo <copia>   (depois: colcon build robo_navigation)
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')

TROCAS = [
    ("""      plugins:  # GLOBAL_OBSTACULOS
      - static_layer
      - obstacle_layer
""",
     """      plugins:  # GLOBAL_OBSTACULOS
      - static_layer
      - bloqueio_layer  # BLOQUEIO_ROTA (24/11)
      - obstacle_layer
"""),
    ("""      # NAV_INTEGRADA (10/11): tirado o 1o bloco obstacle_layer duplicado (5,0/5,5 m); valia o ultimo
""",
     """      bloqueio_layer:  # BLOQUEIO_ROTA (24/11): memoria de obstaculos que barraram (bloqueios_rota.py)
        plugin: nav2_costmap_2d::StaticLayer
        map_topic: /bloqueios_mapa
        map_subscribe_transient_local: true
        subscribe_to_updates: false
        use_maximum: true
      # NAV_INTEGRADA (10/11): tirado o 1o bloco obstacle_layer duplicado (5,0/5,5 m); valia o ultimo
"""),
]


def conferir(velho, novo):
    a = yaml.safe_load(velho)
    b = yaml.safe_load(novo)
    ga = a['global_costmap']['global_costmap']['ros__parameters']
    gb = b['global_costmap']['global_costmap']['ros__parameters']
    assert gb['plugins'] == ['static_layer', 'bloqueio_layer', 'obstacle_layer', 'inflation_layer'], gb['plugins']
    gb2 = dict(gb)
    gb2.pop('bloqueio_layer')
    gb2['plugins'] = ga['plugins']
    assert gb2 == ga, 'mudou outra chave do global'
    b['global_costmap'] = a['global_costmap']
    assert a == b, 'mudou fora do global_costmap'


if __name__ == '__main__':
    rodar('bloqueio_rota', 'BLOQUEIO_ROTA', PADRAO, TROCAS, conferir, doc=__doc__)
