#!/usr/bin/env python3
"""DWB_QUINA (24/11/2026): DWB pesa a proximidade de obstaculo (ObstacleFootprint 0,1 -> 0,5).

Simulacao (s04, 24/11, pontos de toque gravados): voltando da PP01 o robo cortou a quina NE da mesa
central (WS05/WS06) com o canto dianteiro esquerdo, pose (4,01; 5,06; 154 graus), e ficou 20 s preso.
Com 0,1 o DWB praticamente so segue o caminho (PathAlign/PathDist 24) e so recusa colisao certa;
o NavFn traca rente a quina. A inflacao do GLOBAL (QUINA_FOLGA) foi desfeita: quebrava a deteccao de
corredor do alinhador nas ilhas. Aqui so o controle local: custo de inflacao sob o footprint passa a
competir com seguir o caminho (custo ~200 perto da quina x 0,5 = 100 ~ 4 celulas fora do caminho).
Uso: aplicar_dwb_quina.py | --desfazer | --arquivo <copia>   (depois: colcon build robo_navigation)
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')

TROCAS = [
    ("""      ObstacleFootprint.scale: 0.1
""",
     """      ObstacleFootprint.scale: 0.5  # DWB_QUINA (24/11): era 0.1; cortava a quina da mesa central
"""),
]


def conferir(velho, novo):
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    fp = b['controller_server']['ros__parameters']['FollowPath']
    assert fp['ObstacleFootprint.scale'] == 0.5
    fp['ObstacleFootprint.scale'] = 0.1
    assert a == b, 'mudou outra chave'


if __name__ == '__main__':
    rodar('dwb_quina', 'DWB_QUINA', PADRAO, TROCAS, conferir, doc=__doc__)
