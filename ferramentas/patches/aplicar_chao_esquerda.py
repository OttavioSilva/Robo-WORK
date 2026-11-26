#!/usr/bin/env python3
"""CHAO_ESQUERDA (23/11/2026): corte de chao dos lados 1,5 -> 1,15 m.

Medido 23/11 ~18:40 (robo parado, /scan_front): o feixe inclinado bate no chao a ESQUERDA num arco
fixo em relacao ao robo: 1,59 (15-30 graus) / 1,54 / 1,42 / 1,35 / 1,29 / 1,25 m (90-105 graus).
Com o corte em 1,5 m os retornos de 1,25-1,45 m viravam obstaculo fantasma no mapa e no CM.
Agora, nos lados (60 <= |angulo| < 130), retorno >= 1,15 m vira NaN (10 cm abaixo do chao mais perto).
Simetrico: a direita tambem perde o que estiver alem de 1,15 m de lado.
Uso: aplicar_chao_esquerda.py | --desfazer | --arquivo <copia>
"""
import os, sys
import yaml
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_lidar/config/lidar.yaml')
TROCAS = [(
    "    floor_side_range_m: 1.5         # lados: < 1,5 m = obstaculo; >= 1,5 m = chao (NaN)\n",
    "    floor_side_range_m: 1.15        # CHAO_ESQUERDA 23/11: era 1.5; chao a esquerda a 1,25-1,45 m\n",
)]


def conferir(velho, novo):
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    pa, pb = a['scan_filter']['ros__parameters'], b['scan_filter']['ros__parameters']
    dif = {k for k in set(pa) | set(pb) if pa.get(k) != pb.get(k)}
    assert dif == {'floor_side_range_m'} and pb['floor_side_range_m'] == 1.15, dif
    assert {k: v for k, v in a.items() if k != 'scan_filter'} == {k: v for k, v in b.items() if k != 'scan_filter'}


if __name__ == '__main__':
    rodar('chao_esquerda', 'CHAO_ESQUERDA', PADRAO, TROCAS, conferir=conferir, doc=__doc__)
