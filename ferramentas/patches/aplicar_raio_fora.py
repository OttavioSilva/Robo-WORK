#!/usr/bin/env python3
"""RAIO_FORA (25/11/2026, corrige ENTRADA_ASSUME): o raio de aceitacao do ponto de giro nunca deixa o robo
chegar mais perto do FINISH que o proprio ponto de giro menos 15 cm.

Teste FINISH 25/11 09:06: ponto de giro a 32 cm do FINISH e raio 0,30 -> o Nav2 "chegou" ja na boca da caixa
(f_pp01: (0,88; 5,60), bico a 0,505 contra a ponta do toco do START a 0,51 -> COLISAO). Agora o raio e
min(raio_ponto_giro, max(0,10, d(ponto de giro, FINISH) - 0,15)). Uso: aplicar_raio_fora.py | --desfazer
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""                self._raio_tmp = float(self.p('raio_ponto_giro')) or None   # ENTRADA_ASSUME
""",
     """                self._raio_tmp = (min(float(self.p('raio_ponto_giro')),   # ENTRADA_ASSUME; RAIO_FORA (25/11)
                                      max(0.10, math.hypot(gx - p[0], gy - p[1]) - 0.15))
                                  if float(self.p('raio_ponto_giro')) > 0 else None)
"""),
]


def conferir(velho, novo):
    assert novo.count('RAIO_FORA') == 1
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('raio_fora', 'RAIO_FORA', PADRAO, TROCAS, conferir, doc=__doc__)
