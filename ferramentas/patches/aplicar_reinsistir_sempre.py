#!/usr/bin/env python3
"""REINSISTIR_SEMPRE (25/11/2026): Nav2 desistiu -> reanalisa e manda de novo (ate 8x), em vez de FALHOU direto.

Bags 24/11: WS07 (1a), SH01 e 3x WS08 deram FALHOU na 1a desistencia do Nav2: o launch subia o alinhador com
reinsistir_goal_final:=0 (BASELINE_LEGADO_LAUNCH 11/11). Pedido 25/11: replanejar quantas vezes for preciso.
navigation.launch.py: reinsistir_goal_final 0 -> 8. Par: aplicar_reinsistir_limpa.py (limpa costmaps antes).
Uso: aplicar_reinsistir_sempre.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/launch/navigation.launch.py')
TROCAS = [("""                     "-p", "reinsistir_goal_final:=0",  # BASELINE_LEGADO_LAUNCH (11/11)
""", """                     "-p", "reinsistir_goal_final:=8",  # REINSISTIR_SEMPRE (25/11): era 0 (BASELINE_LEGADO_LAUNCH 11/11)
""")]


def conferir(v, n):
    assert n.count('REINSISTIR_SEMPRE') == 1


if __name__ == '__main__':
    rodar('reinsistir_sempre', 'REINSISTIR_SEMPRE', PADRAO, TROCAS, conferir, doc=__doc__)
