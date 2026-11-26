#!/usr/bin/env python3
"""MEMORIA_LONGA (25/11/2026, junto com MEMORIA_BARREIRA): bloqueios_rota.py marca ate 3,0 m (era 1,5) e
lembra por 300 s (era 120): uma barreira que fecha um vao continua fechada na volta da mesma prova.
Uso: aplicar_memoria_longa.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/bloqueios_rota.py')
TROCAS = [
    ("ALCANCE = 1.5\n", "ALCANCE = 3.0   # MEMORIA_LONGA (25/11): era 1.5\n"),
    ("VALIDADE = 120.0\n", "VALIDADE = 300.0   # MEMORIA_LONGA (25/11): era 120\n"),
]

if __name__ == '__main__':
    rodar('memoria_longa', 'MEMORIA_LONGA', PADRAO, TROCAS, None, doc=__doc__)
