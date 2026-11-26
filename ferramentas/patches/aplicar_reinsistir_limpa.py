#!/usr/bin/env python3
"""REINSISTIR_LIMPA (25/11/2026): no REINSISTIR, limpa os costmaps antes de mandar o goal de novo.

Marcas velhas (pose reposta, gente que ja saiu) fazem o planner recusar o mesmo caminho a cada tentativa
(24/11: WS07 1a tentativa, "objeto do momento" na aproximacao). Par de aplicar_reinsistir_sempre.py.
Uso: aplicar_reinsistir_limpa.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [("""                        fim_espera = time.monotonic() + 1.0
                        while time.monotonic() < fim_espera and not self._novo_goal_chegou():
                            time.sleep(0.05)
                        self.hist_nav = None
                        continue
""", """                        self._limpar_costmaps()   # REINSISTIR_LIMPA (25/11): marcas velhas fora antes de replanejar
                        fim_espera = time.monotonic() + 1.5
                        while time.monotonic() < fim_espera and not self._novo_goal_chegou():
                            time.sleep(0.05)
                        self.hist_nav = None
                        continue
""")]


def conferir(v, n):
    assert n.count('REINSISTIR_LIMPA') == 1
    compile(n, 'alin', 'exec')


if __name__ == '__main__':
    rodar('reinsistir_limpa', 'REINSISTIR_LIMPA', PADRAO, TROCAS, conferir, doc=__doc__)
