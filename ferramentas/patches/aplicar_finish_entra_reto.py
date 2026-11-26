#!/usr/bin/env python3
"""FINISH_ENTRA_RETO (25/11/2026, sobre FINISH_CHEGADA + FINISH_PERPENDICULAR): alinha FORA da caixa e entra reto.

Sim 25/11 03:19: so com o yaw do goal em 180 o robo chegava ao FINISH torto (-112/-120 graus: o goal checker
nao cobra angulo) e girava DENTRO da boca da caixa (vao de 1,09 m): s02 e s04 FINISH com folga 0,0 cm.
Agora, para o FINISH com 'finish_perpendicular': volta a ENTRADA_TRANSLADA (24/11), mas com o yaw de frente
para a caixa (START + 180): navega ate o ultimo ponto do caminho onde o giro cabe, gira la (pouco, ja vem
quase nesse rumo) e entra transladando reto. Os outros pontos sem mesa seguem o FINISH_CHEGADA.
Uso: aplicar_finish_entra_reto.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""                if livre and self.p('entrada_translada') and not self.p('finish_chegada'):   # ENTRADA_TRANSLADA; FINISH_CHEGADA
                    r_ent = self._entrada_translada(area, gx, gy, yaw_de(goal.pose.orientation), vigia)
""",
     """                fin_reto = livre and str(area).startswith('FINISH') and self.p('finish_perpendicular')   # FINISH_ENTRA_RETO
                if livre and self.p('entrada_translada') and (fin_reto or not self.p('finish_chegada')):   # ENTRADA_TRANSLADA; FINISH_CHEGADA
                    yaw_ent = yaw_de(goal.pose.orientation) + (math.pi if fin_reto else 0.0)   # FINISH_ENTRA_RETO
                    r_ent = self._entrada_translada(area, gx, gy, yaw_ent, vigia)
"""),
]


def conferir(velho, novo):
    assert novo.count('FINISH_ENTRA_RETO') == 2
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('finish_entra_reto', 'FINISH_ENTRA_RETO', PADRAO, TROCAS, conferir, doc=__doc__)
