#!/usr/bin/env python3
"""ENTRADA_ASSUME (25/11/2026): o Nav2 so leva PERTO do ponto de giro; o alinhador gira e entra.

Teste so de FINISH (25/11 08:53, cenarios_finish.yaml): com FINISH_ENTRA_RETO o ponto de giro e escolhido
certo (ex.: (1,19; 5,42) para 181 graus), mas o DWB nao para fino nele (posicao + rumo em 10 cm): anti-ciscado
em 3 de 4 casos e o destravar girando 127-173 graus perto da parede sob a WS09 -> 2 colisoes (folga 0,0).
Operador faria: chega perto, para, gira onde cabe e entra. Agora (param 'raio_ponto_giro', 0 = como antes):
o ponto de giro e aceito a ate 0,30 m (giro_livre ja foi conferido ali e em volta, no caminho); o
_girar_seguro (confere a area varrida) gira e o _deslocar_ate leva ao FINISH.
Uso: aplicar_entrada_assume.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""            self.declare_parameter('finish_perpendicular', True)   # FINISH_PERPENDICULAR (25/11)
""",
     """            self.declare_parameter('finish_perpendicular', True)   # FINISH_PERPENDICULAR (25/11)
            self.declare_parameter('raio_ponto_giro', 0.30)   # ENTRADA_ASSUME (25/11): 0 = como antes
"""),
    ("""                r = self._navegar(alvo, n + ' (ponto de giro)', vigia, aceitar_perto=True)
""",
     """                self._raio_tmp = float(self.p('raio_ponto_giro')) or None   # ENTRADA_ASSUME
                try:
                    r = self._navegar(alvo, n + ' (ponto de giro)', vigia, aceitar_perto=True)
                finally:
                    self._raio_tmp = None
"""),
    ("""                                                         pm[1] - alvo.pose.position.y) <= self.p('raio_chegada'):
""",
     """                                                         pm[1] - alvo.pose.position.y) <= \\
                                (getattr(self, '_raio_tmp', None) or self.p('raio_chegada')):   # ENTRADA_ASSUME
"""),
]


def conferir(velho, novo):
    assert novo.count('ENTRADA_ASSUME') == 3
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('entrada_assume', 'ENTRADA_ASSUME', PADRAO, TROCAS, conferir, doc=__doc__)
