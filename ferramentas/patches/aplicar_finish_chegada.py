#!/usr/bin/env python3
"""FINISH_CHEGADA (25/11/2026): ponto sem mesa (FINISH) entra de frente, no angulo de chegada; sem giro de 180.

Sim sem barreiras (25/11 02:29, mundo corrigido, LiDAR M2M2): as idas as mesas foram todas OK; o caos era a
volta ao FINISH. O FINISH guarda o yaw do START (0,01 rad, de frente para a arena). O robo chega de leste,
rumo oeste, e a ENTRADA_TRANSLADA (24/11) procurava um ponto de giro na boca da caixa (vao de 1,09 m entre o
toco norte do START e a parede do meio) para girar ~180 e entrar de re transladando:
  s03: ponto de giro (1,31; 5,41) colado na ponta da parede do meio -> frente 1 cm, folga 1,2 cm, destravar
       sem alternativa -> FINISH FALHOU;
  s04: 3 destravamentos (lateral, lateral, re) -> 125 s para 7,2 m.
A prova nao cobra o angulo no FINISH (so estar na area). Agora (param 'finish_chegada', false = como antes):
ponto sem mesa vai direto pelo goal final com o angulo de chegada (v3.10), sem ENTRADA_TRANSLADA.
Uso: aplicar_finish_chegada.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""            self.declare_parameter('entrada_translada', True)  # ENTRADA_TRANSLADA (24/11)
""",
     """            self.declare_parameter('entrada_translada', True)  # ENTRADA_TRANSLADA (24/11)
            self.declare_parameter('finish_chegada', True)     # FINISH_CHEGADA (25/11): sem mesa entra de frente
"""),
    ("""                if livre and self.p('entrada_translada'):   # ENTRADA_TRANSLADA (24/11)
""",
     """                if livre and self.p('entrada_translada') and not self.p('finish_chegada'):   # ENTRADA_TRANSLADA; FINISH_CHEGADA
"""),
]


def conferir(velho, novo):
    assert novo.count('FINISH_CHEGADA') == 2
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('finish_chegada', 'FINISH_CHEGADA', PADRAO, TROCAS, conferir, doc=__doc__)
