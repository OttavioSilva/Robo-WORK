#!/usr/bin/env python3
"""TELEOP_CAIXA2 (24/11/2026): a arena tem 2 caixas de START/FINISH; o teleop marca as duas no mesmo mapa.

D-pad cima/baixo (2x) continuam marcando START/FINISH (caixa 1). Novos:
  D-pad esquerda (2x) -> START_2   D-pad direita (2x) -> FINISH_2   (caixa 2, mesmo frame map)
Gravados no mesmo pontos.yaml. View lista os 4. Y 2x (mapa novo) apaga todos, como antes.
Na prova: ferramentas/subida_unica/escolher_caixa.py <mapa> 1|2 copia a caixa para START/FINISH.
Uso: aplicar_teleop_caixa2.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/ferramentas/teleop/teleop_mapeamento_joy.py')

TROCAS = [
    ("""  D-pad baixo (2x)      marca o FINISH na pose atual
""",
     """  D-pad baixo (2x)      marca o FINISH na pose atual
  D-pad esquerda (2x)   marca o START_2 (caixa 2) na pose atual   # TELEOP_CAIXA2 (24/11)
  D-pad direita (2x)    marca o FINISH_2 (caixa 2) na pose atual
"""),
    ("""        for k in ('START', 'FINISH'):
            if k in pontos:""",
     """        for k in ('START', 'FINISH', 'START_2', 'FINISH_2'):   # TELEOP_CAIXA2 (24/11)
            if k in pontos:"""),
    ("""                    nome = 'START' if novo < 0 else 'FINISH'   # cima = -1
                    if confirmado(nome, 'marcar o %s aqui' % nome):
                        marcar_ponto(nome)
""",
     """                    nome = 'START' if novo < 0 else 'FINISH'   # cima = -1
                    if confirmado(nome, 'marcar o %s aqui' % nome):
                        marcar_ponto(nome)
                elif ev[0] == 'a' and ev[1] == E['hat_x']:   # TELEOP_CAIXA2 (24/11): caixa 2
                    novo, antigo = round(ev[2]), round(ev[3])
                    if novo == 0 or novo == antigo:
                        continue
                    nome = 'START_2' if novo < 0 else 'FINISH_2'   # esquerda = -1
                    if confirmado(nome, 'marcar o %s (caixa 2) aqui' % nome):
                        marcar_ponto(nome)
"""),
]


def conferir(velho, novo):
    assert novo.count('TELEOP_CAIXA2') == 3, novo.count('TELEOP_CAIXA2')
    compile(novo, 'teleop', 'exec')


if __name__ == '__main__':
    rodar('teleop_caixa2', 'TELEOP_CAIXA2', PADRAO, TROCAS, conferir, doc=__doc__)
