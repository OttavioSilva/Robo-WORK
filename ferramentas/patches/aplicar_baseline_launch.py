#!/usr/bin/env python3
"""BASELINE_LEGADO_LAUNCH (11/11/2026) - navigation.launch.py passa reinsistir_goal_final:=0 ao alinhador
(REINSISTIR, 11/11, nao existia na missao24: repetia pre-giro + goal ate 2x apos o Nav2 desistir).
O padrao no codigo do alinhador continua 2 (texto do patch REINSISTIR e test_refinamento intactos).
Mesmo mecanismo ja usado para insistir_curva:=0 e curva_segue_nav2:=false. Vale para os dois perfis.
Depois de aplicar: colcon build --packages-select robo_navigation (o launch e lido do install).
Uso: python3 aplicar_baseline_launch.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/launch/navigation.launch.py')
TROCAS = [
    ('"-p", "insistir_curva:=0", "-p", "curva_segue_nav2:=false",\n',
     '"-p", "insistir_curva:=0", "-p", "curva_segue_nav2:=false",\n'
     '                     "-p", "reinsistir_goal_final:=0",  # BASELINE_LEGADO_LAUNCH (11/11)\n'),
]
if __name__ == '__main__':
    rodar('baseline_launch', 'BASELINE_LEGADO_LAUNCH', ARQ, TROCAS, doc=__doc__)
