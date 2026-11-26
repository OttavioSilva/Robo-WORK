#!/usr/bin/env python3
"""GIRO_FINAL_COMUM (11/11/2026) - Patch 02. Parametro novo do alinhador `giro_final_comum` (PADRAO False:
nada muda). Ligado, o goal SEM area recebe o mesmo tratamento do GIRO_FINAL das mesas: o Nav2 leva so ate a
posicao com o rumo de chegada do caminho e o alinhador faz o giro para o yaw pedido (_girar_seguro, que
para). O yaw final pedido e mantido. Motivo (bag 2nav 12:37:22-41): no goal, o DWB so tinha 1 trajetoria
valida (girar +0,16; min_speed_theta 0,15 + decel_lim_theta -0,2 nao deixam frear nem inverter) e passou
126 graus do yaw do goal.
A/B sem reiniciar:  ros2 param set /alinhador_passagem giro_final_comum true   (volta: false)
Uso: python3 aplicar_giro_final_comum.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("            self.declare_parameter('giro_final_alinhador', True)  # v3.32 GIRO_FINAL nas mesas\n",
     "            self.declare_parameter('giro_final_alinhador', True)  # v3.32 GIRO_FINAL nas mesas\n"
     "            # GIRO_FINAL_COMUM (11/11): goal sem area tambem com o giro final pelo alinhador (padrao: nao)\n"
     "            self.declare_parameter('giro_final_comum', False)\n"),
    ("                    if area is not None and not livre and self.p('giro_final_alinhador'):\n",
     "                    # GIRO_FINAL_COMUM (11/11): com giro_final_comum, goal sem area segue o mesmo caminho\n"
     "                    if (area is not None and not livre and self.p('giro_final_alinhador')) or \\\n"
     "                            (area is None and self.p('giro_final_comum')):\n"),
]
if __name__ == '__main__':
    rodar('giro_final_comum', 'GIRO_FINAL_COMUM (11/11)', ARQ, TROCAS, doc=__doc__)
