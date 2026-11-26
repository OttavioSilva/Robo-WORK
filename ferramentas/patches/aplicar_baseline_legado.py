#!/usr/bin/env python3
"""BASELINE_LEGADO (11/11/2026) - alinhador. Volta o comportamento da missao24 (HEAD 7201367, 2 voltas MISSAO2
completas) no caminho legado (2D Goal Pose -> alinhador -> Nav2/destravar), sem apagar codigo novo:
  - encosto_referencia_mapa False: face da mesa so pelo LiDAR (requer FACE_LIDAR_M24 em face_ws.py);
  - REINSISTIR/CURVA_INSISTE: o padrao do codigo NAO muda (texto de outros patches e testes); o launch passa
    reinsistir_goal_final:=0 (aplicar_baseline_launch.py), como ja passava insistir_curva:=0/curva_segue_nav2:=false.
Tudo continua ajustavel por parametro (ros2 param set /alinhador_passagem ...).
Uso: python3 aplicar_baseline_legado.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("            self.declare_parameter('encosto_linear', True)\n",
     "            self.declare_parameter('encosto_linear', True)\n"
     "            # BASELINE_LEGADO (11/11): True = face escolhida pela referencia do mapa (09/11); False = so LiDAR (missao24)\n"
     "            self.declare_parameter('encosto_referencia_mapa', False)\n"),
    ("                rastreio = RastreioFaceWS(face)\n",
     "                rastreio = RastreioFaceWS(face, usar_mapa=bool(self.p('encosto_referencia_mapa')))  # BASELINE_LEGADO\n"),
]
if __name__ == '__main__':
    rodar('baseline_legado', 'BASELINE_LEGADO (11/11)', ARQ, TROCAS, doc=__doc__)
