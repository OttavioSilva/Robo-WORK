#!/usr/bin/env python3
"""RE_LONGA (12/11/2026) - destravar_node.py: re mais longa e possivel de acontecer.

Bag corredor_re_freio_curva_20261112_113352 (11:44:08-11:44:29): com permitir_re true, a re foi recusada 3x por
"area varrida 66-71% observada (minimo 90%)": o proprio corpo tapa parte da traseira para o LiDAR. O usuario pediu
re mais longa. Agora: re_max 0,30 m (era 0,15 fixo; tenta 0,30 e depois 0,10) e cobertura_re 0,65 (era 0,90).
Continuam: poligono 'backward' do CM livre, pontos do LiDAR na area varrida recusam, ICP precisa verificar e
confirmar, nunca duas res seguidas, reavalia depois.
Em tempo de execucao: ros2 param set /destravar re_max 0.15 ; ros2 param set /destravar cobertura_re 0.9
Uso: python3 aplicar_re_longa.py [--arquivo copia.py] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/destravar_node.py')
TROCAS = [
    ("                                ('cobertura_lateral', 0.5), ('cobertura_giro', 0.5), ('cobertura_re', 0.9),\n"
     "                                ('deriva_lateral', 0.4)):\n",
     "                                # RE_LONGA (12/11): cobertura_re era 0.9 (traseira so 66-71% visivel); re_max era 0.15\n"
     "                                ('cobertura_lateral', 0.5), ('cobertura_giro', 0.5), ('cobertura_re', 0.65),\n"
     "                                ('deriva_lateral', 0.4), ('re_max', 0.30)):\n"),
    ("                            deriva_lateral=float(self.get_parameter('deriva_lateral').value))\n",
     "                            deriva_lateral=float(self.get_parameter('deriva_lateral').value),\n"
     "                            re_max=max(0.10, min(0.40, float(self.get_parameter('re_max').value))))   # RE_LONGA\n"),
]
if __name__ == '__main__':
    rodar('re_longa', 'RE_LONGA (12/11)', ARQ, TROCAS, doc=__doc__)
