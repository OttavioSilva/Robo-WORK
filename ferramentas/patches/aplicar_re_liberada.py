#!/usr/bin/env python3
"""RE_LIBERADA (12/11/2026) - destravar_node.py: permitir_re passa a true por padrao (pedido do usuario).

Teste no corredor 12/11 11:18 (~/logs/12-11_111354/nav2.log): robo no cruzamento com "frente 1 cm, quina_dir";
o destravar recusou retomar, contornar, lateral (pontos na area varrida) e giro, e a re estava BLOQUEADA:
"falhar (sem alternativa segura)". A etapa de re (recuperacao_avancada.py, ETAPA D) continua com as protecoes:
ate 15 cm (re_max), nunca duas seguidas, poligono 'backward' do CM livre, 90% da area varrida observada
(cobertura_re), deslocamento verificavel e confirmado pelo ICP, reavaliacao depois.
Desligar em tempo de execucao: ros2 param set /destravar permitir_re false
Uso: python3 aplicar_re_liberada.py [--arquivo copia.py] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/destravar_node.py')
TROCAS = [
    ("            for nome, valor in (('permitir_lateral', True), ('permitir_giro', True), ('permitir_re', False),\n",
     "            # RE_LIBERADA (12/11): permitir_re era False (re bloqueada ate validacao fisica); pedido do usuario\n"
     "            for nome, valor in (('permitir_lateral', True), ('permitir_giro', True), ('permitir_re', True),\n"),
]
if __name__ == '__main__':
    rodar('re_liberada', 'RE_LIBERADA (12/11)', ARQ, TROCAS, doc=__doc__)
