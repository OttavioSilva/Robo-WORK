#!/usr/bin/env python3
"""ROTA_SEGURA_LAUNCH (11/11/2026) - navigation.launch.py passa rota_mapa_obstaculo:=recusar ao alinhador.
ROTA_MAPA troca o caminho do planner pela rota mais curta do mapa quando o planner da a volta (>1,3x). Com um objeto
do momento a < 35 cm dessa rota, 'ignorar' (padrao do codigo) ainda manda o robo pela rota bloqueada; 'recusar'
mantem o caminho do planner (que ja desvia) e registra no log. Sem objeto novo, ROTA_MAPA continua igual.
O padrao no codigo continua 'ignorar' (texto do patch ROTA_MAPA_OBSTACULO intacto).
Depois de aplicar: colcon build --packages-select robo_navigation.
Uso: python3 aplicar_rota_segura_launch.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/launch/navigation.launch.py')
TROCAS = [
    ('                     "-p", "reinsistir_goal_final:=0",  # BASELINE_LEGADO_LAUNCH (11/11)\n',
     '                     "-p", "reinsistir_goal_final:=0",  # BASELINE_LEGADO_LAUNCH (11/11)\n'
     '                     "-p", "rota_mapa_obstaculo:=recusar",  # ROTA_SEGURA_LAUNCH (11/11)\n'),
]
if __name__ == '__main__':
    rodar('rota_segura_launch', 'ROTA_SEGURA_LAUNCH', ARQ, TROCAS, doc=__doc__)
