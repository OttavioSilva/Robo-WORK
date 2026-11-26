#!/usr/bin/env python3
"""BASELINE_LEGADO_SUBIDA (11/11/2026) - ~/subir_robo.sh sobe por padrao a arquitetura legada
(2D Goal Pose -> alinhador -> Nav2/destravar, nav2_principal:=false, sem arbitro/supervisor).
O supervisor novo continua disponivel, so por opcao explicita: --supervisor (observar) ou --supervisor-aplicar.
--legado continua aceito (agora e o padrao). SLAM/mapa/ordem de subida nao mudam.
Uso: python3 aplicar_baseline_subida.py [--arquivo copia.sh] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/subir_robo.sh')
TROCAS = [
    ("# SUBIDA_SUPERVISOR_20261111: novo perfil por padrao; --legado e rollback explicito.\n",
     "# BASELINE_LEGADO_SUBIDA (11/11): arquitetura legada por padrao; --supervisor para o perfil novo.\n"
     "# SUBIDA_SUPERVISOR_20261111: novo perfil por padrao; --legado e rollback explicito.\n"),
    ("NAV_LEGADO=0; SUPERVISOR_MODO=observar\n",
     "NAV_LEGADO=1; SUPERVISOR_MODO=observar  # BASELINE_LEGADO_SUBIDA: era NAV_LEGADO=0\n"),
    ("    --supervisor-observar) SUPERVISOR_MODO=observar ;;\n    --supervisor-aplicar) SUPERVISOR_MODO=aplicar ;;\n",
     "    --supervisor|--supervisor-observar) NAV_LEGADO=0; SUPERVISOR_MODO=observar ;;\n"
     "    --supervisor-aplicar) NAV_LEGADO=0; SUPERVISOR_MODO=aplicar ;;\n"),
]
if __name__ == '__main__':
    rodar('baseline_subida', 'BASELINE_LEGADO_SUBIDA', ARQ, TROCAS, doc=__doc__)
