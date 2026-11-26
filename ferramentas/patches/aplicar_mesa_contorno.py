#!/usr/bin/env python3
"""MESA_CONTORNO (24/11/2026): subir_robo.sh --manual usa ~/mapas/arena_manual/arena_contorno.yaml.

O arena_contorno.pgm (gerar_contorno.py) e o arena.pgm com as mesas e paredes grossas so em BORDA de 10 cm.
Motivo: AMCL escorregou 23-28 cm para dentro das mesas (WS07/WS08; CORRIGE_POSE_MESA mediu 23,2 e 28,0 cm).
map_server alimenta AMCL, vigia e costmap global: a borda continua lethal e fecha a mesa para o planner.
Uso: aplicar_mesa_contorno.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/subir_robo.sh')

TROCAS = [
    ("""  for f in ~/mapas/arena_manual/arena.yaml "${MAPA}_pontos.yaml" "${MAPA}_service_areas.yaml"; do
""",
     """  for f in ~/mapas/arena_manual/arena_contorno.yaml "${MAPA}_pontos.yaml" "${MAPA}_service_areas.yaml"; do  # MESA_CONTORNO (24/11): era arena.yaml
"""),
    ("""localizacao_amcl.launch.py mapa:=$HOME/mapas/arena_manual/arena.yaml x:=$X""",
     """localizacao_amcl.launch.py mapa:=$HOME/mapas/arena_manual/arena_contorno.yaml x:=$X"""),
]


def conferir(velho, novo):
    assert novo.count('arena_contorno.yaml') == 2 and novo.count('MESA_CONTORNO') == 1
    import subprocess
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.sh', delete=False) as f:
        f.write(novo)
    assert subprocess.run(['bash', '-n', f.name]).returncode == 0
    os.unlink(f.name)


if __name__ == '__main__':
    rodar('mesa_contorno', 'MESA_CONTORNO', PADRAO, TROCAS, conferir, doc=__doc__)
