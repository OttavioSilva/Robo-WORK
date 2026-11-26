#!/usr/bin/env python3
"""PAREDE_SEM_CAMERA (23/11/2026): na arena (mapa fixo) a parede virtual sobe SEM camera.

base_arena_localizacao.launch.py: com PAREDES_FITA_MODO=load (padrao) roda parede_sem_camera.sh
(so tape_keepout com as fitas gravadas); a camera XIFT fica para a garra. Com --gravar-fitas
(record) ou PAREDE_CAMERA=1, roda o parede.sh de antes (camera + virtual_wall).
Uso: aplicar_parede_sem_camera.py | --desfazer | --arquivo <copia>
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/ferramentas/subida_unica/base_arena_localizacao.launch.py')
TROCAS = [
    ("""    wall = str(home / "logs/reabertura_20261116_100105/parede.sh")
""",
     """    wall = str(home / "logs/reabertura_20261116_100105/parede.sh")
    # PAREDE_SEM_CAMERA (23/11): camera so para gravar fitas (mapeamento); na operacao, fitas gravadas sem camera.
    if os.environ.get("PAREDES_FITA_MODO", "load") != "record" and os.environ.get("PAREDE_CAMERA", "0") != "1":
        wall_cmd = ["bash", str(home / "robo_jazzy_ws/ferramentas/subida_unica/parede_sem_camera.sh")]
    else:
        wall_cmd = None
"""),
    ("""        ("parede_virtual", ["bash", "-c", camera_command, "camera", device, wall]),
""",
     """        ("parede_virtual", wall_cmd or ["bash", "-c", camera_command, "camera", device, wall]),  # PAREDE_SEM_CAMERA
"""),
]
if __name__ == '__main__':
    rodar('parede_sem_camera', 'PAREDE_SEM_CAMERA', PADRAO, TROCAS, doc=__doc__)
