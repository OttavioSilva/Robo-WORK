#!/usr/bin/env python3
"""SLAM_IMU_FORTE + SEM_MAPA_NO_GIRO + CONFIRMA (23/11/2026) — so o mapeamento (subir_tudo, SLAM do zero).

Bag mapeamento_orientacao_20261123 (18:41-18:44): IMU e giroscopio concordaram em ~3% nos giros; quem
entortava o mapa era o scan matching (erro de 2-15 graus por giro de 90, -10 numa reta). Replay do bag
(dominio isolado) com estes valores: erro por giro ~2 graus, soma dos erros 44 -> 19,5 graus, 0 saltos > 3 graus.

slam_mapping.yaml:
  - SLAM_IMU_FORTE: minimum_angle_penalty 0.9 -> 0.2, angle_variance_penalty 1.0 -> 0.1,
    coarse_search_angle_offset 0.349 (+-20 graus) -> 0.0873 (+-5 graus): confia mais no rumo da odometria/IMU;
  - CONFIRMA: occupancy_threshold 0.1 -> 0.25 e min_pass_through 2 (padrao) -> 3: celula precisa ser vista
    mais vezes antes de virar parede/livre (menos pontos soltos);
  - SEM_MAPA_NO_GIRO: scan_topic /scan_front -> /scan_slam.
base.launch.py: sobe ferramentas/subida_unica/scan_sem_giro.py (/scan_front -> /scan_slam sem giro; respawn).
A arena (localization) NAO muda: usa o seu proprio perfil com /scan_front.
Uso: aplicar_slam_imu_forte.py | --desfazer | --raiz <copia do workspace>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_multi import rodar  # noqa: E402

ARQUIVOS = [
    ('src/robo_navigation/config/slam_mapping.yaml', [
        ("    scan_topic: /scan_front\n",
         "    scan_topic: /scan_slam  # SEM_MAPA_NO_GIRO 23/11: era /scan_front (scan_sem_giro.py corta o giro)\n"),
        ("    occupancy_threshold: 0.1  # SLAM_OCC01 (11/11): era 0.2 (desde 03/11); 0.1 = padrao do slam_toolbox\n",
         "    occupancy_threshold: 0.25  # CONFIRMA 23/11: era 0.1 (SLAM_OCC01)\n"
         "    min_pass_through: 3  # CONFIRMA 23/11: era 2 (padrao)\n"),
        ("    angle_variance_penalty: 1.0\n",
         "    angle_variance_penalty: 0.1  # SLAM_IMU_FORTE 23/11: era 1.0\n"),
        ("    coarse_search_angle_offset: 0.349\n",
         "    coarse_search_angle_offset: 0.0873  # SLAM_IMU_FORTE 23/11: era 0.349\n"),
        ("    minimum_angle_penalty: 0.9\n",
         "    minimum_angle_penalty: 0.2  # SLAM_IMU_FORTE 23/11: era 0.9\n"),
    ]),
    ('ferramentas/subida_unica/base.launch.py', [
        ("""                                  cmd=["python3", "-u", str(home / "robo_jazzy_ws/ferramentas/subida_unica/mapa_limpo.py")]))
""",
         """                                  cmd=["python3", "-u", str(home / "robo_jazzy_ws/ferramentas/subida_unica/mapa_limpo.py")]))
    # SLAM_IMU_FORTE / SEM_MAPA_NO_GIRO (23/11): o SLAM le /scan_slam (scan sem giro).
    actions.append(ExecuteProcess(name="scan_sem_giro", output="screen", respawn=True, respawn_delay=1.0,
                                  cmd=["python3", "-u", str(home / "robo_jazzy_ws/ferramentas/subida_unica/scan_sem_giro.py")]))
"""),
    ]),
]

if __name__ == '__main__':
    rodar('slam_imu_forte', 'SLAM_IMU_FORTE', ARQUIVOS, doc=__doc__)
