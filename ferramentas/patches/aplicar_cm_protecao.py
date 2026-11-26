#!/usr/bin/env python3
"""CM_PROTECAO (11/11/2026) - missao34: colisao com obstaculo a frente. So AMPLIA a protecao do Collision Monitor:
 1) diagonal para a frente (direcao 45..80 graus, os 2 lados): poligonos novos diag_esq/diag_dir com frente 0,485
    (11 cm alem do para-choque). Antes caia em left/right, frente 0,425 = 5 cm, dentro da zona cega do LiDAR
    (11:09:53: vx 0,02-0,09 + vy 0,08 -> bateu a 0,5 cm). left/right passam a valer de 80 a 135 graus: a saida
    lateral da CURVA (vx<0, ~110 graus) continua com o poligono de hoje.
 2) frente acima de 0,12 m/s: poligono novo forward_veloz com frente 0,545 (17 cm); forward_rapido fica 0,05-0,12.
    (11:07:46: 0,17 m/s com 11 cm de protecao -> parou a 3,7 cm). A TRAVESSIA chega a parede a <= 0,035 m/s
    (poligono forward, nao muda).
Uso: python3 aplicar_cm_protecao.py [--arquivo copia.yaml] [--desfazer]"""
import os, sys, copy, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
import yaml
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
TROCAS = [
    ("      - forward_rapido  # CM_FRENTE\n",
     "      - forward_rapido  # CM_FRENTE\n      - forward_veloz  # CM_PROTECAO (11/11)\n"
     "      - diag_esq  # CM_PROTECAO (11/11)\n      - diag_dir  # CM_PROTECAO (11/11)\n"),
    ("""      forward_rapido:
        points: "[[0.485, 0.325], [0.485, -0.325], [-0.050, -0.325], [-0.050, 0.325]]"
        linear_min: 0.05
        linear_max: 0.35
""", """      forward_rapido:
        points: "[[0.485, 0.325], [0.485, -0.325], [-0.050, -0.325], [-0.050, 0.325]]"
        linear_min: 0.05
        linear_max: 0.12  # CM_PROTECAO (11/11): era 0.35; acima disso vale o forward_veloz
"""),
    ("""      left:
        points:""", """      # CM_PROTECAO (11/11): frente acima de 0,12 m/s com 17 cm (a 0,17 m/s, 11 cm nao bastaram)
      forward_veloz:
        points: "[[0.545, 0.325], [0.545, -0.325], [-0.050, -0.325], [-0.050, 0.325]]"
        linear_min: 0.12
        linear_max: 0.35
        direction_start_angle: -0.785
        direction_end_angle: 0.785
        theta_min: -0.6
        theta_max: 0.6

      # CM_PROTECAO (11/11): diagonal para a frente (45 a 80 graus) com a frente do forward_rapido (11 cm)
      diag_esq:
        points: "[[0.485, 0.375], [0.485, -0.050], [-0.225, -0.050], [-0.225, 0.375]]"
        linear_min: 0.005
        linear_max: 0.35
        direction_start_angle: 0.785
        direction_end_angle: 1.396
        theta_min: -0.60
        theta_max: 0.60

      diag_dir:
        points: "[[0.485, 0.050], [0.485, -0.375], [-0.225, -0.375], [-0.225, 0.050]]"
        linear_min: 0.005
        linear_max: 0.35
        direction_start_angle: -1.396
        direction_end_angle: -0.785
        theta_min: -0.60
        theta_max: 0.60

      left:
        points:"""),
    ("""        direction_start_angle: 0.785
        direction_end_angle: 2.356
""", """        direction_start_angle: 1.396  # CM_PROTECAO (11/11): era 0.785 (45..80 graus agora e diag_esq)
        direction_end_angle: 2.356
"""),
    ("""        direction_start_angle: -2.356
        direction_end_angle: -0.785
""", """        direction_start_angle: -2.356
        direction_end_angle: -1.396  # CM_PROTECAO (11/11): era -0.785 (-80..-45 graus agora e diag_dir)
"""),
]
def conferir(velho, novo):
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    va = a['collision_monitor']['ros__parameters']['VelocityPolygonStop']
    vb = b['collision_monitor']['ros__parameters']['VelocityPolygonStop']
    e = copy.deepcopy(a)
    eb = e['collision_monitor']['ros__parameters']['VelocityPolygonStop']
    eb['velocity_polygons'] = va['velocity_polygons'][:2] + ['forward_veloz', 'diag_esq', 'diag_dir'] + va['velocity_polygons'][2:]
    eb['forward_rapido']['linear_max'] = 0.12
    eb['left']['direction_start_angle'] = 1.396; eb['right']['direction_end_angle'] = -1.396
    for k in ('forward_veloz', 'diag_esq', 'diag_dir'): eb[k] = vb[k]
    if b != e: sys.exit('ERRO: o yaml mudou alem do previsto no Collision Monitor')
    # cobertura de direcoes (velocidade translacional): nenhum buraco entre os poligonos
    for g in range(-180, 180, 5):
        r = math.radians(g + 0.5)
        ok = [n for n in vb['velocity_polygons'] if 'direction_start_angle' in vb[n] and
              (lambda s, f: (s <= r <= f) if s <= f else (r >= s or r <= f))(vb[n]['direction_start_angle'], vb[n]['direction_end_angle'])]
        if not ok: sys.exit('ERRO: direcao %d graus sem poligono' % g)
if __name__ == '__main__':
    rodar('cm_protecao', 'CM_PROTECAO (11/11)', ARQ, TROCAS, conferir=conferir, doc=__doc__)
