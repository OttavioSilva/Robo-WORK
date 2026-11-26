#!/usr/bin/env python3
"""CM_RIGIDO (09/11/2026): Collision Monitor com mais margem no giro rapido e no deslize de lado.

Por que (missao31, medido no bag):
  - 1447 s: giro a 0,40 rad/s com a quina dianteira a 21 cm de um obstaculo; 2 s depois a folga era
    0,2 cm e so entao o CM deu STOP. A quina anda ~20 cm/s nesse giro; com ~0,4 s de atraso do LiDAR
    ela anda ~8 cm antes de o CM reagir: a margem de 7 cm (giro_esq/giro_dir) nao basta.
  - 1468-1494 s: deslizando de lado com a frente a 0-3,6 cm de obstaculos e o CM em "nada": os
    poligonos left/right iam so 1 cm alem do para-choque (x 0,385).

O que muda:
  nav2_params.yaml (collision_monitor, VelocityPolygonStop):
    giro_esq / giro_dir: margem 7 -> 11 cm em volta do footprint;
    left / right: frente 0,385 -> 0,425 (5 cm alem do para-choque, igual ao forward devagar).
  alinhador_passagem.py: RET_GIRO_RAPIDO acompanha o giro_esq/giro_dir (11 cm + 1 cm), para o
    alinhador so escolher o giro rapido quando o CM vai deixar.
Depois: colcon build --packages-select robo_navigation e reiniciar o Nav2.

Uso:
    python3 aplicar_cm_rigido.py               # aplica (backup antes)
    python3 aplicar_cm_rigido.py --desfazer    # volta os backups mais recentes
    python3 aplicar_cm_rigido.py --params <copia.yaml> --alinhador <copia.py>   # teste numa copia
"""
import argparse
import glob
import os
import shutil
import sys
import time

import yaml

WS = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation')
PARAMS = os.path.join(WS, 'config', 'nav2_params.yaml')
ALIN = os.path.join(WS, 'scripts', 'alinhador_passagem.py')
MARCA = 'CM_RIGIDO'
SUF = '.bak_cmrigido_'

GIRO_ANTES = '        points: "[[0.435, 0.385], [0.435, -0.385], [-0.275, -0.385], [-0.275, 0.385]]"\n'
GIRO_DEPOIS = ('        points: "[[0.475, 0.425], [0.475, -0.425], [-0.315, -0.425], [-0.315, 0.425]]"'
               '  # CM_RIGIDO (09/11): margem 11 cm (era 7)\n')
LEFT_ANTES = '        points: "[[0.385, 0.375], [0.385, -0.050], [-0.225, -0.050], [-0.225, 0.375]]"\n'
LEFT_DEPOIS = ('        points: "[[0.425, 0.375], [0.425, -0.050], [-0.225, -0.050], [-0.225, 0.375]]"'
               '  # CM_RIGIDO (09/11): frente 0.385 -> 0.425\n')
RIGHT_ANTES = '        points: "[[0.385, 0.050], [0.385, -0.375], [-0.225, -0.375], [-0.225, 0.050]]"\n'
RIGHT_DEPOIS = ('        points: "[[0.425, 0.050], [0.425, -0.375], [-0.225, -0.375], [-0.225, 0.050]]"'
                '  # CM_RIGIDO (09/11): frente 0.385 -> 0.425\n')
RET_ANTES = 'RET_GIRO_RAPIDO = (-0.285, 0.445, 0.395)   # v3.32: poligono giro_esq/giro_dir do CM (7 cm) + 1 cm\n'
RET_DEPOIS = ('RET_GIRO_RAPIDO = (-0.325, 0.485, 0.435)   # CM_RIGIDO (09/11): giro_esq/giro_dir do CM (11 cm) + 1 cm;'
              ' era (-0.285, 0.445, 0.395)\n')


def poligonos(texto):
    vp = yaml.safe_load(texto)['collision_monitor']['ros__parameters']['VelocityPolygonStop']
    return vp, {k: vp[k] for k in vp['velocity_polygons']}


def main():
    ap = argparse.ArgumentParser(description=MARCA)
    ap.add_argument('--desfazer', action='store_true')
    ap.add_argument('--params', default=PARAMS)
    ap.add_argument('--alinhador', default=ALIN)
    a = ap.parse_args()
    arqs = [a.params, a.alinhador]
    if a.desfazer:
        baks = [sorted(glob.glob(p + SUF + '*')) for p in arqs]
        if not all(baks):
            print('Backup %s* nao encontrado para os 2 arquivos. Nada alterado.' % SUF)
            return 1
        for p, b in zip(arqs, baks):
            shutil.copy2(b[-1], p)
            print('restaurado: %s <- %s' % (p, b[-1]))
        print('Agora: colcon build --packages-select robo_navigation e reiniciar o Nav2')
        return 0

    tp, ta = open(a.params).read(), open(a.alinhador).read()
    if MARCA in tp or MARCA in ta:
        print('%s ja aplicado. Nada alterado.' % MARCA)
        return 0
    for texto, trecho, n in ((tp, GIRO_ANTES, 2), (tp, LEFT_ANTES, 1), (tp, RIGHT_ANTES, 1), (ta, RET_ANTES, 1)):
        if texto.count(trecho) != n:
            print('!!! trecho esperado %dx, achado %dx:\n%sNada alterado.' % (n, texto.count(trecho), trecho))
            return 1
    np_ = tp.replace(GIRO_ANTES, GIRO_DEPOIS).replace(LEFT_ANTES, LEFT_DEPOIS).replace(RIGHT_ANTES, RIGHT_DEPOIS)
    na = ta.replace(RET_ANTES, RET_DEPOIS)

    # conferencia: o yaml carrega e so mudam os pontos de giro_esq, giro_dir, left e right
    da, db = yaml.safe_load(tp), yaml.safe_load(np_)
    va, pa = poligonos(tp)
    vb, pb = poligonos(np_)
    mud = sorted(k for k in pa if pa[k] != pb[k])
    resto_a = {k: v for k, v in da.items() if k != 'collision_monitor'}
    resto_b = {k: v for k, v in db.items() if k != 'collision_monitor'}
    cm_a = {k: v for k, v in da['collision_monitor']['ros__parameters'].items() if k != 'VelocityPolygonStop'}
    cm_b = {k: v for k, v in db['collision_monitor']['ros__parameters'].items() if k != 'VelocityPolygonStop'}
    so_pontos = all({kk: vv for kk, vv in pa[k].items() if kk != 'points'} ==
                    {kk: vv for kk, vv in pb[k].items() if kk != 'points'} for k in pa)
    if mud != ['giro_dir', 'giro_esq', 'left', 'right'] or resto_a != resto_b or cm_a != cm_b or not so_pontos:
        print('!!! conferencia do yaml falhou (mudaram: %s). Nada alterado.' % mud)
        return 1
    compile(na, a.alinhador, 'exec')

    d = time.strftime('%Y%m%d_%H%M%S')
    for p, novo in ((a.params, np_), (a.alinhador, na)):
        shutil.copy2(p, p + SUF + d)
        open(p, 'w').write(novo)
        print('alterado: %s (backup %s)' % (p, p + SUF + d))
    print('mudaram no CM: %s' % ', '.join(mud))
    print('Agora: cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && '
          'source install/setup.bash, e reiniciar o Nav2')
    return 0


if __name__ == '__main__':
    sys.exit(main())
