#!/usr/bin/env python3
"""WS1_REMAP (09/11/2026): recadastra a WS_1 com a face medida pelo LiDAR na missao30.

Por que: na missao30 (SLAM em localizacao, mapa arena_04-11_1235) o robo parou na aproximacao da
WS_1 e o LiDAR viu a face inteira (0,88 m, y -0,15..+0,74 no robo): o centro estava 30 cm a esquerda
do robo, e o cadastro dizia centrado. 17 medicoes: face x 0,841 y 3,615 (3,615..3,622) yaw -3,1193;
o cadastro (marcado em modo mapping) tinha y 3,862. A aproximacao fica 60 cm a frente da face.

Muda os 2 arquivos, porque o subir_robo.sh copia o do mapa por cima do service_areas.yaml.
Uso:
    python3 aplicar_ws1_remap.py              # aplica (backup antes)
    python3 aplicar_ws1_remap.py --desfazer   # volta os backups mais recentes
    python3 aplicar_ws1_remap.py --arquivo <copia> [--arquivo <copia2>]
"""
import argparse
import glob
import math
import os
import shutil
import sys
import time

import yaml

MAPAS = os.path.expanduser('~/mapas')
ARQS = [os.path.join(MAPAS, 'service_areas.yaml'), os.path.join(MAPAS, 'arena_04-11_1235_service_areas.yaml')]
MARCA = 'WS1_REMAP'
SUF = '.bak_ws1remap_'
FX, FY, FYAW, LARG, DIST = 0.841, 3.615, -3.1193, 0.88, 0.60
AX, AY = FX - DIST * math.cos(FYAW), FY - DIST * math.sin(FYAW)
ANTES = """    face:
      x: 0.859
      y: 3.862
      yaw: -3.096
      largura_medida: 0.87
    aproximacao:
      x: 1.458
      y: 3.889
      yaw: -3.096
"""
DEPOIS = """    face:  # WS1_REMAP (09/11): 17 medicoes LiDAR na missao30 (localizacao); era x 0.859 y 3.862 yaw -3.096 larg 0.87
      x: %.3f
      y: %.3f
      yaw: %.4f
      largura_medida: %.2f
    aproximacao:  # WS1_REMAP: 60 cm a frente da face; era x 1.458 y 3.889 yaw -3.096
      x: %.3f
      y: %.3f
      yaw: %.4f
""" % (FX, FY, FYAW, LARG, AX, AY, FYAW)


def main():
    ap = argparse.ArgumentParser(description=MARCA)
    ap.add_argument('--desfazer', action='store_true')
    ap.add_argument('--arquivo', action='append')
    a = ap.parse_args()
    arqs = a.arquivo or ARQS
    if a.desfazer:
        baks = [sorted(glob.glob(p + SUF + '*')) for p in arqs]
        if not all(baks):
            print('Backup %s* nao encontrado para todos os arquivos. Nada alterado.' % SUF)
            return 1
        for p, b in zip(arqs, baks):
            shutil.copy2(b[-1], p)
            print('restaurado: %s <- %s' % (p, b[-1]))
        return 0
    textos = []
    for p in arqs:
        t = open(p).read()
        if MARCA in t:
            print('%s ja aplicado em %s. Nada alterado.' % (MARCA, p))
            return 0
        if t.count(ANTES) != 1:
            print('!!! bloco esperado da WS_1 nao achado em %s. Nada alterado.' % p)
            return 1
        novo = t.replace(ANTES, DEPOIS)
        va, vb = yaml.safe_load(t), yaml.safe_load(novo)
        if va['service_areas']['WS_2'] != vb['service_areas']['WS_2'] or \
                {k: v for k, v in va.items() if k != 'service_areas'} != \
                {k: v for k, v in vb.items() if k != 'service_areas'}:
            print('!!! conferencia falhou: algo alem da WS_1 mudaria em %s. Nada alterado.' % p)
            return 1
        w = vb['service_areas']['WS_1']
        if abs(w['face']['yaw'] - FYAW) > 1e-6 or abs(w['aproximacao']['x'] - round(AX, 3)) > 1e-6:
            print('!!! conferencia dos valores novos falhou em %s. Nada alterado.' % p)
            return 1
        textos.append((p, novo))
    d = time.strftime('%Y%m%d_%H%M%S')
    for p, novo in textos:
        shutil.copy2(p, p + SUF + d)
        open(p, 'w').write(novo)
        print('alterado: %s (backup %s)' % (p, p + SUF + d))
    print('WS_1 face (%.3f, %.3f, %.1f graus) aproximacao (%.3f, %.3f)' % (FX, FY, math.degrees(FYAW), AX, AY))
    print('O alinhador le o arquivo a cada pedido: nao precisa reiniciar.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
