#!/usr/bin/env python3
"""WS2_REMAP (09/11/2026): recadastra a WS_2 com a face medida pelo LiDAR na missao29.

Por que: na missao29 (mapa arena_04-11_1235) 41 medicoes com o robo alinhado (|ang| < 3 graus,
|centro| < 5 cm) deram a face em x 2,625 (2,601..2,635), y 1,998 (1,983..2,025), yaw +8,5 graus
(6,5..11,5), largura 0,89 m. O cadastro dizia yaw -2,9 graus: o robo chegava girado ~15 graus e
11-16 cm fora do centro. A aproximacao fica 60 cm a frente da face, no mesmo rumo.

Muda os 2 arquivos, porque o subir_robo.sh copia o do mapa por cima do service_areas.yaml.
Uso:
    python3 aplicar_ws2_remap.py              # aplica (backup antes)
    python3 aplicar_ws2_remap.py --desfazer   # volta os backups mais recentes
    python3 aplicar_ws2_remap.py --arquivo <copia> [--arquivo <copia2>]
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
MARCA = 'WS2_REMAP'
SUF = '.bak_ws2remap_'
FX, FY, FYAW, LARG, DIST = 2.625, 1.998, 0.148, 0.89, 0.60
AX, AY = FX - DIST * math.cos(FYAW), FY - DIST * math.sin(FYAW)
ANTES = """    face:  # WS2_FACE (05/11): medida nos encostos das missoes 21 e 22; era x 2.698 y 2.049
      x: 2.6
      y: 1.98
      yaw: -0.0508
      largura_medida: 0.87
    aproximacao:  # WS2_FACE: 60 cm da face; era x 2.099 y 2.079
      x: 2.001
      y: 2.01
      yaw: -0.0508
"""
DEPOIS = """    face:  # WS2_REMAP (09/11): 41 medicoes LiDAR na missao29; era x 2.6 y 1.98 yaw -0.0508 larg 0.87
      x: %.3f
      y: %.3f
      yaw: %.4f
      largura_medida: %.2f
    aproximacao:  # WS2_REMAP: 60 cm a frente da face; era x 2.001 y 2.01 yaw -0.0508
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
            print('!!! bloco esperado da WS_2 nao achado em %s. Nada alterado.' % p)
            return 1
        novo = t.replace(ANTES, DEPOIS)
        va, vb = yaml.safe_load(t), yaml.safe_load(novo)
        if va['service_areas']['WS_1'] != vb['service_areas']['WS_1'] or \
                {k: v for k, v in va.items() if k != 'service_areas'} != \
                {k: v for k, v in vb.items() if k != 'service_areas'}:
            print('!!! conferencia falhou: algo alem da WS_2 mudaria em %s. Nada alterado.' % p)
            return 1
        w = vb['service_areas']['WS_2']
        if abs(w['face']['yaw'] - FYAW) > 1e-6 or abs(w['aproximacao']['x'] - round(AX, 3)) > 1e-6:
            print('!!! conferencia dos valores novos falhou em %s. Nada alterado.' % p)
            return 1
        textos.append((p, novo))
    d = time.strftime('%Y%m%d_%H%M%S')
    for p, novo in textos:
        shutil.copy2(p, p + SUF + d)
        open(p, 'w').write(novo)
        print('alterado: %s (backup %s)' % (p, p + SUF + d))
    print('WS_2 face (%.3f, %.3f, %.1f graus) aproximacao (%.3f, %.3f)' % (FX, FY, math.degrees(FYAW), AX, AY))
    print('O alinhador le o arquivo a cada pedido: nao precisa reiniciar.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
