#!/usr/bin/env python3
"""WS2_FACE (05/11/2026): corrige a marca da WS_2 com o que os encostos mediram.

Por que: nas 9 medidas das missoes 21 e 22 a face real estava em x 2,60 (marcada 2,698) e o centro em
y 1,98 (marcado 2,049). O ponto de aproximacao ficava a ~50 cm da face, e a quina do robo girado alcanca
49 cm: ele travava a poucos centimetros do destino.

Muda os 2 arquivos, porque o subir_robo.sh copia o do mapa por cima do service_areas.yaml.
Uso:
    python3 aplicar_ws2_face.py              # aplica (backup antes)
    python3 aplicar_ws2_face.py --desfazer   # volta os backups mais recentes
    python3 aplicar_ws2_face.py --arquivo <copia> [--arquivo <copia2>]
"""
import argparse
import glob
import os
import shutil
import sys
import time

import yaml

MAPAS = os.path.expanduser('~/mapas')
ARQS = [os.path.join(MAPAS, 'service_areas.yaml'), os.path.join(MAPAS, 'arena_04-11_1235_service_areas.yaml')]
MARCA = 'WS2_FACE'
SUF = '.bak_ws2face_'
ANTES = """    face:
      x: 2.698
      y: 2.049
      yaw: -0.0508
      largura_medida: 0.87
    aproximacao:
      x: 2.099
      y: 2.079
      yaw: -0.0508
"""
DEPOIS = """    face:  # WS2_FACE (05/11): medida nos encostos das missoes 21 e 22; era x 2.698 y 2.049
      x: 2.6
      y: 1.98
      yaw: -0.0508
      largura_medida: 0.87
    aproximacao:  # WS2_FACE: 60 cm da face; era x 2.099 y 2.079
      x: 2.001
      y: 2.01
      yaw: -0.0508
"""


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
            print('Restaurado de %s' % b[-1])
        return 0
    novos = {}
    for p in arqs:
        txt = open(p).read()
        if MARCA in txt:
            print('Ja aplicado em %s (achei a marca %s). Nada a fazer.' % (p, MARCA))
            return 0
        try:
            if txt.count(ANTES) != 1:
                raise RuntimeError('a marca da WS_2 nao e a esperada (remarcada depois?)')
            novo = txt.replace(ANTES, DEPOIS)
            d0, d1 = yaml.safe_load(txt), yaml.safe_load(novo)
            w = d1['service_areas']['WS_2']
            assert (w['face']['x'], w['face']['y'], w['aproximacao']['x'], w['aproximacao']['y']) == (2.6, 1.98, 2.001, 2.01)
            d0['service_areas']['WS_2'] = w
            assert d0 == d1, 'mudou mais do que a WS_2'
        except Exception as exc:
            print('Nao consegui aplicar em %s (%s). Nada alterado.' % (p, exc))
            return 1
        novos[p] = novo
    data = time.strftime('%Y%m%d_%H%M%S')
    for p, novo in novos.items():
        shutil.copy2(p, p + SUF + data)
        open(p, 'w').write(novo)
        print('Backup: %s' % (p + SUF + data))
    print('Aplicado. Reinicie o ~/subir_robo.sh.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
