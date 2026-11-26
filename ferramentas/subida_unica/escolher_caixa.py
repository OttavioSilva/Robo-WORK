#!/usr/bin/env python3
"""ESCOLHER_CAIXA (24/11/2026): escolhe a caixa de START/FINISH da prova (1 ou 2) num mapa salvo.

O mapa salvo (<mapa>_pontos.yaml) pode ter START/FINISH (caixa 1) e START_2/FINISH_2 (caixa 2, TELEOP_CAIXA2).
Na 1a vez guarda a caixa 1 como START_1/FINISH_1. Depois copia a caixa escolhida para START/FINISH,
que e o que a subida (pose inicial do SLAM/AMCL) e o executor usam. Backup .bak_caixa_<hora>.
Uso: escolher_caixa.py <mapa> 1|2        ex.: escolher_caixa.py arena_24-11_1500 2
     escolher_caixa.py <mapa>            (so mostra)
Depois: subir normalmente com esse mapa (robo na caixa escolhida, virado como foi marcado).
"""
import math
import os
import shutil
import sys
import time

import yaml


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    arq = os.path.expanduser('~/mapas/%s_pontos.yaml' % sys.argv[1])
    doc = yaml.safe_load(open(arq)) or {}
    p = doc.setdefault('pontos', {})
    if 'START_1' not in p and 'START' in p:
        p['START_1'] = dict(p['START'])
        p['FINISH_1'] = dict(p.get('FINISH', p['START']))
    for k in sorted(p):
        print('  %-9s x=%7.3f y=%7.3f yaw=%6.1f' % (k, p[k]['x'], p[k]['y'], math.degrees(p[k]['yaw'])))
    if len(sys.argv) < 3:
        return
    c = sys.argv[2]
    if c not in ('1', '2') or 'START_' + c not in p:
        sys.exit('!!! caixa %s nao marcada neste mapa (marque START_%s com o teleop)' % (c, c))
    p['START'] = dict(p['START_' + c])
    p['FINISH'] = dict(p.get('FINISH_' + c, p['START_' + c]))
    shutil.copy2(arq, arq + '.bak_caixa_' + time.strftime('%Y%m%d_%H%M%S'))
    with open(arq, 'w') as f:
        f.write('# ESCOLHER_CAIXA: caixa %s ativa (%s)\n' % (c, time.strftime('%d/%m %H:%M')))
        yaml.safe_dump(doc, f, sort_keys=True)
    print('>>> CAIXA %s ativa: START (%.3f, %.3f, %.1f graus)' % (
        c, p['START']['x'], p['START']['y'], math.degrees(p['START']['yaw'])))


if __name__ == '__main__':
    main()
