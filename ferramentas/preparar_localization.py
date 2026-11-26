#!/usr/bin/env python3
"""Valida par serializado e gera perfil reversivel sem iniciar ROS/hardware."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def validar(mapa):
    mapa = Path(mapa).expanduser().resolve()
    if mapa.suffix in ('.posegraph', '.data'):
        mapa = mapa.with_suffix('')
    evidencias = {}
    for ext in ('.posegraph', '.data'):
        p = Path(str(mapa)+ext)
        b = p.read_bytes()
        if len(b) < 1000 or b'serialization::archive' not in b[:64]:
            raise ValueError('arquivo serializado invalido: '+str(p))
        evidencias[ext] = {'bytes':len(b), 'sha256':hashlib.sha256(b).hexdigest()}
    pontos = yaml.safe_load(Path(str(mapa)+'_pontos.yaml').read_text())['pontos']['START']
    pose = [float(pontos[k]) for k in ('x','y','yaw')]
    if not all(math.isfinite(v) for v in pose):
        raise ValueError('START invalido')
    return str(mapa), pose, evidencias


def perfil(mapa):
    nome,pose,evidencias=validar(mapa)
    cfg=yaml.safe_load((ROOT/'src/robo_navigation/config/slam_mapping.yaml').read_text())
    cfg['slam_toolbox']['ros__parameters'].update(mode='localization',map_file_name=nome,map_start_pose=pose)
    return cfg,evidencias


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mapa',default='~/mapas/arena_04-11_1235')
    p.add_argument('--saida',required=True)
    a=p.parse_args()
    cfg,e=perfil(a.mapa)
    dest=Path(a.saida)
    if dest.exists():
        if yaml.safe_load(dest.read_text()) == cfg:
            print('perfil ja confere');raise SystemExit(0)
        raise SystemExit('recusado sobrescrever perfil existente; use outra saida')
    dest.write_text('# SUPERVISOR_LOCALIZATION_20261111: somente modo e inicializacao diferem do mapping.\n'+yaml.safe_dump(cfg,sort_keys=False))
    print(json.dumps(e,indent=2))
