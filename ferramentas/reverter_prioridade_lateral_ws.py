#!/usr/bin/env python3
"""Reverte somente esta rodada. Padrao: conferir sem gravar. Nao inicia ROS."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

ROOT=Path(__file__).resolve().parents[1]
DOC='docs/prioridade_lateral_ws_20261109'
SNAP='backups/prioridade_lateral_ws_20261109/antes.tar.gz'

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def reverter(root,manifest,aplicar=False):
    root=root.resolve();items=manifest['modificados_rodada']+manifest['novos_codigo_rodada']
    def path(name):
        p=root/name
        if Path(name).is_absolute() or not p.resolve().is_relative_to(root) or p.is_symlink():
            raise ValueError('caminho inseguro: '+name)
        return p
    for item in items:
        p=path(item['path'])
        if not p.is_file() or digest(p)!=item['sha256']:
            raise RuntimeError('arquivo mudou apos entrega: '+item['path'])
    snap=path(SNAP)
    if digest(snap)!=manifest['snapshot_sha256']:raise RuntimeError('snapshot mudou')
    originals=[]
    with tarfile.open(snap) as tar:
        for item in manifest['modificados_rodada']:
            m=tar.getmember(item['path'])
            if not m.isfile():raise ValueError('snapshot sem arquivo regular')
            originals.append((path(item['path']),tar.extractfile(m).read(),m.mode))
    if not aplicar:return 'Conferencia OK; nada alterado.'
    backup=root/'backups'/('reversao_lateral_ws_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.mkdir(parents=True,exist_ok=False)
    with tarfile.open(backup/'antes_reverter.tar.gz','w:gz') as tar:
        for item in items:tar.add(path(item['path']),arcname=item['path'])
    for p,content,mode in originals:
        p.write_bytes(content);p.chmod(mode & 0o777)
    for item in manifest['novos_codigo_rodada']:
        p=path(item['path']);dest=backup/'novos'/item['path'];dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.move(p,dest)
    return 'Versao anterior restaurada; arquivos posteriores preservados em '+str(backup)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--aplicar',action='store_true');a=p.parse_args()
    manifest=json.loads((ROOT/DOC/'arquivos.json').read_text())
    print(reverter(ROOT,manifest,a.aplicar))

if __name__=='__main__':main()
