#!/usr/bin/env python3
"""Patch reversivel: suspender giro e validar o caminho realmente seguido.
--arquivo permite ensaiar cada patch em copia; --desfazer restaura o backup local.
"""
import argparse
import ast
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
MARCA='PRIORIDADE_LATERAL_WS_20261109'

def transformar(texto, tipo):
    if MARCA in texto:return texto
    if tipo=='launch':
        texto=texto.replace('choices=["lateral", "giro_frontal"]','choices=["lateral"]')
        texto=texto.replace('# Entrada arbitrada: /cmd_vel_nav_seguro; saida /cmd_vel_smoothed.',
                            '# '+MARCA+': caminho lateral original; giro suspenso.')
        texto=texto.replace('[("cmd_vel", "cmd_vel_nav_seguro")]','[("cmd_vel", "cmd_vel_nav")]')
        inicio=texto.index('            ExecuteProcess(\n                cmd=["python3", os.path.expanduser(\n                    "~/robo_jazzy_ws/src/robo_navigation/scripts/arbitro_t.py")')
        fim=texto.index('            ExecuteProcess(  # NAV_TUDO: alinhador',inicio)
        texto=texto[:inicio]+texto[fim:]
        ast.parse(texto)
        assert 'arbitro_t.py' not in texto and 'choices=["lateral"]' in texto
    else:
        assert texto.count('<IsPathValid path="{path}"/>')==1
        texto=texto.replace('<IsPathValid path="{path}"/>',
            '<!-- '+MARCA+': verificar o caminho executado pelo FollowPath. -->\n                <IsPathValid path="{smoothed_path}"/>')
        ET.fromstring(texto)
    return texto

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tipo',choices=('launch','bt'),required=True)
    p.add_argument('--arquivo',type=Path)
    p.add_argument('--desfazer',action='store_true')
    a=p.parse_args()
    target=a.arquivo or ROOT/('src/robo_navigation/launch/navigation.launch.py' if a.tipo=='launch' else 'src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml')
    backup=target.with_name(target.name+'.antes_lateral_ws')
    if a.desfazer:
        if not backup.exists():p.error('backup inexistente')
        # Nao descartar edicoes feitas depois da aplicacao.
        expected=transformar(backup.read_text(),a.tipo)
        if target.read_text()!=expected:p.error('arquivo mudou depois do patch; reverter manualmente com diff')
        shutil.copy2(backup,target)
    else:
        old=target.read_text();new=transformar(old,a.tipo)
        if old==new:return
        if backup.exists():p.error('backup ja existe; conferir antes de reaplicar')
        shutil.copy2(target,backup)
        target.write_text(new)
    print(target)

if __name__=='__main__':main()
