#!/usr/bin/env python3
"""Auditoria offline da Missao 24; le MCAP, nao inicia ROS nem publica comandos.
Compara selecao antiga e nova; TF amostrado por recepcao, nao prova fisica.
"""
import argparse
import ast
import json
import math
from pathlib import Path
import sys
import tarfile
import tempfile
import time
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
import analisa_missao as analise

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/robo_navigation/scripts'))
from face_ws import RastreioFaceWS, referencia_ws, candidatos_face

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag');p.add_argument('--areas',type=Path,default=Path.home()/'mapas/service_areas.yaml')
    p.add_argument('--saida',type=Path,required=True);a=p.parse_args()
    areas=yaml.safe_load(a.areas.read_text())['service_areas']
    with tarfile.open(ROOT/'backups/prioridade_lateral_ws_20261109/antes.tar.gz') as tar:
        source=tar.extractfile('src/robo_navigation/scripts/alinhador_passagem.py').read().decode()
    func=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='medir_face')
    ns={'math':math};exec(compile(ast.Module(body=[func],type_ignores=[]),'face_previa','exec'),ns)
    with tempfile.TemporaryDirectory() as tmp:d=analise.carregar(analise.achar_mcap(a.bag,tmp)[0])
    x,y=analise.pontos_corpo(d)
    intervals=[];start=None
    for t,_,_,node,msg in d.rosout:
        if node!='alinhador_passagem':continue
        if msg.startswith('ENCOSTAR ') and ': medindo a face da mesa' in msg:
            start=(t,msg.split()[1].rstrip(':'))
        elif start and msg.startswith('ENCOSTAR '+start[1]+': ') and ('SUCESSO' in msg or 'FALHOU' in msg):
            intervals.append((*start,t));start=None
    result=[]
    for t0,name,t1 in intervals:
        r=RastreioFaceWS(areas[name]['face']);row=dict(area=name,inicio=analise.hora(t0),fim=analise.hora(t1),scans=0,face_antiga=0,centro_antigo=0,face_ws_nova=0,motivos={})
        timings=[];row['amostras']=[]
        for i,t in enumerate(d.scan_t):
            if not t0<=t<=t1:continue
            pts=[(float(px),float(py)) for px,py in zip(x[i],y[i]) if math.isfinite(px) and math.isfinite(py)]
            row['scans']+=1
            old=ns['medir_face'](pts)
            row['face_antiga']+=int(old is not None)
            row['centro_antigo']+=int(old is not None and old['centro'] is not None)
            od=tuple(analise.amostra(d.odom_base,t)[1:])
            before=time.perf_counter();med=r.medir(pts,float(t),od,analise.pose_mapa(d,t));timings.append(time.perf_counter()-before)
            row['face_ws_nova']+=int(med is not None)
            if row['scans'] in (1,10,30,60):
                row['amostras'].append(dict(hora=analise.hora(t),pose=analise.pose_mapa(d,t),
                    referencia=referencia_ws(areas[name]['face'],analise.pose_mapa(d,t)),
                    antiga=old,candidatos=candidatos_face(pts)))
            row['motivos'][r.motivo]=row['motivos'].get(r.motivo,0)+1
        row['tempo_max_seletor_ms']=round(max(timings,default=0)*1000,3)
        result.append(row)
    out=dict(limites=['Replay somente geometrico, sem executar maquina de estados ou velocidades.',
        'TF e scan amostrados por recepcao; sem reconstituir aquisicao ou latencia.',
        'Usa cadastro atual fornecido em --areas; nao prova identidade semantica da bancada.',
        'Sem /scan_nav, /bloqueio_virtual e /cmd_vel_ctrl no bag.'],
        bag=str(a.bag),areas=str(a.areas),encostos=result)
    a.saida.write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(out,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
