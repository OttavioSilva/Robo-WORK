#!/usr/bin/env python3
"""Evidencias scan/cadeia de comandos e transicao, sem ROS ativo/publicacao."""
import argparse,bisect,collections,json,math
from pathlib import Path
from ler_mcap_parcial import mensagens
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap=argparse.ArgumentParser();ap.add_argument('mcap',type=Path);ap.add_argument('saida',type=Path);a=ap.parse_args();a.saida.mkdir(parents=True,exist_ok=True)
    tops={'/rosout','/scan_raw','/scan_front','/scan_nav','/cmd_vel_ctrl','/cmd_vel_nav_base','/cmd_vel_nav_especial','/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel','/cmd_vel_hardware','/odom'}
    cls={};counts=collections.Counter();bins={t:collections.Counter() for t in ('/scan_raw','/scan_front','/scan_nav')};cmds=collections.defaultdict(list);logs=[];times=[]
    for top,typ,tns,data in mensagens(a.mcap,tops):
        if typ not in cls:cls[typ]=get_message(typ)
        m=deserialize_message(data,cls[typ]);t=tns/1e9;counts[top]+=1;times.append(t)
        if top=='/rosout':
            if m.name in ('alinhador_passagem','bt_navigator','controller_server','destravar','slam_toolbox'):
                logs.append({'t':t,'no':m.name,'msg':m.msg})
        elif top.startswith('/scan'):
            for i,r in enumerate(m.ranges):
                if not math.isfinite(r) or r<m.range_min or r>m.range_max:continue
                ang=m.angle_min+i*m.angle_increment
                # /scan_raw ainda esta invertido; demais ja estao no sentido base_link.
                if top=='/scan_raw':ang=-ang
                x,y=.33+r*math.cos(ang),.005+r*math.sin(ang)
                if -.215<x<.375 and abs(y)<.325:
                    bins[top][round(math.degrees(ang))]+=1
        elif top.startswith('/cmd_vel'):
            tw=m.twist;cmds[top].append((t,tw.linear.x,tw.linear.y,tw.angular.z))
    starts={}
    for top,rows in cmds.items():
        eps=[];lastlat=-1e20
        for i,row in enumerate(rows):
            t,x,y,w=row
            if abs(y)>.025:
                if t-lastlat>.8:
                    window=[p for p in rows[max(0,i-30):i+1] if t-.6<=p[0]<=t]
                    eps.append({'t':t,'primeiro':row[1:],'vx_max_antes_06s':max((p[1] for p in window),default=0),'amostras_antes':window})
                lastlat=t
        starts[top]=eps
    result={'tamanho_bytes_ao_fim':a.mcap.stat().st_size,'duracao_lida_s':max(times)-min(times),'topicos':dict(counts),'autorretorno_bins_graus':{k:dict(sorted(v.items())) for k,v in bins.items()},'inicios_lateral':starts}
    (a.saida/'evidencias.json').write_text(json.dumps(result,indent=2)+'\n')
    (a.saida/'rosout.jsonl').write_text(''.join(json.dumps(l,ensure_ascii=False)+'\n' for l in logs))
    print(json.dumps({'duracao_s':result['duracao_lida_s'],'topicos':result['topicos'],'inicios_lateral':{k:len(v) for k,v in starts.items()}},ensure_ascii=False))


if __name__=='__main__':main()
