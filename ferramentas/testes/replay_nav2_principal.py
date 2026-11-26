#!/usr/bin/env python3
"""Replay de decisoes sobre bags m33/m34 (leitura MCAP; nao inicia ROS nem publica).
Nao e simulacao fechada da trajetoria: robô/obstaculos seguem os dados gravados.
"""
import argparse, ast, json, math, sys, threading, time
from pathlib import Path
from types import SimpleNamespace as NS
WS=Path(__file__).resolve().parents[2];NAV=WS/'src/robo_navigation'
sys.path.insert(0,str(NAV/'scripts'))
from alinhador_passagem import Mapa, planejar_etapas, medir_corredor
from controle_nav_principal import passagem_confirmada
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def replay(bag):
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    wanted={'/map','/plan','/scan_front','/goal_pose_alinhado','/rosout','/cmd_vel_nav','/tf'}
    tipos={t.name:get_message(t.type) for t in r.get_all_topics_and_types() if t.name in wanted}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(tipos)))
    goals=[];mapa=None;scan=None;trans={};checks=[];last=-1e20;plano=None;plano_t=0
    counts={'caixa_logs':0,'curva_t_logs':0,'giros_comandados':0,'planos':0}
    def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def pose():
        if ('map','odom') not in trans or ('odom','base_link') not in trans:return None
        a,b=trans['map','odom'],trans['odom','base_link'];c,s=math.cos(a[2]),math.sin(a[2])
        return a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],a[2]+b[2]
    while r.has_next():
        top,data,tns=r.read_next();m=deserialize_message(data,tipos[top]);t=tns/1e9
        if top=='/goal_pose_alinhado':goals.append((t,m))
        elif top=='/map':
            i=m.info;mapa=Mapa(i.resolution,i.width,i.height,i.origin.position.x,i.origin.position.y,yaw(i.origin.orientation),m.data)
        elif top=='/tf':
            for tr in m.transforms:trans[tr.header.frame_id,tr.child_frame_id]=(tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation))
        elif top=='/scan_front':scan=(t,m)

        elif top=='/rosout':
            text=m.msg.lower();counts['caixa_logs']+=int(any(w in text for w in ('caixa','peca','obstaculo')))
            counts['curva_t_logs']+=int(any(w in text for w in ('curva','cruzamento')))
        elif top=='/cmd_vel_nav':counts['giros_comandados']+=int(abs(m.twist.angular.z)>.05)
        elif top=='/plan':
            counts['planos']+=1;plano=m;plano_t=t
        if top=='/scan_front' and plano is not None:
            if t-last<1 or t-plano_t>16 or mapa is None or not goals:continue
            last=t;pts=[(p.pose.position.x,p.pose.position.y) for p in plano.poses]
            if not pts:continue
            pm=pose();med=None
            if pm:
                k=min(range(len(pts)),key=lambda k:(pts[k][0]-pm[0])**2+(pts[k][1]-pm[1])**2);pts=pts[k:]
            etapas,info=planejar_etapas(mapa,pts,pts[-1])
            if etapas and pm and t-scan[0]<.5:
                # Laser rigid transform comes from bag TF when present, fallback only recorded production mounting.
                laser=trans.get(('base_link',scan[1].header.frame_id),(.33,.005,0))
                a=math.radians(info['eixo_graus'])-pm[2];c,s=math.cos(a),math.sin(a);lidar=[]
                sm=scan[1]
                for k,dist in enumerate(sm.ranges):
                    if not math.isfinite(dist) or not sm.range_min<=dist<=sm.range_max:continue
                    ang=sm.angle_min+k*sm.angle_increment+laser[2]
                    x,y=laser[0]+dist*math.cos(ang),laser[1]+dist*math.sin(ang)
                    lidar.append((c*x+s*y,-s*x+c*y))
                med=medir_corredor(lidar)
            checks.append({'t':t,'passagem_mapa':bool(etapas),'confirmada':passagem_confirmada(info,pm,med),
                           'curva_t':bool(isinstance(info,dict) and info.get('curva'))})
    # Executa o metodo real do novo coordenador para cada goal gravado, action Nav2 substituida por stub terminal.
    tree=ast.parse((NAV/'scripts/alinhador_passagem.py').read_text())
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_executar_principal')
    env={'time':time,'math':math};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<coordenador real>','exec'),env)
    for t,goal in goals:
        calls=[]
        f=NS(p=lambda k:4,sem_encosto=set(),dono_mov=None,
            _navegar_principal=lambda g,n,v:(calls.append(g),'ok')[1],_drenar_nav=lambda:None)
        env['_executar_principal'](f,goal)
        assert calls==[goal] and calls[0].pose==goal.pose
    assert goals and checks and counts['giros_comandados'] and counts['curva_t_logs']
    return {'bag':str(bag),'goals_diretos_preservados':len(goals),
            'novos_goals_intervalo_menor_30s':sum(b[0]-a[0]<30 for a,b in zip(goals,goals[1:])),
            **counts,'amostras':checks,
            'limite':'replay de decisao; nao comprova trajetoria alternativa, frenagem ou sucesso fisico'}


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('bags',nargs='+',type=Path);ap.add_argument('--saida',required=True,type=Path)
    a=ap.parse_args();res=[replay(b) for b in a.bags];a.saida.write_text(json.dumps(res,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in row.items() if k!='amostras'} for row in res],ensure_ascii=False))
