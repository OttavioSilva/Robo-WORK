#!/usr/bin/env python3
"""Compara correcao angular map->odom e saltos IMU/odom, leitura parcial limitada."""
import bisect,json,math,sys
from pathlib import Path
from ler_mcap_parcial import mensagens
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
p=Path(sys.argv[1]);out=Path(sys.argv[2]);cl={};tf=[];imu=[];odom=[]
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def ang(v):return math.atan2(math.sin(v),math.cos(v))
for top,typ,ns,data in mensagens(p,{'/tf','/imu/data','/imu/yaw_deg','/odom'}):
 if typ not in cl:cl[typ]=get_message(typ)
 m=deserialize_message(data,cl[typ]);t=ns/1e9
 if top=='/tf':
  for tr in m.transforms:
   if tr.header.frame_id=='map' and tr.child_frame_id=='odom':tf.append((t,yaw(tr.transform.rotation)))
 elif top=='/imu/data':imu.append((t,yaw(m.orientation)))
 elif top=='/odom':odom.append((t,yaw(m.pose.pose.orientation)))
def delta(rows):
 return [(b[0],math.degrees(ang(b[1]-a[1])),b[0]-a[0]) for a,b in zip(rows,rows[1:]) if abs(ang(b[1]-a[1]))>1e-8]
dt=delta(tf);di=delta(imu);do=delta(odom);it=[p[0] for p in imu]
events=[]
for t,d,interval in dt:
 if abs(d)<3:continue
 i=bisect.bisect_left(it,t);window=[v for tt,v,gap in di if t-.3<=tt<=t+.3]
 events.append({'t':t,'correcao_slam_graus':d,'intervalo_tf_s':interval,'maior_salto_imu_janela_graus':max(map(abs,window),default=0)})
r={'amostras':{'tf':len(tf),'imu':len(imu),'odom':len(odom)},'correcoes_slam_acima_3graus':len(events),'max_correcao_slam_graus':max((abs(x[1]) for x in dt),default=0),'max_salto_imu_graus':max((abs(x[1]) for x in di),default=0),'max_salto_odom_graus':max((abs(x[1]) for x in do),default=0),'eventos':events}
out.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='eventos'}))
