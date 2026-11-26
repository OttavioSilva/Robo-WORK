#!/usr/bin/env python3
"""Compara filtro angular original e mascara corporal no scan bruto gravado; sem ROS ativo."""
import importlib.util,json,math,sys
from pathlib import Path
from ler_mcap_parcial import mensagens
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
w=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('scan_filter',w/'src/robo_lidar/scripts/scan_filter_node.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
old=m.parse_intervals('-162:-146,-140:-110,100:138,142:160');counts={'scans':0,'corpo_removidos':0,'externos_preservados':0,'externos_antes_cegados_recuperados':0,'corpo_antes_vazava':0}
cl=None;bins={}
for top,typ,t,data in mensagens(sys.argv[1],{'/scan_raw'}):
 if cl is None:cl=get_message(typ)
 sc=deserialize_message(data,cl);counts['scans']+=1
 for i,r in enumerate(sc.ranges):
  if not math.isfinite(r) or not sc.range_min<=r<=sc.range_max:continue
  a=-(sc.angle_min+i*sc.angle_increment)
  body=m.ponto_do_corpo(r,a);blocked=m.angle_is_blocked(a,old)
  if body:
   counts['corpo_removidos']+=1
   if not blocked:counts['corpo_antes_vazava']+=1
   deg=round(math.degrees(a));bins[deg]=bins.get(deg,0)+1
  else:
   counts['externos_preservados']+=1
   if blocked:counts['externos_antes_cegados_recuperados']+=1
result={**counts,'bins_motor_corpo_graus':bins}
Path(sys.argv[2]).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(counts))
