#!/usr/bin/env python3
"""Preflight somente leitura; nunca publica comandos de movimento."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/robo_navigation/scripts'))
from giro_frontal import avaliar_varredura, Observacao, envolve_disco


def evaluate(data):
    checks = dict(data.get('checks', {}))
    checks['sensores_temporais_recentes'] = all(data.get('ages', {}).get(k, math.inf) <= 0.3
        and data['ages'][k] >= 0 for k in ('scan', 'imu', 'odom'))
    raw = data.get('scan')
    checks['envelope_observado_livre'] = (raw is not None and
        avaliar_varredura(raw['ranges'], raw['amin'], raw['inc'], raw['rmin'], raw['rmax'],
                          tuple(data.get('laser', (0.330,0.005,0))),
                          math.hypot(0.365,0.315)+0.05) is Observacao.LIVRE)
    publishers = data.get('publishers', {})
    expected = {'/cmd_vel_nav_seguro':'arbitro_t','/cmd_vel_nav_t':'alinhador_passagem',
                '/cmd_vel_smoothed':'velocity_smoother','/cmd_vel':'collision_monitor'}
    checks['cadeia_exclusiva'] = all(publishers.get(topic) == [node] for topic,node in expected.items())
    checks['sensores_parados'] = (data.get('gyro_abs', math.inf) <= 0.03 and
                                  data.get('linear_abs', math.inf) <= 0.01)
    required = ['sensores_temporais_recentes','envelope_observado_livre','cadeia_exclusiva',
                'sensores_parados','modo_experimental','cm_envelope','cm_ativo','smoother_ativo',
                'hardware_timing_valido','limites_temporais_configurados', 'tf_laser_valida',
                'arbitro_disponivel','saida_zero', 'arbitro_parado','sem_correcao_pos_cm','limites_de_movimento_configurados','autoridade_no_hardware']
    ok = all(checks.get(k,False) for k in required)
    return dict(parecer='APTO' if ok else 'NAO APTO', checks=checks,
                pendencias=[k for k in required if not checks.get(k,False)])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=5)
    parser.add_argument('--snapshot',type=Path,help='JSON de simulacao; nao consulta ROS')
    args=parser.parse_args(argv)
    if args.snapshot:
        data=json.loads(args.snapshot.read_text())
        r=evaluate(data)
        r['simulado']=True
        print(json.dumps(r,indent=2))
        return 0 if r['parecer']=='APTO' else 1
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.parameter import parameter_value_to_python
    from lifecycle_msgs.srv import GetState
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time
    rclpy.init()
    node=Node('preflight_giro_t')
    data={'ages':{}, 'checks':{}, 'publishers':{}}
    received={}
    def scan(msg):
        received['scan']=msg.header.stamp
        data['scan']=dict(ranges=list(msg.ranges),amin=msg.angle_min,inc=msg.angle_increment,
                          rmin=msg.range_min,rmax=msg.range_max)
    def imu(msg):
        received['imu']=msg.header.stamp
        data['gyro_abs']=abs(msg.angular_velocity.z)
    def odom(msg):
        received['odom']=msg.header.stamp
        data['linear_abs']=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
    def timing(msg):
        try:
            item=json.loads(msg.data)
            if not isinstance(item,dict): return
            if item.get('valid'):
                timing_seen[item['sensor']]=time.monotonic()
            elif item.get('timed_frame'):
                timing_seen[item['sensor']]=-math.inf
        except (ValueError,KeyError):
            pass
    def state(msg):
        try:
            item=json.loads(msg.data)
            if not isinstance(item,dict): return
            data['checks']['arbitro_disponivel']=True
            data['checks']['arbitro_parado']=not item['ativo'] and not item['falha']
            received['arb_state_mono']=time.monotonic()
        except (ValueError,KeyError):
            pass
    def command(msg):
        tw=msg.twist
        data['checks']['saida_zero']=all(abs(x)<1e-5 for x in (tw.linear.x,tw.linear.y,tw.angular.z))
        received['cmd_mono']=time.monotonic()
    node.create_subscription(LaserScan,'/scan_temporal_front',scan,qos_profile_sensor_data)
    node.create_subscription(Imu,'/imu/data_temporal',imu,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/odom',odom,20)
    node.create_subscription(String,'/hardware/sensor_timing',timing,20)
    node.create_subscription(String,'/arbitro_t/estado',state,20)
    node.create_subscription(TwistStamped,'/cmd_vel',command,20)
    timing_seen={}
    buffer=Buffer()
    listener=TransformListener(buffer,node)
    requested={
        'alinhador_passagem':['modo_curva_t','limite_vel_angular_real_t','limite_vel_linear_real_t'],
        'collision_monitor':['polygons','SweptTurnStop.enabled','SweptTurnStop.min_points',
            'SweptTurnStop.left_turn.points','SweptTurnStop.right_turn.points'],
        'robo_hardware':['temporal_enabled','device_clock_drift_bound_ppm',
                         'lidar_capture_lag_bound_s','imu_capture_lag_bound_s','heading_hold_enabled','command_authority']}
    futures={}
    for name,keys in requested.items():
        client=AsyncParameterClient(node,name)
        futures[name]=(keys,client,client.get_parameters(keys))
    lifecycle={}
    for name in ('collision_monitor','velocity_smoother'):
        client=node.create_client(GetState,'/'+name+'/get_state')
        lifecycle[name]=(client,client.call_async(GetState.Request()))
    end=time.monotonic()+max(1,args.seconds)
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.05)
        for key in ('scan','imu','odom'):
            stamp=received.get(key)
            data['ages'][key]=((node.get_clock().now().nanoseconds-
                stamp.sec*1000000000-stamp.nanosec)/1e9) if stamp else math.inf
        for topic in ('/cmd_vel_nav_seguro','/cmd_vel_nav_t','/cmd_vel_smoothed','/cmd_vel'):
            data['publishers'][topic]=[i.node_name for i in node.get_publishers_info_by_topic(topic)]
        params={}
        for name,(keys,client,f) in futures.items():
            if f.done() and f.result() is not None:
                params[name]=dict(zip(keys,[parameter_value_to_python(v) for v in f.result().values]))
        a=params.get('alinhador_passagem',{})
        cm=params.get('collision_monitor',{})
        hw=params.get('robo_hardware',{})
        data['checks']['modo_experimental']=a.get('modo_curva_t')=='giro_frontal'
        data['checks']['limites_de_movimento_configurados']=(a.get('limite_vel_angular_real_t',-1)>=0.10
            and a.get('limite_vel_linear_real_t',-1)>=0.03)
        data['checks']['sem_correcao_pos_cm']=hw.get('heading_hold_enabled') is False
        data['checks']['autoridade_no_hardware']=hw.get('command_authority')=='collision_monitor'
        try:
            geometria_cm = all(envolve_disco(json.loads(cm.get('SweptTurnStop.'+name+'.points','[]')),
                math.hypot(.365,.315)+.05) for name in ('left_turn','right_turn'))
        except (TypeError, ValueError):
            geometria_cm = False
        data['checks']['cm_envelope']=('SweptTurnStop' in cm.get('polygons',[]) and geometria_cm and
            cm.get('SweptTurnStop.enabled') is True and cm.get('SweptTurnStop.min_points')==1)
        data['checks']['limites_temporais_configurados']=(hw.get('temporal_enabled') is True and
            all(math.isfinite(hw.get(k,-1)) and hw.get(k,-1)>=0 for k in
                ('device_clock_drift_bound_ppm','lidar_capture_lag_bound_s','imu_capture_lag_bound_s')))
        for name,key in [('collision_monitor','cm_ativo'),('velocity_smoother','smoother_ativo')]:
            f=lifecycle[name][1]
            data['checks'][key]=f.done() and f.result() is not None and f.result().current_state.id==3
        data['checks']['hardware_timing_valido']=all(time.monotonic()-timing_seen.get(k,-math.inf)<=0.3 for k in ('lidar','imu'))
        data['checks']['arbitro_disponivel']=time.monotonic()-received.get('arb_state_mono',-math.inf)<=0.3
        data['checks']['saida_zero']=data['checks'].get('saida_zero',False) and time.monotonic()-received.get('cmd_mono',-math.inf)<=0.3
        try:
            tr=buffer.lookup_transform('base_link','laser_raw',Time())
            q=tr.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            data['laser']=[tr.transform.translation.x,tr.transform.translation.y,yaw]
            data['checks']['tf_laser_valida']=True
        except Exception:
            data['checks']['tf_laser_valida']=False
        r=evaluate(data)
        print(json.dumps(r,indent=2,allow_nan=False))
        return 0 if r['parecer']=='APTO' else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    raise SystemExit(main())
