#!/usr/bin/env python3
"""Diagnostico passivo: nao envia goals, cancela actions ou publica velocidades."""
import json,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,TwistStamped
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Log
from action_msgs.msg import GoalStatusArray
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy


def main():
    rclpy.init();n=Node('diagnostico_nav_principal_leitura')
    counts={t:0 for t in ('/goal_pose_alinhado','/goal_pose','/cmd_vel_ctrl','/cmd_vel_nav_base','/cmd_vel_nav_especial','/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel','/navegacao/heartbeat')}
    nonzero={t:0 for t in counts if t.startswith('/cmd_vel')}
    status={};logs=[];subs=[]
    def receber(top,m):
        counts[top]+=1
        if top in nonzero:
            tw=m.twist
            nonzero[top]+=int(any(abs(v)>1e-5 for v in (tw.linear.x,tw.linear.y,tw.angular.z)))
    for t in counts:
        subs.append(n.create_subscription(PoseStamped if t.startswith('/goal_pose') else TwistStamped,t,lambda m,t=t:receber(t,m),10))
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for t in ('/navigate_to_pose/_action/status','/follow_path/_action/status'):
        subs.append(n.create_subscription(GoalStatusArray,t,lambda m,t=t:status.update({t:[s.status for s in m.status_list]}),qos))
    def log(m):
        if m.name in ('alinhador_passagem','bt_navigator','controller_server','arbitro_nav_principal'):
            logs.append(m.name+': '+m.msg[:220])
    subs.append(n.create_subscription(Log,'/rosout',log,10))
    end=time.monotonic()+3
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    result={'nos':[x for x,ns in n.get_node_names_and_namespaces() if x in ('alinhador_passagem','arbitro_nav_principal','bt_navigator','controller_server')]}
    for name in ('bt_navigator','controller_server','collision_monitor'):
        cli=n.create_client(GetState,'/'+name+'/get_state')
        if cli.wait_for_service(timeout_sec=.5):
            f=cli.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=1)
            result[name]=f.result().current_state.label if f.done() and f.result() else 'sem resposta'
        else:result[name]='servico ausente'
    cli=n.create_client(GetParameters,'/alinhador_passagem/get_parameters')
    if cli.wait_for_service(timeout_sec=.5):
        rq=GetParameters.Request();rq.names=['nav2_principal','topico_goal','use_sim_time']
        f=cli.call_async(rq);rclpy.spin_until_future_complete(n,f,timeout_sec=1)
        if f.done() and f.result():result['parametros']={k:(v.bool_value if v.type==1 else v.string_value) for k,v in zip(rq.names,f.result().values)}
    result['entrada_goal']={t:[i.node_name for i in n.get_subscriptions_info_by_topic(t) if i.node_name!=n.get_name()] for t in ('/goal_pose','/goal_pose_alinhado')}
    result['publicadores']={t:[i.node_name for i in n.get_publishers_info_by_topic(t)] for t in ('/cmd_vel_nav_base','/cmd_vel_nav_especial','/cmd_vel_nav','/navegacao/heartbeat')}
    result['amostras_recebidas']=counts;result['comandos_nao_nulos']=nonzero;result['status_actions']=status;result['logs_recentes']=logs[-8:]
    for chave,valor in result.items():
        print(chave+': '+json.dumps(valor,ensure_ascii=False,separators=(',',':')))
    n.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
