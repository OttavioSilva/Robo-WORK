#!/usr/bin/env python3
"""PARTIDA_ARBITRO_20261111: descoberta persistente e verificacao somente leitura.
Nao chama SetBool, nao renova lease e nao publica velocidades/goals.
"""
import argparse
import json
import time
from pathlib import Path


def avaliar(d, modo):
    pendencias=[]
    if not d['servico']:pendencias.append('servico /navegacao/especial ausente')
    if d['principal'] is not True:pendencias.append('nav2_principal nao e true')
    if d['modo'] != modo:pendencias.append('supervisor_modo divergente/ausente')
    if d['publicadores'] != ['/arbitro_nav_principal']:
        pendencias.append('saida /cmd_vel_nav exige exatamente um publisher do arbitro')
    if '/alinhador_passagem' not in d['heartbeat']:
        pendencias.append('publisher heartbeat do alinhador ausente')
    return pendencias


def conferir(modo='observar', timeout=25):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType
    from std_srvs.srv import SetBool
    rclpy.init()
    n=rclpy.create_node('verificar_supervisor_ativo')
    params=n.create_client(GetParameters,'/alinhador_passagem/get_parameters')
    lease=n.create_client(SetBool,'/navegacao/especial')
    limite=time.monotonic()+timeout
    d=dict(servico=False,principal=None,modo=None,publicadores=[],heartbeat=[])
    futuro=None;estavel=None;consulta=0
    try:
        while rclpy.ok() and time.monotonic()<limite:
            rclpy.spin_once(n,timeout_sec=.1)
            agora=time.monotonic()
            if futuro is None and params.service_is_ready() and agora>=consulta:
                req=GetParameters.Request();req.names=['nav2_principal','supervisor_modo']
                futuro=params.call_async(req)
            if futuro is not None and futuro.done():
                res=futuro.result()
                if res is not None and len(res.values)==2:
                    a,b=res.values
                    d['principal']=a.bool_value if a.type==ParameterType.PARAMETER_BOOL else None
                    d['modo']=b.string_value if b.type==ParameterType.PARAMETER_STRING else None
                futuro=None;consulta=agora+.5
            def nomes(topico):
                return sorted((p.node_namespace.rstrip('/')+'/'+p.node_name) for p in n.get_publishers_info_by_topic(topico))
            d.update(servico=lease.service_is_ready(),publicadores=nomes('/cmd_vel_nav'),heartbeat=nomes('/navegacao/heartbeat'))
            pendencias=avaliar(d,modo)
            if not pendencias:
                estavel=estavel or agora
                if agora-estavel>=1.5:
                    return dict(ok=True,evidencias=d,pendencias=[])
            else:estavel=None
        return dict(ok=False,evidencias=d,pendencias=avaliar(d,modo))
    finally:
        n.destroy_node();rclpy.try_shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--modo',choices=['observar','aplicar'],default='observar')
    p.add_argument('--timeout',type=float,default=25)
    p.add_argument('--saida',type=Path)
    a=p.parse_args()
    try:r=conferir(a.modo,a.timeout)
    except Exception as e:r=dict(ok=False,erro=str(e))
    texto=json.dumps(r,ensure_ascii=False,indent=2)
    if a.saida:a.saida.write_text(texto+'\n')
    print(texto)
    return 0 if r['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
