"""Politica e metodos reais do coordenador, sem inicializar ROS/hardware."""
import ast
import importlib.util
import math
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
NAV=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(NAV/'scripts'))
from controle_nav_principal import Autoridade, passagem_confirmada


def metodo(nome):
    tree=ast.parse((NAV/'scripts/alinhador_passagem.py').read_text())
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==nome)
    env={'time':time,'math':math,'GoalStatus':NS(STATUS_SUCCEEDED=4,STATUS_ABORTED=6,STATUS_CANCELED=5),
         '_ang':lambda v:math.atan2(math.sin(v),math.cos(v)),
         'Cancelado':InterruptedError,'NavigateToPose':NS(Goal=lambda:NS()),
         'SetBool':NS(Request=lambda:NS())}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'<alinhador real>','exec'),env)
    return env[nome]


def test_autoridade_lease_troca_fila_e_protecao():
    a=Autoridade()
    assert not a.permite('nav2',1,1,1)
    a.trocar('nav2',1);a.desde_ros=10
    assert a.permite('nav2',1.1,10.1,10.1)
    assert not a.permite('especial',1.1,10.1,10.1)
    a.trocar('especial',1.2);a.desde_ros=10.2
    assert not a.permite('nav2',1.3,10.3,10.3)
    assert not a.permite('especial',1.3,10.1,10.3)
    assert not a.permite('especial',2,10.5,10.5)
    a.renovar('nav2',2)
    assert not a.permite('especial',2,10.5,10.5)
    a.renovar('especial',2)
    assert a.permite('especial',2.1,10.6,10.6)
    assert not a.permite('especial',2.1,10.8,10.6)


@pytest.mark.parametrize('largura,lados,mapa,dist,ok',[(.82,2,.82,1,True),(.82,1,.82,1,False),
    (.60,2,.82,1,False),(1.2,2,.82,1,False),(.82,2,.82,3,False),(.82,2,1.2,1,False)])
def test_passagem_dupla_confirmacao(largura,lados,mapa,dist,ok):
    assert passagem_confirmada({'largura_mapa':mapa,'entrada':(0,0)},(dist,0,0),
        {'largura':largura,'lados':lados}) is ok


def fake_exec():
    calls=[]
    f=NS(_log_nav_principal=lambda *a:None,p=lambda k:{'max_passagens':4,'supervisor_modo':'aplicar','supervisor_regioes':str(NAV/'config/regioes_passagem.yaml')}[k],sem_encosto={'FINISH'},lock=threading.Lock(),dono_mov=None)
    from supervisor_passagem import Passagens
    f.passagens_supervisor=Passagens()
    f.get_logger=lambda:NS(info=lambda m:None)
    f._esperar_parada_supervisor=lambda:calls.append('parada_confirmada')
    f._iniciar_evidencia_passagem=lambda:calls.append('evidencia')
    f._confirmar_saida_supervisor=lambda *a:calls.append('saida_confirmada')
    f._drenar_nav=lambda:calls.append('terminal')
    f._parar=lambda:calls.append('zero')
    f._autoridade_mov=lambda d:(calls.append(d),setattr(f,'dono_mov',d))
    f._encostar=lambda a:calls.append(('bancada',a))
    f._publicar_resultado=lambda *a:calls.append(a)
    f._navegar_principal=lambda g,n,v:(calls.append(('nav2',g)), 'ok')[1]
    return f,calls


@pytest.mark.parametrize('area',[None,'FINISH','WS_1'])
def test_goal_comum_bancada_preserva_interface_e_yaw(area):
    f,calls=fake_exec();goal=NS(pose=NS(position=NS(x=1,y=2),orientation=NS(z=.7,w=.7)))
    metodo('_executar_principal')(f,goal,area)
    assert calls[0]==('nav2',goal)
    assert (('bancada','WS_1') in calls)==(area=='WS_1')
    assert 'terminal' in calls and f.dono_mov is None


@pytest.mark.parametrize('curva',[False,True])
def test_corredor_t_devolve_nav2_com_novo_goal_e_sem_insistir(curva,monkeypatch):
    f,calls=fake_exec();f.p=lambda k:{'max_passagens':4,'largura_estreita':.9,
        'distancia_preparo':.6,'distancia_saida':.45,'centro_corpo':.08,'supervisor_modo':'aplicar','supervisor_regioes':str(NAV/'config/regioes_passagem.yaml')}[k]
    f.plano=[(0,0),(2,0)];f.plano_t=time.monotonic()+100;f.scan=(time.monotonic(),[],None)
    f._pose_mapa_yaw=lambda **kw:(0,0,0);f._mapa=lambda:object()
    info={'entrada':(0,0),'saida':(1,0),'largura_mapa':.82,'eixo_graus':0}
    if curva: info['curva']={'juncao':(1,0),'lado':1,'entrada_b':(1,0),'saida_b':(1,1),'eixo_b':(0,1)}
    fn=metodo('_executar_principal')
    fn.__globals__['planejar_etapas']=lambda *a,**k:([{'x':0,'y':0,'yaw':0}],info)
    import supervisor_passagem
    monkeypatch.setattr(supervisor_passagem,'regiao_conhecida',lambda *a:True)
    fn.__globals__['medir_corredor']=lambda pts:{'largura':.82,'lados':2}
    n=[0]
    def nav(goal,nome,vigia):
        n[0]+=1;calls.append('navigate')
        if n[0]==1: assert vigia();return 'interrompido'
        return 'ok'
    f._navegar_principal=nav;f._chegar_preparo=lambda *a:(calls.append('preparo'),'ok')[1]
    f._travessia_lidar=lambda *a:(calls.append('corredor'),'ok')[1]
    f._travessia_lateral=lambda *a:(calls.append('T'),'ok')[1]
    goal=NS(pose=NS(position=NS(x=2,y=0)))
    fn(f,goal)
    assert n[0]==2 and calls.index('especial')<calls.index('corredor')
    assert calls.index('nav2')<len(calls)-1
    assert calls.index('parada_confirmada') < calls.index('especial')
    assert calls.index('saida_confirmada') < calls.index('nav2')
    assert ('T' in calls)==curva
    assert 'preparo' not in calls  # ja esta alinhado dentro do corredor, nao volta para tras


class Future:
    def __init__(self,result=None,done=True):self.value=result;self.ready=done
    def done(self):return self.ready
    def result(self):return self.value
    def add_done_callback(self,cb):
        if self.ready:cb(self)


def test_cancelamento_espera_terminal_antes_de_nova_autoridade():
    calls=[];result=Future(NS(status=5),False)
    h=NS(cancel_goal_async=lambda:(calls.append('cancel'),setattr(result,'ready',True)))
    f=NS(_log_nav_principal=lambda *a:None,nav_pendente=None,handle_ativo=h,nav_resultado=result,dono_mov='nav2')
    metodo('_drenar_nav')(f)
    assert calls==['cancel'] and f.handle_ativo is None and f.nav_resultado is None


def test_aceitacao_tardia_nao_permite_takeover(monkeypatch):
    clock=iter([0,10]);fn=metodo('_drenar_nav')
    fn.__globals__['time']=NS(monotonic=lambda:next(clock),sleep=lambda s:None)
    f=NS(_log_nav_principal=lambda *a:None,nav_pendente=Future(done=False),handle_ativo=None,nav_resultado=None,dono_mov='nav2')
    with pytest.raises(RuntimeError,match='aceitacao'):fn(f)
    assert f.dono_mov is None and f.nav_pendente is not None


def test_novo_goal_durante_aceitacao_cancela_e_drena():
    calls=[];h=NS(accepted=True,cancel_goal_async=lambda:calls.append('cancel'))
    f=NS(_log_nav_principal=lambda *a:None,_autoridade_mov=lambda d:None,nav=NS(wait_for_server=lambda **k:True,
        send_goal_async=lambda req:Future(h)),_novo_goal_chegou=lambda:False,
        dono_mov='nav2',_drenar_nav=lambda:calls.append('drenar'))
    def espera(futuro,limite):raise InterruptedError()
    f._esperar=espera
    with pytest.raises(InterruptedError):metodo('_navegar_principal')(f,object(),'goal')
    assert calls==['drenar']  # futuro persistente sera drenado pelo trabalhador
    f.dono_mov=None


def test_perfil_nao_ativa_patches_m34_e_preserva_invariantes():
    import yaml
    import xml.etree.ElementTree as ET
    c=yaml.safe_load((NAV/'config/nav2_params_principal.yaml').read_text())
    assert c['controller_server']['ros__parameters']['FollowPath']['plugin']=='dwb_core::DWBLocalPlanner'
    assert 'forward_veloz' not in c['collision_monitor']['ros__parameters']['VelocityPolygonStop']
    tree=ET.parse(NAV/'behavior_trees/navigate_to_pose_principal.xml')
    assert len(list(tree.iter('SmoothPath')))==2
    assert tree.find('.//CaminhoSemCarimbo') is not None
    assert tree.find('.//DriveOnHeading').get('server_name')=='destravar'
    assert tree.find('.//Sequence[@name="GoalNovo"]/Script').get('code')=='caminho_do_goal := false'


def test_launch_flag_remapeia_todos_publicadores_sem_mexer_cm():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, ExecuteProcess
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    from launch.utilities import perform_substitutions
    spec=importlib.util.spec_from_file_location('launch_principal',NAV/'launch/navigation.launch.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    m.get_package_share_directory=lambda _:str(NAV)
    for enabled in ('false','true'):
        d=m.generate_launch_description();ctx=LaunchContext();ctx.launch_configurations['nav2_principal']=enabled
        for a in d.entities:
            if isinstance(a,DeclareLaunchArgument):a.execute(ctx)
        nodes=[a for a in d.entities if isinstance(a,Node)]
        ctrl=next(n for n in nodes if n.node_executable=='controller_server')
        assert Path(evaluate_parameters(ctx,ctrl._Node__parameters)[0]).name==('nav2_params_principal.yaml' if enabled=='true' else 'nav2_params.yaml')
        for exe in ('behavior_server','progressive_spin_node','mecanum_escape_node'):
            n=next(n for n in nodes if n.node_executable==exe)
            pairs=[(perform_substitutions(ctx,a),perform_substitutions(ctx,b)) for a,b in n._Node__remappings]
            assert ('cmd_vel','cmd_vel_nav_base' if enabled=='true' else 'cmd_vel_nav') in pairs
        for proc in [a for a in d.entities if isinstance(a,ExecuteProcess) and not isinstance(a,Node)]:
            cmd=[perform_substitutions(ctx,p) for p in proc.cmd]
            if any('suavizador_nav.py' in p or 'destravar_node.py' in p for p in cmd):
                assert '/cmd_vel_nav:='+('cmd_vel_nav_base' if enabled=='true' else 'cmd_vel_nav') in cmd


def test_nav2_real_metodo_sem_pre_giro_reenvio_ou_yaw_alterado():
    calls=[];goal=object();result=Future(NS(status=4))
    h=NS(accepted=True,get_result_async=lambda:result)
    f=NS(_log_nav_principal=lambda *a:None,_autoridade_mov=lambda d:calls.append(('autoridade',d)),
        nav=NS(wait_for_server=lambda **k:True,send_goal_async=lambda req:(calls.append(('goal',req.pose)),Future(h))[1]),
        _novo_goal_chegou=lambda:False,dono_mov='nav2',_esperar=lambda fut,t:fut.result(),
        p=lambda k:1,_drenar_nav=lambda:calls.append('terminal'))
    assert metodo('_navegar_principal')(f,goal,'comum')=='ok'
    assert calls==[('autoridade','nav2'),('goal',goal),'terminal']


def test_preempcao_durante_manobra_para_e_nao_reenvia():
    f,calls=fake_exec();f.dono_mov='especial'
    def cancelar(*a):raise InterruptedError('goal novo')
    f._navegar_principal=cancelar
    with pytest.raises(InterruptedError):metodo('_executar_principal')(f,object())
    assert calls==['terminal','zero'] and f.dono_mov is None


def test_novo_goal_bloqueia_comando_especial_antes_da_publicacao():
    calls=[]
    f=NS(_log_nav_principal=lambda *a:None,p=lambda k:True,_novo_goal_chegou=lambda:True,
         _autoridade_mov=lambda d:calls.append('autoridade'),
         pub_cmd=NS(publish=lambda m:calls.append('movimento')))
    with pytest.raises(InterruptedError):metodo('_publicar_vel')(f,.03,0,0)
    assert calls==[]


def test_vigia_interrompe_somente_depois_de_cancelar_e_terminar_nav2():
    calls=[];clock=[0]
    fn=metodo('_navegar_principal')
    def mono():clock[0]+=1;return clock[0]
    fn.__globals__['time']=NS(monotonic=mono)
    h=NS(accepted=True,get_result_async=lambda:Future(done=False))
    f=NS(_log_nav_principal=lambda *a:None,_autoridade_mov=lambda d:calls.append(d),dono_mov='nav2',
        nav=NS(wait_for_server=lambda **k:True,send_goal_async=lambda req:Future(h)),
        _novo_goal_chegou=lambda:False,p=lambda k:.1,
        _drenar_nav=lambda:calls.append('terminal'))
    n=[0]
    def esperar(futuro,limite):
        n[0]+=1;return h if n[0]==1 else None
    f._esperar=esperar
    assert fn(f,object(),'passagem',lambda:True)=='interrompido'
    assert calls==['nav2','terminal']


def test_arbitro_real_filtra_fonte_e_para_sem_lease():
    tree=ast.parse((NAV/'scripts/arbitro_nav_principal.py').read_text())
    cls=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='Arbitro')
    sent=[];mono=[1.0];ros=[100.0]
    def msg(stamp=100.1,dono='nav2'):
        return NS(header=NS(stamp=NS(sec=int(stamp),nanosec=int((stamp-int(stamp))*1e9)),frame_id=dono))
    class Node:
        def __init__(self,*a):pass
        def create_publisher(self,*a):return NS(publish=lambda m:sent.append(m))
        def create_subscription(self,*a):pass
        def create_service(self,*a):pass
        def create_timer(self,*a):pass
        def get_logger(self):return NS(info=lambda mensagem:None)
        def get_clock(self):return NS(now=lambda:NS(nanoseconds=int(ros[0]*1e9),to_msg=lambda:msg(ros[0]).header.stamp))
    env={'Node':Node,'Autoridade':Autoridade,'TwistStamped':msg,'SetBool':object,'time':NS(monotonic=lambda:mono[0])}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),'<arbitro real>','exec'),env)
    n=env['Arbitro']();n.cmd('nav2',msg());assert sent==[]
    n.trocar(NS(data=False),NS());assert len(sent)==1
    mono[0]=1.1;ros[0]=100.1
    n.cmd('especial',msg());assert len(sent)==1
    n.cmd('nav2',msg());assert len(sent)==2
    mono[0]=2;ros[0]=101;n.vigiar();assert len(sent)==3
    n.cmd('nav2',msg(101));assert len(sent)==3
    # heartbeat atrasado nao renova; novo heartbeat da fonte correta permite retomar.
    n.heartbeat(msg(100.1));n.cmd('nav2',msg(101));assert len(sent)==3
    n.heartbeat(msg(101));n.cmd('nav2',msg(101));assert len(sent)==4
