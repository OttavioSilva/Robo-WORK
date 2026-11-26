"""Regressoes medidas da missao35; nenhuma publicacao ROS."""
import importlib.util,math,sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest,yaml
NAV=Path(__file__).resolve().parents[1];WS=NAV.parents[1]
sys.path.insert(0,str(NAV/'scripts'))
from suavizador_nav import Filtro
from filtro_corpo_scan import filtrar,MemoriaPerto
from test_nav2_principal import metodo,Future


def lidar():
    spec=importlib.util.spec_from_file_location('scan_filter_m35',WS/'src/robo_lidar/scripts/scan_filter_node.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def test_re_limitada_e_cm_ampliado_sem_mudar_criticas():
    a=yaml.safe_load((NAV/'config/nav2_params.yaml').read_text())
    b=yaml.safe_load((NAV/'config/nav2_params_principal.yaml').read_text())
    x=a['controller_server']['ros__parameters']['FollowPath'].copy();y=b['controller_server']['ros__parameters']['FollowPath'].copy()
    assert x.pop('min_vel_x')==0 and y.pop('min_vel_x')==-.10
    assert x==y
    assert b['velocity_smoother']['ros__parameters']['min_velocity'][0]<=-.10
    pts=yaml.safe_load(b['collision_monitor']['ros__parameters']['VelocityPolygonStop']['backward']['points'])
    assert min(p[0] for p in pts)==-.315
    assert b['controller_server']['ros__parameters']['FollowPath']['vx_samples']==7


@pytest.mark.parametrize('vy',[.08,-.08])
def test_lateral_sem_retenção_frontal_apos_avanco_e_zero(vy):
    f=Filtro(preservar_eixos_zero=True)
    f.passo(0,.2,0,0);f.passo(.05,0,0,0)
    for t in (.1,.15,.2,.25):
        x,y,w=f.passo(t,0,vy,0)
        assert x==0 and y*vy>0 and w==0
        assert abs(y)<=abs(vy)+1e-9


def test_diagonal_intencional_e_giro_preservados():
    f=Filtro(preservar_eixos_zero=True)
    f.passo(0,.2,0,0)
    x,y,w=f.passo(.1,.08,-.053,.1)
    assert x>0 and y<0 and w>0
    assert f.passo(.2,0,0,.16)==(0,0,.16)
    assert f.passo(.25,0,0,0)==(0,0,0)


def test_reproduz_pulso_no_filtro_legado():
    f=Filtro();f.passo(0,.2,0,0);f.passo(.05,0,0,0)
    assert f.passo(.1,0,.08,0)[0]>.01


@pytest.mark.parametrize('degrees',[-164,-150,-146,-144,-142,104,126,138,140,142,162,164])
def test_motor_vira_nan_sem_apagar_obstaculo_alem_no_mesmo_angulo(degrees):
    m=lidar();a=math.radians(degrees)
    # Alcance dentro do envelope; mesmo angulo em parede distante tem que permanecer.
    inside=[r/1000 for r in range(146,650) if m.ponto_do_corpo(r/1000,a)]
    assert inside
    assert m.ponto_do_corpo(inside[len(inside)//2],a)
    assert not m.ponto_do_corpo(1.5,a)
    r=inside[len(inside)//2]
    filtered,n=filtrar([r,1.5],a,0,(.33,.005,0))
    assert n==1 and math.isnan(filtered[0]) and filtered[1]==1.5


def test_motor_da_borda_antigamente_escapava_e_nao_entra_na_memoria():
    x,y=.35,-.32;r=math.hypot(x-.33,y-.005);a=math.atan2(y-.005,x-.33)
    assert lidar().ponto_do_corpo(r,a)
    vals,n=filtrar([r],a,.01,(.33,.005,0));assert n==1 and math.isnan(vals[0])
    memory=MemoriaPerto();memory.passo(0,(0,0,0),[r],a,.01,(.33,.005,0))
    assert not memory.pts


def test_filtro_ativo_sem_mascaras_angulares_cegas():
    c=yaml.safe_load((WS/'src/robo_lidar/config/lidar.yaml').read_text())['scan_filter']['ros__parameters']
    assert c['body_filter_enabled'] and c['blocked_intervals_deg']==''
    assert not lidar().ponto_do_corpo(1.5,math.radians(140))


@pytest.mark.parametrize('status,preempt,confirmed,expected',[(6,False,True,'interrompido'),(6,False,False,'falha'),(5,False,True,'falha'),(6,True,True,'falha'),(4,False,True,'ok')])
def test_abortado_com_passagem_confirmada_pode_transferir_mas_cancelado_nao(status,preempt,confirmed,expected):
    calls=[];h=NS(accepted=True,get_result_async=lambda:Future(NS(status=status)))
    count=[0]
    def novo():
        count[0]+=1
        return preempt if count[0]>=3 else False
    f=NS(_log_nav_principal=lambda *a:None,_autoridade_mov=lambda d:None,dono_mov='nav2',
         nav=NS(wait_for_server=lambda **k:True,send_goal_async=lambda req:Future(h)),
         _novo_goal_chegou=novo,_esperar=lambda fut,t:fut.result(),p=lambda k:1,
         _drenar_nav=lambda:calls.append('terminal'))
    assert metodo('_navegar_principal')(f,object(),'goal',lambda:confirmed)==expected
    assert calls==['terminal']


def test_callback_real_filtra_scan_front_sem_injetar_livre_atras_do_motor():
    from sensor_msgs.msg import LaserScan
    source=LaserScan();source.angle_min=math.radians(140);source.angle_max=source.angle_min
    source.angle_increment=.01;source.range_min=.146;source.range_max=5.8
    source.ranges=[.40,1.5];source.intensities=[1.0,2.0]
    result=[]
    f=NS(_log_nav_principal=lambda *a:None,invert_angles=False,force_out_of_range=True,body_filter_enabled=True,
         body_laser=(.33,.005,0),blocked_intervals=[],publisher=NS(publish=lambda m:result.append(m)))
    lidar().ScanFilterNode.scan_callback(f,source)
    assert len(result)==1
    assert math.isnan(result[0].ranges[0]) and result[0].ranges[1]==1.5
    assert result[0].intensities[0]==0
    assert source.ranges[0]==pytest.approx(.4)  # nao modifica scan bruto


def test_mascara_compartilha_montagem_tf_inclusive_temporal():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    spec=importlib.util.spec_from_file_location('launch_lidar_m35',WS/'src/robo_lidar/launch/lidar_filter.launch.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    m.get_package_share_directory=lambda _:str(WS/'src/robo_lidar')
    ctx=LaunchContext();ctx.launch_configurations.update({'lidar_x':'.31','lidar_y':'.02','lidar_yaw':'.1'})
    description=m.generate_launch_description()
    for a in description.entities:
        if isinstance(a,DeclareLaunchArgument):a.execute(ctx)
    filters=[n for n in description.entities if isinstance(n,Node) and n.node_executable=='scan_filter_node']
    assert len(filters)==2
    for n in filters:
        mount=evaluate_parameters(ctx,n._Node__parameters)[-1]
        assert mount=={'body_laser_x':.31,'body_laser_y':.02,'body_laser_yaw':.1}
