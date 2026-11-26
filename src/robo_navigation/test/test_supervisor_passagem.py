import math
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/robo_navigation/scripts'))
sys.path.insert(0,str(ROOT/'ferramentas'))
from supervisor_passagem import Supervisor, Evidencia, Passagens, saida_livre
from controle_nav_principal import passagem_confirmada
from preparar_localization import perfil, validar
FOOT=[(-.215,-.325),(.375,-.325),(.375,.325),(-.215,.325)]


def medida(**kw):
    return SimpleNamespace(**dict(dict(ok=True,dx=.1,dy=0,dth=0,residuo=.002,ambiguidade=0),**kw))


def evidencia():
    e=Evidencia((0,0,0),0,0)
    for i in range(1,7):
        e.atualizar((i*.1,0,0),0,i*.3,medida())
    return e


def test_caminho_livre():
    assert not passagem_confirmada(None,(0,0),None)


def test_caixa():
    assert not passagem_confirmada({'entrada':(.5,0),'largura_mapa':.8},(0,0),{'largura':.8,'lados':1})


def test_corredor():
    assert passagem_confirmada({'entrada':(.5,0),'largura_mapa':.8},(0,0),{'largura':.81,'lados':2})


def test_saida_incompleta():
    e=evidencia()
    assert not e.completa((.5,0),(1,0),FOOT,True,True)
    assert e.completa((.1,0),(1,0),FOOT,True,True)
    assert not e.completa((.1,0),(1,0),FOOT,True,False)
    assert not e.completa((.1,0),(1,0),FOOT,False,True)


def test_T_footprint_rotacionado():
    e=evidencia();e.pose=(0,.7,0);e.mapa=e.pose
    assert e.completa((0,.1),(0,1),FOOT,True,True)
    assert not e.completa((0,.4),(0,1),FOOT,True,True)


@pytest.mark.parametrize('kw,pose,imu', [({'ok':False},(.1,0,0),0), ({},(.5,0,0),0), ({},(.1,0,.2),0), ({'dth':.2},(.1,0,.2),0), ({'dy':.3},(.1,.3,0),0), ({'dx':float('nan')},(.1,0,0),0)])
def test_falha_icp_salto_slam_imu_deriva(kw,pose,imu):
    e=Evidencia((0,0,0),0,0)
    with pytest.raises(RuntimeError):e.atualizar(pose,imu,.3,medida(**kw))


def test_timeout_sensor():
    with pytest.raises(RuntimeError):Evidencia((0,0,0),0,0).atualizar((.1,0,0),0,1,medida())


def test_preempcao_retomada_goal():
    goal={'pose':{'x':1,'y':2,'quaternion':[0,0,.2,.98]}}
    s=Supervisor(goal,lambda m:None,identificador='original')
    goal['pose']['x']=9
    s.mudar('RETOMADA','teste')
    assert s.goal['pose']['x']==1 and s.id=='original'
    novo=Supervisor(goal,lambda m:None)
    assert novo.id != s.id and novo.goal['pose']['x']==9


def test_latch_entre_goals():
    p=Passagens();p.concluir((0,0),(1,0))
    assert p.bloqueadas((2,0))==[(0,0)]
    assert p.bloqueadas((-.8,2))==[(0,0)]
    assert p.bloqueadas((-.8,0))==[]


def test_livre_exige_observacao():
    assert not saida_livre([], (1,0))
    pts=[(1,.6)]*5+[(1,-.6)]*5
    assert saida_livre(pts,(1,0))
    assert not saida_livre(pts+[(.2,0)],(1,0))


def test_perfil_somente_modo_e_inicializacao():
    cfg,_=perfil('/home/marcos/mapas/arena_04-11_1235')
    mapping=yaml.safe_load((ROOT/'src/robo_navigation/config/slam_mapping.yaml').read_text())
    p=cfg['slam_toolbox']['ros__parameters']
    assert p.pop('mode')=='localization';p.pop('map_file_name');p.pop('map_start_pose')
    mapping['slam_toolbox']['ros__parameters'].pop('mode')
    assert cfg==mapping


def test_mapa_ausente(tmp_path):
    with pytest.raises(FileNotFoundError):validar(tmp_path/'ausente')


def test_regioes_caixa_nao_cadastrada():
    from supervisor_passagem import regiao_conhecida
    r=[dict(entrada=(0,0),saida=(1,0),largura=.8)]
    assert regiao_conhecida(dict(entrada=(0,0),saida=(1,0),largura_mapa=.8),r)
    assert not regiao_conhecida(dict(entrada=(2,0),saida=(3,0),largura_mapa=.8),r)
    assert not regiao_conhecida(dict(entrada=(0,0),saida=(1,0),largura_mapa=.8),[])


@pytest.mark.parametrize('zerado,fresco,evidencia,gyro,ok',[(True,True,False,0,True),(False,True,False,0,False),(True,False,False,0,False),(True,True,True,.1,False),(True,True,True,0,True)])
def test_metodo_real_espera_parada(zerado,fresco,evidencia,gyro,ok):
    from test_nav2_principal import metodo
    import threading
    t=[0]
    def agora():t[0]+=.02;return t[0]
    class No:
        lock=threading.Lock()
        dono_mov='especial'
        evidencia_passagem=object() if evidencia else None
        gyro_z=gyro
        def _novo_goal_chegou(self):return False
        def _conferir_evidencia_passagem(self):
            if self.icp_parado_desde is None:self.icp_parado_desde=t[0]
        @property
        def cmd_final(self):return (t[0] if fresco else -9,zerado)
    fn=metodo('_esperar_parada_supervisor')
    fn.__globals__['time']=SimpleNamespace(monotonic=agora,sleep=lambda _:None)
    n=No()
    if ok:
        fn(n);assert t[0]>=.4 and n.dono_mov is None
    else:
        with pytest.raises(RuntimeError,match='parada'):fn(n)
