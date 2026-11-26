"""Partida fail-closed; resolve arquivos em memoria sem executar processos."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess
from launch_ros.actions import Node
from test_nav2_principal import metodo
NAV=Path(__file__).resolve().parents[1]


def load(share):
    spec=importlib.util.spec_from_file_location('partida_nav',NAV/'launch/navigation.launch.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    m.get_package_share_directory=lambda _:str(share)
    return m


def context(m):
    d=m.generate_launch_description();ctx=LaunchContext();ctx.launch_configurations['nav2_principal']='true'
    for a in d.entities:
        if isinstance(a,DeclareLaunchArgument):a.execute(ctx)
    return ctx,d


def test_instalacao_antiga_sem_perfil_recusa_antes_de_subir_no(tmp_path):
    m=load(tmp_path);ctx,d=context(m)
    with pytest.raises(RuntimeError,match='params_file ausente'):m.validar_arquivos(ctx)
    validator=next(i for i,a in enumerate(d.entities) if isinstance(a,OpaqueFunction))
    assert all(not isinstance(a,(Node,ExecuteProcess)) for a in d.entities[:validator])


def test_perfil_e_arvore_existentes_aprovam_sem_iniciar_nos():
    m=load(NAV);ctx,d=context(m)
    assert m.validar_arquivos(ctx)==[]


@pytest.mark.parametrize('argumento',['params_file','bt_xml'])
def test_override_inexistente_nao_continua_com_defaults(tmp_path,argumento):
    m=load(NAV);ctx,d=context(m);ctx.launch_configurations[argumento]=str(tmp_path/'inexistente')
    with pytest.raises(RuntimeError,match=argumento+' ausente'):m.validar_arquivos(ctx)


def test_nav2_indisponivel_exibe_motivo_em_vez_de_silencio():
    logs=[]
    f=NS(_autoridade_mov=lambda d:None,nav=NS(wait_for_server=lambda **k:False),
         _log_nav_principal=lambda *a:logs.append(a))
    assert metodo('_navegar_principal')(f,object(),'goal')=='falha'
    assert logs and logs[0][0]=='error' and 'indisponivel' in logs[0][1]
