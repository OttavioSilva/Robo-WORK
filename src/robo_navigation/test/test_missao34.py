"""Validacoes offline dos patches: copias temporarias, sem ROS/hardware."""
import ast
import importlib.util
import math
import subprocess
import sys
from pathlib import Path
import pytest
import yaml
WS = Path(__file__).resolve().parents[3]
PATCH = WS / 'ferramentas/patches'
NAV = WS / 'src/robo_navigation'

@pytest.mark.parametrize('nome,arquivo', [
    ('cm_protecao', 'config/nav2_params.yaml'),
    ('sem_suavizador', 'behavior_trees/navigate_to_pose_smooth.xml'),
    ('curva_insiste', 'scripts/alinhador_passagem.py'),
])
def test_patch_roundtrip_idempotencia(tmp_path, nome, arquivo):
    p = tmp_path / Path(arquivo).name
    atual = (NAV / arquivo).read_text()
    if nome == 'curva_insiste':
        spec = importlib.util.spec_from_file_location('seguro', PATCH / 'aplicar_curva_insiste_seguro.py')
        mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        for antes, depois in reversed(mod.TROCAS):
            assert atual.count(depois) == 1
            atual = atual.replace(depois, antes)
    p.write_text(atual)
    cmd = [sys.executable, str(PATCH / ('aplicar_' + nome + '.py')), '--arquivo', str(p)]
    subprocess.run(cmd + ['--desfazer'], check=True, capture_output=True)
    baseline = p.read_bytes()
    subprocess.run(cmd, check=True, capture_output=True)
    assert p.read_text() == atual
    subprocess.run(cmd, check=True, capture_output=True)
    assert p.read_text() == atual
    subprocess.run(cmd + ['--desfazer'], check=True, capture_output=True)
    assert p.read_bytes() == baseline


def test_cm_protecao_selecao_e_cobertura():
    v = yaml.safe_load((NAV / 'config/nav2_params.yaml').read_text())['collision_monitor']['ros__parameters']['VelocityPolygonStop']
    def selecionar(speed, angle, theta=0):
        r=math.radians(angle)
        for nome in v['velocity_polygons']:
            p=v[nome]; a,b=p['direction_start_angle'],p['direction_end_angle']
            if p['linear_min'] <= speed <= p['linear_max'] and p['theta_min'] <= theta <= p['theta_max'] and (a <= r <= b if a <= b else r >= a or r <= b):
                return nome
        raise AssertionError((speed,angle,theta))
    for speed in (.005,.035,.05,.051,.12,.120001,.17,.35):
        for angle in range(-180,181):
            for theta in (-.16,0,.16): selecionar(speed,angle,theta)
    assert selecionar(.035,0)=='forward'
    assert selecionar(.17,0)=='forward_veloz'
    assert selecionar(.10,60)=='diag_esq'
    assert selecionar(.10,-60)=='diag_dir'
    assert selecionar(.10,110)=='left'
    assert selecionar(.10,-110)=='right'
    assert max(x for x,y in yaml.safe_load(v['forward_veloz']['points'])) == .545
    for n in ('diag_esq','diag_dir'):
        assert max(x for x,y in yaml.safe_load(v[n]['points'])) == .485


def test_curva_seguranca_condicoes_reais():
    tree=ast.parse((NAV/'scripts/alinhador_passagem.py').read_text())
    loops=[n for n in ast.walk(tree) if isinstance(n,ast.While) and 'k_ins' in ast.unparse(n.test)]
    assert len(loops)==1
    expr=compile(ast.Expression(loops[0].test),'<retry>','eval')
    class Fake:
        def p(self,key): return 200
        def _novo_goal_chegou(self): return self.cancelado
    f=Fake(); f.cancelado=False
    for r in ('falha','ok'):
        assert not eval(expr, {'self':f,'r':r,'k_ins':0})
    assert eval(expr, {'self':f,'r':'repetir','k_ins':0})
    assert not eval(expr, {'self':f,'r':'repetir','k_ins':2})
    f.cancelado=True
    assert not eval(expr, {'self':f,'r':'repetir','k_ins':0})
    fallback=[n for n in ast.walk(tree) if isinstance(n,ast.If) and 'curva_segue_nav2' in ast.unparse(n.test)]
    assert len(fallback)==1
    f.cancelado=False
    def param(k): return 'giro_frontal' if k=='modo_curva_t' else True
    f.p=param
    assert not eval(compile(ast.Expression(fallback[0].test),'<fallback>','eval'),{'self':f,'r':'repetir'})


def test_curva_resultado_nao_rearma_falhas_de_seguranca():
    from types import SimpleNamespace
    tree=ast.parse((NAV/'scripts/alinhador_passagem.py').read_text())
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_percorrer_de_lado')
    result=next(n for n in ast.walk(method) if isinstance(n,ast.Return))
    expr=compile(ast.Expression(result.value),'<resultado>','eval')
    for motivo in ('Collision Monitor bloqueou 4 vezes','sem /odom recente',
                   'TF map->base_link ausente/desatualizada; restante desconhecido'):
        assert eval(expr,{'ok':False,'tl':SimpleNamespace(motivo=motivo)})=='falha'
    assert eval(expr,{'ok':False,'tl':SimpleNamespace(motivo='tempo maximo')})=='repetir'
    assert eval(expr,{'ok':True})=='ok'


def test_patch_seguranca_roundtrip(tmp_path):
    p=tmp_path/'alinhador.py'
    atual=(NAV/'scripts/alinhador_passagem.py').read_bytes(); p.write_bytes(atual)
    cmd=[sys.executable,str(PATCH/'aplicar_curva_insiste_seguro.py'),'--arquivo',str(p)]
    subprocess.run(cmd+['--desfazer'],check=True,capture_output=True)
    assert p.read_bytes()!=atual
    subprocess.run(cmd,check=True,capture_output=True)
    assert p.read_bytes()==atual
    subprocess.run(cmd,check=True,capture_output=True)
    assert p.read_bytes()==atual
