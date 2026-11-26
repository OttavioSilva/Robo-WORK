"""Melhorias isoladas sobre a BASELINE_LEGADO (11/11/2026), cada uma com parametro para voltar ao antes.

PRE_GIRO_UNICO: nao repete o pre-giro para o mesmo alvo em 20 s.
CISCADO_PROGRESSO: da 2a intervencao do ANTI_CISCADO, so reenvia o goal se o destravar resolveu.
ROTA_SEGURA_LAUNCH: launch passa rota_mapa_obstaculo:=recusar.
Os metodos sao os do fonte real (extraidos por AST) rodando com um 'self' falso. Sem ROS, sem robo.
"""
import ast
import math
import time
import types
from pathlib import Path
import sys

import pytest

NAV = Path(__file__).resolve().parents[1]
SRC = NAV / 'scripts/alinhador_passagem.py'
sys.path.insert(0, str(NAV / 'scripts'))
import alinhador_passagem as ap  # noqa: E402


def metodo(nome, extra=None):
    arv = ast.parse(SRC.read_text())
    fn = next(n for n in ast.walk(arv) if isinstance(n, ast.FunctionDef) and n.name == nome)
    ns = dict(time=time, math=math, rumo_do_caminho=ap.rumo_do_caminho, _ang=ap._ang)
    ns.update(extra or {})
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(SRC), 'exec'), ns)
    return ns[nome]


class Log:
    def __init__(self): self.msgs = []
    def info(self, m): self.msgs.append(m)
    warn = warning = error = info


def alvo(x, y):
    return types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y)))


def robo_virado(rep=20.0):
    s = types.SimpleNamespace(pre_giro_memo=None, giros=0, log=Log())
    s.p = lambda n: {'virar_caminho_graus': 45.0, 'pre_giro_repetir_s': rep}[n]
    s._pose_mapa_yaw = lambda: (0.0, 0.0, 0.0)                     # robo virado para +x
    s._caminho = lambda a: [(0.0, -0.1 * k) for k in range(15)]    # caminho saindo a -90 graus
    s.get_logger = lambda: s.log

    def girar(rumo, n, tempo_max, movs_max):
        s.giros += 1
        return True
    s._girar_seguro = girar
    return s


def test_pre_giro_primeiro_envio_igual_a_antes():
    f = metodo('_virar_para_caminho')
    s = robo_virado()
    f(s, alvo(0.0, -1.4), 'GOAL FINAL')
    assert s.giros == 1


def test_pre_giro_nao_repete_para_o_mesmo_alvo():
    f = metodo('_virar_para_caminho')
    s = robo_virado()
    f(s, alvo(0.0, -1.4), 'GOAL FINAL')
    f(s, alvo(0.05, -1.45), 'GOAL FINAL')          # reenvio do mesmo alvo (ANTI_CISCADO/REINSISTIR)
    assert s.giros == 1
    assert any('PRE_GIRO_UNICO' in m for m in s.log.msgs)


def test_pre_giro_alvo_novo_ou_tempo_vencido_gira():
    f = metodo('_virar_para_caminho')
    s = robo_virado()
    f(s, alvo(0.0, -1.4), 'A')
    f(s, alvo(0.0, -0.9), 'B')                      # outro alvo (50 cm)
    assert s.giros == 2
    s.pre_giro_memo = (0.0, -0.9, time.monotonic() - 25.0)
    f(s, alvo(0.0, -0.9), 'B')
    assert s.giros == 3


def test_pre_giro_parametro_zero_volta_ao_antes():
    f = metodo('_virar_para_caminho')
    s = robo_virado(rep=0.0)
    for _ in range(3):
        f(s, alvo(0.0, -1.4), 'GOAL FINAL')
    assert s.giros == 3


def test_pre_giro_caminho_a_frente_nao_gira_nem_memoriza():
    f = metodo('_virar_para_caminho')
    s = robo_virado()
    s._caminho = lambda a: [(0.1 * k, 0.0) for k in range(15)]
    f(s, alvo(1.4, 0.0), 'GOAL FINAL')
    assert s.giros == 0 and s.pre_giro_memo is None


# ---------------------------------------------------------------- CISCADO_PROGRESSO
GoalStatus = types.SimpleNamespace(STATUS_SUCCEEDED=4)


def destravar_falso(servidor=True, aceito=True, status=4):
    s = types.SimpleNamespace(log=Log())
    s.get_logger = lambda: s.log
    h = types.SimpleNamespace(accepted=aceito, get_result_async=lambda: ('res', status))
    s.destravar_cli = types.SimpleNamespace(wait_for_server=lambda timeout_sec: servidor,
                                            send_goal_async=lambda g: ('goal', h))
    s._esperar = lambda fut, t: fut[1] if fut[0] == 'goal' else types.SimpleNamespace(status=fut[1])
    # ENVIO_GOAL_SEGURO (11/11): o envio passa por _enviar_goal (testado com ROS em test_substituicao_goal_ros.py)
    s._enviar_goal = lambda cli, req, t: s._esperar(cli.send_goal_async(req), t)
    return s


class DriveOnHeadingFalso:
    class Goal:
        def __init__(self):
            self.target = types.SimpleNamespace(x=0.0)
            self.speed = 0.0
            self.time_allowance = types.SimpleNamespace(sec=0)


@pytest.mark.parametrize('kw,esperado', [({}, True), (dict(status=6), False), (dict(aceito=False), False),
                                         (dict(servidor=False), False)])
def test_chamar_destravar_devolve_se_resolveu(kw, esperado):
    f = metodo('_chamar_destravar', dict(DriveOnHeading=DriveOnHeadingFalso, GoalStatus=GoalStatus))
    assert f(destravar_falso(**kw), 'ANTI_CISCADO 1') is esperado


def test_anti_ciscado_so_desiste_da_2a_em_diante_e_com_parametro():
    fonte = ast.get_source_segment(SRC.read_text(), next(
        n for n in ast.walk(ast.parse(SRC.read_text())) if isinstance(n, ast.FunctionDef) and n.name == '_navegar_legado'))
    assert "destravou = self._chamar_destravar('ANTI_CISCADO %d' % (_cisc + 1))" in fonte
    assert "if not destravou and _cisc >= 1 and self.p('ciscado_exige_destravar'):" in fonte
    i = fonte.index('if not destravou'); j = fonte.index("return 'falha'", i)
    k = fonte.index('return self._navegar(alvo, nome, vigia, aceitar_perto, virar, _cisc + 1)', i)
    assert j < k                      # desiste antes de reenviar
    assert "self.declare_parameter('ciscado_exige_destravar', True)" in SRC.read_text()


# ---------------------------------------------------------------- ROTA_SEGURA_LAUNCH
def test_launch_rota_segura():
    src = (NAV / 'launch/navigation.launch.py').read_text()
    assert '"rota_mapa_obstaculo:=recusar"' in src
    # o padrao do codigo nao mudou (patch ROTA_MAPA_OBSTACULO intacto)
    assert "self.declare_parameter('rota_mapa_obstaculo', 'ignorar')" in SRC.read_text()


def test_obstaculo_novo_na_rota_detecta_objeto_perto():
    # mapa vazio 4x4 m; objeto do momento a 20 cm da rota -> detectado (base do modo recusar)
    m = ap.Mapa(0.05, 80, 80, -2.0, -2.0, 0.0, [0] * 6400)
    rota = [(-1.0 + 0.05 * k, 0.0) for k in range(41)]

    def obs_em(x, y):
        d = [0] * 6400
        d[int((y + 2.0) / 0.05) * 80 + int((x + 2.0) / 0.05)] = 100
        return ap.Mapa(0.05, 80, 80, -2.0, -2.0, 0.0, d)
    assert ap.obstaculo_novo_na_rota(m, obs_em(0.0, 0.20), rota)
    assert not ap.obstaculo_novo_na_rota(m, obs_em(0.0, 0.80), rota)
