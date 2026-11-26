"""SUBSTITUICAO_GOAL (11/11/2026): substituicao de goal no alinhador LEGADO, com ROS real em dominio isolado.

Roda o codigo real do alinhador (_navegar_legado, _esperar, _novo_goal_chegou, _cancelar_nav, extraidos do fonte)
contra um NavigateToPose falso que conta os goals ativos. O goal novo chega (como em ao_goal / cancelar_t) em
momentos diferentes. A regra e sempre a mesma: quando o trabalhador sai do goal velho, o goal velho nao pode
continuar ativo no Nav2, nem se a aceitacao chegar depois. Sem robo e sem Nav2.
"""
import ast
import math
import os
import random
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
# DOMINIO_FORCADO (11/11): o ~/.bashrc exporta ROS_DOMAIN_ID=0 (o do robo); setdefault nao trocava e os servidores
# falsos (NavigateToPose/FollowPath, /cmd_vel_ctrl) entravam no grafo do robo ligado. Sempre isolado e so localhost.
os.environ['ROS_DOMAIN_ID'] = str(150 + random.randint(0, 50))
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
assert os.environ['ROS_DOMAIN_ID'] not in ('', '0'), 'teste ROS nunca roda no dominio do robo'

from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402

SRC = Path(__file__).resolve().parents[1] / 'scripts/alinhador_passagem.py'
METODOS = ('_esperar', '_novo_goal_chegou', '_cancelar_nav', '_enviar_goal', '_navegar_legado')


class Cancelado(Exception):
    pass


def _extrair():
    arv = ast.parse(SRC.read_text())
    fns = {n.name: n for n in ast.walk(arv) if isinstance(n, ast.FunctionDef) and n.name in METODOS}
    assert set(fns) == set(METODOS), 'metodos nao encontrados no fonte: %s' % (set(METODOS) - set(fns))
    ns = {'time': time, 'math': math, 'threading': threading, 'Cancelado': Cancelado,
          'NavigateToPose': NavigateToPose, 'GoalStatus': GoalStatus,
          'yaw_de': lambda q: 2.0 * math.atan2(q.z, q.w),
          'ciscando': lambda *a: (False, 0, 0.0, 0.0)}
    exec(compile(ast.Module(body=list(fns.values()), type_ignores=[]), str(SRC), 'exec'), ns)
    return {k: ns[k] for k in METODOS}


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Nav2Falso(Node):
    """NavigateToPose falso: aceita depois de 'atraso' s, fica ativo ate cancelarem, conta os ativos."""

    def __init__(self):
        super().__init__('bt_navigator_falso')
        self.atraso = 0.0
        self.encerrar = False   # desmontagem: goal orfao nao pode prender o executor
        self.ativos = {}      # id do goal (x) -> True enquanto executa
        self.max_simult = 0
        self.lk = threading.Lock()
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self.executar,
                     callback_group=ReentrantCallbackGroup(),
                     goal_callback=self.aceitar, cancel_callback=lambda _: CancelResponse.ACCEPT)

    def aceitar(self, req):
        time.sleep(self.atraso)
        return GoalResponse.ACCEPT

    def executar(self, gh):
        x = round(gh.request.pose.pose.position.x, 2)
        with self.lk:
            self.ativos[x] = True
            self.max_simult = max(self.max_simult, len(self.ativos))
        try:
            while not gh.is_cancel_requested and not self.encerrar:
                time.sleep(0.05)
            if gh.is_cancel_requested:
                gh.canceled()
            else:
                gh.abort()
            return NavigateToPose.Result()
        finally:
            with self.lk:
                self.ativos.pop(x, None)

    def ativos_agora(self):
        with self.lk:
            return sorted(self.ativos)


class Log:
    def __init__(self):
        self.msgs = []

    def info(self, m): self.msgs.append(('info', m))
    def warn(self, m): self.msgs.append(('warn', m))
    warning = warn
    def error(self, m): self.msgs.append(('error', m))


class AlinhadorFalso:
    """So o necessario para o _navegar_legado real: o resto (pre-giro, sensores, ciscado) fica desligado."""

    PARAMS = {'nav2_principal': False, 'ciscado_janela': 8.0, 'periodo_vigia': 0.5, 'anti_ciscado': False,
              'ciscado_max': 0, 'ciscado_inversoes': 4, 'raio_chegada': 0.1, 'tempo_chegada': 1.0,
              'ciscado_exige_destravar': True}

    def __init__(self, no):
        fns = _extrair()
        for k, f in fns.items():
            setattr(self, k, f.__get__(self))
        self.nav = ActionClient(no, NavigateToPose, 'navigate_to_pose')
        self.lock = threading.Lock()
        self.cancelamento_t = threading.Event()
        self.goal_pendente = None
        self.handle_ativo = None
        self.hist_cmd, self.hist_odom = [], []
        self.log = Log()

    def get_logger(self): return self.log
    def p(self, nome): return self.PARAMS[nome]
    def _virar_para_caminho(self, alvo, nome): pass
    def _esperar_sensores(self, nome): pass
    def _sensor_parado(self, s): return False
    def _parar(self): pass
    def _pose_mapa(self): return None
    def _verificar_controlador_orfao(self, nome): return 'ok'

    def _navegar(self, alvo, nome, vigia=None, aceitar_perto=False, virar=True, _cisc=0):
        return self._navegar_legado(alvo, nome, vigia, aceitar_perto, virar, _cisc)

    # o que ao_goal e cancelar_t fazem no no real
    def chega_goal_novo(self, msg='novo'):
        with self.lock:
            self.goal_pendente = msg
        self._cancelar_nav()

    def cancelar_t(self):
        self.cancelamento_t.set()
        self._cancelar_nav()

    def proximo_goal(self):
        with self.lock:
            self.goal_pendente = None
        self.cancelamento_t.clear()


def _alvo(x):
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.pose.position.x = float(x)
    ps.pose.orientation.w = 1.0
    return ps


@pytest.fixture
def ambiente(ros):
    srv = Nav2Falso()
    cli = Node('alinhador_teste_subst')
    ex = MultiThreadedExecutor(6)
    ex.add_node(srv)
    ex.add_node(cli)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    al = AlinhadorFalso(cli)
    assert al.nav.wait_for_server(timeout_sec=5)
    yield srv, al
    srv.encerrar = True
    time.sleep(0.2)
    ex.shutdown()
    srv.destroy_node()
    cli.destroy_node()


def _navegar_em_thread(al, alvo):
    out = {}

    def corre():
        try:
            out['r'] = al._navegar_legado(alvo, 'TESTE', virar=False)
        except Cancelado:
            out['r'] = 'Cancelado'
        except Exception as exc:  # pragma: no cover
            out['r'] = 'erro: %r' % exc
    th = threading.Thread(target=corre, daemon=True)
    th.start()
    return th, out


def _esperar_ate(cond, tmax):
    t0 = time.monotonic()
    while time.monotonic() - t0 < tmax:
        if cond():
            return True
        time.sleep(0.05)
    return cond()


def test_substituicao_durante_execucao_cancela_o_velho(ambiente):
    srv, al = ambiente
    th, out = _navegar_em_thread(al, _alvo(1.0))
    assert _esperar_ate(lambda: srv.ativos_agora() == [1.0], 3)
    al.chega_goal_novo()
    th.join(3)
    assert out.get('r') == 'Cancelado'
    assert _esperar_ate(lambda: srv.ativos_agora() == [], 2), srv.ativos_agora()


def test_substituicao_antes_da_aceitacao_cancela_o_aceito_depois(ambiente):
    """Goal novo enquanto o Nav2 ainda nao aceitou o velho: handle_ativo e None, ninguem cancelava o velho."""
    srv, al = ambiente
    srv.atraso = 1.0
    th, out = _navegar_em_thread(al, _alvo(2.0))
    time.sleep(0.3)
    al.chega_goal_novo()
    th.join(3)
    assert out.get('r') == 'Cancelado'
    time.sleep(1.2)   # a aceitacao atrasada chega aqui
    assert _esperar_ate(lambda: srv.ativos_agora() == [], 2), \
        'goal velho ficou ativo no Nav2 depois da substituicao: %s' % srv.ativos_agora()


def test_aceitacao_depois_do_timeout_nao_fica_orfa(ambiente):
    """O Nav2 demora mais que 5 s para aceitar: o alinhador devolve 'falha', mas o goal aceito depois nao pode rodar."""
    srv, al = ambiente
    srv.atraso = 5.6
    th, out = _navegar_em_thread(al, _alvo(3.0))
    th.join(8)
    assert out.get('r') == 'falha'
    time.sleep(1.0)
    assert _esperar_ate(lambda: srv.ativos_agora() == [], 2), \
        'goal aceito depois do timeout ficou ativo: %s' % srv.ativos_agora()


def test_cancelar_t_para_o_goal(ambiente):
    srv, al = ambiente
    th, out = _navegar_em_thread(al, _alvo(4.0))
    assert _esperar_ate(lambda: srv.ativos_agora() == [4.0], 3)
    al.cancelar_t()
    th.join(3)
    assert out.get('r') == 'Cancelado'
    assert _esperar_ate(lambda: srv.ativos_agora() == [], 2)


def test_sequencia_de_substituicoes_nunca_tem_dois_goals(ambiente):
    """Tres goals seguidos, um deles substituido antes de ser aceito. No fim so o ultimo esta ativo e nunca
    houve dois goals do alinhador rodando juntos por mais que o instante do cancelamento."""
    srv, al = ambiente
    srv.atraso = 0.6
    th, out = _navegar_em_thread(al, _alvo(5.0))
    time.sleep(0.2)
    al.chega_goal_novo()          # substitui antes da aceitacao
    th.join(3)
    assert out.get('r') == 'Cancelado'
    al.proximo_goal()
    srv.atraso = 0.0
    th, out = _navegar_em_thread(al, _alvo(6.0))
    assert _esperar_ate(lambda: 6.0 in srv.ativos_agora(), 3)
    al.chega_goal_novo()          # substitui durante a execucao
    th.join(3)
    assert out.get('r') == 'Cancelado'
    al.proximo_goal()
    th, out = _navegar_em_thread(al, _alvo(7.0))
    time.sleep(1.2)
    assert srv.ativos_agora() == [7.0], srv.ativos_agora()
    al.cancelar_t()
    th.join(3)
    assert _esperar_ate(lambda: srv.ativos_agora() == [], 2)
