"""CONTROLADOR_ORFAO (11/11/2026) com ROS real em dominio isolado: FollowPath falso que comanda /cmd_vel_ctrl
enquanto tem goal ativo. Confere o mecanismo usado pelo destravar e pelo alinhador: CancelGoal com goal_info vazio
em /follow_path/_action/cancel_goal cancela o goal orfao e o controlador para. E a logica do alinhador
(_verificar_controlador_orfao) extraida do fonte e rodada com um 'self' falso: so cancela quando o controlador
continua comandando depois do fim do NavigateToPose. Sem robo, sem Nav2.
"""
import ast
import os
import random
import threading
import time
import types
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
from geometry_msgs.msg import TwistStamped  # noqa: E402
from nav2_msgs.action import FollowPath  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402

SRC = Path(__file__).resolve().parents[1] / 'scripts/alinhador_passagem.py'


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class ControladorFalso(Node):
    def __init__(self):
        super().__init__('controller_server_falso')
        g = ReentrantCallbackGroup()
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', 10)
        self.ativos = 0
        ActionServer(self, FollowPath, 'follow_path', self.executar, callback_group=g,
                     goal_callback=lambda _: GoalResponse.ACCEPT, cancel_callback=lambda _: CancelResponse.ACCEPT)

    def executar(self, gh):
        self.ativos += 1
        try:
            while not gh.is_cancel_requested:
                m = TwistStamped(); m.header.stamp = self.get_clock().now().to_msg(); m.twist.linear.x = -0.10
                self.pub.publish(m)
                time.sleep(0.1)
            gh.canceled()
            return FollowPath.Result()
        finally:
            self.ativos -= 1


def test_cancel_goal_vazio_cancela_follow_path_orfao(ros):
    srv = ControladorFalso()
    cli = Node('cliente_teste')
    ex = MultiThreadedExecutor(4); ex.add_node(srv); ex.add_node(cli)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()
    try:
        ac = ActionClient(cli, FollowPath, 'follow_path')
        assert ac.wait_for_server(timeout_sec=5)
        ultimo = {'t': None}
        cli.create_subscription(TwistStamped, '/cmd_vel_ctrl', lambda m: ultimo.__setitem__('t', time.monotonic()), 10)
        f = ac.send_goal_async(FollowPath.Goal())
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < 5:
            time.sleep(0.02)
        assert f.result().accepted
        time.sleep(0.5)
        assert srv.ativos == 1 and ultimo['t'] is not None and time.monotonic() - ultimo['t'] < 0.3
        # o goal fica "orfao": o cliente nao usa o handle; cancela pelo servico com goal_info vazio
        cc = cli.create_client(CancelGoal, '/follow_path/_action/cancel_goal')
        assert cc.wait_for_service(timeout_sec=3)
        r = cc.call_async(CancelGoal.Request())
        t0 = time.monotonic()
        while not r.done() and time.monotonic() - t0 < 3:
            time.sleep(0.02)
        assert r.done() and len(r.result().goals_canceling) == 1
        time.sleep(0.6)
        assert srv.ativos == 0
        assert time.monotonic() - ultimo['t'] >= 0.3
    finally:
        ex.shutdown(); srv.destroy_node(); cli.destroy_node()


# ---------------------------------------------------------------- logica do alinhador (fonte real)
def _metodo():
    arv = ast.parse(SRC.read_text())
    fn = next(n for n in ast.walk(arv) if isinstance(n, ast.FunctionDef) and n.name == '_verificar_controlador_orfao')
    mod = ast.Module(body=[fn], type_ignores=[])
    ns = {'time': time}
    exec(compile(mod, str(SRC), 'exec'), ns)
    return ns['_verificar_controlador_orfao']


class Log:
    def __init__(self): self.msgs = []
    def warn(self, m): self.msgs.append(('warn', m))
    def info(self, m): self.msgs.append(('info', m))
    def error(self, m): self.msgs.append(('error', m))


def falso(ctrl_ativo_ate, principal=False, servico=True):
    chamadas = []

    class Futuro:
        def done(self): return True

    class Cli:
        def wait_for_service(self, timeout_sec): return servico
        def call_async(self, req):
            chamadas.append(req)
            return Futuro()
    s = types.SimpleNamespace(lock=threading.Lock(), ctrl_nz_t=None, cli_cancel_fp=Cli(), log=Log())
    s.p = lambda n: {'nav2_principal': principal, 'cancelar_controlador_orfao': True}[n]
    s.get_logger = lambda: s.log

    def bater():   # controlador publicando a 10 Hz ate ctrl_ativo_ate() ficar falso
        while not s.parar:
            if ctrl_ativo_ate(s, chamadas):
                with s.lock:
                    s.ctrl_nz_t = time.monotonic()
            time.sleep(0.1)
    s.parar = False
    th = threading.Thread(target=bater, daemon=True); th.start()
    return s, chamadas


@pytest.mark.parametrize('principal', [False])
def test_alinhador_cancela_orfao_e_confirma(principal):
    fn = _metodo()
    s, ch = falso(lambda s, ch: not ch, principal)   # comanda ate alguem cancelar
    time.sleep(0.15)
    try:
        assert fn(s, 'GOAL FINAL') == 'cancelado'
        assert len(ch) == 1 and isinstance(ch[0], CancelGoal.Request)
        assert any('CONTROLADOR_ORFAO' in m for _, m in s.log.msgs)
    finally:
        s.parar = True


def test_alinhador_nao_cancela_quando_o_controlador_parou_junto():
    fn = _metodo()
    t_fim = time.monotonic() + 0.05
    s, ch = falso(lambda s, ch: time.monotonic() < t_fim)
    try:
        assert fn(s, 'GOAL FINAL') == 'ok' and ch == []
    finally:
        s.parar = True


def test_alinhador_avisa_quando_o_orfao_persiste():
    fn = _metodo()
    s, ch = falso(lambda s, ch: True)
    try:
        assert fn(s, 'GOAL FINAL', calar=0.5) == 'persistente'
        assert len(ch) == 1 and any(n == 'error' for n, _ in s.log.msgs)
    finally:
        s.parar = True


def test_perfil_novo_nao_mexe():
    fn = _metodo()
    s, ch = falso(lambda s, ch: True, principal=True)
    try:
        t0 = time.monotonic()
        assert fn(s, 'X') == 'ok' and ch == [] and time.monotonic() - t0 < 0.1
    finally:
        s.parar = True
