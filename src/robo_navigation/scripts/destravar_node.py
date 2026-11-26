#!/usr/bin/env python3
"""Recuperacao "destravar" do Nav2 (DESTRAVAR v4 / RECUPERACAO_AVANCADA / NAV_INTEGRADA, 10/11/2026).

v4: a DECISAO sai de recuperacao_avancada.py (mesmo diretorio, sem ROS, testado em simulacao):
  diagnostico (caminho invalido / DWB sem trajetoria / CM / sem progresso fisico / oscilacao / quina) ->
  contorno normal (devolve ao Nav2 sem mexer) -> CONTORNAR -> lateral com continuacao -> andar no caminho ->
  giro pelo smart_spin -> re (BLOQUEADA por padrao: permitir_re) -> falha explicita.
  Cada manobra: para, foto nova do LiDAR, confirma o deslocamento pelo ICP (giro: IMU) e registra no
  historico; o que falhou ou nao resolveu no mesmo lugar nao se repete. Depois de uma re o no SEMPRE
  reavalia com foto nova antes de outra manobra. Comando estrangeiro em /cmd_vel_nav, controlador ativo,
  CM inativo ou sensor parado: para e nao autoriza manobra.
  Parametros (ros2 param set /destravar ...): permitir_lateral (true), permitir_giro (true),
  permitir_re (false), cobertura_lateral/giro/re (0.5/0.5/0.9), deriva_lateral (0.4).
  O servidor continua o mesmo (nav2_msgs/DriveOnHeading "destravar"): a arvore e o ANTI_CISCADO do
  alinhador chamam do mesmo jeito.

v3 (missao18): girar para o caminho / andar na direcao dele / re ate o giro ficar livre / de lado.
  No v4 o "re_girar" (re seguida de giro sem nova avaliacao) deixou de existir.
BLOQUEIO_LOCAL e CONTORNAR (05/11): mantidos (bloqueio virtual 10 s so no local; CONTORNAR agora
  confere tambem a deriva para a frente do deslize e o poligono do CM).
v2 (missao17): afastar do que encosta; BLOQUEIO VIRTUAL.  v1: servidor DriveOnHeading "destravar".

Uso:
    python3 destravar_node.py      # o NAV_TUDO sobe junto com o Nav2
"""
import math
import threading
import time

FOOTPRINT = (-0.205, 0.365, 0.315)   # x traseira, x frente, meia largura (base_link)
LASER = (0.33, 0.005, 0.0)           # laser em base_link (fallback se nao houver TF)
FOLGA_MOV = 0.07                     # o Collision Monitor para a 6 cm do lado para onde anda
MAX_TRAS = 0.10
DIRECOES = {'frente': (1.0, 0.0), 'esq': (0.0, 1.0), 'dir': (0.0, -1.0), 'tras': (-1.0, 0.0)}


def folga(pts, direcao, m=0.02):
    """Distancia do corpo ate o ponto mais perto andando reto nessa direcao (m); 9 = nada visto."""
    x0, x1, y1 = FOOTPRINT
    if direcao == 'frente':
        d = [x - x1 for x, y in pts if x > x1 and abs(y) <= y1 + m]
    elif direcao == 'tras':
        d = [x0 - x for x, y in pts if x < x0 and abs(y) <= y1 + m]
    elif direcao == 'esq':
        d = [y - y1 for x, y in pts if y > y1 and x0 - m <= x <= x1 + m]
    else:
        d = [-y1 - y for x, y in pts if y < -y1 and x0 - m <= x <= x1 + m]
    return float(min(d)) if d else 9.0


OPOSTA = {'frente': 'tras', 'tras': 'frente', 'esq': 'dir', 'dir': 'esq'}


def escolher(pts, dist):
    """-> (direcao ou None, quanto andar, {direcao: folga}). Sem ROS (testavel)."""
    folgas = {d: folga(pts, d) for d in DIRECOES}
    # v2: encostado num lado: primeiro se afasta para o lado oposto (o CM barra deslizar encostado)
    perto = min(folgas, key=lambda d: folgas[d])
    if folgas[perto] < 0.05:
        op = OPOSTA[perto]
        u = min(dist, folgas[op] - FOLGA_MOV, 0.12)
        if op == 'tras':
            u = min(u, MAX_TRAS) if folgas[op] < 9.0 else min(u, 0.05)
        if u >= 0.04:
            return op, u, folgas
    util = {}
    for d, f in folgas.items():
        u = min(dist, f - FOLGA_MOV)
        if d == 'tras':
            # traseira quase cega: so anda o que enxerga, e no maximo 10 cm
            u = min(u, MAX_TRAS) if f < 9.0 else min(dist, 0.05)
        util[d] = u
    # mais espaco primeiro; empate (2 cm): lados, depois frente, depois tras
    ordem = {'esq': 0, 'dir': 0, 'frente': 1, 'tras': 2}
    melhor = max(util, key=lambda d: (round(min(folgas[d], 1.0) / 0.02), -ordem[d]))
    if util[melhor] < 0.03:
        # a de mais espaco nao deixa andar 3 cm: tenta a que deixa andar mais
        melhor = max(util, key=lambda d: util[d])
    if util[melhor] < 0.03:
        return None, 0.0, folgas
    return melhor, util[melhor], folgas



# ---------------- v3: geometria do giro e do deslocamento ----------------

def _dentro(x, y, m):
    x0, x1, y1 = FOOTPRINT
    return x0 - m <= x <= x1 + m and abs(y) <= y1 + m

def fora_do_corpo(pts, alcance=1.2):
    x0, x1, y1 = FOOTPRINT
    return [(x, y) for x, y in pts if abs(x) < alcance and abs(y) < alcance and not (x0 <= x <= x1 and abs(y) <= y1)]

def giro_livre(pts, ang, margem=0.045, min_pts=2):
    """O retangulo do corpo + margem gira 'ang' rad no lugar sem pegar pontos?"""
    p = fora_do_corpo(pts)
    n = max(2, int(abs(ang) / math.radians(4)) + 1)
    for k in range(1, n + 1):
        t = ang * k / n
        c, s = math.cos(t), math.sin(t)
        if sum(1 for x, y in p if _dentro(c * x + s * y, -s * x + c * y, margem)) >= min_pts:
            return False
    return True

def translacao_livre(pts, ux, uy, d, margem=0.03, min_pts=2):
    """O corpo anda d m na direcao (ux, uy) do base_link sem pegar pontos?"""
    p = fora_do_corpo(pts)
    s = 0.03
    while s <= d + 1e-6:
        if sum(1 for x, y in p if _dentro(x - s * ux, y - s * uy, margem)) >= min_pts:
            return False
        s += 0.03
    return True

def visivel_tras(pts, d):
    """Ha pontos vistos atras do robo alem de d (a traseira nao e totalmente cega)?"""
    x0, _, y1 = FOOTPRINT
    return any(x < x0 - d and abs(y) <= y1 + 0.1 for x, y in pts)

def plano_destravar(pts, ang_plano, dist=0.25, re_max=0.30):
    """Decide: ('girar', ang) | ('mover', ux, uy, d, motivo) | ('re_girar', d, ang) | None.
    ang_plano: direcao do caminho (rad, base_link), ou None."""
    if ang_plano is not None:
        a = math.atan2(math.sin(ang_plano), math.cos(ang_plano))
        # 1) girar para o caminho (o mais curto; se travar, o outro sentido)
        if abs(a) > math.radians(20):
            for ang in (a, a - math.copysign(2 * math.pi, a)):
                if abs(ang) <= math.radians(200) and giro_livre(pts, ang):
                    return ('girar', ang)
        # 2) andar direto na direcao do caminho (mecanum anda em qualquer direcao)
        ux, uy = math.cos(a), math.sin(a)
        for d in (dist, 0.15, 0.10):
            if (ux >= -0.3 or visivel_tras(pts, d)) and translacao_livre(pts, ux, uy, d):
                return ('mover', ux, uy, d, 'na direcao do caminho')
        # 3) re ate o giro ficar livre
        if abs(a) > math.radians(20):
            for dr in (0.05, 0.10, 0.15, 0.20, 0.25, re_max):
                if not translacao_livre(pts, -1.0, 0.0, dr):
                    break
                movidos = [(x + dr, y) for x, y in pts]
                for ang in (a, a - math.copysign(2 * math.pi, a)):
                    if abs(ang) <= math.radians(200) and giro_livre(movidos, ang):
                        return ('re_girar', dr, ang)
        # 4) de lado: o lado do caminho primeiro, depois o outro
        for uy2 in ((1.0, -1.0) if a >= 0 else (-1.0, 1.0)):
            for d in (dist, 0.15, 0.10):
                if translacao_livre(pts, 0.0, uy2, d):
                    return ('mover', 0.0, uy2, d, 'de lado')
    return None


# ---------------- CONTORNAR (05/11): ver o fim do obstaculo e passar por ele ----------------
FAIXA_FRENTE = 0.15      # o que esta ate aqui alem do para-choque barra o caminho
MARGEM_LADO = 0.05       # folga lateral para dar o obstaculo por vencido
OBJETO_MAX = 0.25        # maior que isto e parede ou mesa, nao obstaculo
DESLIZE_MAX = 0.30
AVANCO = 0.25
DESTINO_MIN = 0.60       # perto do destino quem barra costuma ser a mesa


def barrando(pts, meia=None):
    """Pontos na faixa a frente do corpo."""
    _, x1, y1 = FOOTPRINT
    lim = y1 + 0.03 if meia is None else meia
    return [(x, y) for x, y in pts if x1 - 0.02 <= x <= x1 + FAIXA_FRENTE and abs(y) <= lim]


def aglomerado(pts, sementes, gap=0.10):
    """Pontos ligados as sementes por saltos de ate gap: o objeto inteiro que o LiDAR ve."""
    grupo, novos, resto = set(sementes), list(sementes), [p for p in pts if p not in set(sementes)]
    while novos:
        ax, ay = novos.pop()
        perto = [p for p in resto if math.hypot(p[0] - ax, p[1] - ay) <= gap]
        for p in perto:
            grupo.add(p)
            resto.remove(p)
        novos.extend(perto)
    return grupo


def plano_contornar(pts, ang_plano):
    """-> {'uy', 's', 'ymin', 'ymax'} se ha um obstaculo pequeno a vencer de lado, ou None."""
    if ang_plano is None or abs(ang_plano) > math.radians(60):
        return None
    _, _, y1 = FOOTPRINT
    barra = barrando(pts)
    if not barra:
        return None
    grupo = aglomerado(pts, barra)
    xs, ys = [p[0] for p in grupo], [p[1] for p in grupo]
    if max(xs) - min(xs) > OBJETO_MAX or max(ys) - min(ys) > OBJETO_MAX:
        return None
    outros = [p for p in pts if p not in grupo]
    opcoes = []
    for uy, s in ((1.0, max(ys) + y1 + MARGEM_LADO), (-1.0, -min(ys) + y1 + MARGEM_LADO)):
        if s > DESLIZE_MAX or not translacao_livre(outros, 0.0, uy, s + 0.03):
            continue
        if not translacao_livre([(x, y - uy * s) for x, y in pts], 1.0, 0.0, AVANCO, margem=0.02):
            continue
        # empate (2 cm): o lado do caminho
        opcoes.append((round(s / 0.02), 0 if uy * ang_plano >= 0 else 1, s, uy))
    if not opcoes:
        return None
    _, _, s, uy = min(opcoes)
    return {'uy': uy, 's': s, 'ymin': min(ys), 'ymax': max(ys)}



def main():
    import json
    import os
    import sys
    from collections import deque
    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import TwistStamped
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry, Path
    from nav2_msgs.action import ComputePathToPose, DriveOnHeading, Spin
    from nav2_msgs.srv import ClearEntireCostmap, IsPathValid  # BLOQUEIO_LOCAL
    from nav2_msgs.msg import CollisionMonitorState
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
    from tf2_ros import Buffer, TransformListener

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import recuperacao_avancada as ra

    def yaw_q(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    geo = sys.modules[__name__]

    class Destravar(Node):
        def __init__(self):
            super().__init__('destravar')
            g = ReentrantCallbackGroup()
            self.g = g
            self.lock = threading.Lock()
            # RE_LIBERADA (12/11): permitir_re era False (re bloqueada ate validacao fisica); pedido do usuario
            for nome, valor in (('permitir_lateral', True), ('permitir_giro', True), ('permitir_re', True),
                                # RE_LONGA (12/11): cobertura_re era 0.9 (traseira so 66-71% visivel); re_max era 0.15
                                ('cobertura_lateral', 0.5), ('cobertura_giro', 0.5), ('cobertura_re', 0.65),
                                ('deriva_lateral', 0.4), ('re_max', 0.30)):
                self.declare_parameter(nome, valor)
            # ORIENTAR_CAMINHO (13/11): giro de orientacao para o caminho (false = como antes)
            for nome, valor in (('giro_orientar', True), ('giro_orientar_max_graus', 360.0),
                                ('giro_orientar_wz', 0.20),
                                # v2 ORIENTAR_ROBUSTO: paradas para reavaliar o arco (0 = v1: aborta no 1o ponto)
                                ('giro_orientar_reavaliar', 3)):
                self.declare_parameter(nome, valor)
            self.pts = None
            self.raios = None
            self.declare_parameter('giro_abertura', False)  # GIRO_ABERTURA_20261116: opt-in
            self.declare_parameter('sem_caminho_unico', False)   # SEM_CAMINHO_UNICO (13/11): opt-in
            self.odom = None
            self.plano = None      # (t, [(x, y)] no map)
            self.plano_msg = None
            self.goal_plano = None # ultima pose do /plan (o destino do Nav2)
            self.cm_stop = False
            self.cm_stop_t = None
            self.meus = deque(maxlen=400)   # stamps (ns) que eu publiquei em /cmd_vel_nav
            self.estrangeiro_t = None       # ultimo comando nao nulo de OUTRO no em /cmd_vel_nav
            self.ctrl_t = None              # ultimo comando nao nulo do controlador (/cmd_vel_ctrl)
            self.ctrl_hist = deque(maxlen=200)  # (t, vx, vy, wz)
            self.scans = deque(maxlen=60)   # (t, pontos) para o ICP da janela
            self.yaws = deque(maxlen=300)   # (t, yaw imu)
            self.em_spin = False
            self.gh_atual = None
            self.create_subscription(LaserScan, '/scan_nav', self.ao_scan, qos_profile_sensor_data, callback_group=g)
            self.create_subscription(Odometry, '/odom', self.ao_odom, 20, callback_group=g)
            self.create_subscription(Path, '/plan', self.ao_plano, 5, callback_group=g)
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.ao_cm, 10, callback_group=g)
            self.create_subscription(TwistStamped, '/cmd_vel_nav', self.ao_cmd_nav, 20, callback_group=g)
            self.create_subscription(TwistStamped, '/cmd_vel_ctrl', self.ao_ctrl, 20, callback_group=g)
            self.pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
            self.pub_estado = self.create_publisher(String, '/destravar/estado', 10)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.bloqueios = []
            self.pub_bloq = self.create_publisher(PointCloud2, '/bloqueio_virtual', 1)
            self.create_timer(0.5, self.publicar_bloqueio, callback_group=g)
            self.cli_plano = ActionClient(self, ComputePathToPose, 'compute_path_to_pose', callback_group=g)
            self.cli_spin = ActionClient(self, Spin, 'smart_spin', callback_group=g)
            self.cli_valido = self.create_client(IsPathValid, 'is_path_valid', callback_group=g)
            self.cli_cm = self.create_client(GetState, 'collision_monitor/get_state', callback_group=g)
            # CONTROLADOR_ORFAO_NO (11/11): cancelar FollowPath orfao (todos os goals)
            from action_msgs.srv import CancelGoal
            self.cli_cancel_fp = self.create_client(CancelGoal, 'follow_path/_action/cancel_goal', callback_group=g)
            self.limpar_local_pendente = False
            self.t_aviso_sensor = 0.0  # SENSOR_PARADO
            self.cli_limpar_global = self.create_client(
                ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap', callback_group=g)
            self.cli_limpar_local = self.create_client(
                ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap', callback_group=g)
            self.hist = ra.Historico()
            ActionServer(self, DriveOnHeading, 'destravar', self.executar, callback_group=g,
                         cancel_callback=lambda _: CancelResponse.ACCEPT)
            self.get_logger().info('Destravar v4 pronto (acao /destravar): diagnostica, devolve ao Nav2 se ainda '
                                   'contorna, desliza com continuacao, gira pelo smart_spin; re %s.' % (
                                       'LIBERADA' if self.get_parameter('permitir_re').value else 'BLOQUEADA'))

        # ---------------- entradas ----------------
        def ao_scan(self, m):
            lx, ly, lt = LASER
            c, s = math.cos(lt), math.sin(lt)
            pts = []
            for i, r in enumerate(m.ranges):
                if math.isfinite(r) and m.range_min <= r <= min(m.range_max, 3.0):
                    a = m.angle_min + i * m.angle_increment
                    px, py = r * math.cos(a), r * math.sin(a)
                    pts.append((lx + c * px - s * py, ly + s * px + c * py))
            agora = time.monotonic()
            raios = ra.Raios(m.angle_min, m.angle_increment, tuple(m.ranges), max(m.range_min, 0.146),
                             m.range_max, LASER, agora)
            with self.lock:
                self.pts = (agora, pts)
                self.raios = raios
                self.scans.append((agora, pts))

        def ao_odom(self, m):
            with self.lock:
                self.odom = (time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y,
                             yaw_q(m.pose.pose.orientation))
                self.yaws.append((self.odom[0], self.odom[3]))

        def ao_plano(self, m):
            with self.lock:
                self.plano = (time.monotonic(), [(p.pose.position.x, p.pose.position.y) for p in m.poses])
                self.plano_msg = m
                if m.poses:
                    self.goal_plano = m.poses[-1]

        def ao_cm(self, m):
            self.cm_stop = m.action_type == 1
            if self.cm_stop:
                self.cm_stop_t = time.monotonic()

        def ao_cmd_nav(self, m):
            st = m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
            t = m.twist
            if st in self.meus or not (abs(t.linear.x) > 1e-3 or abs(t.linear.y) > 1e-3 or abs(t.angular.z) > 1e-3):
                return
            if self.em_spin:      # o smart_spin (behavior_server) comanda por mim durante o giro
                return
            self.estrangeiro_t = time.monotonic()

        def ao_ctrl(self, m):
            t = m.twist
            agora = time.monotonic()
            self.ctrl_hist.append((agora, t.linear.x, t.linear.y, t.angular.z))
            if abs(t.linear.x) > 1e-3 or abs(t.linear.y) > 1e-3 or abs(t.angular.z) > 1e-3:
                self.ctrl_t = agora

        def _esperar(self, fut, timeout):
            t1 = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < t1:
                time.sleep(0.02)
            return fut.result() if fut.done() else None

        def cancelar_follow_path(self, timeout=1.0):
            """CONTROLADOR_ORFAO_NO (11/11): cancela todos os goals do FollowPath. True = servidor respondeu."""
            from action_msgs.srv import CancelGoal
            if not self.cli_cancel_fp.wait_for_service(timeout_sec=0.5):
                return False
            r = self._esperar(self.cli_cancel_fp.call_async(CancelGoal.Request()), timeout)
            return r is not None

        def replanejar(self, timeout=5.0):
            """Pede um caminho novo ao planner. True: veio; False: o planner respondeu sem caminho;
            None: nao deu para perguntar (sem goal, sem servidor ou sem resposta a tempo)."""
            with self.lock:
                goal = self.goal_plano
            if goal is None or not self.cli_plano.wait_for_server(timeout_sec=1.0):
                return None
            req = ComputePathToPose.Goal()
            req.goal = goal
            req.planner_id = 'GridBased'
            req.use_start = False
            h = self._esperar(self.cli_plano.send_goal_async(req), timeout)
            if h is None or not h.accepted:
                return None
            r = self._esperar(h.get_result_async(), timeout)
            if r is None:
                return None
            if not r.result.path.poses:
                return False
            pts = [(p.pose.position.x, p.pose.position.y) for p in r.result.path.poses]
            with self.lock:
                self.plano = (time.monotonic(), pts)
                self.plano_msg = r.result.path
            self.get_logger().info('DESTRAVAR: caminho novo (%d pontos)' % len(pts))
            return True

        def caminho_valido(self):
            with self.lock:
                m = self.plano_msg
            if m is None or not m.poses or not self.cli_valido.wait_for_service(timeout_sec=0.5):
                return None
            req = IsPathValid.Request()
            req.path = m
            r = self._esperar(self.cli_valido.call_async(req), 2.0)
            return None if r is None else bool(r.is_valid)

        def cm_ativo(self):
            """Collision Monitor no estado 'active' do ciclo de vida (3)."""
            if not self.cli_cm.wait_for_service(timeout_sec=0.5):
                return False
            r = self._esperar(self.cli_cm.call_async(GetState.Request()), 1.0)
            return r is not None and r.current_state.id == 3

        # ---------------- bloqueio virtual (v3, igual) ----------------
        def pose_map(self):
            try:
                tr = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            except Exception:
                return None
            return tr.transform.translation.x, tr.transform.translation.y, yaw_q(tr.transform.rotation)

        def marcar_frente(self, profundidade=0.30, validade=10.0):  # BLOQUEIO_LOCAL: era 30 s
            p = self.pose_map()
            if p is None:
                return
            x0, y0, yaw = p
            c, s = math.cos(yaw), math.sin(yaw)
            pts = []
            xf = FOOTPRINT[1] + 0.03
            while xf <= FOOTPRINT[1] + profundidade + 1e-6:
                y = -FOOTPRINT[2]
                while y <= FOOTPRINT[2] + 1e-6:
                    pts.append((x0 + c * xf - s * y, y0 + s * xf + c * y))
                    y += 0.05
                xf += 0.05
            self.bloqueios.append((time.monotonic() + validade, pts))
            self.get_logger().info('BLOQUEIO: marquei %.0f cm a frente do robo por %.0f s' % (100 * profundidade, validade))

        def publicar_bloqueio(self):
            agora = time.monotonic()
            if self.limpar_local_pendente:  # BLOQUEIO_LOCAL: a nuvem vazia ja saiu no ciclo anterior
                self.limpar_local_pendente = False
                if self.cli_limpar_local.service_is_ready():
                    self.cli_limpar_local.call_async(ClearEntireCostmap.Request())
            havia = bool(self.bloqueios)
            self.bloqueios = [b for b in self.bloqueios if b[0] > agora]
            if havia and not self.bloqueios:
                self.limpar_local_pendente = True
            h = Header()
            h.stamp = self.get_clock().now().to_msg()
            h.frame_id = 'map'
            self.pub_bloq.publish(point_cloud2.create_cloud_xyz32(
                h, [(x, y, 0.10) for _, ps in self.bloqueios for x, y in ps]))

        def soltar_bloqueio(self):
            """BLOQUEIO_LOCAL: o planner nao achou caminho: tira as marcas e limpa os costmaps."""
            n = len(self.bloqueios)
            self.bloqueios = []
            self.limpar_local_pendente = False
            self.publicar_bloqueio()
            time.sleep(0.6)  # o costmap guarda a ultima nuvem: limpar antes marcaria tudo de novo
            for cli in (self.cli_limpar_global, self.cli_limpar_local):
                if cli.service_is_ready():
                    self._esperar(cli.call_async(ClearEntireCostmap.Request()), 1.0)
            time.sleep(0.6)  # o costmap global atualiza a 2 Hz
            self.get_logger().info('BLOQUEIO: sem caminho: apaguei %d bloqueio(s), limpei os costmaps e planejo de novo' % n)

        # ---------------- movimentos ----------------
        def vel(self, vx, vy, wz=0.0):
            # SENSOR_PARADO (06/11): scan ou odom com mais de 0,5 s: nao manda movimento
            agora = time.monotonic()
            pts, od = self.pts, self.odom
            if (vx or vy or wz) and (pts is None or od is None or agora - pts[0] > 0.5 or agora - od[0] > 0.5):
                vx, vy, wz = 0.0, 0.0, 0.0
                if agora - self.t_aviso_sensor > 2.0:
                    self.t_aviso_sensor = agora
                    self.get_logger().warn('DESTRAVAR: sensor parado: parei')
            t = TwistStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.twist.linear.x, t.twist.linear.y, t.twist.angular.z = float(vx), float(vy), float(wz)
            self.meus.append(t.header.stamp.sec * 1000000000 + t.header.stamp.nanosec)
            self.pub.publish(t)

        def parar(self):
            for _ in range(5):
                self.vel(0.0, 0.0, 0.0)
                time.sleep(0.05)

        def direcao_caminho(self):
            """Angulo (base_link) do ponto do /plan a ~0,45 m do robo, ou None."""
            with self.lock:
                pl = self.plano
            p = self.pose_map()
            if pl is None or p is None or time.monotonic() - pl[0] > 30.0 or len(pl[1]) < 2:
                return None
            x, y, yaw = p
            alvo = next(((px, py) for px, py in pl[1] if math.hypot(px - x, py - y) > 0.45), pl[1][-1])
            if math.hypot(alvo[0] - x, alvo[1] - y) < 0.15:
                return None
            a = math.atan2(alvo[1] - y, alvo[0] - x) - yaw
            return math.atan2(math.sin(a), math.cos(a))

        def smart_spin(self, ang, tmax):
            """Giro pelo smart_spin (micro-giros do Spin nativo, arco conferido no costmap, yaw por TF)."""
            if not self.cli_spin.wait_for_server(timeout_sec=1.0):
                return False, 'smart_spin indisponivel'
            req = Spin.Goal()
            req.target_yaw = float(ang)
            req.time_allowance.sec = int(tmax)
            self.em_spin = True
            try:
                h = self._esperar(self.cli_spin.send_goal_async(req), 2.0)
                if h is None or not h.accepted:
                    return False, 'smart_spin recusou'
                fr = h.get_result_async()
                t1 = time.monotonic() + tmax + 2.0
                while not fr.done() and time.monotonic() < t1:
                    if self.gh_atual is not None and self.gh_atual.is_cancel_requested:
                        h.cancel_goal_async()
                        return False, 'cancelado'
                    time.sleep(0.05)
                if not fr.done():
                    h.cancel_goal_async()
                    return False, 'smart_spin sem resposta'
                r = fr.result()
                ok = r.status == 4 and r.result.error_code == 0   # STATUS_SUCCEEDED
                msg = getattr(r.result, 'error_msg', '') or ('ok' if ok else 'falhou (%d)' % r.result.error_code)
                return ok, msg
            finally:
                time.sleep(0.3)
                self.em_spin = False

        def janela(self, seg=4.0):
            """Evidencias dos ultimos seg: controlador, CM, inversoes, deslocamento fisico (ICP) e giro (IMU)."""
            agora = time.monotonic()
            hist = [h for h in list(self.ctrl_hist) if agora - h[0] <= seg]
            ativos = [h for h in hist if abs(h[1]) > 1e-3 or abs(h[2]) > 1e-3 or abs(h[3]) > 1e-3]
            ativo_s = 0.0
            if ativos:
                ativo_s = min(seg, ativos[-1][0] - ativos[0][0])
            inv = 0
            for k in (2, 3):
                sinais = [math.copysign(1, h[k]) for h in ativos if abs(h[k]) > 0.02]
                inv += sum(1 for a, b in zip(sinais, sinais[1:]) if a != b)
            scans = [s for s in list(self.scans) if agora - s[0] <= seg + 0.5]
            desloc = None
            if len(scans) >= 2 and agora - scans[0][0] >= seg * 0.6:
                m = ra.medir_deslocamento(scans[0][1], scans[-1][1], res_max=0.04, amb_max=0.06)
                if m.ok:
                    desloc = math.hypot(m.dx, m.dy)
            ys = [y for y in list(self.yaws) if agora - y[0] <= seg]
            giro = ra.ang_norm(ys[-1][1] - ys[0][1]) if len(ys) >= 2 else None
            return {'ctrl_parado_s': None if self.ctrl_t is None else agora - self.ctrl_t,
                    'ctrl_ativo_s': ativo_s,
                    'cm_stop_s': None if self.cm_stop_t is None else agora - self.cm_stop_t,
                    'desloc_janela': desloc, 'giro_janela': giro, 'inversoes': inv}

        # ---------------- a acao ----------------
        def executar(self, gh):
            t0 = time.monotonic()
            req = gh.request
            dist = max(0.10, min(0.40, abs(req.target.x) or 0.15))
            vcmd = max(0.02, min(0.06, abs(req.speed) or 0.03))
            tmax = (req.time_allowance.sec + req.time_allowance.nanosec * 1e-9) or 10.0
            tmax = max(tmax, 12.0)
            cfg = ra.Config(permitir_lateral=bool(self.get_parameter('permitir_lateral').value),
                            permitir_giro=bool(self.get_parameter('permitir_giro').value),
                            permitir_re=bool(self.get_parameter('permitir_re').value),
                            cobertura_lateral=float(self.get_parameter('cobertura_lateral').value),
                            cobertura_giro=float(self.get_parameter('cobertura_giro').value),
                            cobertura_re=float(self.get_parameter('cobertura_re').value),
                            deriva_lateral=float(self.get_parameter('deriva_lateral').value),
                            re_max=max(0.10, min(0.40, float(self.get_parameter('re_max').value))))   # RE_LONGA
            # ORIENTAR_CAMINHO (13/11): wz limitado a 0,15-0,20 (nunca mais rapido que o DWB)
            cfg.giro_abertura = bool(self.get_parameter('giro_abertura').value)  # GIRO_ABERTURA_20261116
            cfg.giro_orientar = bool(self.get_parameter('giro_orientar').value)
            cfg.giro_orientar_max = math.radians(max(0.0, min(360.0, float(
                self.get_parameter('giro_orientar_max_graus').value))))
            cfg.giro_orientar_wz = max(0.15, min(0.20, float(self.get_parameter('giro_orientar_wz').value)))
            cfg.giro_orientar_reavaliar = max(0, min(5, int(self.get_parameter('giro_orientar_reavaliar').value)))
            self.gh_atual = gh
            res = DriveOnHeading.Result()
            cfg.sem_caminho_unico = bool(self.get_parameter('sem_caminho_unico').value)   # SEM_CAMINHO_UNICO (13/11)
            try:
                ok, msg, _ = ra.Recuperador(IoRos(self), geo, cfg, self.hist).executar(dist, vcmd, tmax)
            except Exception as exc:  # nunca deixa o robo andando
                ok, msg = False, 'erro interno: %r' % (exc,)
                self.get_logger().error('DESTRAVAR: ' + msg)
            finally:
                self.parar()
                self.gh_atual = None
            dt = time.monotonic() - t0
            res.total_elapsed_time = Duration(sec=int(dt), nanosec=int(dt % 1 * 1e9))
            res.error_code = 0 if ok else 1
            try:
                res.error_msg = msg[:250]
            except Exception:
                pass
            if ok:
                self.get_logger().info('DESTRAVAR: ' + msg)
                gh.succeed()
            else:
                self.get_logger().warn('DESTRAVAR: ' + msg)
                if gh.is_cancel_requested:
                    gh.canceled()
                else:
                    gh.abort()
            return res

    class IoRos:
        """Interface do Recuperador sobre o no ROS."""

        def __init__(self, no):
            self.no = no

        def agora(self):
            return time.monotonic()

        def dormir(self, s):
            time.sleep(s)

        def scan(self):
            return self.no.raios

        def odom(self):
            return self.no.odom

        def pose_ref(self):
            p = self.no.pose_map()
            return None if p is None else (p[0], p[1])

        def goal(self):
            g = self.no.goal_plano
            return None if g is None else (g.pose.position.x, g.pose.position.y)

        def enviar(self, vx, vy, wz):
            self.no.vel(vx, vy, wz)

        def parar(self):
            self.no.vel(0.0, 0.0, 0.0)

        def cancelado(self):
            h = self.no.gh_atual
            return h is not None and h.is_cancel_requested

        def cm_stop(self):
            return self.no.cm_stop

        def cm_ativo(self):
            return self.no.cm_ativo()

        def ultimo_estrangeiro(self):
            return self.no.estrangeiro_t

        def ultimo_ctrl(self):
            return self.no.ctrl_t

        def cancelar_controlador(self):   # CONTROLADOR_ORFAO_NO (11/11)
            return self.no.cancelar_follow_path()

        def replanejar(self):
            return self.no.replanejar()

        def soltar_bloqueio(self):
            self.no.soltar_bloqueio()

        def marcar_frente(self):
            self.no.marcar_frente()
            time.sleep(0.4)      # BLOQUEIO_LOCAL: o costmap local atualiza a 5 Hz

        def caminho_valido(self):
            return self.no.caminho_valido()

        def direcao_caminho(self):
            return self.no.direcao_caminho()

        def janela(self):
            return self.no.janela()

        def smart_spin(self, ang, tmax):
            return self.no.smart_spin(ang, tmax)

        def log(self, nivel, txt):
            # o rclpy nao aceita a mesma linha de log com severidades diferentes: uma chamada por nivel
            if nivel == 'warn':
                self.no.get_logger().warn('DESTRAVAR: ' + txt)
            else:
                self.no.get_logger().info('DESTRAVAR: ' + txt)

        def estado(self, rel):
            try:
                self.no.pub_estado.publish(String(data=json.dumps(rel, ensure_ascii=False, default=str)))
            except Exception:
                pass

    rclpy.init()
    no = Destravar()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(no)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            no.parar()          # pode falhar se o SIGINT ja fechou o contexto
        except Exception:
            pass
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
