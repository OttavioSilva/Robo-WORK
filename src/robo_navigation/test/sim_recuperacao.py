"""Simulador 2D sem ROS para o Recuperador (RECUPERACAO_AVANCADA). Nao publica nada.

Mundo: caixas (x0, x1, y0, y1) no referencial do mundo. Robo: footprint fisico medido.
LiDAR: 360 raios a 1 grau, 5,9 Hz, alcance minimo 0,146 m, fracao de raios 'inf' (31-38% no robo) e
setor cego opcional (sombra_traseira). Movimento real: o deslize lateral rende 'rend_lateral' do que a
odometria (feita pelo comando) acredita e avanca 'deriva' por metro de deslize. Collision Monitor:
poligonos reais (min 3 pontos), para o comando. Colisao: o footprint encostou numa caixa.
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import recuperacao_avancada as ra  # noqa: E402
import destravar_node as geo  # noqa: E402  (so as funcoes puras do v3; main() nao roda)

DT = 0.05


def _ray_box(ox, oy, dx, dy, b):
    x0, x1, y0, y1 = b
    tmin, tmax = -1e9, 1e9
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
    if tmax < max(tmin, 0.0):
        return None
    return tmin if tmin > 0 else None


class SimIO:
    def __init__(self, caixas, pose=(0.0, 0.0, 0.0), goal=(3.0, 0.0), inf=0.30, semente=1,
                 rend_lateral=0.70, deriva=0.35, sombra_traseira=None, planner=True, valido=False,
                 janela=None, cancelar_em=None, estrangeiro_em=None, cm_ativo=True, spin_ok=True,
                 waypoint=None):
        self.caixas = list(caixas)
        self.x, self.y, self.th = pose
        self.ox, self.oy, self.oth = pose      # odometria (comando + yaw da IMU)
        self.t = 0.0
        self.goal_xy = goal
        self.waypoint = waypoint
        self.inf, self.rng = inf, random.Random(semente)
        self.rend_lateral, self.deriva = rend_lateral, deriva
        self.sombra = sombra_traseira
        self.cmd = (0.0, 0.0, 0.0)
        self._planner = planner
        self._valido = valido
        self._janela = janela or {}
        self.cancelar_em, self.estrangeiro_em = cancelar_em, estrangeiro_em
        self._cm_ativo, self.spin_ok = cm_ativo, spin_ok
        self.colisoes = 0
        self.logs, self.estados = [], []
        self.marcou = 0
        self.spins = []
        self.max_vel = 0.0
        self._scan = None
        self._t_scan = -1.0
        self._gerar_scan()

    # ---------------- mundo
    def corpo_livre(self, x, y, th, m=0.0):
        x0, x1, y1 = ra.FOOT
        c, s = math.cos(th), math.sin(th)
        for b in self.caixas:
            # cantos da caixa dentro do corpo, ou cantos do corpo dentro da caixa, ou amostras da borda
            for i in range(9):
                for j in range(9):
                    bx = b[0] + (b[1] - b[0]) * i / 8
                    by = b[2] + (b[3] - b[2]) * j / 8
                    px, py = bx - x, by - y
                    if ra.dentro(c * px + s * py, -s * px + c * py, ra.FOOT, m):
                        return False
            for cx, cy in ((x0, y1), (x0, -y1), (x1, y1), (x1, -y1), (x1, 0), (x0, 0), (0, y1), (0, -y1)):
                wx, wy = x + c * cx - s * cy, y + s * cx + c * cy
                if b[0] <= wx <= b[1] and b[2] <= wy <= b[3]:
                    return False
        return True

    def _gerar_scan(self):
        lx, ly, _ = ra.LASER
        c, s = math.cos(self.th), math.sin(self.th)
        ox, oy = self.x + c * lx - s * ly, self.y + s * lx + c * ly
        rs = []
        for i in range(360):
            a = -math.pi + math.radians(i)
            if self.sombra and self.sombra[0] <= a <= self.sombra[1]:
                rs.append(float('nan'))
                continue
            wa = self.th + a
            dx, dy = math.cos(wa), math.sin(wa)
            best = None
            for b in self.caixas:
                t = _ray_box(ox, oy, dx, dy, b)
                if t is not None and (best is None or t < best):
                    best = t
            if best is None or best > 8.0 or best < 0.146 or self.rng.random() < self.inf:
                rs.append(float('inf'))
            else:
                rs.append(best + self.rng.gauss(0.0, 0.004))
        self._scan = ra.Raios(-math.pi, math.radians(1), tuple(rs), 0.146, 12.0, ra.LASER, self.t)
        self._t_scan = self.t

    def _cm_para(self, vx, vy, wz):
        if not self._cm_ativo:
            return False
        if abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(wz) < 1e-4:
            return False
        pol = ra.poligono_de((vx, vy, 0.0)) if math.hypot(vx, vy) > 0.005 else 'stopped'
        return ra.cm_barraria(self._scan.pontos(), pol)

    # ---------------- interface do Recuperador
    def agora(self):
        return self.t

    def dormir(self, s):
        n = max(1, int(round(s / DT)))
        for _ in range(n):
            vx, vy, wz = self.cmd
            self.cm = self._cm_para(vx, vy, wz)
            if self.cm:
                vx = vy = wz = 0.0
            # odometria: acredita no comando
            c, s_ = math.cos(self.oth), math.sin(self.oth)
            self.ox += (c * vx - s_ * vy) * DT
            self.oy += (s_ * vx + c * vy) * DT
            # real: deslize rende menos e avanca
            rvy = vy * self.rend_lateral
            rvx = vx + self.deriva * abs(rvy)
            c, s_ = math.cos(self.th), math.sin(self.th)
            nx = self.x + (c * rvx - s_ * rvy) * DT
            ny = self.y + (s_ * rvx + c * rvy) * DT
            nth = self.th + wz * DT
            if self.corpo_livre(nx, ny, nth):
                self.x, self.y, self.th = nx, ny, nth
                self.oth = self.th          # yaw da IMU e real
            elif (vx, vy, wz) != (0.0, 0.0, 0.0):
                self.colisoes += 1
            self.max_vel = max(self.max_vel, math.hypot(vx, vy))
            self.t += DT
            if self.t - self._t_scan >= 1.0 / 5.9 - 1e-9:
                self._gerar_scan()

    def scan(self):
        return self._scan

    def odom(self):
        return (self.t, self.ox, self.oy, self.oth)

    def pose_ref(self):
        return (self.x, self.y)

    def goal(self):
        return self.goal_xy

    def enviar(self, vx, vy, wz):
        self.cmd = (vx, vy, wz)

    def parar(self):
        self.cmd = (0.0, 0.0, 0.0)

    def cancelado(self):
        return self.cancelar_em is not None and self.t >= self.cancelar_em

    def cm_stop(self):
        return getattr(self, 'cm', False)

    def cm_ativo(self):
        return self._cm_ativo

    def ultimo_estrangeiro(self):
        if self.estrangeiro_em is not None and self.t >= self.estrangeiro_em:
            return self.t
        return None

    def ultimo_ctrl(self):
        return None

    def replanejar(self):
        return self._planner(self) if callable(self._planner) else self._planner

    def soltar_bloqueio(self):
        self.limpezas = getattr(self, 'limpezas', 0) + 1

    def marcar_frente(self):
        self.marcou += 1

    def caminho_valido(self):
        return self._valido

    def direcao_caminho(self):
        if self._planner is False:
            return None                    # sem caminho global: sem direcao
        alvo = self.waypoint(self) if callable(self.waypoint) else self.goal_xy
        if alvo is None:
            return None
        a = math.atan2(alvo[1] - self.y, alvo[0] - self.x) - self.th
        return ra.ang_norm(a)

    def janela(self):
        base = {'ctrl_parado_s': 2.0, 'ctrl_ativo_s': 0.0, 'cm_stop_s': None,
                'desloc_janela': None, 'giro_janela': None, 'inversoes': 0}
        base.update(self._janela)
        return base

    def smart_spin(self, ang, tmax):
        self.spins.append(ang)
        if not self.spin_ok:
            return False, 'sentido bloqueado'
        alvo = self.th + ang
        passo = math.copysign(0.15 * DT, ang)
        t1 = self.t + tmax
        while abs(ra.ang_norm(alvo - self.th)) > abs(passo) and self.t < t1:
            if self.cancelado():
                self.cmd = (0.0, 0.0, 0.0)
                return False, 'cancelado'
            if not self.corpo_livre(self.x, self.y, self.th + passo, 0.02):
                self.cmd = (0.0, 0.0, 0.0)
                return False, 'arco bloqueado no costmap'
            self.cmd = (0.0, 0.0, math.copysign(0.15, ang))
            self.dormir(DT)
        self.cmd = (0.0, 0.0, 0.0)
        return True, 'girou'

    def log(self, nivel, txt):
        self.logs.append((nivel, txt))

    def estado(self, rel):
        self.estados.append(dict(rel))


def recuperador(io, **cfg):
    hist = cfg.pop('hist', None)
    return ra.Recuperador(io, geo, ra.Config(**cfg), hist)
