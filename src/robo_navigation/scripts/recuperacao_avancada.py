#!/usr/bin/env python3
"""RECUPERACAO_AVANCADA v1 (10/11/2026): decisao da recuperacao do destravar, sem ROS (testavel).

O destravar_node.py (servidor "destravar" da arvore e do ANTI_CISCADO do alinhador) chama este modulo:

  ETAPA A  diagnosticar()  -> por que travou (caminho invalido, DWB sem trajetoria, CM, sem progresso
                              fisico, oscilacao, quina, frente barrada)
  ETAPA B  'retomar'       -> o planner tem caminho e o corpo passa na direcao dele: nao mexe, devolve ao DWB
  ETAPA C  'contornar' / 'lateral' -> desliza de lado (corpo inteiro + deriva para a frente medida) so se
                              depois do deslize o caminho fica livre (continuacao)
  ETAPA D  're'            -> so com permitir_re, area varrida livre E observada (cobertura), ICP capaz de
                              confirmar o deslocamento, CM ativo e comando exclusivo
  ETAPA E  depois da re o no PARA, tira foto nova, confirma pelo ICP e decide de novo (nunca encadeia)
  ETAPA F  'giro'          -> pelo smart_spin, so se nada de lado resolve, com area varrida livre/observada
  ETAPA G  Historico       -> nao repete o que falhou ou nao resolveu no mesmo lugar; falha explicita

Nada aqui usa a odometria (que vem do comando) como prova de deslocamento: a prova e o ICP entre fotos
do LiDAR com o robo parado (ou, no giro, o yaw da IMU).
"""
import math
import time
from dataclasses import dataclass, field, replace

try:
    import numpy as np
except ImportError:  # pragma: no cover - o robo tem numpy; sem ele o ICP fica indisponivel
    np = None

# Fisico medido (CLAUDE.md): x -0,215 .. 0,375, y +-0,325. O Nav2 usa este menos 1 cm + padding.
FOOT = (-0.215, 0.375, 0.325)
LASER = (0.33, 0.005, 0.0)


def ang_norm(a):
    return math.atan2(math.sin(a), math.cos(a))


# ------------------------------------------------------------------ scan
@dataclass
class Raios:
    """Um LaserScan em base_link (o laser em LASER)."""
    angle_min: float
    inc: float
    ranges: tuple
    rmin: float = 0.146
    rmax: float = 12.0
    laser: tuple = LASER
    t: float = 0.0

    def pontos(self, alcance=3.0):
        lx, ly, lt = self.laser
        out = []
        for i, r in enumerate(self.ranges):
            if math.isfinite(r) and self.rmin <= r <= min(self.rmax, alcance):
                a = self.angle_min + i * self.inc + lt
                out.append((lx + r * math.cos(a), ly + r * math.sin(a)))
        return out

    def observado_livre(self, x, y, res=0.03):
        """Ha raio passando por (x, y) com retorno ALEM dele? inf/NaN = desconhecido (31-38% dos raios)."""
        lx, ly, lt = self.laser
        d = math.hypot(x - lx, y - ly)
        if d < self.rmin or d > self.rmax - res or not self.ranges or self.inc <= 0:
            return False
        a = ang_norm(math.atan2(y - ly, x - lx) - lt)
        n = len(self.ranges)
        k = (a - self.angle_min) / self.inc
        cheio = n * self.inc >= 2 * math.pi - 1.5 * self.inc
        # janela angular do tamanho da celula (no minimo +-1 raio)
        meia = max(1, int(math.ceil(math.atan2(res, max(d, 1e-3)) / self.inc)))
        base = int(round(k))
        for j in range(base - meia, base + meia + 1):
            if cheio:
                j %= n
            elif j < 0 or j >= n:
                continue
            r = self.ranges[j]
            if math.isfinite(r) and self.rmin <= r <= self.rmax and r > d + res:
                return True
        return False


def dentro(x, y, foot=FOOT, m=0.0):
    x0, x1, y1 = foot
    return x0 - m <= x <= x1 + m and abs(y) <= y1 + m


def _amostras(mov, passo_lin=0.02, passo_ang=math.radians(3)):
    dx, dy, dth = mov
    n = max(1, int(math.ceil(max(math.hypot(dx, dy) / passo_lin, abs(dth) / passo_ang))))
    return [(dx * k / n, dy * k / n, dth * k / n) for k in range(n + 1)]


def _no_corpo_movido(x, y, pose, foot, m):
    cx, cy, a = pose
    c, s = math.cos(a), math.sin(a)
    px, py = x - cx, y - cy
    return dentro(c * px + s * py, -s * px + c * py, foot, m)


@dataclass
class Avaliacao:
    livre: bool
    obstaculos: int
    cobertura: float
    celulas: int
    motivo: str = ''


def avaliar(raios, movs, foot=FOOT, margem=0.03, grade=0.03, min_pts=1, cobertura_min=0.0):
    """Corpo inteiro (+margem) em TODAS as poses intermediarias de cada movimento de movs.

    movs: lista de (dx, dy, dth) a partir da pose atual, em base_link (a uniao e avaliada, p.ex. o
    deslize reto e o deslize com deriva). livre exige: menos de min_pts pontos do scan na area e
    fracao observada livre >= cobertura_min nas celulas novas (fora do corpo atual)."""
    pts = [p for p in raios.pontos() if not dentro(p[0], p[1], foot)]
    poses = []
    for mv in movs:
        poses.extend(_amostras(mv))
    obst = 0
    for x, y in pts:
        # ponto ja dentro da margem agora: so veta se o movimento o levar a CONTATO (afastar-se e o escape)
        m = 0.0 if dentro(x, y, foot, margem) else margem
        if any(_no_corpo_movido(x, y, ps, foot, m) for ps in poses[1:] or poses):
            obst += 1
    x0, x1, y1 = foot
    cel = set()
    for cx, cy, a in poses[1:] if len(poses) > 1 else poses:
        c, s = math.cos(a), math.sin(a)
        nx = int(math.ceil((x1 - x0 + 2 * margem) / grade))
        ny = int(math.ceil((2 * y1 + 2 * margem) / grade))
        for i in range(nx + 1):
            bx = x0 - margem + i * (x1 - x0 + 2 * margem) / nx
            for j in range(ny + 1):
                by = -y1 - margem + j * (2 * y1 + 2 * margem) / ny
                wx, wy = cx + c * bx - s * by, cy + s * bx + c * by
                if not dentro(wx, wy, foot):
                    cel.add((round(wx / grade), round(wy / grade)))
    vistos = sum(1 for gx, gy in cel if raios.observado_livre(gx * grade, gy * grade, grade))
    cob = vistos / len(cel) if cel else 1.0
    livre = obst < min_pts and cob >= cobertura_min
    motivo = ''
    if obst >= min_pts:
        motivo = '%d ponto(s) do LiDAR na area varrida' % obst
    elif cob < cobertura_min:
        motivo = 'area varrida %.0f%% observada (minimo %.0f%%)' % (100 * cob, 100 * cobertura_min)
    return Avaliacao(livre, obst, cob, len(cel), motivo)


def deslocar(pts, mov):
    """Pontos vistos da pose final de mov (dx, dy, dth)."""
    dx, dy, dth = mov
    c, s = math.cos(dth), math.sin(dth)
    return [(c * (x - dx) + s * (y - dy), -s * (x - dx) + c * (y - dy)) for x, y in pts]


def folga_corpo(pts, foot=FOOT):
    """Menor distancia de um ponto do scan ao retangulo do corpo (m); 9 = nada perto."""
    x0, x1, y1 = foot
    best = 9.0
    for x, y in pts:
        if dentro(x, y, foot):
            continue
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(abs(y) - y1, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


def folga_direcao(pts, ux, uy, foot=FOOT, margem=0.03, alcance=0.60, passo=0.02):
    """Quanto o CORPO INTEIRO anda na direcao (ux, uy) antes de pegar um ponto (m, ate alcance)."""
    p = [q for q in pts if not dentro(q[0], q[1], foot)]
    s = passo
    while s <= alcance + 1e-9:
        if any(dentro(x - s * ux, y - s * uy, foot, margem) for x, y in p):
            return s - passo
        s += passo
    return alcance


# ------------------------------------------------------------------ ICP (de ferramentas/testes/teste_deslize_icp.py)
def icp(a, b, chute=(0.0, 0.0, 0.0), voltas=40):
    """(dx, dy, dth, residuo, pares): pose do scan b no referencial do scan a = deslocamento do robo."""
    if np is None:
        return None
    th, tx, ty = chute[2], chute[0], chute[1]
    raio = 0.30
    res, n = 9.0, 0
    for _ in range(voltas):
        c, s = math.cos(th), math.sin(th)
        q = np.column_stack((c * b[:, 0] - s * b[:, 1] + tx, s * b[:, 0] + c * b[:, 1] + ty))
        d2 = ((q[:, None, :] - a[None, :, :]) ** 2).sum(axis=2)
        j = d2.argmin(axis=1)
        dmin = np.sqrt(d2[np.arange(len(q)), j])
        ok = dmin < raio
        if ok.sum() < 15:
            return None
        lim = np.percentile(dmin[ok], 75)
        ok &= dmin <= lim
        p, r = b[ok], a[j[ok]]
        mp, mr = p.mean(axis=0), r.mean(axis=0)
        h = (p - mp).T @ (r - mr)
        th = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        c, s = math.cos(th), math.sin(th)
        tx = mr[0] - (c * mp[0] - s * mp[1])
        ty = mr[1] - (s * mp[0] + c * mp[1])
        res, n = float(dmin[ok].mean()), int(ok.sum())
        raio = max(0.06, raio * 0.85)
    return tx, ty, th, res, n


def icp_varios(a, b, chute=(0.0, 0.0, 0.0)):
    """ICP de varias partidas: (dx, dy, dth, residuo, pares, ambiguidade_m). None se nao casou."""
    sols = []
    for fx in (-0.15, 0.0, 0.15):
        for fy in (-0.15, 0.0, 0.15):
            r = icp(a, b, chute=(chute[0] + fx, chute[1] + fy, chute[2]))
            if r is not None:
                sols.append(r)
    if not sols:
        return None
    mais = float(max(r[4] for r in sols))
    nota = lambda r: r[3] + 0.5 * (1.0 - r[4] / mais)  # noqa: E731
    sols.sort(key=nota)
    m = sols[0]
    amb = max((math.hypot(r[0] - m[0], r[1] - m[1]) for r in sols if nota(r) <= nota(m) + 0.004), default=0.0)
    return m + (amb,)


def nuvem(pts, max_pts=260, alcance=3.0):
    if np is None:
        return None
    p = [q for q in pts if math.hypot(q[0], q[1]) <= alcance and not dentro(q[0], q[1], FOOT, 0.03)]
    if len(p) > max_pts:
        passo = len(p) / max_pts
        p = [p[int(i * passo)] for i in range(max_pts)]
    return np.array(p, dtype=float) if p else None


@dataclass
class Medida:
    ok: bool
    dx: float = 0.0
    dy: float = 0.0
    dth: float = 0.0
    residuo: float = 9.0
    pares: int = 0
    ambiguidade: float = 9.0
    motivo: str = ''


def medir_deslocamento(pts0, pts1, chute=(0.0, 0.0, 0.0), res_max=0.025, pares_min=30, amb_max=0.04):
    """Deslocamento real do robo entre duas fotos paradas do LiDAR (ICP). ok so se a medida e confiavel."""
    a, b = nuvem(pts0), nuvem(pts1)
    if a is None or b is None or len(a) < 30 or len(b) < 30:
        return Medida(False, motivo='poucos pontos para o ICP')
    r = icp_varios(a, b, chute)
    if r is None:
        return Medida(False, motivo='ICP nao casou')
    dx, dy, dth, res, n, amb = r
    m = Medida(True, dx, dy, dth, res, n, amb)
    if res > res_max:
        m.ok, m.motivo = False, 'ICP residuo %.1f cm' % (100 * res)
    elif n < pares_min:
        m.ok, m.motivo = False, 'ICP so %d pares' % n
    elif amb > amb_max:
        m.ok, m.motivo = False, 'ICP ambiguo (%.0f cm): ambiente sem referencia nessa direcao' % (100 * amb)
    return m


def icp_verificavel(pts, ux, uy, d=0.10):
    """Antes de mover: o ICP conseguiria confirmar um deslocamento d na direcao (ux, uy) aqui?
    Simula a foto depois do movimento (mesmos pontos deslocados) e exige medida confiavel e certa."""
    depois = deslocar(pts, (d * ux, d * uy, 0.0))
    m = medir_deslocamento(pts, depois, (0.0, 0.0, 0.0))
    if not m.ok:
        return False, m.motivo
    if math.hypot(m.dx - d * ux, m.dy - d * uy) > 0.03:
        return False, 'ICP erraria %.0f cm nessa direcao' % (100 * math.hypot(m.dx - d * ux, m.dy - d * uy))
    return True, ''


# Poligonos do Collision Monitor (nav2_params.yaml, CM_RIGIDO 09/11), em base_link: (x0, x1, y0, y1)
CM_POLIGONOS = {
    'left': (-0.225, 0.425, -0.050, 0.375),
    'right': (-0.225, 0.425, -0.375, 0.050),
    'backward': (-0.265, 0.050, -0.325, 0.325),
    'forward': (-0.050, 0.425, -0.325, 0.325),
    'stopped': (-0.245, 0.405, -0.355, 0.355),
}
CM_MIN_PONTOS = 3


def cm_barraria(pts, poligono):
    """O CM (min_points 3) pararia um movimento cujo poligono ja tem pontos dentro agora?"""
    x0, x1, y0, y1 = CM_POLIGONOS[poligono]
    n = sum(1 for x, y in pts if x0 <= x <= x1 and y0 <= y <= y1 and not dentro(x, y, FOOT))
    return n >= CM_MIN_PONTOS


def poligono_de(mov):
    dx, dy, dth = mov
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return 'stopped'
    a = math.atan2(dy, dx)
    if -0.785 <= a <= 0.785:
        return 'forward'
    if 0.785 < a <= 2.356:
        return 'left'
    if -2.356 <= a < -0.785:
        return 'right'
    return 'backward'


# ------------------------------------------------------------------ ETAPA A: diagnostico
@dataclass
class Evidencias:
    planner: str = 'desconhecido'         # 'ok' | 'sem_caminho' | 'indisponivel' | 'desconhecido'
    caminho_valido: object = None         # True | False | None (IsPathValid no /plan atual)
    ctrl_parado_s: object = None          # s desde o ultimo comando nao nulo do controlador (None: nunca)
    ctrl_ativo_s: float = 0.0             # s de comando nao nulo do controlador na janela
    cm_stop_s: object = None              # s desde o ultimo STOP do Collision Monitor (None: nao houve)
    desloc_janela: object = None          # deslocamento fisico (ICP) na janela (m), None: sem medida
    giro_janela: object = None            # giro pela IMU na janela (rad)
    inversoes: int = 0                    # inversoes de sentido do comando em 4 s
    frente: float = 9.0                   # folga do para-choque (m)
    quina: object = None                  # 'esq' | 'dir' | None
    chamador_oscilou: bool = False


def detectar_quina(pts, foot=FOOT):
    """Canto da frente encostado com o meio da frente livre: 'esq'/'dir'/None."""
    _, x1, y1 = foot
    canto = {'esq': 0, 'dir': 0}
    meio = 0
    for x, y in pts:
        if x1 - 0.10 <= x <= x1 + 0.10:
            if y1 - 0.12 <= y <= y1 + 0.08:
                canto['esq'] += 1
            elif -y1 - 0.08 <= y <= -y1 + 0.12:
                canto['dir'] += 1
            elif abs(y) < y1 - 0.12 and x <= x1 + 0.10:
                meio += 1
    lados = [k for k, v in canto.items() if v >= 2]
    if len(lados) == 1 and meio == 0:
        return lados[0]
    return None


def folga_frente(pts, foot=FOOT, m=0.02):
    x0, x1, y1 = foot
    d = [x - x1 for x, y in pts if x > x1 and abs(y) <= y1 + m]
    return float(min(d)) if d else 9.0


def diagnosticar(ev):
    causas = []
    if ev.caminho_valido is False:
        causas.append('caminho_invalido')
    if ev.planner == 'sem_caminho':
        causas.append('sem_caminho_global')
    if ev.cm_stop_s is not None and ev.cm_stop_s < 3.0:
        causas.append('cm_bloqueou')
    if ev.planner == 'ok' and (ev.ctrl_parado_s is None or ev.ctrl_parado_s > 1.0):
        causas.append('dwb_sem_trajetoria')
    parado = (ev.desloc_janela is not None and ev.desloc_janela < 0.03 and
              (ev.giro_janela is None or abs(ev.giro_janela) < math.radians(6)))
    if ev.ctrl_ativo_s >= 2.0 and parado:
        causas.append('sem_progresso_fisico')
    if ev.inversoes >= 4 or ev.chamador_oscilou:
        causas.append('oscilacao')
    if ev.quina:
        causas.append('quina_' + ev.quina)
    if ev.frente < 0.15:
        causas.append('frente_barrada')
    return causas


# ------------------------------------------------------------------ ETAPA G: historico
@dataclass
class Tentativa:
    t: float
    x: float
    y: float
    acao: str
    resultado: str      # 'ok' | 'falhou' | 'sem_efeito' | 'nao_verificado'
    detalhe: str = ''


class Historico:
    """Por goal. Tentativas no mesmo lugar (raio) nos ultimos 'validade' s."""

    def __init__(self, raio=0.15, validade=120.0, max_retornos=3, max_acoes=10):
        self.raio, self.validade = raio, validade
        self.max_retornos, self.max_acoes = max_retornos, max_acoes
        self.itens = []
        self.chamadas = []   # (t, x, y)
        self.goal = None

    def novo_goal(self, goal):
        """goal: (x, y) ou None. Goal novo (> 0,30 m) zera o historico."""
        if goal is None:
            return False
        if self.goal is None or math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) > 0.30:
            self.goal = goal
            self.itens, self.chamadas = [], []
            return True
        return False

    def _aqui(self, t, x, y, lista):
        return [i for i in lista if t - i[0] <= self.validade and math.hypot(i[1] - x, i[2] - y) <= self.raio]

    def nova_chamada(self, t, x, y):
        """Registra a chamada. Se o robo voltou a um lugar onde algo 'deu certo', aquilo nao resolveu.
        Devolve quantas vezes ja travou aqui antes."""
        antes = len(self._aqui(t, x, y, [(c[0], c[1], c[2]) for c in self.chamadas]))
        if antes:
            for it in self.itens:
                if (it.resultado == 'ok' and t - it.t <= self.validade and
                        math.hypot(it.x - x, it.y - y) <= self.raio):
                    it.resultado, it.detalhe = 'sem_efeito', 'voltou ao mesmo bloqueio'
        self.chamadas.append((t, x, y))
        return antes

    def registrar(self, t, x, y, acao, resultado, detalhe=''):
        self.itens.append(Tentativa(t, x, y, acao, resultado, detalhe))

    def bloqueadas(self, t, x, y):
        """Acoes que nao devem ser repetidas aqui: falharam, nao resolveram ou nao se confirmaram."""
        return {i.acao for i in self.itens
                if i.resultado != 'ok' and t - i.t <= self.validade and math.hypot(i.x - x, i.y - y) <= self.raio}

    def total(self):
        return len([i for i in self.itens if i.acao != 'retomar'])


# ------------------------------------------------------------------ ETAPAS B-F: decisao
@dataclass
class Config:
    permitir_lateral: bool = True
    permitir_giro: bool = True
    permitir_re: bool = False          # BLOQUEADA ate validacao fisica (cobertura traseira + ICP)
    cobertura_lateral: float = 0.50
    cobertura_giro: float = 0.50
    cobertura_re: float = 0.90
    deriva_lateral: float = 0.40       # avanco por metro de deslize (06/11: 0,31-0,37; folga a mais)
    giro_max: float = 0.70             # limite do smart_spin
    re_max: float = 0.15
    sem_caminho_unico: bool = False    # SEM_CAMINHO_UNICO (13/11): espera de 2x2 s so na 1a chamada no lugar
    continuacao: float = 0.15          # o corpo precisa andar isto na direcao do caminho depois
    # ORIENTAR_CAMINHO (13/11): giro de orientacao para o caminho (False = como antes: so a varredura de 40 graus)
    giro_orientar: bool = False
    giro_orientar_max: float = 2 * math.pi      # giro acumulado maximo (pelo outro lado: 360 - |angulo|)
    giro_orientar_wz: float = 0.20              # rad/s (o DWB usa ate 0,20; o motor gira parado no minimo ~0,15)
    giro_orientar_tol: float = math.radians(20.0)   # sobra aceita no fim (o DWB acerta o resto andando)
    giro_orientar_reavaliar: int = 0            # v2 ORIENTAR_ROBUSTO: paradas p/ reavaliar o arco (0 = v1: aborta)
    giro_abertura: bool = False        # GIRO_ABERTURA_20261116: opt-in; caminho atual + continuacao comprovada


@dataclass
class Estado:
    raios: Raios
    ang_caminho: object                # rad em base_link ou None
    causas: list
    bloqueadas: set
    cm_ativo: bool = True
    exclusivo: bool = True
    planner_ok: bool = False
    apos_re: bool = False
    dist: float = 0.25
    quina: object = None


@dataclass
class Decisao:
    acao: str                          # retomar|contornar|lateral|mover|giro|afastar|re|falhar
    mov: tuple = (0.0, 0.0, 0.0)
    motivo: str = ''
    recusas: list = field(default_factory=list)
    extra: dict = field(default_factory=dict)


DWB_DIRECAO_MAX = math.radians(95)   # vx 0..0,20 e vy +-0,08: o DWB so anda de -90 a +90 graus


def _lateral_movs(dy, k):
    return [(0.0, dy, 0.0), (k * abs(dy), dy, 0.0)]


def _giro_abertura(es, cfg):
    """GIRO_ABERTURA_20261116: compara poses, sem aliviar margem ou cobertura."""
    if (not es.planner_ok or es.ang_caminho is None or not es.cm_ativo or not es.exclusivo
            or not cfg.permitir_giro or 'giro' in es.bloqueadas):
        return None
    pts, ang = es.raios.pontos(), es.ang_caminho
    if cm_barraria(pts, 'stopped'):
        return None
    f0 = folga_direcao(pts, math.cos(ang), math.sin(ang), alcance=0.50)
    opcoes = []
    for graus in (-40, -30, -20, -10, 10, 20, 30, 40):
        giro = math.radians(graus)
        novo_ang = ang_norm(ang - giro)
        if abs(giro) > cfg.giro_max + 1e-9 or abs(novo_ang) > DWB_DIRECAO_MAX:
            continue
        c, s = math.cos(giro), math.sin(giro)
        lx, ly, lt = es.raios.laser
        # A foto permanece a mesma; pontos e origem do laser mudam juntos de referencial.
        foto = replace(es.raios, angle_min=es.raios.angle_min - giro,
                       laser=(c * lx + s * ly, -s * lx + c * ly, lt))
        f1 = folga_direcao(foto.pontos(), math.cos(novo_ang), math.sin(novo_ang), alcance=0.50)
        if f1 < cfg.continuacao or f1 < f0 + 0.04:
            continue
        continuacao = avaliar(foto, [(cfg.continuacao * math.cos(novo_ang),
                                     cfg.continuacao * math.sin(novo_ang), 0.0)],
                              cobertura_min=cfg.cobertura_lateral)
        if not continuacao.livre:
            continue
        varredura = avaliar(es.raios, [(0.0, 0.0, giro)], margem=0.045,
                            cobertura_min=cfg.cobertura_giro)
        if varredura.livre:
            opcoes.append((f1, -abs(giro), giro, varredura.cobertura))
    if not opcoes:
        return None
    f1, _, giro, cobertura = max(opcoes)
    return {'giro_graus': math.degrees(giro), 'folga_antes_m': f0,
            'folga_depois_m': f1, 'cobertura': cobertura}


def decidir(es, cfg=None, plano_contornar=None):
    """Escolhe UMA manobra. plano_contornar: funcao(pts, ang) do destravar v3 (CONTORNAR), opcional."""
    cfg = cfg or Config()
    rec = []
    if not es.cm_ativo:
        return Decisao('falhar', motivo='Collision Monitor inativo: nenhuma manobra autorizada')
    if not es.exclusivo:
        return Decisao('falhar', motivo='outro no comandando /cmd_vel_nav: sem controle exclusivo')
    pts = es.raios.pontos()
    ang = es.ang_caminho
    ux = math.cos(ang) if ang is not None else 1.0
    uy = math.sin(ang) if ang is not None else 0.0

    def cont_livre(mov):
        """Depois de mov, o corpo anda cfg.continuacao na direcao do caminho (pelos pontos ja vistos)?"""
        if ang is None or abs(ang) > DWB_DIRECAO_MAX:
            return False, 0.0          # o DWB nao anda para tras (min_vel_x 0): continuacao impossivel
        p2 = deslocar(pts, mov)
        f = folga_direcao(p2, ux, uy, alcance=0.50)
        return f >= cfg.continuacao, f

    # ---- ETAPA B: o Nav2 ainda contorna sozinho?
    if 'retomar' in es.bloqueadas:
        rec.append(('retomar', 'ja devolvido ao Nav2 aqui e voltou a travar'))
    elif not es.planner_ok or ang is None:
        rec.append(('retomar', 'planner sem caminho'))
    elif {'oscilacao', 'sem_progresso_fisico', 'cm_bloqueou'} & set(es.causas):
        rec.append(('retomar', 'causa %s: o mesmo controle repetiria' % ','.join(
            sorted({'oscilacao', 'sem_progresso_fisico', 'cm_bloqueou'} & set(es.causas)))))
    elif abs(ang) > DWB_DIRECAO_MAX:
        rec.append(('retomar', 'caminho para tras (%+.0f graus): o DWB nao da re' % math.degrees(ang)))
    elif cm_barraria(pts, poligono_de((ux, uy, 0.0))):
        rec.append(('retomar', 'o Collision Monitor barraria o movimento na direcao do caminho'))
    else:
        f = folga_direcao(pts, ux, uy, alcance=0.50)
        av = avaliar(es.raios, [(cfg.continuacao * ux, cfg.continuacao * uy, 0.0)],
                     cobertura_min=cfg.cobertura_lateral)
        if f >= cfg.continuacao and av.livre:
            return Decisao('retomar', motivo='caminho novo e corpo passa %.0f cm na direcao dele' % (100 * f),
                           recusas=rec)
        rec.append(('retomar', 'corpo so anda %.0f cm na direcao do caminho%s' % (
            100 * f, '' if av.livre else ' (' + av.motivo + ')')))

    # ---- ETAPA C1: CONTORNAR (objeto pequeno na frente; codigo do destravar v3)
    if cfg.permitir_lateral and plano_contornar is not None and 'frente_barrada' in es.causas:
        if 'contornar' in es.bloqueadas:
            rec.append(('contornar', 'ja tentado aqui sem resultado'))
        else:
            pc = plano_contornar(pts, ang)
            if pc is None:
                rec.append(('contornar', 'nao e objeto pequeno com lado livre'))
            else:
                dy = pc['uy'] * pc['s']
                av = avaliar(es.raios, _lateral_movs(dy, cfg.deriva_lateral), cobertura_min=cfg.cobertura_lateral)
                barra = cm_barraria(pts, 'left' if dy > 0 else 'right')
                if barra and not cfg.permitir_re:
                    rec.append(('contornar', 'frente a < 5 cm: o CM barraria o deslize e a re de 4 cm esta bloqueada'))
                elif av.livre:
                    return Decisao('contornar', (0.0, dy, 0.0), 'objeto de y %+.0f a %+.0f cm: deslizo %.0f cm' % (
                        100 * pc['ymin'], 100 * pc['ymax'], 100 * pc['s']), rec, {'plano': pc})
                else:
                    rec.append(('contornar', av.motivo))

    # ---- ETAPA C2: escape lateral com continuacao util
    if not cfg.permitir_lateral:
        rec.append(('lateral', 'desligado (permitir_lateral=false)'))
    elif 'lateral' in es.bloqueadas:
        rec.append(('lateral', 'ja tentado aqui sem resultado'))
    else:
        if es.quina == 'esq':
            lados = (-1.0, 1.0)
        elif es.quina == 'dir':
            lados = (1.0, -1.0)
        else:
            lados = (1.0, -1.0) if (ang or 0.0) >= 0 else (-1.0, 1.0)
        opcoes, motivos = [], []
        for lado in lados:
            for d in sorted({d for d in (0.40, 0.30, 0.25) if d <= es.dist + 1e-9} | {0.15, 0.10}, reverse=True):
                dy = lado * d
                if cm_barraria(pts, 'left' if lado > 0 else 'right'):
                    motivos.append('%s: o Collision Monitor barraria (obstaculo a < 5 cm do corpo)' % (
                        'esq' if lado > 0 else 'dir'))
                    break
                av = avaliar(es.raios, _lateral_movs(dy, cfg.deriva_lateral), cobertura_min=cfg.cobertura_lateral)
                if not av.livre:
                    motivos.append('%s %.0f cm: %s' % ('esq' if lado > 0 else 'dir', 100 * d, av.motivo))
                    continue
                ok, f = cont_livre((cfg.deriva_lateral * abs(dy), dy, 0.0))   # onde o robo para de verdade
                if not ok:
                    motivos.append('%s %.0f cm: depois o caminho so anda %.0f cm' % (
                        'esq' if lado > 0 else 'dir', 100 * d, 100 * f))
                    continue
                opcoes.append((round(f / 0.02), -abs(dy), lado == lados[0], dy, f, av))
                break
        if opcoes:
            opcoes.sort(key=lambda o: (o[0], o[2], o[1]), reverse=True)
            _, _, _, dy, f, av = opcoes[0]
            return Decisao('lateral', (0.0, dy, 0.0), '%.0f cm para %s; depois o corpo anda %.0f cm no caminho '
                           '(area %.0f%% observada)' % (100 * abs(dy), 'esq' if dy > 0 else 'dir', 100 * f,
                                                        100 * av.cobertura), rec)
        rec.append(('lateral', '; '.join(motivos[:4]) or 'sem lado livre'))

    # ---- mover na direcao do caminho (mecanum), sem re
    if ang is not None and abs(ang) <= DWB_DIRECAO_MAX and 'mover' not in es.bloqueadas:
        for d in (0.15, 0.10):
            if cm_barraria(pts, poligono_de((ux, uy, 0.0))):
                break
            av = avaliar(es.raios, [(d * ux, d * uy, 0.0)], cobertura_min=cfg.cobertura_lateral)
            if av.livre and folga_direcao(pts, ux, uy, alcance=0.60) >= d + cfg.continuacao:
                return Decisao('mover', (d * ux, d * uy, 0.0), '%.0f cm na direcao do caminho (%+.0f graus)' % (
                    100 * d, math.degrees(ang)), rec)
        rec.append(('mover', 'corpo nao passa 10 cm + continuacao na direcao do caminho'))
    elif 'mover' in es.bloqueadas:
        rec.append(('mover', 'ja tentado aqui sem resultado'))

    # GIRO_ABERTURA_20261116: apos falharem as translacoes, antes da recusa dos 45 graus.
    if cfg.giro_abertura:
        abertura = _giro_abertura(es, cfg)
        if abertura is not None:
            g = math.radians(abertura['giro_graus'])
            return Decisao('giro', (0.0, 0.0, g),
                           'abertura %+.0f graus: continuacao no caminho de %.0f para %.0f cm; area %.0f%% observada' % (
                               abertura['giro_graus'], 100 * abertura['folga_antes_m'],
                               100 * abertura['folga_depois_m'], 100 * abertura['cobertura']),
                           rec, {'abertura': abertura, 'ang_caminho': ang})

    # ---- ORIENTAR_CAMINHO (13/11): giro de ORIENTACAO (ficar de frente para o caminho). A varredura do smart_spin
    # (abaixo) para na 1a abertura frontal e no maximo 40 graus: no bag de 13/11 11:41 girou 7 de 162 graus.
    # So com caminho ATUAL do planner (skill robo-work-nav-melhoria: nada de giro por rota nao validada).
    if cfg.giro_orientar and cfg.permitir_giro and ang is not None and abs(ang) >= math.radians(45):
        if not es.planner_ok:
            rec.append(('orientar', 'planner sem caminho agora: nao giro por direcao antiga'))
        elif 'orientar' in es.bloqueadas:
            rec.append(('orientar', 'ja tentei orientar aqui sem resultado'))
        elif cm_barraria(pts, 'stopped'):
            rec.append(('orientar', 'o Collision Monitor barraria (ponto a < 3 cm do corpo)'))
        else:
            mot = []
            longo = ang - math.copysign(2 * math.pi, ang)
            for g, lado in ((ang, 'pelo lado curto'), (longo, 'pelo outro lado')):
                if abs(g) > cfg.giro_orientar_max + 1e-9:
                    mot.append('%+.0f graus %s: acima do limite de %.0f' % (
                        math.degrees(g), lado, math.degrees(cfg.giro_orientar_max)))
                    continue
                av = avaliar(es.raios, [(0.0, 0.0, g)], margem=0.045, cobertura_min=cfg.cobertura_giro)
                if av.livre:
                    return Decisao('orientar', (0.0, 0.0, g), 'gira %+.0f graus %s para ficar de frente para o '
                                   'caminho (%+.0f); area %.0f%% observada' % (
                                       math.degrees(g), lado, math.degrees(ang), 100 * av.cobertura), rec)
                mot.append('%+.0f graus %s: %s' % (math.degrees(g), lado, av.motivo))
            rec.append(('orientar', '; '.join(mot)))

    # ---- ETAPA F: giro (pelo smart_spin)
    if not cfg.permitir_giro:
        rec.append(('giro', 'desligado (permitir_giro=false)'))
    elif 'giro' in es.bloqueadas:
        rec.append(('giro', 'ja girou aqui sem resultado'))
    elif ang is None or abs(ang) < math.radians(45):
        rec.append(('giro', 'caminho a menos de 45 graus: giro nao ajuda'))
    else:
        g = max(-cfg.giro_max, min(cfg.giro_max, ang))
        av = avaliar(es.raios, [(0.0, 0.0, g)], margem=0.045, cobertura_min=cfg.cobertura_giro)
        if cm_barraria(pts, 'stopped'):
            rec.append(('giro', 'o Collision Monitor barraria (ponto a < 3 cm do corpo)'))
        elif av.livre:
            return Decisao('giro', (0.0, 0.0, g), 'gira %+.0f graus para o caminho (%+.0f); area %.0f%% observada' % (
                math.degrees(g), math.degrees(ang), 100 * av.cobertura), rec)
        else:
            rec.append(('giro', av.motivo))

    # ---- AFASTAR (v2/v3, 10/11): sem direcao de caminho e encostado: abre folga do lado mais livre
    if ang is None:
        f0 = folga_corpo(pts)
        if 'afastar' in es.bloqueadas:
            rec.append(('afastar', 'ja tentado aqui sem resultado'))
        elif not cfg.permitir_lateral:
            rec.append(('afastar', 'desligado (permitir_lateral=false)'))
        elif f0 >= 0.08:
            rec.append(('afastar', 'corpo a %.0f cm do mais perto: nao esta encostado' % (100 * f0)))
        else:
            dirs = [('frente', 1.0, 0.0), ('esq', 0.0, 1.0), ('dir', 0.0, -1.0)]
            if cfg.permitir_re:
                dirs.append(('tras', -1.0, 0.0))
            ops, mot = [], []
            for nome, dx, dy in dirs:
                if cm_barraria(pts, poligono_de((dx, dy, 0.0))):
                    mot.append('%s: CM barraria' % nome)
                    continue
                for d in (0.10, 0.06):
                    k = cfg.deriva_lateral if dy else 0.0
                    movs = [(d * dx, d * dy, 0.0)] + ([(k * d, d * dy, 0.0)] if dy else [])
                    av = avaliar(es.raios, movs, cobertura_min=cfg.cobertura_re if dx < 0 else cfg.cobertura_lateral)
                    if not av.livre:
                        mot.append('%s %.0f cm: %s' % (nome, 100 * d, av.motivo))
                        continue
                    f1 = min(folga_corpo(deslocar(pts, m)) for m in movs)
                    if f1 >= f0 + 0.03:
                        ops.append((f1, nome, (d * dx, d * dy, 0.0)))
                        break
                    mot.append('%s %.0f cm: folga so iria a %.0f cm' % (nome, 100 * d, 100 * f1))
            if ops:
                f1, nome, mov = max(ops)
                return Decisao('afastar', mov, 'encostado (%.0f cm): %.0f cm para %s, folga vai a %.0f cm' % (
                    100 * f0, 100 * math.hypot(mov[0], mov[1]), nome, 100 * f1), rec)
            rec.append(('afastar', '; '.join(mot[:4]) or 'nenhum lado abre folga'))

    # ---- ETAPA D: re estrategica (intermediaria)
    if es.apos_re:
        rec.append(('re', 'acabou de dar re: nunca duas seguidas'))
    elif not cfg.permitir_re:
        rec.append(('re', 'BLOQUEADA: permitir_re=false (falta validacao fisica da cobertura traseira e do ICP)'))
    elif 're' in es.bloqueadas:
        rec.append(('re', 'ja tentado aqui sem resultado'))
    else:
        mot = []
        for d in (cfg.re_max, 0.10):
            if cm_barraria(pts, 'backward'):
                mot.append('o Collision Monitor barraria')
                break
            av = avaliar(es.raios, [(-d, 0.0, 0.0)], cobertura_min=cfg.cobertura_re)
            if not av.livre:
                mot.append('%.0f cm: %s' % (100 * d, av.motivo))
                continue
            ver, why = icp_verificavel(pts, -1.0, 0.0, d)
            if not ver:
                mot.append('%.0f cm: deslocamento nao verificavel (%s)' % (100 * d, why))
                continue
            # util? (estimativa com os pontos atuais; a autorizacao da manobra seguinte usa foto NOVA)
            p2 = deslocar(pts, (-d, 0.0, 0.0))
            util = (folga_direcao(p2, ux, uy) >= cfg.continuacao if ang is not None and
                    abs(ang) <= DWB_DIRECAO_MAX else False) or \
                folga_direcao(p2, 0.0, 1.0) >= 0.10 or folga_direcao(p2, 0.0, -1.0) >= 0.10
            if not util:
                mot.append('%.0f cm: nao abre espaco util' % (100 * d))
                continue
            return Decisao('re', (-d, 0.0, 0.0), 're %.0f cm para abrir espaco (area %.0f%% observada); '
                           'depois reavalia' % (100 * d, 100 * av.cobertura), rec)
        rec.append(('re', '; '.join(mot)))

    return Decisao('falhar', motivo='sem alternativa segura', recusas=rec)


def resumo_recusas(rec):
    return ' | '.join('%s: %s' % r for r in rec)


# ------------------------------------------------------------------ ETAPA E: verificacao
def verificar_translacao(med, mov, frac_min=0.40, minimo=0.03):
    """A medida do ICP confirma o movimento pedido? (a odom de lado superestima: aceita >= 40%)."""
    d = math.hypot(mov[0], mov[1])
    if not med.ok or d < 1e-6:
        return False, med.motivo or 'sem medida'
    proj = (med.dx * mov[0] + med.dy * mov[1]) / d
    if proj >= max(minimo, frac_min * d):
        return True, 'ICP: %.0f cm na direcao pedida (dx %+.0f, dy %+.0f cm)' % (
            100 * proj, 100 * med.dx, 100 * med.dy)
    return False, 'ICP: so %.0f cm na direcao pedida' % (100 * proj)


# ------------------------------------------------------------------ executor (mesmo codigo no no ROS e na simulacao)
class Recuperador:
    """Uma chamada do destravar = diagnostico + ate max_manobras manobras verificadas.

    io: interface do robo (ver destravar_node.IoRos e test/sim_recuperacao.SimIO):
      agora() dormir(s) scan()->Raios odom()->(t,x,y,yaw_imu) pose_ref()->(x,y)|None goal()->(x,y)|None
      enviar(vx,vy,wz) parar() cancelado() cm_stop() cm_ativo() ultimo_estrangeiro() ultimo_ctrl()
      replanejar()->True|False|None soltar_bloqueio() marcar_frente() caminho_valido()->True|False|None
      direcao_caminho()->rad|None janela()->dict smart_spin(ang, tmax)->(ok, msg) log(nivel, txt) estado(dict)
    geo: modulo com as funcoes do destravar v3 (plano_contornar, barrando, translacao_livre, folga, FOOTPRINT,
      MARGEM_LADO, AVANCO).
    """

    def __init__(self, io, geo, cfg=None, hist=None, max_manobras=2):
        self.io, self.geo = io, geo
        self.cfg = cfg or Config()
        self.hist = hist or Historico()
        self.max_manobras = max_manobras

    # ---------------- primitivas
    def _sensores_ok(self):
        sc, od = self.io.scan(), self.io.odom()
        agora = self.io.agora()
        return sc is not None and od is not None and agora - sc.t <= 0.5 and agora - od[0] <= 0.5

    def _exclusivo(self):
        agora = self.io.agora()
        e, c = self.io.ultimo_estrangeiro(), self.io.ultimo_ctrl()
        return (e is None or agora - e > 1.0) and (c is None or agora - c > 0.5)

    def _garantir_exclusivo(self, t_fim, espera=2.0):
        """CONTROLADOR_ORFAO (11/11): controlador ainda comandando quando a recuperacao comeca = FollowPath orfao
        (a arvore so chama a recuperacao com o FollowPath parado). Cancela o FollowPath e espera a exclusividade
        parado; nunca anda nem ignora o outro comando. Devolve se ficou exclusivo."""
        if self._exclusivo():
            return True
        io = self.io
        c = io.ultimo_ctrl()
        if c is not None and io.agora() - c <= 0.5 and hasattr(io, 'cancelar_controlador'):
            aceito = io.cancelar_controlador()
            io.log('warn', 'CONTROLADOR_ORFAO: o controlador ainda comandava com a recuperacao chamada; '
                           'pedi cancelar o FollowPath (%s)' % ('aceito' if aceito else 'sem resposta'))
        t1 = min(t_fim - 1.0, io.agora() + espera)
        t_ini = io.agora()
        while io.agora() < t1:
            if io.cancelado():
                return False
            if self._exclusivo():
                io.log('info', 'CONTROLADOR_ORFAO: comando exclusivo depois de %.1f s' % (io.agora() - t_ini))
                return True
            io.dormir(0.1)
        ok = self._exclusivo()
        if not ok:
            io.log('warn', 'CONTROLADOR_ORFAO: outro no continua comandando depois de %.1f s' % (io.agora() - t_ini))
        return ok

    def foto(self, t_fim):
        """Para e espera um scan novo com o robo parado (>= 0,4 s depois de parar)."""
        self.io.parar()
        t_stop = self.io.agora()
        while self.io.agora() < min(t_fim, t_stop + 2.0):
            self.io.parar()
            self.io.dormir(0.05)
            sc = self.io.scan()
            if sc is not None and sc.t >= t_stop + 0.4 and self.io.agora() - t_stop >= 0.4:
                return sc
        return None

    def _delta_odom(self, o0, o1):
        c, s = math.cos(o0[3]), math.sin(o0[3])
        dx, dy = o1[1] - o0[1], o1[2] - o0[2]
        return (c * dx + s * dy, -s * dx + c * dy, ang_norm(o1[3] - o0[3]))

    def transladar(self, ux, uy, d, v, t_fim, estrito=False, deriva=0.0):
        """Anda ate d m (criterio de parada pela odom, que superestima de lado: anda MENOS, nunca mais).
        Para por: cancelamento, tempo, sensor parado, CM (re: na hora), comando estrangeiro, LiDAR."""
        o0 = self.io.odom()
        t_mov = min(t_fim, self.io.agora() + 1.5 + d / 0.04)
        cm_desde, motivo = None, 'chegou'
        while True:
            agora = self.io.agora()
            if self.io.cancelado():
                motivo = 'cancelado'
                break
            if agora >= t_mov:
                motivo = 'tempo'
                break
            if not self._sensores_ok():
                motivo = 'sensor parado'
                break
            e = self.io.ultimo_estrangeiro()
            if e is not None and agora - e < 0.3:
                motivo = 'comando estrangeiro em /cmd_vel_nav'
                break
            c = self.io.ultimo_ctrl()
            if c is not None and agora - c < 0.3:
                motivo = 'controlador comandando'
                break
            if self.io.cm_stop():
                cm_desde = cm_desde if cm_desde is not None else agora
                if estrito or agora - cm_desde > 0.5:
                    motivo = 'Collision Monitor parou'
                    break
            else:
                cm_desde = None
            od = self.io.odom()
            bx, by, _ = self._delta_odom(o0, od)
            if bx * ux + by * uy >= d - 0.01:
                break
            pts = self.io.scan().pontos()
            dirs = [(ux, uy)]
            if deriva:
                n = math.hypot(deriva, 1.0)
                dirs.append(((ux + deriva) / n, uy / n))
            if any(folga_direcao(pts, a, b, margem=0.02, alcance=0.08) < 0.04 for a, b in dirs):
                motivo = 'LiDAR: obstaculo a menos de 4 cm na direcao'
                break
            self.io.enviar(v * ux, v * uy, 0.0)
            self.io.dormir(0.05)
        self.io.parar()
        od = self.io.odom()
        return motivo, self._delta_odom(o0, od)

    def contornar(self, pl, t_fim):
        """CONTORNAR do destravar v3 (mesma logica): desliza ate a faixa da frente ficar livre e avanca."""
        g = self.geo
        uy = pl['uy']
        y1 = g.FOOTPRINT[2]
        sc = self.io.scan()
        if g.folga(sc.pontos(), 'frente') < 0.04 and g.translacao_livre(sc.pontos(), -1.0, 0.0, 0.05) \
                and self.cfg.permitir_re:
            self.transladar(-1.0, 0.0, 0.04, 0.05, t_fim, estrito=True)
        o0 = self.io.odom()
        lat, livres, ult, cm_desde, venci, motivo = 0.0, 0, None, None, False, ''
        while self.io.agora() < t_fim:
            if self.io.cancelado():
                motivo = 'cancelado'
                break
            if not self._sensores_ok():
                motivo = 'sensor parado'
                break
            e = self.io.ultimo_estrangeiro()
            if e is not None and self.io.agora() - e < 0.3:
                motivo = 'comando estrangeiro'
                break
            sc, od = self.io.scan(), self.io.odom()
            lat = self._delta_odom(o0, od)[1] * uy
            if sc.t != ult:
                ult = sc.t
                livres = 0 if g.barrando(sc.pontos(), y1 + g.MARGEM_LADO) else livres + 1
            if livres >= 2 and lat >= 0.5 * pl['s']:
                venci = True
                break
            pts = sc.pontos()
            lado = [p for p in pts if p[1] * uy > 0]
            if lat >= pl['s'] + 0.10 or not g.translacao_livre(lado, 0.0, uy, 0.04):
                motivo = 'lado fechou'
                break
            # RECUPERACAO_AVANCADA: o deslize tambem avanca (deriva): a frente tem de continuar livre
            if folga_direcao(pts, 1.0, 0.0, margem=0.0, alcance=0.06) < 0.02:
                motivo = 'deriva levaria a frente ao obstaculo'
                break
            if self.io.cm_stop():
                cm_desde = cm_desde if cm_desde is not None else self.io.agora()
                if self.io.agora() - cm_desde > 0.5:
                    motivo = 'Collision Monitor parou'
                    break
            else:
                cm_desde = None
            self.io.enviar(0.0, 0.08 * uy, 0.0)
            self.io.dormir(0.05)
        self.io.parar()
        if not venci:
            return False, 'nao venci depois de %.0f cm (%s)' % (100 * lat, motivo or 'tempo')
        motivo, dl = self.transladar(1.0, 0.0, g.AVANCO, 0.10, t_fim)
        return True, 'venci: %.0f cm de lado (odom) e %.0f cm para frente (%s)' % (100 * lat, 100 * dl[0], motivo)

    def girar_orientar(self, g, t_fim):
        """ORIENTAR_CAMINHO (13/11): gira g rad parado (sentido fixo), medindo pelo yaw da IMU ACUMULADO.
        A cada scan novo confere o arco dos proximos 30 graus (corpo + 3 cm) pelo LiDAR. Para por: chegou (falta
        < 3 graus), cancelamento, tempo, sensor parado, comando estrangeiro, controlador comandando, Collision
        Monitor (na hora), LiDAR no arco seguinte ou IMU sem progresso por 2 s. -> (motivo, girou_rad)"""
        io, cfg = self.io, self.cfg
        sentido = 1.0 if g > 0 else -1.0
        yaw_ant = io.odom()[3]
        girou, ult_scan, motivo = 0.0, None, 'chegou'
        reav = 0                           # v2 ORIENTAR_ROBUSTO: paradas para reavaliar o arco
        wz = 0.15
        t_ini = io.agora()
        t_prog = t_ini
        t_lim = min(t_ini + 3.0 + abs(g) / 0.10, t_fim + 10.0)
        while True:
            agora = io.agora()
            od = io.odom()
            d = ang_norm(od[3] - yaw_ant)
            yaw_ant = od[3]
            girou += d
            if abs(d) > 0.003:
                t_prog = agora
            falta = g - girou
            if falta * sentido <= math.radians(3.0):
                break
            if io.cancelado():
                motivo = 'cancelado'
                break
            if agora >= t_lim:
                motivo = 'tempo'
                break
            if not self._sensores_ok():
                motivo = 'sensor parado'
                break
            e = io.ultimo_estrangeiro()
            if e is not None and agora - e < 0.3:
                motivo = 'comando estrangeiro em /cmd_vel_nav'
                break
            c = io.ultimo_ctrl()
            if c is not None and agora - c < 0.3:
                motivo = 'controlador comandando'
                break
            if io.cm_stop():
                motivo = 'Collision Monitor parou'
                break
            if agora - t_prog > 2.0 and agora - t_ini > 2.0:
                motivo = 'IMU sem progresso angular (giro nao comecou ou travou)'
                break
            sc = io.scan()
            if sc.t != ult_scan:
                ult_scan = sc.t
                prox = sentido * min(abs(falta), math.radians(30.0))
                av = avaliar(sc, [(0.0, 0.0, prox)], margem=0.03)
                if not av.livre and reav < cfg.giro_orientar_reavaliar:
                    # v2 ORIENTAR_ROBUSTO (teste 13/11 13:17:34): 1 ponto num scan tirado GIRANDO derrubou o giro
                    # com 6 de 164 graus, embora a foto parada desse a area livre. Agora: para, foto PARADA e
                    # reavalia o arco que FALTA com o mesmo criterio do inicio. O CM continua parando na hora.
                    reav += 1
                    io.parar()
                    foto = self.foto(t_lim)
                    od = io.odom()
                    girou += ang_norm(od[3] - yaw_ant)
                    yaw_ant = od[3]
                    falta = g - girou
                    if foto is None:
                        motivo = 'LiDAR: %s no arco seguinte (sem foto parada para reavaliar)' % av.motivo
                        break
                    av2 = avaliar(foto, [(0.0, 0.0, falta)], margem=0.045, cobertura_min=cfg.cobertura_giro)
                    io.log('info' if av2.livre else 'warn',
                           'ORIENTAR_ROBUSTO %d: girando vi %s; parado, o arco que falta (%+.0f graus) %s' % (
                               reav, av.motivo, math.degrees(falta),
                               'esta livre: continuo' if av2.livre else 'NAO esta livre (%s): paro' % av2.motivo))
                    if not av2.livre:
                        motivo = 'LiDAR (parado): %s no arco que falta' % av2.motivo
                        break
                    ult_scan, t_prog, t_ini = foto.t, io.agora(), io.agora()
                    continue
                if not av.livre:
                    motivo = 'LiDAR: %s no arco seguinte' % av.motivo
                    break
                folgado = avaliar(sc, [(0.0, 0.0, prox)], margem=0.07).livre
                wz = cfg.giro_orientar_wz if folgado and abs(falta) > math.radians(15.0) else 0.15
            io.enviar(0.0, 0.0, sentido * wz)
            io.dormir(0.05)
        io.parar()
        io.dormir(0.3)                     # o motor ainda gira um pouco depois do zero
        girou += ang_norm(io.odom()[3] - yaw_ant)
        return motivo, girou

    # ---------------- a chamada
    def executar(self, dist=0.25, vcmd=0.03, tmax=15.0, chamador_oscilou=False):
        """-> (ok, mensagem, relatorio)"""
        io, cfg = self.io, self.cfg
        t0 = io.agora()
        t_fim = t0 + tmax
        rel = {'causas': [], 'decisoes': [], 'resultado': ''}

        def fim(ok, msg):
            io.parar()
            rel['resultado'] = ('OK: ' if ok else 'FALHA: ') + msg
            io.estado(rel)
            io.log('info' if ok else 'warn', rel['resultado'])
            return ok, msg, rel

        if not self._sensores_ok():
            return fim(False, 'sem scan ou odom recente')
        self.hist.novo_goal(io.goal())
        ref = io.pose_ref() or io.odom()[1:3]
        retornos = self.hist.nova_chamada(t0, ref[0], ref[1])
        rel['retornos'] = retornos
        if retornos >= self.hist.max_retornos:
            return fim(False, 'mesmo bloqueio pela %da vez: alternativas esgotadas' % (retornos + 1))
        if self.hist.total() >= self.hist.max_acoes:
            return fim(False, '%d manobras neste goal: desisto' % self.hist.total())

        self._garantir_exclusivo(t_fim)   # CONTROLADOR_ORFAO (11/11): a decisao continua exigindo exclusividade
        # ---- ETAPA A
        jan = io.janela()
        planner = io.replanejar()
        tent = 0
        # SEM_CAMINHO_UNICO (13/11): voltando ao mesmo lugar, a espera ja foi feita e nao trouxe caminho
        tent_max = 0 if (cfg.sem_caminho_unico and retornos >= 1) else 2
        if planner is False and tent_max == 0:
            io.log('info', 'SEM CAMINHO de novo no mesmo lugar (%da chamada aqui): nao repito a espera de '
                           '2 x 2 s; vou direto as manobras' % (retornos + 1))
        while planner is False and tent < tent_max and io.agora() < t_fim - 6.0:
            # BLOQUEIO_LOCAL do v3 + SEM_CAMINHO (10/11): no bag de 10/11 a passagem fechou por paredes
            # fantasmas (SLAM saltando); limpa, espera PARADO a localizacao assentar e o costmap remarcar
            io.soltar_bloqueio()
            io.parar()
            io.log('info', 'SEM CAMINHO: limpei os costmaps; espero 2 s parado e planejo de novo (%d/2)' % (tent + 1))
            io.dormir(2.0)
            planner = io.replanejar()
            tent += 1
        foto0 = self.foto(t_fim)
        if foto0 is None:
            return fim(False, 'sem scan novo com o robo parado')
        pts0 = foto0.pontos()
        ev = Evidencias(planner={True: 'ok', False: 'sem_caminho', None: 'indisponivel'}[planner],
                        caminho_valido=io.caminho_valido(), frente=folga_frente(pts0),
                        quina=detectar_quina(pts0), chamador_oscilou=chamador_oscilou, **jan)
        causas = diagnosticar(ev)
        rel['causas'] = causas
        io.log('info', 'DIAGNOSTICO: %s (planner %s, frente %s, retornos %d)' % (
            ','.join(causas) or 'nada evidente', ev.planner,
            '-' if ev.frente >= 9 else '%.0f cm' % (100 * ev.frente), retornos))
        marcou = False
        feitos, apos_re, manobras = [], False, 0
        while manobras < self.max_manobras + (1 if apos_re else 0):
            if io.cancelado():
                return fim(False, 'cancelado')
            if io.agora() > t_fim - 1.0:
                break
            es = Estado(raios=foto0, ang_caminho=io.direcao_caminho(), causas=causas,
                        bloqueadas=self.hist.bloqueadas(io.agora(), ref[0], ref[1]),
                        cm_ativo=io.cm_ativo(), exclusivo=self._exclusivo(),
                        planner_ok=planner is True, apos_re=apos_re, dist=dist, quina=ev.quina)
            dec = decidir(es, cfg, getattr(self.geo, 'plano_contornar', None))
            rel['decisoes'].append({'acao': dec.acao, 'motivo': dec.motivo,
                                    'recusas': ['%s: %s' % r for r in dec.recusas]})
            io.log('info', 'DECISAO: %s (%s) | recusadas: %s' % (dec.acao, dec.motivo, resumo_recusas(dec.recusas) or '-'))
            io.estado(rel)
            if dec.acao == 'falhar':
                if feitos:
                    return fim(True, ' + '.join(feitos) + '; sem 2a manobra segura: devolvo ao Nav2 para replanejar')
                return fim(False, '%s (%s)' % (dec.motivo, resumo_recusas(dec.recusas)))
            if dec.acao == 'retomar':
                self.hist.registrar(io.agora(), ref[0], ref[1], 'retomar', 'ok', dec.motivo)
                feitos.append('contorno normal: ' + dec.motivo)
                return fim(True, ' + '.join(feitos))
            if 'frente_barrada' in causas and planner is not False and not marcou and dec.acao != 'contornar':
                io.marcar_frente()      # BLOQUEIO_LOCAL do v3 (so no costmap local, 10 s)
                marcou = True
            o0 = io.odom()
            manobras += 1
            if dec.acao == 'orientar':      # ORIENTAR_CAMINHO (13/11): o proprio destravar gira
                motivo, self._orientar_girou = self.girar_orientar(dec.mov[2], t_fim)
                ok_mov = motivo == 'chegou'
                motivo = 'orientar: ' + motivo
            elif dec.acao == 'giro':
                ok_mov, txt_mov = io.smart_spin(dec.mov[2], max(3.0, min(12.0, t_fim - io.agora() - 2.0)))
                motivo = 'smart_spin: ' + txt_mov
            elif dec.acao == 'contornar':
                ok_mov, motivo = self.contornar(dec.extra['plano'], t_fim)
            else:
                d = math.hypot(dec.mov[0], dec.mov[1])
                ux, uy = dec.mov[0] / d, dec.mov[1] / d
                re_ = dec.mov[0] < -1e-6           # re (ou afastar para tras): CM para na hora
                v = vcmd if not re_ else min(vcmd, 0.05)
                motivo, _ = self.transladar(ux, uy, d, v, t_fim, estrito=re_,
                                            deriva=cfg.deriva_lateral if abs(uy) > 0.7 else 0.0)
                ok_mov = motivo in ('chegou', 'tempo', 'LiDAR: obstaculo a menos de 4 cm na direcao')
            if io.cancelado():
                self.hist.registrar(io.agora(), ref[0], ref[1], dec.acao, 'falhou', 'cancelado')
                return fim(False, 'cancelado durante %s' % dec.acao)
            foto1 = self.foto(t_fim + 2.0 if dec.acao != 'orientar' else max(t_fim, io.agora()) + 2.0)
            o1 = io.odom()
            if foto1 is None:
                self.hist.registrar(io.agora(), ref[0], ref[1], dec.acao, 'nao_verificado', 'sem foto')
                return fim(False, '%s sem foto depois: resultado nao verificado' % dec.acao)
            # ---- verificacao com evidencia independente da odometria
            if dec.acao == 'orientar':
                girou = self._orientar_girou
                falta = dec.mov[2] - girou
                ok = girou * dec.mov[2] > 0 and abs(falta) <= cfg.giro_orientar_tol
                evid = 'IMU: girou %+.0f de %+.0f graus (falta %+.0f)' % (
                    math.degrees(girou), math.degrees(dec.mov[2]), math.degrees(falta))
            elif dec.acao == 'giro':
                dyaw = ang_norm(o1[3] - o0[3])
                ok = ok_mov and abs(dyaw) >= 0.08 and dyaw * dec.mov[2] > 0
                evid = 'IMU: girou %+.0f graus' % math.degrees(dyaw)
                if 'abertura' in dec.extra:
                    a1 = ang_norm(dec.extra['ang_caminho'] - dyaw)
                    f1 = folga_direcao(foto1.pontos(), math.cos(a1), math.sin(a1), alcance=0.50)
                    av1 = avaliar(foto1, [(cfg.continuacao * math.cos(a1),
                                          cfg.continuacao * math.sin(a1), 0.0)],
                                  cobertura_min=cfg.cobertura_lateral)
                    ok = (ok and f1 >= cfg.continuacao and av1.livre and
                          f1 >= dec.extra['abertura']['folga_antes_m'] + 0.04)
                    evid += '; abertura medida: %.0f cm, continuacao %s' % (100 * f1, 'livre' if av1.livre else av1.motivo)

            else:
                med = medir_deslocamento(pts0, foto1.pontos(), self._delta_odom(o0, o1))
                mov_real = dec.mov
                if dec.acao == 'contornar':
                    mov_real = (0.0, dec.mov[1], 0.0)
                ok, evid = verificar_translacao(med, mov_real)
                if not ok and dec.acao == 'afastar':
                    f0, f1 = folga_corpo(pts0), folga_corpo(foto1.pontos())
                    if ok_mov and f1 >= f0 + 0.02:
                        ok, evid = True, 'LiDAR: folga do corpo de %.0f para %.0f cm (%s)' % (100 * f0, 100 * f1, evid)
                if not ok and dec.acao in ('contornar', 'lateral', 'mover'):
                    f0, f1 = folga_frente(pts0), folga_frente(foto1.pontos())
                    if dec.acao == 'contornar' and ok_mov and f1 >= 0.15:
                        ok, evid = True, 'LiDAR: faixa da frente livre (%s)' % evid
                    elif f0 < 0.15 <= f1:
                        ok, evid = True, 'LiDAR: frente abriu de %.0f para %.0f cm (%s)' % (100 * f0, 100 * f1, evid)
                ok = ok and ok_mov
            res = 'ok' if ok else ('falhou' if not ok_mov else 'nao_verificado')
            self.hist.registrar(io.agora(), ref[0], ref[1], dec.acao, res, '%s | %s' % (motivo, evid))
            io.log('info' if ok else 'warn', 'RESULTADO %s: %s | %s | %s' % (dec.acao, res, motivo, evid))
            foto0, pts0 = foto1, foto1.pontos()
            if ok:
                feitos.append('%s (%s; %s)' % (dec.acao, dec.motivo, evid))
                if dec.acao != 're':
                    return fim(True, ' + '.join(feitos))
                # ---- ETAPA E: depois da re, NOVA avaliacao (foto nova, planner novo) antes de qualquer outra coisa
                apos_re = True
                planner = io.replanejar()
                ev.frente, ev.quina = folga_frente(pts0), detectar_quina(pts0)
                causas = [c for c in diagnosticar(ev) if c not in ('sem_progresso_fisico', 'oscilacao',
                                                                  'cm_bloqueou', 'dwb_sem_trajetoria')]
                io.log('info', 'REAVALIACAO depois da re: frente %.0f cm, planner %s' % (
                    100 * min(ev.frente, 9), planner))
            elif apos_re:
                break
        if feitos:
            return fim(True, ' + '.join(feitos) + '; devolvo ao Nav2 para replanejar')
        return fim(False, 'nenhuma manobra confirmada (%s)' % '; '.join(
            '%s=%s' % (i.acao, i.resultado) for i in self.hist.itens[-self.max_manobras:]))


if __name__ == '__main__':  # pragma: no cover
    print(__doc__)
    print(time.strftime('%H:%M:%S'))
