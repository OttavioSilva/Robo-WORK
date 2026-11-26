"""ROTA_RETAS (25/11/2026): rota so de retas nos eixos do mapa para o robo mecanum.

Ideia do operador (teste real na arena Fusion, 25/11): em vez da curva do Nav2 (que derivou para a Shelf e
se perdeu), o robo anda RETO (frente/re) e DE LADO, com o rumo fixo, e so gira parado onde o giro cabe:
no inicio (para acertar o rumo) ou no fim (angulo da mesa; ex.: Shelf a 45 graus = reto, lateral, gira 45).

plano_retas(mapa, inicio, goal) -> lista de passos ou None:
  ('girar', yaw)            gira parado ate o yaw
  ('mover', x, y)           anda reto ate (x, y) sem girar (frente/re ou lateral, conforme o rumo)
Cada reta e conferida no mapa com o corpo inteiro (+margem) no espaco de configuracao do rumo; o giro
exige o circulo do corpo livre. No maximo 3 retas (reta, L ou Z). Sem rota -> None (o chamador usa o Nav2).
Funcoes puras (numpy), testaveis sem ROS.
"""
import math

import numpy as np

CORPO = (-0.215, 0.375, -0.325, 0.325)   # x0, x1, y0, y1 no base_link (medido)
R_GIRO = math.hypot(0.375, 0.325)        # 0,496 m: quina mais longe do centro
CUSTO_LADO = 1.8                         # lateral ~0,6x a frente
CUSTO_GIRO = 0.5                         # m equivalentes por giro
CUSTO_RE = 1.3                           # re: anda igual, mas prefere ver a frente


def _ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def _snap90(a):
    return _ang(round(a / (math.pi / 2)) * (math.pi / 2))


class Grade:
    """Ocupacao do mapa (desconhecido = ocupado) e espacos de configuracao por rumo de 90 graus."""

    def __init__(self, mapa, margem=0.04):
        if abs(mapa.s) > 1e-6 or mapa.c < 0:
            raise ValueError('mapa girado: ROTA_RETAS so com origem sem yaw')
        self.m, self.res, self.margem = mapa, mapa.res, margem
        d = np.asarray(mapa.dados, dtype=np.int16).reshape(mapa.h, mapa.w)
        self.occ = ((d < 0) | (d >= mapa.limiar)).astype(np.int32)
        self.ii = np.pad(self.occ, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
        self._cs = {}
        self._giro = None

    def _soma(self, i0, i1, j0, j1):
        """ocupadas no retangulo de celulas [i0,i1] x [j0,j1] (vetorizado, i = coluna, j = linha)."""
        h, w = self.occ.shape
        i0 = np.clip(i0, 0, w); i1 = np.clip(i1 + 1, 0, w)
        j0 = np.clip(j0, 0, h); j1 = np.clip(j1 + 1, 0, h)
        ii = self.ii
        return ii[j1, i1] - ii[j0, i1] - ii[j1, i0] + ii[j0, i0]

    def livre_rumo(self, k):
        """bool[h, w]: centro do robo livre com rumo k*90 graus."""
        if k in self._cs:
            return self._cs[k]
        x0, x1, y0, y1 = CORPO
        m = self.margem
        cs = [(x0 - m, y0 - m), (x1 + m, y0 - m), (x1 + m, y1 + m), (x0 - m, y1 + m)]
        a = k * math.pi / 2
        c, s = round(math.cos(a)), round(math.sin(a))
        xs = [c * u - s * v for u, v in cs]
        ys = [s * u + c * v for u, v in cs]
        di0, di1 = int(math.floor(min(xs) / self.res)), int(math.ceil(max(xs) / self.res))
        dj0, dj1 = int(math.floor(min(ys) / self.res)), int(math.ceil(max(ys) / self.res))
        h, w = self.occ.shape
        J, I = np.mgrid[0:h, 0:w]
        livre = self._soma(I + di0, I + di1, J + dj0, J + dj1) == 0
        # fora da grade = ocupado (o retangulo cortado pela borda nao conta como livre)
        livre &= (I + di0 >= 0) & (I + di1 < w) & (J + dj0 >= 0) & (J + dj1 < h)
        self._cs[k] = livre
        return livre

    def livre_giro(self):
        """bool[h, w]: circulo do giro (R_GIRO + margem) sem celula ocupada."""
        if self._giro is not None:
            return self._giro
        r = int(math.ceil((R_GIRO + self.margem) / self.res))
        h, w = self.occ.shape
        ys, xs = np.nonzero(self.occ)
        livre = np.ones((h, w), dtype=bool)
        disco = [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1) if di * di + dj * dj <= r * r]
        for di, dj in disco:
            jj, ii = ys + dj, xs + di
            ok = (jj >= 0) & (jj < h) & (ii >= 0) & (ii < w)
            livre[jj[ok], ii[ok]] = False
        livre[:r, :] = livre[-r:, :] = False
        livre[:, :r] = livre[:, -r:] = False
        self._giro = livre
        return livre

    def cel(self, x, y):
        return (int(math.floor((x - self.m.ox) / self.res)), int(math.floor((y - self.m.oy) / self.res)))

    def ok(self, grade, x, y):
        i, j = self.cel(x, y)
        h, w = grade.shape
        return 0 <= i < w and 0 <= j < h and bool(grade[j, i])

    def reta_livre(self, k, a, b):
        g = self.livre_rumo(k)
        n = max(2, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.res / 2))))
        return all(self.ok(g, a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n) for t in range(n + 1))


def _custo_reta(a, b, rumo):
    dx, dy = b[0] - a[0], b[1] - a[1]
    f = math.cos(rumo) * dx + math.sin(rumo) * dy
    l = abs(-math.sin(rumo) * dx + math.cos(rumo) * dy)
    return (f if f >= 0 else -CUSTO_RE * f) + CUSTO_LADO * l


def _caminhos(S, E, passo):
    """retas candidatas de S a E (pontos intermediarios): reta, L (2), Z por linha y ou x."""
    yield []
    yield [(E[0], S[1])]
    yield [(S[0], E[1])]
    lo, hi = sorted((S[1], E[1]))
    for ym in np.arange(min(lo, hi) - 1.5, max(lo, hi) + 1.5, passo):
        yield [(S[0], ym), (E[0], ym)]
    lo, hi = sorted((S[0], E[0]))
    for xm in np.arange(lo - 1.5, hi + 1.5, passo):
        yield [(xm, S[1]), (xm, E[1])]
    # 4 retas (escada): S -> (S.x, ym) -> (xm, ym) -> (xm, E.y) -> E, grade de 10 cm
    for ym in np.arange(min(S[1], E[1]) - 1.0, max(S[1], E[1]) + 1.0, 2 * passo):
        for xm in np.arange(min(S[0], E[0]) - 1.0, max(S[0], E[0]) + 1.0, 2 * passo):
            yield [(S[0], ym), (xm, ym), (xm, E[1])]
            yield [(xm, S[1]), (xm, ym), (E[0], ym)]


def _avaliar(g, pts, syaw, tol_ang, giro):
    """custo minimo e rumos por trecho (DP: em cada esquina com giro livre pode girar para outro rumo)."""
    INF = float('inf')
    custo = [INF] * 4
    rumos_ant = [None] * 4
    for k in range(4):
        if abs(_ang(syaw - k * math.pi / 2)) <= tol_ang:
            custo[k] = 0.0
        elif g.ok(giro, *pts[0]):
            custo[k] = CUSTO_GIRO
    hist = []
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        pode_girar = i == 0 or g.ok(giro, *a)
        novo, de = [INF] * 4, [None] * 4
        for k in range(4):
            if not g.reta_livre(k, a, b):
                continue
            seg = _custo_reta(a, b, k * math.pi / 2)
            for k0 in range(4):
                if custo[k0] == INF or (k0 != k and not pode_girar and i > 0):
                    continue
                c = custo[k0] + seg + (CUSTO_GIRO if (k0 != k and i > 0) else 0.0)
                if c < novo[k]:
                    novo[k], de[k] = c, k0
        hist.append(de)
        custo = novo
    return custo, hist


def plano_retas(mapa, inicio, goal, margem=0.04, passo=0.05, tol_ang=math.radians(8), pose_livre=None,
                grade=None):
    """Melhor plano de retas (ver doc do modulo) ou None. pose_livre(mapa, x, y, yaw, margem=) confere o
    trecho final fora dos eixos (mesa a 45 graus)."""
    g = grade or Grade(mapa, margem)
    sx, sy, syaw = inicio
    gx, gy, gyaw = goal
    giro = g.livre_giro()
    # fins: (ponto E, rumo exigido no ultimo trecho ou None, passos depois de E, custo)
    fins = []
    ka = round(gyaw / (math.pi / 2)) % 4
    if abs(_ang(gyaw - ka * math.pi / 2)) <= tol_ang:
        fins.append(((gx, gy), ka, [], 0.0))
    if g.ok(giro, gx, gy):
        fins.append(((gx, gy), None, [('girar', gyaw)], CUSTO_GIRO))
    elif abs(_ang(gyaw - ka * math.pi / 2)) > tol_ang:
        c, s = math.cos(gyaw), math.sin(gyaw)
        for d in np.arange(0.05, 0.85, passo):
            R = (gx - c * d, gy - s * d)
            if not g.ok(giro, *R):
                continue
            if pose_livre is not None and not all(
                    pose_livre(mapa, R[0] + c * d * t / 8, R[1] + s * d * t / 8, gyaw, margem=margem)
                    for t in range(9)):
                continue
            fins.append((R, None, [('girar', gyaw), ('mover', gx, gy)], CUSTO_GIRO + d))
            break
    # inicio: pose atual, ou (fora dos eixos e sem giro no lugar) recua reto no rumo atual ate o giro caber
    inicios = [((sx, sy), syaw, [], 0.0)]
    k0 = round(syaw / (math.pi / 2)) % 4
    alinhado = abs(_ang(syaw - k0 * math.pi / 2)) <= tol_ang
    apertado = not g.ok(giro, sx, sy) and not (alinhado and g.ok(g.livre_rumo(k0), sx, sy))
    if apertado:   # fora dos eixos sem giro, ou colado em algo: recua reto no rumo atual ate o giro caber
        c, s_ = math.cos(syaw), math.sin(syaw)
        for d in np.arange(0.05, 0.85, passo):
            R = (sx - c * d, sy - s_ * d)
            if not g.ok(giro, *R):
                continue
            # os primeiros 15 cm nao exigem mapa livre: o robo ja esta ali (o _andar_reto para pelo LiDAR)
            if pose_livre is not None and not all(
                    pose_livre(mapa, sx - c * d * t / 8, sy - s_ * d * t / 8, syaw, margem=0.02)
                    for t in range(9) if d * t / 8 >= 0.15):
                continue
            inicios.append((R, syaw, [('mover', R[0], R[1])], CUSTO_RE * d))
            break
    melhor = None
    for (S0, syaw0, passos_ini, custo_ini) in inicios:
      for E, k_fim, passos_fim, custo_fim in fins:
        for meio in _caminhos(S0, E, passo):
            pts = [S0] + [tuple(float(v) for v in p) for p in meio] + [E]
            pts = [p for i, p in enumerate(pts) if i == 0 or math.hypot(p[0] - pts[i - 1][0], p[1] - pts[i - 1][1]) >= 0.02]
            if len(pts) < 2:
                pts = [S0, E]
            if not all(abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6 for a, b in zip(pts, pts[1:])):
                continue
            lb = custo_ini + custo_fim + sum(abs(b[0] - a[0]) + abs(b[1] - a[1]) for a, b in zip(pts, pts[1:]))
            if melhor is not None and lb >= melhor[0]:
                continue
            custo, hist = _avaliar(g, pts, syaw0, tol_ang, giro)
            ks = [k_fim] if k_fim is not None else range(4)
            for k in ks:
                c = custo[k] + custo_fim + custo_ini
                if c == float('inf') or (melhor is not None and c >= melhor[0]):
                    continue
                rumos = [k]
                for de in reversed(hist[1:]):
                    rumos.append(de[rumos[-1]])
                rumos.reverse()
                passos = list(passos_ini)
                if abs(_ang(syaw0 - rumos[0] * math.pi / 2)) > tol_ang:
                    passos.append(('girar', _ang(rumos[0] * math.pi / 2)))
                atual = rumos[0]
                for (a, b), r in zip(zip(pts, pts[1:]), rumos):
                    if r != atual:
                        passos.append(('girar', _ang(r * math.pi / 2)))
                        atual = r
                    passos.append(('mover', b[0], b[1]))
                melhor = (c, passos + passos_fim)
    return None if melhor is None else melhor[1]


def descrever(plano):
    out = []
    for p in plano:
        out.append('gira p/ %.0f g' % math.degrees(p[1]) if p[0] == 'girar' else 'reta ate (%.2f, %.2f)' % p[1:])
    return ' -> '.join(out)
