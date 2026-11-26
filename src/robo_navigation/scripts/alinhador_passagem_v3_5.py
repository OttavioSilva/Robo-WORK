#!/usr/bin/env python3
"""Alinhador de passagens estreitas para o Nav2 (v3.5: travessia pelo LiDAR + ir ate Service Area e encostar).

Recebe um goal (topico /goal_pose_alinhado, ferramenta "2D Goal Pose" do RViz
com o topico trocado), pede um caminho ao planner, procura no mapa trechos
estreitos ao longo desse caminho e, antes de cada um, manda o robo (pelo Nav2)
a uma pose de preparo no eixo da passagem.

v3: a TRAVESSIA nao usa mais o DWB. Da pose de preparo o robo avanca devagar ate
o LiDAR enxergar as duas paredes, para na boca, mede o angulo e o centro do
corredor pelas paredes reais (nao pelo mapa), gira/desliza para alinhar e
atravessa reto corrigindo pelo LiDAR. Os comandos vao para /cmd_vel_nav, ou
seja, continuam passando pelo velocity_smoother e pelo Collision Monitor.
Se algo der errado, volta ao comportamento antigo (TRAVESSIA pelo Nav2).

Uso:
    python3 alinhador_passagem.py
    python3 alinhador_passagem.py --ros-args -p modo_travessia:=nav2   (comportamento v2)
"""
import math
import os
import threading
import time

# ---------------------------------------------------------------------------
# Nucleo geometrico (sem ROS: pode ser testado fora do robo)
# ---------------------------------------------------------------------------

FOOTPRINT = [(0.365, 0.315), (0.365, -0.315), (-0.205, -0.315), (-0.205, 0.315)]


class Mapa:
    """Acesso a um OccupancyGrid: 1 = ocupado, 0 = livre, -1 = desconhecido/fora."""

    def __init__(self, resolucao, largura, altura, origem_x, origem_y, origem_yaw, dados,
                 limiar_ocupado=65):
        self.res = resolucao
        self.w = largura
        self.h = altura
        self.ox = origem_x
        self.oy = origem_y
        self.c = math.cos(origem_yaw)
        self.s = math.sin(origem_yaw)
        self.dados = dados
        self.limiar = limiar_ocupado

    def estado(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        gx = self.c * dx + self.s * dy
        gy = -self.s * dx + self.c * dy
        i, j = int(math.floor(gx / self.res)), int(math.floor(gy / self.res))
        if i < 0 or j < 0 or i >= self.w or j >= self.h:
            return -1
        v = self.dados[j * self.w + i]
        if v < 0:
            return -1
        return 1 if v >= self.limiar else 0

    def ocupado_firme(self, x, y, minimo=3):
        """Ocupado e parte de um grupo de >= minimo celulas (8-vizinhanca).

        Ignora pontinhos soltos do SLAM (ex.: o auto-retorno do LiDAR que deixa um
        rastro de celulas isoladas por onde o robo passa).
        """
        if self.estado(x, y) != 1:
            return False
        r = self.res
        vistos = {(0, 0)}
        fila = [(0, 0)]
        while fila:
            i, j = fila.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    k = (i + di, j + dj)
                    if k in vistos or abs(k[0]) > 3 or abs(k[1]) > 3:
                        continue
                    if self.estado(x + k[0] * r, y + k[1] * r) == 1:
                        vistos.add(k)
                        if len(vistos) >= minimo:
                            return True
                        fila.append(k)
        return False


def _tangentes(pontos):
    n = len(pontos)
    tang = []
    for i in range(n):
        a = pontos[max(0, i - 2)]
        b = pontos[min(n - 1, i + 2)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        d = math.hypot(dx, dy)
        tang.append((dx / d, dy / d) if d > 1e-9 else (1.0, 0.0))
    return tang


def _raio(mapa, x, y, ux, uy, alcance):
    passo = mapa.res / 2.0
    s = 0.0
    while s <= alcance:
        if mapa.estado(x + ux * s, y + uy * s) == 1:
            return s
        s += passo
    return None


def larguras_no_caminho(mapa, pontos, alcance=0.60):
    """Para cada ponto do caminho: (dist_esquerda, dist_direita, tangente)."""
    saida = []
    for (x, y), (tx, ty) in zip(pontos, _tangentes(pontos)):
        nx, ny = -ty, tx  # normal para a esquerda
        de = _raio(mapa, x, y, nx, ny, alcance)
        dd = _raio(mapa, x, y, -nx, -ny, alcance)
        saida.append((de, dd, (tx, ty)))
    return saida


def _comprimento(pontos, i0, i1):
    return sum(math.hypot(pontos[k + 1][0] - pontos[k][0], pontos[k + 1][1] - pontos[k][1])
               for k in range(i0, i1))


def achar_passagens(pontos, larg, largura_estreita, comprimento_min=0.15, buraco_max=2):
    """Trechos contiguos onde as duas paredes existem e a soma < largura_estreita."""
    estreito = [de is not None and dd is not None and de + dd < largura_estreita
                for de, dd, _ in larg]
    trechos = []
    i, n = 0, len(pontos)
    while i < n:
        if not estreito[i]:
            i += 1
            continue
        j, ultimo, buraco = i, i, 0
        while j + 1 < n:
            j += 1
            if estreito[j]:
                ultimo, buraco = j, 0
            else:
                buraco += 1
                if buraco > buraco_max:
                    break
        if _comprimento(pontos, i, ultimo) >= comprimento_min:
            trechos.append((i, ultimo))
        i = ultimo + 1
    return trechos


def geometria_passagem(pontos, larg, i0, i1):
    """Centro de entrada, centro de saida, eixo unitario e largura mediana (no mapa)."""
    centros = []
    larguras = []
    for k in range(i0, i1 + 1):
        de, dd, (tx, ty) = larg[k]
        if de is None or dd is None:
            continue
        nx, ny = -ty, tx
        desloc = (de - dd) / 2.0
        centros.append((pontos[k][0] + nx * desloc, pontos[k][1] + ny * desloc))
        larguras.append(de + dd)
    mx = sum(c[0] for c in centros) / len(centros)
    my = sum(c[1] for c in centros) / len(centros)
    if len(centros) >= 3 and _comprimento(pontos, i0, i1) >= 0.25:
        sxx = sum((c[0] - mx) ** 2 for c in centros)
        syy = sum((c[1] - my) ** 2 for c in centros)
        sxy = sum((c[0] - mx) * (c[1] - my) for c in centros)
        ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
        ax, ay = math.cos(ang), math.sin(ang)
    else:
        tx = sum(larg[k][2][0] for k in range(i0, i1 + 1))
        ty = sum(larg[k][2][1] for k in range(i0, i1 + 1))
        d = math.hypot(tx, ty)
        ax, ay = tx / d, ty / d
    sentido_x = pontos[i1][0] - pontos[i0][0]
    sentido_y = pontos[i1][1] - pontos[i0][1]
    if ax * sentido_x + ay * sentido_y < 0:
        ax, ay = -ax, -ay

    def projetar(c):
        t = (c[0] - mx) * ax + (c[1] - my) * ay
        return (mx + t * ax, my + t * ay)

    larguras.sort()
    return dict(entrada=projetar(centros[0]), saida=projetar(centros[-1]),
                eixo=(ax, ay), largura=larguras[len(larguras) // 2])


def pose_livre(mapa, x, y, yaw, footprint=FOOTPRINT, margem=0.03):
    """Contorno (com margem) e seu interior sem celulas ocupadas."""
    c, s = math.cos(yaw), math.sin(yaw)
    xs = [p[0] for p in footprint]
    ys = [p[1] for p in footprint]
    x0, x1 = min(xs) - margem, max(xs) + margem
    y0, y1 = min(ys) - margem, max(ys) + margem
    passo = mapa.res / 2.0
    u = x0
    while u <= x1 + 1e-9:
        v = y0
        while v <= y1 + 1e-9:
            if mapa.ocupado_firme(x + c * u - s * v, y + s * u + c * v):
                return False
            v += passo
        u += passo
    return True


def planejar_etapas(mapa, pontos, goal_xy, largura_estreita=0.90, distancia_preparo=0.60,
                    distancia_saida=0.45, ignorar_inicio=0.30, ja_tratadas=()):
    """Primeira passagem estreita a frente: devolve (etapas, info) ou ([], motivo).

    ja_tratadas: entradas (x, y) de passagens em que o robo ja se alinhou; sao puladas.
    """
    if len(pontos) < 5:
        return [], 'caminho curto demais'
    larg = larguras_no_caminho(mapa, pontos)
    for i0, i1 in achar_passagens(pontos, larg, largura_estreita):
        if _comprimento(pontos, 0, i0) < ignorar_inicio:
            continue  # o robo ja esta na entrada ou dentro deste trecho
        g = geometria_passagem(pontos, larg, i0, i1)
        if any(math.hypot(g['entrada'][0] - ex, g['entrada'][1] - ey) < 0.40
               for ex, ey in ja_tratadas):
            continue
        ax, ay = g['eixo']
        yaw = math.atan2(ay, ax)
        preparo = None
        for d in (distancia_preparo, distancia_preparo + 0.10, distancia_preparo - 0.10,
                  distancia_preparo + 0.20, distancia_preparo - 0.15):
            px, py = g['entrada'][0] - d * ax, g['entrada'][1] - d * ay
            if pose_livre(mapa, px, py, yaw):
                preparo = (px, py, yaw, d)
                break
        if preparo is None:
            return [], 'passagem em (%.2f, %.2f) sem espaco livre para a pose de preparo' % g['entrada']
        etapas = [dict(tipo='preparo', x=preparo[0], y=preparo[1], yaw=yaw)]
        # Pose de saida no eixo da passagem. Se a distancia padrao nao estiver livre no mapa,
        # tenta mais perto da saida: sem ela, o robo seguiria direto para o goal final e
        # poderia sair do centro da passagem (aconteceu no teste de 21/10 08:43).
        restante_goal = (goal_xy[0] - g['saida'][0]) * ax + (goal_xy[1] - g['saida'][1]) * ay
        for ds in (distancia_saida, 0.35, 0.25, 0.15, 0.0):
            if ds > distancia_saida or restante_goal <= ds + 0.10:
                continue
            sx, sy = g['saida'][0] + ds * ax, g['saida'][1] + ds * ay
            if pose_livre(mapa, sx, sy, yaw, margem=0.0 if ds < 0.2 else 0.03):
                etapas.append(dict(tipo='saida', x=sx, y=sy, yaw=yaw))
                break
        info = dict(entrada=g['entrada'], saida=g['saida'], eixo_graus=math.degrees(yaw),
                    largura_mapa=g['largura'], distancia_preparo=preparo[3])
        return etapas, info
    return [], 'nenhuma passagem estreita no caminho'


# ---------------------------------------------------------------------------
# Corredor medido pelo LiDAR + controle da travessia (sem ROS)
# ---------------------------------------------------------------------------
def _reta(pts, tol=0.025):
    """Parede quase paralela ao robo: y = a + b*x.

    Varre angulos de -20 a +20 graus e fica com a faixa de 3 cm que junta mais pontos
    (robusto contra pontos da face da bancada, pes de mesa etc.); depois refina por
    minimos quadrados so com esses pontos.
    """
    if len(pts) < 6:
        return None
    melhor = None
    for k in range(-40, 41):
        ang = math.radians(k * 0.5)
        c, s = math.cos(ang), math.sin(ang)
        offs = sorted(p[1] * c - p[0] * s for p in pts)
        j = 0
        for i in range(len(offs)):
            while offs[i] - offs[j] > 0.03:
                j += 1
            n = i - j + 1
            if melhor is None or n > melhor[0]:
                melhor = (n, ang, (offs[i] + offs[j]) / 2.0)
    n, ang, centro = melhor
    c, s = math.cos(ang), math.sin(ang)
    sel = [p for p in pts if abs(p[1] * c - p[0] * s - centro) <= 0.02]
    for _ in range(2):
        m = len(sel)
        if m < 6:
            return None
        mx = sum(p[0] for p in sel) / m
        my = sum(p[1] for p in sel) / m
        sxx = sum((p[0] - mx) ** 2 for p in sel)
        if sxx < 1e-4:
            return None
        b = sum((p[0] - mx) * (p[1] - my) for p in sel) / sxx
        a = my - b * mx
        sel = [p for p in sel if abs(p[1] - (a + b * p[0])) < tol]
    if len(sel) < 6:
        return None
    xs = [p[0] for p in sel]
    if max(xs) - min(xs) < 0.15 or abs(b) > 0.35:  # parede curta demais ou torta demais
        return None
    return a, b, len(sel)


def medir_corredor(pts, x_min=0.20, x_max=1.40, y_min=0.22, y_max=0.80, x_ref=0.08,
                   largura_conhecida=None):
    """pts: pontos do LiDAR em base_link. Devolve dict ou None.

    angulo: quanto o robo precisa girar (rad, + = anti-horario) para ficar paralelo as paredes.
    desvio: quanto o centro do corredor esta a esquerda do centro do robo (m) em x_ref.
    """
    esq = _reta([p for p in pts if x_min <= p[0] <= x_max and y_min < p[1] < y_max])
    dir_ = _reta([p for p in pts if x_min <= p[0] <= x_max and -y_max < p[1] < -y_min])
    if esq is None and dir_ is None:
        return None
    r = dict(esq=None, dir=None, largura=None, desvio=None)
    if esq:
        r['esq'] = esq[0] + esq[1] * x_ref
    if dir_:
        r['dir'] = dir_[0] + dir_[1] * x_ref
    if esq and dir_:
        # paredes quase paralelas? (senao uma delas nao e parede do corredor)
        if abs(math.atan(esq[1]) - math.atan(dir_[1])) > math.radians(8):
            # fica com a que tem mais pontos
            if esq[2] >= dir_[2]:
                dir_, r['dir'] = None, None
            else:
                esq, r['esq'] = None, None
    if esq and dir_:
        r['angulo'] = math.atan((esq[1] * esq[2] + dir_[1] * dir_[2]) / (esq[2] + dir_[2]))
        r['largura'] = r['esq'] - r['dir']
        r['desvio'] = (r['esq'] + r['dir']) / 2.0
        r['n'] = esq[2] + dir_[2]
        r['lados'] = 2
    else:
        lado = esq or dir_
        r['angulo'] = math.atan(lado[1])
        r['n'] = lado[2]
        r['lados'] = 1
        if largura_conhecida:
            if esq:
                r['desvio'] = r['esq'] - largura_conhecida / 2.0
            else:
                r['desvio'] = r['dir'] + largura_conhecida / 2.0
    return r

def _ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def _sat(v, vmin, vmax):
    """Satura |v| entre vmin e vmax mantendo o sinal (vmin vence a zona morta do smoother)."""
    if v == 0.0:
        return 0.0
    return math.copysign(min(max(abs(v), vmin), vmax), v)


class Travessia:
    """Maquina de estados: aproximar -> medir -> girar/deslocar -> medir ... -> atravessar -> fim.

    Coordenadas: pose do odom (x, y, yaw); a medicao do LiDAR vem em base_link.
    """

    def __init__(self, v_aprox=0.03, v_travessia=0.035, giro_min=0.02, giro_max=0.03,
                 lateral_cmd=0.02, tol_giro=math.radians(2.0), tol_lateral=0.015,
                 antecipacao_giro=math.radians(2.0), aceita_giro=math.radians(3.0),
                 largura_min=0.66, largura_max=1.00, aprox_max=0.60, ciclos_max=8,
                 tempo_max=60.0, largura_padrao=0.82, log=print):
        self.v_aprox = v_aprox
        self.v_trav = v_travessia
        self.giro_min, self.giro_max = giro_min, giro_max
        self.lat_cmd = lateral_cmd
        self.tol_giro, self.tol_lat = tol_giro, tol_lateral
        # o robo ainda gira ~2 graus depois que o comando para (medido no teste de 11:43):
        # para antes, senao fica oscilando +-2-3 graus em volta do alvo
        self.antecipacao = antecipacao_giro
        self.aceita_giro = aceita_giro
        self.larg_min, self.larg_max = largura_min, largura_max
        self.aprox_max = aprox_max
        self.ciclos_max = ciclos_max
        self.tempo_max = tempo_max
        self.log = log
        self.estado = 'aproximar'
        self.motivo = ''
        self.t0 = None
        self.t_estado = None
        self.inicio_estado = None  # pose do odom quando o estado comecou
        self.meds = []             # (t, med, pose_no_scan) com as 2 paredes
        self.meds1 = []            # idem, so uma parede (centro estimado pela largura)
        self.largura_padrao = largura_padrao
        self.ult = None            # ultima medicao boa de 2 paredes
        self.largura = None
        self.ciclos = 0
        self.n_lat = 0
        self.n_bloq = 0
        self.giro_bloqueado = False
        self.frente = 9.0          # folga livre a frente do para-choque (m), do LiDAR
        self.yaw_ref = None
        self.alvo = None
        self.p_aprox_odom = None

    # ------------------------------------------------------------------
    def _boa(self, med):
        return (med is not None and med['lados'] == 2 and med['n'] >= 14 and
                self.larg_min < med['largura'] < self.larg_max)

    def nova_medicao(self, t, med, pose, frente=None):
        if frente is not None:
            self.frente = frente
        if med is None:
            return
        if (med['lados'] == 1 and med['n'] >= 10 and med.get('desvio') is not None and
                abs(med['angulo']) < math.radians(12)):
            self.meds1.append((t, med, pose))
            self.meds1 = self.meds1[-10:]
        if self._boa(med):
            self.meds.append((t, med, pose))
            self.meds = self.meds[-10:]
            self.ult = (t, med, pose)
            self.largura = med['largura'] if self.largura is None else (
                0.8 * self.largura + 0.2 * med['largura'])
        elif (self.estado == 'atravessar' and med['n'] >= 10 and med.get('desvio') is not None
              and self.ult is not None):
            # uma parede so (dentro do corredor): aceita se concordar com o angulo atual
            yaw_eixo = pose[2] + med['angulo']
            if abs(_ang(yaw_eixo - self.yaw_ref)) < math.radians(4):
                self.ult = (t, med, pose)

    def _mudar(self, estado, t, pose):
        self.estado, self.t_estado, self.inicio_estado = estado, t, pose

    def _desloc_robo(self, p0, p1):
        """Deslocamento de p0 ate p1 no referencial do robo em p0."""
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s = math.cos(p0[2]), math.sin(p0[2])
        return c * dx + s * dy, -s * dx + c * dy

    def _erro_lateral(self, pose):
        """Centro do corredor (m, + = a esquerda), corrigido pelo movimento desde o scan."""
        t, med, p_scan = self.ult
        dx, dy = self._desloc_robo(p_scan, pose)
        return med['desvio'] + math.tan(med['angulo']) * dx - dy

    def bloqueio(self, t, pose):
        """O Collision Monitor esta segurando o robo: para e mede de novo (ou desiste)."""
        if self.estado == 'atravessar':
            # dentro da passagem, seguir empurrando so piora (pivota na quina):
            # desiste; o no sai de re e tenta de novo a partir do preparo
            self.abortar('Collision Monitor bloqueando dentro da passagem')
            return
        self.n_bloq += 1
        if self.n_bloq > 4:
            self.abortar('Collision Monitor bloqueou %d vezes' % self.n_bloq)
            return
        self.log('Collision Monitor segurando em "%s": parando para medir de novo' % self.estado)
        self.giro_bloqueado = self.estado == 'girar'
        self._mudar('assentar', t, pose)

    def abortar(self, motivo):
        self.estado, self.motivo = 'abortar', motivo
        self.log('Travessia LiDAR abortada: %s' % motivo)

    # ------------------------------------------------------------------
    def passo(self, t, pose, restante):
        """Devolve (vx, vy, wz). restante: metros ate o centro do robo passar da saida."""
        if self.t0 is None:
            self.t0 = t
            self.p_aprox = pose
            self.p_aprox_odom = pose
            self._mudar('aproximar', t, pose)
        if self.estado in ('fim', 'abortar'):
            return 0.0, 0.0, 0.0
        if t - self.t0 > self.tempo_max:
            self.abortar('tempo maximo')
            return 0.0, 0.0, 0.0
        e = self.estado

        if e == 'aproximar':
            vistas = [m for m in self.meds if m[0] >= self.t_estado]
            if len(vistas) >= 2:  # duas leituras boas seguidas: nao e ruido
                self.log('Paredes vistas (largura %.2f m): parando na boca para alinhar' %
                         self.ult[1]['largura'])
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            andou = math.hypot(pose[0] - self.p_aprox[0], pose[1] - self.p_aprox[1])
            if andou > self.aprox_max or self.frente < 0.06:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return self.v_aprox, 0.0, 0.0

        if e == 'assentar':  # espera o robo parar e o LiDAR atualizar
            if t - self.t_estado >= 0.4:
                self._mudar('medir', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'medir':
            novas = [m for m in self.meds if m[0] >= self.t_estado]
            if len(novas) < 4 and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            if len(novas) < 2:
                # Muito fora do centro o LiDAR so ve uma parede: usa ela + a largura conhecida
                novas1 = [m for m in self.meds1 if m[0] >= self.t_estado]
                if len(novas1) >= 2:
                    novas = novas1
                    if self.ult is None or self.ult[0] < novas1[-1][0]:
                        self.ult = novas1[-1]
                    self.log('So vejo uma parede: usando a largura de %.2f m para achar o centro' %
                             (self.largura or self.largura_padrao))
                else:
                    # ainda longe: avanca mais um pouco, se a frente estiver livre
                    andou = math.hypot(pose[0] - self.p_aprox[0], pose[1] - self.p_aprox[1])
                    if andou < self.aprox_max and self.frente >= 0.06:
                        self.estado, self.t_estado, self.inicio_estado = 'aproximar', t, pose
                        return 0.0, 0.0, 0.0
                    self.abortar('nao vejo as paredes da passagem')
                    return 0.0, 0.0, 0.0
            ang = sorted(m[1]['angulo'] for m in novas)[len(novas) // 2]
            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]
            yaw_scan = novas[-1][2][2]
            self.ciclos += 1
            larg = self.largura or self.largura_padrao
            self.log('Medicao %d: girar %+.1f graus, centro a %+.1f cm, largura %.2f m' % (
                self.ciclos, math.degrees(ang), des * 100, larg))
            if larg < self.larg_min + 0.02:
                self.abortar('passagem estreita demais (%.2f m)' % larg)
                return 0.0, 0.0, 0.0
            # Ordem: muito fora do centro -> desliza primeiro (perto da bancada o giro e
            # bloqueado pelo Collision Monitor); depois giro grande; depois deslizes finos
            # (no maximo 2, cada um da tranco de yaw); o giro fino fica por ultimo.
            pode = self.ciclos <= self.ciclos_max
            lat_ok = self.n_lat < 2 or (abs(des) > 0.04 and self.n_lat < 5)
            bloq, self.giro_bloqueado = self.giro_bloqueado, False
            if pode and lat_ok and (abs(des) > 0.06 or (bloq and abs(des) > self.tol_lat)):
                self.n_lat += 1
                self.alvo = des
                self._mudar('deslocar', t, pose)
            elif pode and abs(ang) > 2 * self.tol_giro:
                self.alvo = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif pode and abs(des) > self.tol_lat and lat_ok:
                self.n_lat += 1
                self.alvo = des
                self._mudar('deslocar', t, pose)
            elif pode and abs(ang) > self.tol_giro:
                self.alvo = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif abs(ang) <= self.aceita_giro and (
                    abs(des) < 2 * self.tol_lat or (self.n_lat >= 2 and abs(des) < 0.045)):
                # o resto do erro lateral e corrigido andando
                self.yaw_ref = _ang(yaw_scan + ang)
                self.log('Alinhado: atravessando')
                self._mudar('atravessar', t, pose)
            else:
                self.abortar('nao consegui alinhar em %d tentativas' % self.ciclos_max)
            return 0.0, 0.0, 0.0

        if e == 'girar':
            err = _ang(self.alvo - pose[2])
            if abs(err) < max(math.radians(0.6), self.antecipacao) or t - self.t_estado > 8.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, _sat(0.5 * err, self.giro_min, self.giro_max)

        if e == 'deslocar':
            _, dy = self._desloc_robo(self.inicio_estado, pose)
            falta = self.alvo - dy
            # Fecha pelo LiDAR (o odom lateral erra 2-3x): para quando a leitura nova das
            # paredes mostra o centro perto/cruzado, ou pelo odom com folga de seguranca.
            novas = ([m for m in self.meds if m[0] >= self.t_estado + 0.25] or
                     [m for m in self.meds1 if m[0] >= self.t_estado + 0.25])
            if novas:
                des_agora = novas[-1][1]['desvio']
                if abs(des_agora) < 0.6 * self.tol_lat or des_agora * self.alvo < 0:
                    self._mudar('assentar', t, pose)
                    return 0.0, 0.0, 0.0
                falta = des_agora
            if abs(dy) > 2.5 * abs(self.alvo) + 0.02 or t - self.t_estado > 6.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            # segura o yaw pelo IMU: deslizar da trancos de yaw no mecanum
            err = _ang(self.inicio_estado[2] - pose[2])
            wz = _sat(0.5 * err, 0.016, 0.03) if abs(err) > math.radians(0.8) else 0.0
            return 0.0, math.copysign(self.lat_cmd, falta), wz

        if e == 'atravessar':
            if restante <= 0.0:
                self.estado = 'fim'
                self.log('Travessia LiDAR concluida')
                return 0.0, 0.0, 0.0
            # eixo do corredor: corrige devagar com as medicoes de 2 paredes
            tm, med, p_scan = self.ult
            if med['lados'] == 2 and t - tm < 0.5:
                eixo = _ang(p_scan[2] + med['angulo'])
                self.yaw_ref = _ang(self.yaw_ref + max(-0.01, min(0.01, 0.3 * _ang(eixo - self.yaw_ref))))
            err_yaw = _ang(self.yaw_ref - pose[2])
            err_lat = self._erro_lateral(pose) if t - tm < 1.5 else 0.0
            wz = _sat(0.4 * err_yaw, 0.016, 0.03) if abs(err_yaw) > math.radians(1.2) else 0.0
            # dentro da passagem so corrige o lado com erro > 2.5 cm: cada deslize da tranco de yaw
            vy = _sat(0.5 * err_lat, 0.016, 0.025) if abs(err_lat) > 0.025 else 0.0
            vx = self.v_trav
            if abs(err_yaw) > math.radians(4) or abs(err_lat) > 0.035:
                vx = 0.0  # corrige parado antes de seguir
            return vx, vy, wz
        return 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------
# Encostar de frente numa Service Area pelo LiDAR (sem ROS)
# ---------------------------------------------------------------------------
def medir_face(pts, x_min=0.30, x_max=1.30, y_lim=0.70, tol=0.02):
    """Face da mesa a frente do robo (pontos em base_link).

    Devolve dict(dist, ang, centro, span) ou None:
      dist   = distancia (m) da origem do base_link ate a face, no eixo x do robo
      ang    = quanto girar (rad, + = anti-horario) para ficar de frente (perpendicular)
      centro = y do centro da face (m, + = a esquerda) ou None se uma ponta nao aparece
    """
    sel = [p for p in pts if x_min <= p[0] <= x_max and abs(p[1]) <= y_lim]
    if len(sel) < 6:
        return None
    # reta x = a + b*y (face quase perpendicular ao robo): varre angulos de -25 a +25 graus
    melhor = None
    for k in range(-50, 51):
        th = math.radians(k * 0.5)
        c, s = math.cos(th), math.sin(th)
        offs = sorted((p[0] * c - p[1] * s, p) for p in sel)
        j = 0
        for i in range(len(offs)):
            while offs[i][0] - offs[j][0] > 0.03:
                j += 1
            n = i - j + 1
            # empate: prefere a reta mais proxima (a face, nao a parede atras da mesa)
            if melhor is None or n > melhor[0] or (n == melhor[0] and offs[j][0] < melhor[2]):
                melhor = (n, th, offs[j][0], (offs[i][0] + offs[j][0]) / 2.0)
    n, th, _, centro_off = melhor
    c, s = math.cos(th), math.sin(th)
    inl = [p for p in sel if abs(p[0] * c - p[1] * s - centro_off) <= 0.02]
    for _ in range(2):
        m = len(inl)
        if m < 6:
            return None
        my = sum(p[1] for p in inl) / m
        mx = sum(p[0] for p in inl) / m
        syy = sum((p[1] - my) ** 2 for p in inl)
        if syy < 1e-4:
            return None
        b = sum((p[1] - my) * (p[0] - mx) for p in inl) / syy
        a = mx - b * my
        inl = [p for p in sel if abs(p[0] - (a + b * p[1])) < tol]
    if len(inl) < 6:
        return None
    ys = sorted(p[1] for p in inl)
    span = ys[-1] - ys[0]
    if span < 0.25:
        return None
    # centro so vale se as duas pontas da face aparecem (nao foram cortadas pela janela)
    centro = None
    if 0.60 <= span <= 0.98 and ys[0] > -y_lim + 0.05 and ys[-1] < y_lim - 0.05:
        centro = (ys[0] + ys[-1]) / 2.0
    # normal da face no robo: (1, -b) -> girar -atan(b) para ficar perpendicular
    return dict(dist=a, ang=-math.atan(b), centro=centro, span=span, n=len(inl))


class Encosto:
    def __init__(self, distancia_alvo=0.08, para_choque=0.365, v_aprox=0.025, v_final=0.016,
                 giro_min=0.02, giro_max=0.03, tol_giro=math.radians(2.0),
                 antecipacao=math.radians(2.0), tol_lat=0.015, lateral_cmd=0.02,
                 tempo_max=60.0, log=print):
        self.alvo = distancia_alvo
        self.pc = para_choque
        self.v_aprox, self.v_final = v_aprox, v_final
        self.giro_min, self.giro_max = giro_min, giro_max
        self.tol_giro, self.antecip = tol_giro, antecipacao
        self.tol_lat, self.lat_cmd = tol_lat, lateral_cmd
        self.tempo_max = tempo_max
        self.log = log
        self.estado = 'assentar'
        self.motivo = ''
        self.t0 = self.t_estado = None
        self.ini = None
        self.meds = []
        self.ciclos = 0
        self.n_lat = 0
        self.avancos = 0
        self.alvo_giro = self.alvo_lat = self.yaw_ref = None
        self.resultado = None

    def nova_medicao(self, t, med, pose):
        if med is not None:
            self.meds.append((t, med, pose))
            self.meds = self.meds[-12:]

    def abortar(self, motivo):
        self.estado, self.motivo = 'abortar', motivo
        self.log('Encosto abortado: %s' % motivo)

    def bloqueio(self, t, pose):
        """Collision Monitor segurando: perto da mesa e esperado no avanco final."""
        if self.estado == 'avancar':
            self.log('Collision Monitor parou o avanco: medindo onde ficou')
            self._mudar('assentar_final', t, pose)
        else:
            self.abortar('Collision Monitor bloqueando em "%s"' % self.estado)

    def _mudar(self, e, t, pose):
        self.estado, self.t_estado, self.ini = e, t, pose

    def _novas(self, desde):
        return [m for m in self.meds if m[0] >= desde]

    @staticmethod
    def _desloc(p0, p1):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s = math.cos(p0[2]), math.sin(p0[2])
        return c * dx + s * dy, -s * dx + c * dy

    def passo(self, t, pose):
        if self.t0 is None:
            self.t0 = t
            self._mudar('assentar', t, pose)
        if self.estado in ('fim', 'abortar'):
            return 0.0, 0.0, 0.0
        if t - self.t0 > self.tempo_max:
            self.abortar('tempo maximo')
            return 0.0, 0.0, 0.0
        e = self.estado

        if e in ('assentar', 'assentar_final'):
            if t - self.t_estado >= 0.4:
                self._mudar('medir' if e == 'assentar' else 'medir_final', t, pose)
            return 0.0, 0.0, 0.0

        if e in ('medir', 'medir_final'):
            novas = self._novas(self.t_estado)
            if len(novas) < 4 and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            if len(novas) < 2:
                self.abortar('nao vejo a face da mesa a frente')
                return 0.0, 0.0, 0.0
            med = lambda k: sorted(m[1][k] for m in novas)[len(novas) // 2]
            dist, ang = med('dist'), med('ang')
            centros = [m[1]['centro'] for m in novas if m[1]['centro'] is not None]
            centro = sorted(centros)[len(centros) // 2] if len(centros) >= 2 else None
            folga = dist - self.pc
            yaw_scan = novas[-1][2][2]
            if e == 'medir_final':
                self.resultado = dict(folga=folga, ang=ang, centro=centro)
                self.log('Encostado: frente a %.1f cm da mesa, %.1f graus, centro %s' % (
                    folga * 100, math.degrees(ang),
                    ('%+.1f cm' % (centro * 100)) if centro is not None else 'nao medido'))
                if folga > self.alvo + 0.025 and self.avancos < 3:
                    self.yaw_ref = _ang(yaw_scan + ang)
                    self._mudar('avancar', t, pose)
                    return 0.0, 0.0, 0.0
                self.estado = 'fim'
                return 0.0, 0.0, 0.0
            self.ciclos += 1
            self.log('Medicao %d: face a %.0f cm (para-choque), girar %+.1f graus, centro %s' % (
                self.ciclos, folga * 100, math.degrees(ang),
                ('%+.1f cm' % (centro * 100)) if centro is not None else 'nao visto'))
            if folga < 0.03:
                self.abortar('ja esta perto demais da mesa (%.0f cm)' % (folga * 100))
                return 0.0, 0.0, 0.0
            pode = self.ciclos <= 8
            if pode and abs(ang) > self.tol_giro:
                self.alvo_giro = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif pode and centro is not None and abs(centro) > self.tol_lat and self.n_lat < 3:
                self.n_lat += 1
                self.alvo_lat = centro
                self._mudar('deslocar', t, pose)
            else:
                self.yaw_ref = _ang(yaw_scan + ang)
                self.log('Alinhado com a mesa: avancando ate %.0f cm' % (self.alvo * 100))
                self._mudar('avancar', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'girar':
            err = _ang(self.alvo_giro - pose[2])
            if abs(err) < max(math.radians(0.6), self.antecip) or t - self.t_estado > 8.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, _sat(0.5 * err, self.giro_min, self.giro_max)

        if e == 'deslocar':
            _, dy = self._desloc(self.ini, pose)
            falta = self.alvo_lat - dy
            novas = [m for m in self._novas(self.t_estado + 0.25) if m[1]['centro'] is not None]
            if novas:
                falta = novas[-1][1]['centro']
                if abs(falta) < 0.6 * self.tol_lat or falta * self.alvo_lat < 0:
                    self._mudar('assentar', t, pose)
                    return 0.0, 0.0, 0.0
            if abs(dy) > 2.5 * abs(self.alvo_lat) + 0.02 or t - self.t_estado > 6.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            err = _ang(self.ini[2] - pose[2])
            wz = _sat(0.5 * err, 0.016, 0.03) if abs(err) > math.radians(0.8) else 0.0
            return 0.0, math.copysign(self.lat_cmd, falta), wz

        if e == 'avancar':
            if self.ini is not None and self.t_estado == t:
                self.avancos += 1
            novas = self._novas(self.t_estado)
            if novas:
                tm, med, p_scan = novas[-1]
                dx, _ = self._desloc(p_scan, pose)
                # o odom superestima o quanto andou: para antes (seguro), o medir_final corrige
                folga = med['dist'] - self.pc - dx
            else:
                folga = 9.0
            if folga <= self.alvo + 0.01:
                self._mudar('assentar_final', t, pose)
                return 0.0, 0.0, 0.0
            if t - self.t_estado > 15.0:
                self._mudar('assentar_final', t, pose)
                return 0.0, 0.0, 0.0
            err = _ang(self.yaw_ref - pose[2])
            wz = _sat(0.4 * err, 0.016, 0.03) if abs(err) > math.radians(1.0) else 0.0
            vx = self.v_aprox if folga > self.alvo + 0.07 else self.v_final
            return vx, 0.0, wz
        return 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------
# No ROS 2
# ---------------------------------------------------------------------------

def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    def yaw_de(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    class Cancelado(Exception):
        pass

    class Alinhador(Node):
        def __init__(self):
            super().__init__('alinhador_passagem')
            self.declare_parameter('topico_goal', '/goal_pose_alinhado')
            self.declare_parameter('largura_estreita', 0.90)
            self.declare_parameter('distancia_preparo', 0.60)
            self.declare_parameter('distancia_saida', 0.45)
            self.declare_parameter('planner_id', 'GridBased')
            self.declare_parameter('max_passagens', 4)
            self.declare_parameter('periodo_vigia', 1.0)
            # v3: travessia pelo LiDAR ('lidar') ou pelo Nav2 como na v2 ('nav2')
            self.declare_parameter('modo_travessia', 'lidar')
            self.declare_parameter('vel_aproximacao', 0.03)   # comando (o robo real anda ~3x)
            self.declare_parameter('vel_travessia', 0.035)
            self.declare_parameter('margem_saida', 0.30)      # centro do robo alem da saida
            self.declare_parameter('topico_scan', '/scan_front')
            # se a travessia pelo LiDAR falhar 2 vezes: True = tenta pelo Nav2 (v2),
            # False = para fora da passagem e avisa (o DWB dentro da passagem so piora)
            self.declare_parameter('fallback_nav2', False)
            self.declare_parameter('largura_passagem', 0.82)  # usada quando so uma parede aparece
            # v3.5: ir ate uma Service Area e encostar de frente
            self.declare_parameter('arquivo_areas', os.path.expanduser('~/mapas/service_areas.yaml'))
            self.declare_parameter('distancia_encosto', 0.08)   # para-choque ate a face (m)
            self.declare_parameter('recuo_saida', 0.25)         # re ao sair da mesa (odom)
            self.area_pendente = None
            self.encostado = False
            grupo = ReentrantCallbackGroup()
            self.cm_stop = False
            self.odom = None          # (t, x, y, yaw)
            self.scan = None          # (t, pontos em base_link, pose do odom no scan)
            self.scan_seq = 0
            self.laser_tf = None      # (x, y, yaw) do laser em base_link
            self.cmd_final = (0.0, False)  # (t, se o /cmd_vel final esta zerado)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(Odometry, '/odom', self.ao_odom, 20, callback_group=grupo)
            self.create_subscription(LaserScan, self.get_parameter('topico_scan').value,
                                     self.ao_scan, qos_profile_sensor_data, callback_group=grupo)
            self.create_subscription(TwistStamped, '/cmd_vel', self.ao_cmd_final, 20,
                                     callback_group=grupo)
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                     self.ao_cm, 10, callback_group=grupo)
            self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
            self.lock = threading.Lock()
            self.mapa_msg = None
            self.plano = None
            self.plano_t = 0.0
            self.goal_pendente = None
            self.evento = threading.Event()
            self.handle_ativo = None
            qos_mapa = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, '/map', self.ao_mapa, qos_mapa,
                                     callback_group=grupo)
            self.create_subscription(Path, '/plan', self.ao_plano, 5, callback_group=grupo)
            self.create_subscription(PoseStamped, self.get_parameter('topico_goal').value,
                                     self.ao_goal, 10, callback_group=grupo)
            self.create_subscription(String, '/ir_para_area', self.ao_ir_area, 10,
                                     callback_group=grupo)
            self.pub_poses = self.create_publisher(PoseArray, '/alinhador/etapas', 1)
            self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=grupo)
            self.plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose',
                                     callback_group=grupo)
            threading.Thread(target=self.trabalhador, daemon=True).start()
            self.get_logger().info('Alinhador pronto. Envie goals em %s.' %
                                   self.get_parameter('topico_goal').value)

        def p(self, nome):
            return self.get_parameter(nome).value

        # ---------------- entradas ----------------
        def ao_mapa(self, msg):
            with self.lock:
                self.mapa_msg = msg

        def ao_plano(self, msg):
            with self.lock:
                self.plano = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
                self.plano_t = time.monotonic()

        def ao_goal(self, msg):
            if msg.header.frame_id not in ('map', ''):
                self.get_logger().warn('Goal em "%s" ignorado: use Fixed Frame = map.' %
                                       msg.header.frame_id)
                return
            with self.lock:
                self.goal_pendente = msg
                self.area_pendente = None
            self.evento.set()
            self._cancelar_nav()
            self.get_logger().info('Novo goal: x=%.2f y=%.2f yaw=%.0f graus' % (
                msg.pose.position.x, msg.pose.position.y, math.degrees(yaw_de(msg.pose.orientation))))

        def ao_ir_area(self, msg):
            nome = msg.data.strip().upper()
            caminho = self.p('arquivo_areas')
            try:
                import yaml
                with open(caminho) as fa:
                    areas = (yaml.safe_load(fa) or {}).get('service_areas') or {}
            except Exception as exc:
                self.get_logger().error('Nao consegui ler %s: %s' % (caminho, exc))
                return
            area = areas.get(nome) or areas.get(nome.replace('_', ''))
            if area is None or 'aproximacao' not in area:
                self.get_logger().error('Area %s nao esta em %s (tem: %s)' % (
                    nome, caminho, ', '.join(areas) or 'nenhuma'))
                return
            ap = area['aproximacao']
            goal = self._pose(ap['x'], ap['y'], ap['yaw'])
            with self.lock:
                self.goal_pendente = goal
                self.area_pendente = nome
            self.evento.set()
            self._cancelar_nav()
            self.get_logger().info('Indo para %s: aproximacao em x=%.2f y=%.2f yaw=%.0f graus' % (
                nome, ap['x'], ap['y'], math.degrees(ap['yaw'])))

        def ao_odom(self, msg):
            p = msg.pose.pose
            with self.lock:
                self.odom = (time.monotonic(), p.position.x, p.position.y, yaw_de(p.orientation))

        def ao_cmd_final(self, msg):
            tw = msg.twist
            zerado = abs(tw.linear.x) < 1e-4 and abs(tw.linear.y) < 1e-4 and abs(tw.angular.z) < 1e-4
            with self.lock:
                self.cmd_final = (time.monotonic(), zerado)

        def ao_cm(self, msg):
            self.cm_stop = msg.action_type == 1  # STOP

        def ao_scan(self, msg):
            if self.laser_tf is None:
                try:
                    tr = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
                    self.laser_tf = (tr.transform.translation.x, tr.transform.translation.y,
                                     yaw_de(tr.transform.rotation))
                    self.get_logger().info('Laser em base_link: x=%.3f y=%.3f yaw=%.1f graus' % (
                        self.laser_tf[0], self.laser_tf[1], math.degrees(self.laser_tf[2])))
                except Exception:
                    return
            lx, ly, lth = self.laser_tf
            pts = []
            a = msg.angle_min
            for r in msg.ranges:
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    pts.append((lx + r * math.cos(a + lth), ly + r * math.sin(a + lth)))
                a += msg.angle_increment
            # folga livre na frente do para-choque (x=0.365), na largura do robo
            frente = min([p[0] - 0.365 for p in pts if p[0] > 0.30 and abs(p[1]) < 0.33] or [9.0])
            with self.lock:
                od = self.odom
                self.scan = (time.monotonic(), pts, od[1:] if od else None, frente)
                self.scan_seq += 1

        def _novo_goal_chegou(self):
            with self.lock:
                return self.goal_pendente is not None

        # ---------------- utilidades ----------------
        def _esperar(self, futuro, timeout):
            ev = threading.Event()
            futuro.add_done_callback(lambda _: ev.set())
            limite = time.monotonic() + timeout
            while not ev.is_set():
                if self._novo_goal_chegou():
                    raise Cancelado()
                if time.monotonic() > limite:
                    return None
                ev.wait(0.05)
            return futuro.result()

        def _cancelar_nav(self):
            h = self.handle_ativo
            if h is not None:
                try:
                    h.cancel_goal_async()
                except Exception:
                    pass

        def _pose(self, x, y, yaw):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            return ps

        def _mapa(self):
            with self.lock:
                m = self.mapa_msg
            if m is None:
                return None
            i = m.info
            return Mapa(i.resolution, i.width, i.height, i.origin.position.x, i.origin.position.y,
                        yaw_de(i.origin.orientation), m.data)

        def _caminho(self, goal):
            if not self.plan.wait_for_server(timeout_sec=3.0):
                self.get_logger().warn('compute_path_to_pose indisponivel')
                return None
            req = ComputePathToPose.Goal()
            req.goal = goal
            req.planner_id = self.p('planner_id')
            req.use_start = False
            h = self._esperar(self.plan.send_goal_async(req), 5.0)
            if h is None or not h.accepted:
                return None
            r = self._esperar(h.get_result_async(), 10.0)
            if r is None or r.status != GoalStatus.STATUS_SUCCEEDED:
                return None
            return [(ps.pose.position.x, ps.pose.position.y) for ps in r.result.path.poses]

        def _navegar(self, alvo, nome, vigia=None):
            """Devolve 'ok', 'falha' ou 'interrompido' (vigia pediu para parar)."""
            if not self.nav.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('navigate_to_pose indisponivel')
                return 'falha'
            req = NavigateToPose.Goal()
            req.pose = alvo
            self.get_logger().info('%s: x=%.2f y=%.2f yaw=%.0f graus' % (
                nome, alvo.pose.position.x, alvo.pose.position.y,
                math.degrees(yaw_de(alvo.pose.orientation))))
            h = self._esperar(self.nav.send_goal_async(req), 5.0)
            if h is None or not h.accepted:
                self.get_logger().warn('%s: goal recusado' % nome)
                return 'falha'
            self.handle_ativo = h
            futuro = h.get_result_async()
            try:
                proxima = time.monotonic() + self.p('periodo_vigia')
                while True:
                    r = self._esperar(futuro, 0.2)
                    if r is not None:
                        break
                    if vigia is not None and time.monotonic() >= proxima:
                        proxima = time.monotonic() + self.p('periodo_vigia')
                        if vigia():
                            h.cancel_goal_async()
                            self._esperar(futuro, 5.0)
                            self.get_logger().info('%s: interrompido para alinhar' % nome)
                            return 'interrompido'
            finally:
                self.handle_ativo = None
            ok = r.status == GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('%s: %s' % (nome, 'SUCESSO' if ok else 'FALHOU'))
            return 'ok' if ok else 'falha'

        def _publicar_vel(self, vx, vy, wz):
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = float(vx), float(vy), float(wz)
            self.pub_cmd.publish(m)

        def _parar(self):
            for _ in range(5):
                self._publicar_vel(0.0, 0.0, 0.0)
                time.sleep(0.05)

        def _pose_mapa(self):
            try:
                tr = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            except Exception:
                return None
            return tr.transform.translation.x, tr.transform.translation.y

        def _travessia_lidar(self, info, nome):
            """Devolve 'ok' ou 'falha' (o chamador cai para a travessia pelo Nav2)."""
            sx, sy = info['saida']
            ax, ay = math.cos(math.radians(info['eixo_graus'])), math.sin(math.radians(info['eixo_graus']))
            margem = self.p('margem_saida')
            trav = Travessia(v_aprox=self.p('vel_aproximacao'), v_travessia=self.p('vel_travessia'),
                             largura_padrao=self.p('largura_passagem'),
                             log=lambda m: self.get_logger().info('%s: %s' % (nome, m)))
            self.get_logger().info('%s: travessia pelo LiDAR ate (%.2f, %.2f) + %.2f m' % (
                nome, sx, sy, margem))
            seq = self.scan_seq
            bloqueado_desde = None
            sem_tf_desde = None
            restante = 9.0
            try:
                while rclpy.ok():
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    agora = time.monotonic()
                    with self.lock:
                        od, sc, sq, cf = self.odom, self.scan, self.scan_seq, self.cmd_final
                    if od is None or agora - od[0] > 0.5:
                        trav.abortar('sem /odom recente')
                        break
                    if sq != seq and sc is not None and sc[2] is not None:
                        seq = sq
                        med = medir_corredor(sc[1], largura_conhecida=trav.largura or
                                             self.p('largura_passagem'))
                        trav.nova_medicao(sc[0], med, sc[2], sc[3])
                    pm = self._pose_mapa()
                    if pm is None:
                        sem_tf_desde = sem_tf_desde or agora
                        if agora - sem_tf_desde > 1.0:
                            trav.abortar('sem TF map->base_link')
                            break
                    else:
                        sem_tf_desde = None
                        restante = margem - ((pm[0] - sx) * ax + (pm[1] - sy) * ay)
                    vx, vy, wz = trav.passo(agora, od[1:], restante)
                    if trav.estado in ('fim', 'abortar'):
                        break
                    self._publicar_vel(vx, vy, wz)
                    # Collision Monitor segurando o robo: STOP no estado do CM, /cmd_vel final
                    # zerado, ou /cmd_vel final parado de publicar (o CM em STOP para de publicar)
                    mandando = abs(vx) >= 0.015 or abs(vy) >= 0.015 or abs(wz) >= 0.015
                    preso = self.cm_stop or agora - cf[0] > 0.5 or cf[1]
                    if mandando and preso:
                        bloqueado_desde = bloqueado_desde or agora
                        if agora - bloqueado_desde > 1.5:
                            bloqueado_desde = None
                            trav.bloqueio(agora, od[1:])
                            if trav.estado == 'abortar':
                                break
                    else:
                        bloqueado_desde = None
                    time.sleep(0.05)
            finally:
                self._parar()
            ok = trav.estado == 'fim'
            if not ok and trav.p_aprox_odom is not None:
                # se entrou na passagem, sai de re reto (mesmo eixo) antes de qualquer outra coisa
                with self.lock:
                    od = self.odom
                if od is not None:
                    p0 = trav.p_aprox_odom
                    avancou = (od[1] - p0[0]) * math.cos(p0[2]) + (od[2] - p0[1]) * math.sin(p0[2])
                    if avancou > 0.05:
                        self._recuar(avancou, od[3], nome)
            self.get_logger().info('%s (LiDAR): %s' % (nome, 'SUCESSO' if ok else 'FALHOU - ' + trav.motivo))
            return 'ok' if ok else 'falha'

        def _encostar(self, nome):
            """Da pose de aproximacao: alinha com a face pelo LiDAR e avanca ate a distancia."""
            enc = Encosto(distancia_alvo=self.p('distancia_encosto'),
                          log=lambda m: self.get_logger().info('ENCOSTAR %s: %s' % (nome, m)))
            self.get_logger().info('ENCOSTAR %s: medindo a face da mesa' % nome)
            seq = self.scan_seq
            bloqueado_desde = None
            try:
                while rclpy.ok():
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    agora = time.monotonic()
                    with self.lock:
                        od, sc, sq, cf = self.odom, self.scan, self.scan_seq, self.cmd_final
                    if od is None or agora - od[0] > 0.5:
                        enc.abortar('sem /odom recente')
                        break
                    if sq != seq and sc is not None and sc[2] is not None:
                        seq = sq
                        enc.nova_medicao(sc[0], medir_face(sc[1]), sc[2])
                    vx, vy, wz = enc.passo(agora, od[1:])
                    if enc.estado in ('fim', 'abortar'):
                        break
                    self._publicar_vel(vx, vy, wz)
                    mandando = abs(vx) >= 0.015 or abs(vy) >= 0.015 or abs(wz) >= 0.015
                    if mandando and (self.cm_stop or agora - cf[0] > 0.5 or cf[1]):
                        bloqueado_desde = bloqueado_desde or agora
                        if agora - bloqueado_desde > 1.5:
                            bloqueado_desde = None
                            enc.bloqueio(agora, od[1:])
                            if enc.estado == 'abortar':
                                break
                    else:
                        bloqueado_desde = None
                    time.sleep(0.05)
            finally:
                self._parar()
            if enc.estado == 'fim':
                self.encostado = True
                self.get_logger().info('ENCOSTAR %s: SUCESSO' % nome)
            else:
                self.get_logger().warn('ENCOSTAR %s: FALHOU - %s' % (nome, enc.motivo))

        def _recuar(self, distancia, yaw_alvo, nome):
            """Re reta segurando o yaw pelo IMU (distancia em unidades do odom)."""
            self.get_logger().info('%s: saindo de re %.2f m (odom) para tentar de novo' % (
                nome, distancia))
            with self.lock:
                od = self.odom
            if od is None:
                return
            x0, y0 = od[1], od[2]
            t0 = time.monotonic()
            bloqueado_desde = None
            try:
                while rclpy.ok() and time.monotonic() - t0 < 12.0:
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    with self.lock:
                        od, cf = self.odom, self.cmd_final
                    if math.hypot(od[1] - x0, od[2] - y0) >= distancia:
                        break
                    agora = time.monotonic()
                    if self.cm_stop or agora - cf[0] > 0.5 or cf[1]:
                        bloqueado_desde = bloqueado_desde or agora
                        if agora - bloqueado_desde > 2.0:
                            self.get_logger().warn('%s: re bloqueada pelo Collision Monitor' % nome)
                            break
                    else:
                        bloqueado_desde = None
                    err = _ang(yaw_alvo - od[3])
                    wz = _sat(0.5 * err, 0.016, 0.03) if abs(err) > math.radians(1.0) else 0.0
                    self._publicar_vel(-0.03, 0.0, wz)
                    time.sleep(0.05)
            finally:
                self._parar()

        def _publicar_etapas(self, etapas):
            pa = PoseArray()
            pa.header.frame_id = 'map'
            pa.header.stamp = self.get_clock().now().to_msg()
            for e in etapas:
                pose = Pose()
                pose.position.x, pose.position.y = e['x'], e['y']
                pose.orientation.z = math.sin(e['yaw'] / 2.0)
                pose.orientation.w = math.cos(e['yaw'] / 2.0)
                pa.poses.append(pose)
            self.pub_poses.publish(pa)

        # ---------------- logica principal ----------------
        def trabalhador(self):
            while rclpy.ok():
                self.evento.wait(0.5)
                with self.lock:
                    goal = self.goal_pendente
                    area = self.area_pendente
                    self.goal_pendente = None
                    self.area_pendente = None
                    self.evento.clear()
                if goal is None:
                    continue
                try:
                    if self.encostado:
                        # de frente para a mesa a 8 cm, girar bate a quina: sai de re reto antes
                        self.encostado = False
                        with self.lock:
                            od = self.odom
                        if od is not None:
                            self._recuar(self.p('recuo_saida'), od[3], 'SAIR DA MESA')
                    self.executar(goal, area)
                except Cancelado:
                    self.get_logger().info('Goal substituido por um novo.')
                except Exception as exc:
                    self.get_logger().error('Erro: %s' % exc)

        def _analisar(self, gx, gy, tratadas, pontos=None, goal=None):
            mapa = self._mapa()
            if pontos is None:
                pontos = self._caminho(goal)
            if mapa is None or pontos is None:
                return [], 'sem mapa ou sem caminho'
            return planejar_etapas(mapa, pontos, (gx, gy), self.p('largura_estreita'),
                                   self.p('distancia_preparo'), self.p('distancia_saida'),
                                   ja_tratadas=tratadas)

        def executar(self, goal, area=None):
            gx, gy = goal.pose.position.x, goal.pose.position.y
            tratadas = []
            alerta = {}

            def vigia():
                with self.lock:
                    pontos, idade = self.plano, time.monotonic() - self.plano_t
                if pontos is None or idade > 3.0:
                    return False
                etapas, info = self._analisar(gx, gy, tratadas, pontos=pontos)
                if etapas:
                    alerta['etapas'], alerta['info'] = etapas, info
                    return True
                return False

            while True:
                if len(tratadas) >= int(self.p('max_passagens')):
                    etapas, info = [], 'limite de passagens atingido'
                elif 'etapas' in alerta:
                    etapas, info = alerta.pop('etapas'), alerta.pop('info')
                else:
                    etapas, info = self._analisar(gx, gy, tratadas, goal=goal)
                if etapas:
                    tratadas.append(info['entrada'])
                    self.get_logger().info(
                        'Passagem %d: entrada (%.2f, %.2f), eixo %.0f graus, largura no mapa '
                        '%.2f m, preparo a %.2f m da entrada' % (
                            len(tratadas), info['entrada'][0], info['entrada'][1],
                            info['eixo_graus'], info['largura_mapa'], info['distancia_preparo']))
                    self._publicar_etapas(etapas)
                    e = etapas[0]
                    ok = self._navegar(self._pose(e['x'], e['y'], e['yaw']),
                                       'PREPARO %d (alinhar fora da passagem)' % len(tratadas)) == 'ok'
                    if ok:
                        # De perto o mapa da passagem fica melhor: refina uma vez.
                        novas, ninfo = self._analisar(gx, gy, tratadas[:-1], goal=goal)
                        if novas and math.hypot(ninfo['entrada'][0] - info['entrada'][0],
                                                ninfo['entrada'][1] - info['entrada'][1]) < 0.40:
                            n0 = novas[0]
                            dyaw = abs(math.atan2(math.sin(n0['yaw'] - e['yaw']),
                                                  math.cos(n0['yaw'] - e['yaw'])))
                            dpos = math.hypot(n0['x'] - e['x'], n0['y'] - e['y'])
                            if dyaw > math.radians(4.0) or dpos > 0.04:
                                self.get_logger().info(
                                    'Refinando preparo: %.0f graus e %.0f cm de diferenca' % (
                                        math.degrees(dyaw), dpos * 100))
                                etapas, info = novas, ninfo
                                tratadas[-1] = info['entrada']
                                self._publicar_etapas(etapas)
                                e = etapas[0]
                                ok = self._navegar(self._pose(e['x'], e['y'], e['yaw']),
                                                   'PREPARO %d refinado' % len(tratadas)) == 'ok'
                    if not ok and self.p('modo_travessia') == 'lidar' and not self.p('fallback_nav2'):
                        self.get_logger().error(
                            'Preparo %d falhou. Parei fora da passagem; mande o goal de novo.' % len(tratadas))
                        return
                    if not ok:
                        self.get_logger().warn('Preparo falhou: seguindo para o goal final.')
                    elif self.p('modo_travessia') == 'lidar':
                        n = len(tratadas)
                        r = self._travessia_lidar(info, 'TRAVESSIA %d' % n)
                        if r != 'ok':
                            self.get_logger().warn('Travessia pelo LiDAR falhou: voltando ao preparo '
                                                   'para tentar de novo.')
                            if self._navegar(self._pose(e['x'], e['y'], e['yaw']),
                                             'PREPARO %d (2a tentativa)' % n) == 'ok':
                                r = self._travessia_lidar(info, 'TRAVESSIA %d (2a tentativa)' % n)
                        if r == 'ok':
                            continue  # reanalisa o caminho a partir da saida
                        if not (self.p('fallback_nav2') and len(etapas) > 1):
                            self.get_logger().error(
                                'Nao consegui atravessar a passagem %d. Parei fora dela; mande o '
                                'goal de novo (ou ajude o robo).' % n)
                            return
                        e = etapas[1]
                        if self._navegar(self._pose(e['x'], e['y'], e['yaw']),
                                         'TRAVESSIA %d (Nav2)' % n) == 'ok':
                            continue
                        self.get_logger().warn('Travessia falhou: seguindo para o goal final.')
                    elif len(etapas) > 1:
                        if self.p('modo_travessia') == 'lidar':
                            self.get_logger().warn('Travessia pelo LiDAR falhou: tentando pelo Nav2.')
                        e = etapas[1]
                        if self._navegar(self._pose(e['x'], e['y'], e['yaw']),
                                         'TRAVESSIA %d' % len(tratadas)) == 'ok':
                            continue  # reanalisa o caminho a partir da saida
                        self.get_logger().warn('Travessia falhou: seguindo para o goal final.')
                else:
                    self.get_logger().info('Analise: %s.' % info)
                # goal final, vigiando o caminho para passagens que aparecerem no mapa
                resultado = self._navegar(goal, 'GOAL FINAL',
                                          vigia if len(tratadas) < int(self.p('max_passagens')) else None)
                if resultado != 'interrompido':
                    if resultado == 'ok' and area is not None:
                        self._encostar(area)
                    return

    rclpy.init()
    no = Alinhador()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(no)
    try:
        executor.spin()
    except KeyboardInterrupt:
        no._cancelar_nav()
        no._parar()
    finally:
        executor.shutdown()
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
