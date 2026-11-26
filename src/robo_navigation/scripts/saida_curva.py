"""Reconhecimento da saída do corredor durante o deslocamento lateral."""
import math


def abertura_observada(pontos, laser, sentido, distancia, corpo):
    """Raios com retorno além da abertura devem cobrir a largura do robô."""
    lx, ly = laser[:2]
    face = max(sentido * x for x, _ in corpo)
    meia = max(abs(y) for _, y in corpo) + 0.025
    limite = face + distancia
    faixas = []
    for x, y in pontos:
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        proj = sentido * x
        if face < proj < limite and abs(y) <= meia:
            return False
        if proj >= limite and proj > sentido * lx:
            yc = ly + (y - ly) * (limite - sentido * lx) / (proj - sentido * lx)
            if abs(yc) <= meia + 0.07:
                faixas.append(max(-meia, min(meia, yc)))
    faixas.sort()
    return (len(faixas) >= 5 and faixas[0] <= -meia + 0.07 and faixas[-1] >= meia - 0.07
            and all(b - a <= 2 * meia / 3 for a, b in zip(faixas, faixas[1:])))


class SaidaCurva:
    def __init__(self, curva, corpo, abertura):
        self.curva, self.corpo, self.abertura = curva, corpo, abertura
        self.odom_dentro = None
        self.ultimo_scan = None
        self.poses_anteriores = None
        self.lado = 0
        self.leituras = 0
        self.inicio = 0.0
        self.finalizando = 0
        self.inicio_final = None
        self.odom_final = None
        self.parado_desde = None
        self.odom_parado = None
        self.pronto_afastar = False
        self.primeiro_afastamento = None

    def interromper(self):
        self.lado = self.leituras = 0

    def _area_aberta(self, mapa, pose, sentido, cantos):
        # Um T pode abrir de um lado; um + pode abrir dos dois. Procura no
        # mapa a parede que reaparece depois do vão, inclusive no eixo de saída.
        bx, by = self.curva['eixo_b']
        nx, ny = sentido * self.curva['lado'] * by, -sentido * self.curva['lado'] * bx
        largura = max(x * nx + y * ny for x, y in cantos) + self.abertura
        fundo = min(x * bx + y * by for x, y in cantos)
        passo = mapa.res / 2
        # 1,30 m é o vão entre trechos que o planejador considera uma interseção.
        n = math.ceil((1.30 - fundo) / passo)
        m = math.ceil(largura / passo)
        for i in range(n + 1):
            u = fundo + (1.30 - fundo) * i / n
            x, y = pose[0] + bx * u, pose[1] + by * u
            for j in range(m + 1):
                v = largura * j / m
                if mapa.estado(x + nx * v, y + ny * v) != 0:
                    return False
        return True

    def observar(self, stamp, pontos, laser, mapa, pose, odom, duas_paredes):
        """Devolve +1 (frente), -1 (trás) ou 0. Ausência de retorno é inconclusiva."""
        if self.ultimo_scan is not None:
            intervalo = stamp - self.ultimo_scan
            if intervalo <= 0:
                if intervalo < 0:
                    self.interromper()
                return 0
            if intervalo > 0.45:
                self.interromper()
        self.ultimo_scan = stamp
        if self.poses_anteriores is not None:
            pm, od = self.poses_anteriores
            delta = pm[2] - od[2]
            cx, sx = math.cos(delta), math.sin(delta)
            ox, oy = odom[0] - od[0], odom[1] - od[1]
            erro_dist = math.hypot(pose[0] - pm[0] - (cx*ox - sx*oy),
                                   pose[1] - pm[1] - (sx*ox + cx*oy))
            erro_giro = math.remainder((pose[2] - pm[2]) - (odom[2] - od[2]), 2 * math.pi)
            if erro_dist > 0.15 or abs(erro_giro) > math.radians(8):
                self.odom_dentro = None
                self.interromper()
        self.poses_anteriores = (pose, odom)
        bx, by = self.curva['eixo_b']
        dx, dy = pose[0] - self.curva['saida_b'][0], pose[1] - self.curva['saida_b'][1]
        avanco = dx * bx + dy * by
        if avanco <= 0 and duas_paredes and self.odom_dentro is None:
            self.odom_dentro = odom
        c, s = math.cos(pose[2]), math.sin(pose[2])
        cantos = [(c*x - s*y, s*x + c*y) for x, y in self.corpo]
        lado = 0
        if (mapa is not None and self.odom_dentro is not None
                and not self.curva.get('outra_juncao') and self.abertura > 0
                and abs(dx * by - dy * bx) <= 0.30
                and self.curva['lado'] * (-s * bx + c * by) >= math.cos(math.radians(20))
                and avanco + min(x * bx + y * by for x, y in cantos) >= 0.05):
            ox, oy, yaw = self.odom_dentro
            progresso = self.curva['lado'] * (-(odom[0]-ox)*math.sin(yaw) + (odom[1]-oy)*math.cos(yaw))
            if progresso >= 0.15:
                for sentido in (1, -1):
                    if (abertura_observada(pontos, laser, sentido, self.abertura, self.corpo)
                            and self._area_aberta(mapa, pose, sentido, cantos)):
                        lado = sentido
                        break
        if lado != self.lado or not lado:
            self.interromper()
            self.lado = lado
            self.inicio = stamp
        if lado:
            self.leituras += 1
        return lado if self.leituras >= 3 and stamp - self.inicio >= 0.25 else 0

    def finalizar(self, t, lado, odom, folga_oposta, oposto_aberto, protecao_ok, parado):
        """Só afasta da parede oposta com abertura confirmada e movimento limitado."""
        if self.inicio_final is None:
            self.finalizando = lado
            self.inicio_final, self.odom_final = t, odom
        if t - self.inicio_final > 6.0 or math.dist(odom[:2], self.odom_final[:2]) > 0.12:
            return 'bloqueada'
        if lado == self.finalizando and protecao_ok:
            if folga_oposta is not None and 0 <= folga_oposta < 0.07:
                self.parado_desde = None
                if not self.pronto_afastar:
                    self.pronto_afastar = parado and t - self.inicio_final >= 0.25
                    return 'parar'
                if self.primeiro_afastamento is None:
                    self.primeiro_afastamento = t
                if t - self.primeiro_afastamento > 1.5:
                    return 'bloqueada'
                return 'afastar'
            folga_ok = oposto_aberto if folga_oposta is None else folga_oposta >= 0.07
            if folga_ok and parado:
                if (self.parado_desde is None or math.dist(odom[:2], self.odom_parado[:2]) > 0.005
                        or abs(math.remainder(odom[2] - self.odom_parado[2], 2*math.pi)) > math.radians(1)):
                    self.parado_desde, self.odom_parado = t, odom
                if t - self.parado_desde >= 0.45:
                    return 'fim'
                return 'parar'
        self.parado_desde = None
        self.pronto_afastar = False
        return 'parar'
