"""Nucleo conservador da curva em T; sem dependencias ROS."""
import math
import time
from dataclasses import dataclass
from enum import Enum

ZERO = (0.0, 0.0, 0.0)


def ang(a):
    return math.atan2(math.sin(a), math.cos(a))


class Observacao(Enum):
    LIVRE = 'livre'
    OBSTACULO = 'obstaculo'
    DESCONHECIDA = 'desconhecida'


def avaliar_varredura(ranges, amin, inc, rmin, rmax, laser, raio, caixa=None,
                       movimento_angular=0.0, movimento_linear=0.0):
    """Cobertura por TODOS os raios nativos, sem preencher setores com vizinhos.

    Fechamento de 360 graus e nenhum raio invalido. Bin pode representar um
    retorno em qualquer angulo de sua celula. A meia lacuna entre dois retornos
    adjacentes no pior caso (desvio de um bin) deve caber na reserva de 5 cm;
    esse valor e ADICIONADO a folga, nunca subtraido das margens existentes.
    Nao comprova detectar objetos menores que a lacuna entre feixes.
    Obstaculo tem precedencia sobre desconhecido; infinito nao prova espaco livre.
    """
    lx, ly, yaw = laser
    if not all(math.isfinite(v) for v in (amin, inc, rmin, rmax, lx, ly, yaw, raio)):
        return Observacao.DESCONHECIDA
    if (not math.isfinite(movimento_angular) or movimento_angular < 0 or
            not math.isfinite(movimento_linear) or movimento_linear < 0):
        return Observacao.DESCONHECIDA
    if (len(ranges) < 3 or inc == 0
            or rmin < 0 or rmax <= rmin or math.hypot(lx, ly) >= raio):
        return Observacao.DESCONHECIDA
    # Cobertura deve fechar uma volta, permitindo apenas a celula do ultimo raio.
    cobertura = abs(inc)*len(ranges)
    desconhecida = abs(cobertura-2*math.pi) > 1e-5 or abs(inc) >= math.pi/2
    if caixa is not None:
        xmin, xmax, ymin, ymax = caixa
        if not xmin < lx < xmax or not ymin < ly < ymax:
            return Observacao.DESCONHECIDA
    for i, r in enumerate(ranges):
        a = amin+i*inc+yaw
        c, sn = math.cos(a), math.sin(a)
        proj = lx*c+ly*sn
        if caixa is None:
            saida = -proj+math.sqrt(proj*proj+raio*raio-lx*lx-ly*ly)
        else:
            tx = ((xmax if c > 0 else xmin)-lx)/c if abs(c) > 1e-12 else math.inf
            ty = ((ymax if sn > 0 else ymin)-ly)/sn if abs(sn) > 1e-12 else math.inf
            saida = min(tx, ty)
        # Sem ordem temporal por bin: o pior fechamento inclui toda a excursao
        # angular permitida durante a janela, alem da incerteza de binning.
        lacuna = saida*math.sin(min(abs(inc)+movimento_angular/2, math.pi/2))
        if lacuna > 0.05:
            desconhecida = True
        if not math.isfinite(r) or not rmin <= r <= rmax:
            desconhecida = True
        elif r <= saida+0.025+lacuna+movimento_linear:
            return Observacao.OBSTACULO
    return Observacao.DESCONHECIDA if desconhecida else Observacao.LIVRE


def disco_observado(*args, **kwargs):
    return avaliar_varredura(*args, **kwargs) is Observacao.LIVRE


def fonte_exclusiva(endpoints, gid, esperado='alinhador_passagem'):
    gid = bytes(gid)
    return (bool(gid) and any(gid) and len(endpoints) == 1
            and endpoints[0].node_name == esperado and bytes(endpoints[0].endpoint_gid) == gid)


def envolve_disco(points, radius):
    """Poligono convexo CCW envolvendo um disco; usado na auditoria dos params ativos."""
    if len(points) < 3 or any(len(p) != 2 or not all(math.isfinite(x) for x in p) for p in points):
        return False
    for i, a in enumerate(points):
        b = points[(i+1)%len(points)]
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if length == 0 or (a[0]*b[1]-a[1]*b[0])/length < radius-1e-6:
            return False
        # Todos os vertices devem estar na mesma meia-regiao de cada aresta.
        if any((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]) < -1e-6 for p in points):
            return False
    return True


@dataclass(frozen=True)
class TempoSensor:
    stamp_ns: int          # carimbo original, nunca substituido por recepcao
    ros_rx_ns: int
    mono_rx: float
    atraso_pre_stamp: float

    @classmethod
    def receber(cls, stamp, ros_ns, mono, atraso=0.0, anterior=None):
        sec, nano = getattr(stamp, 'sec', None), getattr(stamp, 'nanosec', None)
        if (not isinstance(sec, int) or not isinstance(nano, int) or sec < 0
                or not 0 <= nano < 1_000_000_000 or sec+nano == 0
                or not math.isfinite(atraso) or atraso < 0):
            return None
        ns = sec*1_000_000_000+nano
        if ns > ros_ns or (anterior is not None and ns <= anterior):
            return None
        sample = cls(ns, ros_ns, mono, atraso)
        return sample if sample.recente(ros_ns, mono) else None

    @property
    def aquisicao_mono(self):
        return self.mono_rx-(self.ros_rx_ns-self.stamp_ns)/1e9-self.atraso_pre_stamp

    def recente(self, ros_ns, mono, limite=0.3):
        elapsed = mono-self.mono_rx
        ros_elapsed = (ros_ns-self.ros_rx_ns)/1e9
        # Mudanca de relogio/pausa simulada nao renova a validade.
        if elapsed < 0 or abs(ros_elapsed-elapsed) > 0.05:
            return False
        idade = max(mono-self.aquisicao_mono,
                    (ros_ns-self.stamp_ns)/1e9+self.atraso_pre_stamp)
        return 0 <= idade <= limite


def devolver_controle(cliente, pedido, publicar_zero, relogio=time.monotonic,
                      esperar=time.sleep, timeout=1.0):
    """RPC de devolucao para estado PARADO; jamais presume sucesso por timeout.

    Publica somente na entrada experimental durante a espera. O servidor estaciona
    antes de responder, e seu watchdog estaciona se a requisicao nao chegar.
    Resposta tardia e idempotente e nao desbloqueia fontes normais.
    """
    try:
        futuro = cliente.call_async(pedido)
        limite = relogio()+timeout
        while not futuro.done():
            publicar_zero()
            if relogio() >= limite:
                return False, 'timeout na devolucao'
            esperar(0.02)
        resposta = futuro.result()
        if resposta is None or not resposta.success or resposta.message != 'parado_confirmado':
            return False, 'devolucao recusada ou estado nao confirmado'
        return True, ''
    except Exception as exc:
        return False, 'excecao na devolucao: '+str(exc)
    finally:
        publicar_zero()


class GiroFrontal:
    def __init__(self, centro, eixo, saida, margem=0.60, timeout=90.0):
        self.centro, self.eixo, self.saida = centro, eixo, saida
        self.margem, self.timeout = margem, timeout
        self.estado, self.motivo = 'parar', ''
        self.t0 = self.estavel = self.progresso_t = None
        self.ref_pose = self.alvo_imu = self.alvo_mapa = None
        self.melhor = None

    def abortar(self, motivo):
        self.estado, self.motivo = 'abortar', motivo
        return ZERO

    def passo(self, t, pose, imu, scan_t, livre, parado, cancelado=False, bloqueado=False):
        if self.estado in ('fim', 'abortar'):
            return ZERO
        if self.t0 is None:
            self.t0 = self.progresso_t = t
        if cancelado:
            return self.abortar('cancelamento')
        if t - self.t0 > self.timeout:
            return self.abortar('timeout')
        if pose is None or not all(math.isfinite(v) for v in pose):
            return self.abortar('pose ausente/invalida')
        if imu is None or not all(math.isfinite(v) for v in imu) or not 0 <= t-imu[0] <= 0.3:
            return self.abortar('IMU ausente/invalida/desatualizada')
        if scan_t is None or not 0 <= t-scan_t <= 0.3:
            return self.abortar('LiDAR ausente/desatualizado')
        if not livre:
            return self.abortar('obstaculo ou area nao observada')
        if bloqueado:
            return self.abortar('protecao ou conflito de controle')
        if self.estado == 'parar':
            if math.hypot(pose[0]-self.centro[0], pose[1]-self.centro[1]) > 0.06:
                return self.abortar('fora do centro seguro do T')
            if self.ref_pose is None:
                self.ref_pose = pose
            moveu = math.hypot(pose[0]-self.ref_pose[0], pose[1]-self.ref_pose[1]) > 0.005
            if not parado or abs(imu[2]) > 0.03 or moveu:
                self.estavel, self.ref_pose = None, pose
                return ZERO
            if self.estavel is None:
                self.estavel = t
            if t-self.estavel >= 0.6:
                self.alvo_mapa = math.atan2(self.eixo[1], self.eixo[0])
                delta = ang(self.alvo_mapa-pose[2])
                if not math.radians(45) <= abs(delta) <= math.radians(135):
                    return self.abortar('direcao de destino incompativel com curva em T')
                self.alvo_imu = ang(imu[1]+delta)
                self.estado, self.estavel = 'girar', None
                self.melhor, self.progresso_t = abs(delta), t
            return ZERO
        if self.estado in ('girar', 'confirmar'):
            if math.hypot(pose[0]-self.centro[0], pose[1]-self.centro[1]) > 0.06:
                return self.abortar('base deslocou durante o giro')
            erro = ang(self.alvo_imu-imu[1])
            if abs(erro) < self.melhor - math.radians(1):
                self.melhor, self.progresso_t = abs(erro), t
            if t-self.progresso_t > 6.0:
                return self.abortar('giro sem progresso')
            if abs(erro) <= math.radians(3):
                self.estado = 'confirmar'
                if not parado or abs(imu[2]) > 0.03:
                    self.estavel = None
                    return ZERO
                if self.estavel is None:
                    self.estavel = t
                if t-self.estavel >= 0.5:
                    if abs(ang(self.alvo_mapa-pose[2])) > math.radians(5):
                        return self.abortar('orientacao IMU/mapa divergente')
                    self.estado, self.melhor, self.progresso_t = 'avancar', None, t
                return ZERO
            self.estado, self.estavel = 'girar', None
            return 0.0, 0.0, math.copysign(min(0.10, max(0.025, 0.5*abs(erro))), erro)
        erro = ang(self.alvo_imu-imu[1])
        if abs(erro) > math.radians(5):
            return self.abortar('orientacao perdida no avanco')
        restante = self.margem - ((pose[0]-self.saida[0])*self.eixo[0] +
                                  (pose[1]-self.saida[1])*self.eixo[1])
        if restante <= 0.02:
            self.estado = 'fim'
            return ZERO
        if self.melhor is None or restante < self.melhor-0.015:
            self.melhor, self.progresso_t = restante, t
        if t-self.progresso_t > 5.0:
            return self.abortar('avanco sem progresso')
        return 0.03, 0.0, max(-0.025, min(0.025, 0.4*erro))


class ArbitragemT:
    """Uma unica saida, lease experimental, conflito e watchdog travam em zero."""
    def __init__(self):
        self.ativo = self.falha = False
        self.ultimo_normal = self.ultimo_t = None
        self.estacionado = False
        self.zero_desde = None

    def adquirir(self, t):
        if self.ativo or self.falha or self.estacionado or (self.ultimo_normal is not None and t-self.ultimo_normal < 0.6):
            return False
        self.ativo, self.falha, self.ultimo_t = True, False, t
        return True

    def normal(self, t, cmd):
        if not all(math.isfinite(v) for v in cmd):
            self.falha, self.estacionado, self.ativo = True, True, False
            return ZERO
        if any(abs(v) > 1e-5 for v in cmd):
            self.ultimo_normal = t
            if self.ativo:
                self.falha = True
        return ZERO if self.ativo or self.falha or self.estacionado else cmd

    def experimental(self, t, cmd):
        if not all(math.isfinite(v) for v in cmd):
            self.falha, self.estacionado, self.ativo = True, True, False
            return ZERO
        if not self.ativo:
            return ZERO
        self.ultimo_t = t
        if cmd == ZERO:
            if self.zero_desde is None:
                self.zero_desde = t
        else:
            self.zero_desde = None
        return ZERO if self.falha else cmd

    def vigiar(self, t):
        if self.ativo and t-self.ultimo_t > 0.25:
            self.falha, self.estacionado, self.ativo = True, True, False
        return self.ativo

    def liberar(self, t):
        # Idempotencia permite recuperar uma resposta perdida sem reativar nada.
        if self.estacionado:
            return not self.falha
        ok = (self.ativo and not self.falha and self.zero_desde is not None
              and t-self.zero_desde >= 0.2 and t-self.ultimo_t <= 0.25)
        self.ativo, self.estacionado = False, True
        self.falha = self.falha or not ok
        return ok

    def resetar(self, t):
        # Somente apos o operador cancelar as fontes normais e confirmar a area.
        if (self.ativo or (self.ultimo_t is not None and t-self.ultimo_t < 0.6)
                or (self.ultimo_normal is not None and t-self.ultimo_normal < 0.6)):
            return False
        self.ativo = self.falha = self.estacionado = False
        self.zero_desde = None
        return True
