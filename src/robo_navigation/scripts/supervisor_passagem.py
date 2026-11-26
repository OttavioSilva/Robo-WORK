"""SUPERVISOR_PASSAGEM_20261111: politica offline; integrado ao trabalhador existente.
Nao usa odometria de comandos como evidencia. Falha fechada em ambiente ICP ambiguo.
"""
import copy
import math
import uuid


def ang(a):
    return math.atan2(math.sin(a), math.cos(a))


class Supervisor:
    estados = {'NAV2', 'APROXIMACAO', 'TRANSFERENCIA', 'ALINHADOR', 'RETOMADA', 'FALHA'}

    def __init__(self, goal, log, modo='observar', identificador=None):
        if modo not in ('observar', 'aplicar'):
            raise ValueError(modo)
        self.goal = copy.deepcopy(goal)
        self.id = identificador or str(uuid.uuid4())
        self.modo, self.log = modo, log
        self.estado = None
        self.tratadas = []
        self.mudar('NAV2', 'novo pedido')

    def mudar(self, estado, motivo):
        if estado not in self.estados:
            raise ValueError(estado)
        self.log('SUPERVISOR goal=%s modo=%s %s -> %s: %s' % (
            self.id, self.modo, self.estado, estado, motivo))
        self.estado = estado


class Evidencia:
    """Integra incrementos ICP no mapa inicial e confere SLAM e IMU independentes.
    Cada incremento precisa ser observavel. Incerteza cresce; nunca reduz por tempo.
    """
    def __init__(self, pose, imu, stamp):
        if pose is None or imu is None or not all(math.isfinite(v) for v in (*pose, imu)):
            raise RuntimeError('pose/IMU inicial invalida')
        self.pose = tuple(pose)
        self.mapa = tuple(pose)
        self.imu = imu
        self.stamp = stamp
        self.incerteza = .06  # translacao inicial + projecao angular do footprint
        self.progresso = 0.0
        self.amostras = 0

    def atualizar(self, pose, imu, stamp, m):
        if stamp <= self.stamp:
            return
        dt = stamp-self.stamp
        if (pose is None or imu is None or not m.ok or dt > .8
                or not all(math.isfinite(v) for v in (*pose, imu, m.dx, m.dy, m.dth, m.residuo, m.ambiguidade))):
            raise RuntimeError('perda de confianca SLAM/ICP/IMU')
        if abs(ang(m.dth-ang(imu-self.imu))) > math.radians(4):
            raise RuntimeError('ICP diverge da IMU')
        c,s = math.cos(self.pose[2]), math.sin(self.pose[2])
        dx,dy = c*m.dx-s*m.dy, s*m.dx+c*m.dy
        if math.hypot(dx,dy) > .5*dt+.03:
            raise RuntimeError('salto ICP')
        if (math.hypot(pose[0]-self.mapa[0]-dx, pose[1]-self.mapa[1]-dy) > .10
                or abs(ang(pose[2]-self.mapa[2]-m.dth)) > math.radians(4)):
            raise RuntimeError('salto SLAM')
        self.pose = (self.pose[0]+dx,self.pose[1]+dy,ang(self.pose[2]+m.dth))
        if math.hypot(self.pose[0]-pose[0], self.pose[1]-pose[1]) > .15:
            raise RuntimeError('deriva acumulada SLAM/ICP')
        self.mapa, self.imu, self.stamp = tuple(pose), imu, stamp
        self.incerteza += max(.001, m.residuo*.1, m.ambiguidade*.1)
        self.progresso += math.hypot(dx,dy)
        self.amostras += 1
        if self.incerteza > .14:
            raise RuntimeError('incerteza ICP acumulada excessiva')

    def completa(self, saida, eixo, footprint, livre, parado, margem=.05):
        def alem(pose):
            c,s=math.cos(pose[2]),math.sin(pose[2])
            return min((pose[0]+c*x-s*y-saida[0])*eixo[0]+
                       (pose[1]+s*x+c*y-saida[1])*eixo[1] for x,y in footprint)
        return (self.amostras >= 2 and self.progresso > .10 and livre and parado
                and min(alem(self.pose),alem(self.mapa)) > margem+self.incerteza)


def saida_livre(pontos, eixo_local, distancia=.45):
    # Exige retornos dos dois lados: ausencia de retornos nao significa espaco livre.
    ux,uy=eixo_local
    frente=[(x*ux+y*uy,-x*uy+y*ux) for x,y in pontos if math.isfinite(x) and math.isfinite(y)]
    return (sum(a > .4 and b > .4 for a,b in frente) >= 5
            and sum(a > .4 and b < -.4 for a,b in frente) >= 5
            and not any(0 < a < distancia and abs(b) < .38 for a,b in frente))


class Passagens:
    """Latch entre goals: rearma somente numa nova aproximacao pelo lado da entrada."""
    def __init__(self):
        self.concluidas = []

    def concluir(self, entrada, eixo):
        self.concluidas.append((tuple(entrada),tuple(eixo)))

    def bloqueadas(self, pose):
        restantes=[]
        for entrada,eixo in self.concluidas:
            dx,dy=pose[0]-entrada[0],pose[1]-entrada[1]
            if not (dx*eixo[0]+dy*eixo[1] < -.65 and abs(-dx*eixo[1]+dy*eixo[0]) < .30):
                restantes.append((entrada,eixo))
        self.concluidas=restantes
        return [entrada for entrada,_ in restantes]


def regiao_conhecida(info, regioes):
    """Whitelist geometrica independente do obstaculo transitorio no costmap/SLAM."""
    if not info or 'saida' not in info:
        return False
    for r in regioes:
        tolerancia = min(.30, float(r.get('tolerancia', .20)))
        if tolerancia <= 0:
            continue
        for entrada,saida in ((r['entrada'],r['saida']),(r['saida'],r['entrada'])):
            comprimento = math.hypot(saida[0]-entrada[0],saida[1]-entrada[1])
            if (comprimento >= .45 and math.dist(info['entrada'],entrada) <= tolerancia
                    and math.dist(info['saida'],saida) <= tolerancia
                    and abs(info['largura_mapa']-r['largura']) <= .12):
                return True
    return False
