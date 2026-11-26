#!/usr/bin/env python3
"""Service Areas (mesas 80 x 50 cm do RoboCup@Work Entry-Level): deteccao no mapa ao vivo.

- Le o /map do SLAM, acha mesas de ~80 x 50 cm que se projetam da parede ou ficam soltas,
  e acompanha cada uma ao longo do tempo (C1, C2, ...). Uma candidata so aparece depois
  de ser vista em 3 mapas seguidos.
- RViz: adicione um MarkerArray no topico /service_areas/marcadores.
  Amarelo = candidata sem nome; verde = com nome. A seta mostra a pose de aproximacao
  (60 cm na frente da face, olhando para a mesa).
- Dar nome (ID da prova) a uma candidata:
    ros2 topic pub --once /service_areas/nomear std_msgs/msg/String "{data: 'C1 WS_1 10'}"
  (o ultimo numero e a altura da mesa em cm: 5, 10 ou 15; opcional, padrao 10)
  Apagar: "{data: 'apagar C3'}"
- Salva tudo em ~/mapas/service_areas.yaml (formato aceito pelo atwork_commander,
  com a pose da face e da aproximacao a mais). As coordenadas valem para o mapa desta
  sessao (o SLAM recomeca do zero a cada vez que o robo liga).

Uso:
    python3 service_areas.py
    python3 service_areas.py --ros-args -p arquivo:=/home/marcos/mapas/service_areas.yaml
"""
import math
import os
import threading
import time

import numpy as np

# ---------------------------------------------------------------------------
# Deteccao (sem ROS)
# ---------------------------------------------------------------------------

VIZ8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _celulas_borda(g, limiar=65):
    occ = g >= limiar
    free = g == 0
    h, w = g.shape
    pad_free = np.zeros((h + 2, w + 2), bool)
    pad_free[1:-1, 1:-1] = free
    nx = np.zeros((h, w))
    ny = np.zeros((h, w))
    for di, dj in VIZ8:
        f = pad_free[1 + di:h + 1 + di, 1 + dj:w + 1 + dj]
        nx += f * dj
        ny += f * di
    borda = occ & ((nx != 0) | (ny != 0))
    jj, ii = np.nonzero(borda)  # jj = linha (y), ii = coluna (x)
    n = np.hypot(nx[jj, ii], ny[jj, ii])
    return ii, jj, nx[jj, ii] / n, ny[jj, ii] / n


def segmentos(g, res, ox, oy, passo_graus=3, tol_normal=50, comp_min=0.25, gap=0.20):
    """Segmentos de borda: dict(c, t, n, L, nc, pts)."""
    ii, jj, nx, ny = _celulas_borda(g)
    if len(ii) == 0:
        return []
    px = ox + (ii + 0.5) * res
    py = oy + (jj + 0.5) * res
    cand = []
    cos_tol = math.cos(math.radians(tol_normal))
    for k in range(0, 360, passo_graus):
        a = math.radians(k)
        n = np.array([math.cos(a), math.sin(a)])
        t = np.array([-n[1], n[0]])
        sel = (nx * n[0] + ny * n[1]) >= cos_tol
        if sel.sum() < 5:
            continue
        x, y = px[sel], py[sel]
        v = x * n[0] + y * n[1]
        u = x * t[0] + y * t[1]
        bins = np.round(v / 0.02).astype(int)
        valores, contagem = np.unique(bins, return_counts=True)
        for b, cnt in zip(valores, contagem):
            if cnt < 5:
                continue
            m = np.abs(v - b * 0.02) <= 0.03
            us = np.sort(u[m])
            corte = np.nonzero(np.diff(us) > gap)[0]
            ini = 0
            for fim in list(corte) + [len(us) - 1]:
                pedaco = us[ini:fim + 1]
                ini = fim + 1
                comp = pedaco[-1] - pedaco[0] + res
                if comp >= comp_min and len(pedaco) >= 5:
                    cand.append((len(pedaco), k, b * 0.02, pedaco[0], pedaco[-1]))
    # supressao de repetidos (mesmo segmento achado em angulos/offsets vizinhos)
    cand.sort(key=lambda c: -c[0])
    finais = []
    for c in cand:
        cnt, k, v0, u0, u1 = c
        dup = False
        for f in finais:
            dk = abs((k - f[1] + 180) % 360 - 180)
            sob = min(u1, f[4]) - max(u0, f[3])
            if dk <= 9 and abs(v0 - f[2]) <= 0.05 and sob > 0.5 * (u1 - u0):
                dup = True
                break
        if not dup:
            finais.append(c)
    segs = []
    for cnt, k, v0, u0, u1 in finais:
        a = math.radians(k)
        n = np.array([math.cos(a), math.sin(a)])
        t = np.array([-n[1], n[0]])
        um = (u0 + u1) / 2.0
        c = n * v0 + t * um
        segs.append(dict(c=c, t=t, n=n, L=u1 - u0 + res, nc=cnt, ang=k))
    return segs


def _livre(g, res, ox, oy, x, y):
    i = int(math.floor((x - ox) / res))
    j = int(math.floor((y - oy) / res))
    if 0 <= j < g.shape[0] and 0 <= i < g.shape[1]:
        return g[j, i]
    return -1



def parede_continua(g, res, ox, oy, c, t, n, L,
                    limite=1.00, tolerancia=0.07):
    """Verifica se a face continua alinhada alem das extremidades."""
    if L >= limite:
        return True

    extensao = 0.0
    passo = min(res / 2.0, 0.025)

    for sinal in (-1, 1):
        comprimento = 0.0
        falha = 0.0

        for d in np.arange(passo, limite, passo):
            ponta = c + t * sinal * (L / 2 + d)

            ocupado = False
            for desvio in np.arange(-tolerancia,
                                     tolerancia + 0.001, passo):
                pos = ponta + n * desvio
                if _livre(g, res, ox, oy,
                          pos[0], pos[1]) >= 65:
                    ocupado = True
                    break

            if ocupado:
                comprimento = d
                falha = 0.0
            else:
                falha += passo
                if falha > 0.08:
                    break

        extensao += comprimento

        if L + extensao >= limite:
            return True

    return False


def detectar(g, res, ox, oy, largura=0.80, profundidade=0.50, tol_face=0.15):
    """Lista de mesas: dict(face=(x,y), yaw=direcao do robo olhando a mesa, centro, ...)."""
    segs = segmentos(g, res, ox, oy)
    mesas = []
    for f in segs:
        if not (0.70 <= f['L'] <= largura + tol_face):
            continue
        c, t, n, L = f['c'], f['t'], f['n'], f['L']
        if parede_continua(g, res, ox, oy, c, t, n, L):
            continue

        # 1) frente livre: faixa de 10 a 40 cm na frente da face
        tot = livres = ocup = 0
        for d in np.arange(0.10, 0.41, 0.05):
            for s in np.arange(-L / 2 + 0.05, L / 2 - 0.049, 0.05):
                p = c + n * d + t * s
                v = _livre(g, res, ox, oy, p[0], p[1])
                tot += 1
                livres += v == 0
                ocup += v >= 65
        if tot == 0 or livres / tot < 0.6 or ocup / tot > 0.05:
            continue
        # 2) laterais: segmentos com normal ~ +-t (para fora da mesa), comecando na ponta
        #    da face e indo para tras (sentido -n) por ~profundidade
        lados = []
        for sinal in (+1, -1):
            ponta = c + t * sinal * L / 2
            melhor = None
            for s in segs:
                if s is f:
                    continue
                if np.dot(s['n'], t * sinal) < math.cos(math.radians(30)):
                    continue  # lateral tem de olhar para fora da mesa
                # distancia da ponta da face a reta da lateral
                if abs(np.dot(ponta - s['c'], s['n'])) > 0.07:
                    continue
                # extensao da lateral medida para tras da face
                a = np.dot((s['c'] + s['t'] * s['L'] / 2) - c, -n)
                b = np.dot((s['c'] - s['t'] * s['L'] / 2) - c, -n)
                ini, fim = min(a, b), max(a, b)
                if ini > 0.10 or fim < 0.20:
                    continue  # tem de comecar perto da face e ir para tras
                if melhor is None or fim > melhor:
                    melhor = fim
            if melhor is not None:
                lados.append(min(melhor, 2.0))
        if lados and max(lados) > profundidade + 0.25:
            continue  # lateral longa demais: canto de parede, nao mesa
        # 3) a mesa se projeta: dos dois lados, logo atras da linha da face, tem chao livre
        #    (num degrau/canto de parede um dos lados e parede)
        livre_lado = []
        for sinal in (+1, -1):
            tl = fl = 0
            for d in (0.15, 0.25, 0.35):
                for e in (0.10, 0.15, 0.20):
                    p = c - n * d + t * sinal * (L / 2 + e)
                    tl += 1
                    fl += _livre(g, res, ox, oy, p[0], p[1]) == 0
            livre_lado.append(fl / tl)
        # 4) atras da face nao e chao livre (mesa ocupada ou nao vista)
        tot = nlivre = 0
        for d in np.arange(0.05, 0.46, 0.05):
            for s in np.arange(-L / 2 + 0.10, L / 2 - 0.099, 0.05):
                p = c - n * d + t * s
                tot += 1
                nlivre += _livre(g, res, ox, oy, p[0], p[1]) != 0
        if tot == 0 or nlivre / tot < 0.4:
            continue
        if not lados or (min(livre_lado) < 0.6 and len(lados) < 2):
            continue  # precisa de pelo menos uma lateral e de se projetar da parede
        yaw = math.atan2(-n[1], -n[0])  # robo de frente para a mesa
        mesas.append(dict(face=(float(c[0]), float(c[1])), yaw=yaw, L=float(L),
                          lados=[round(x, 2) for x in lados],
                          centro=(float(c[0] - n[0] * profundidade / 2),
                                  float(c[1] - n[1] * profundidade / 2)),
                          livre_lado=[round(x, 2) for x in livre_lado],
                          pontuacao=len(lados) + min(livre_lado) + (1 - abs(L - largura) / tol_face)))
    # duas faces da mesma mesa (ex.: face e uma lateral confundida): fica a melhor
    mesas.sort(key=lambda m: -m['pontuacao'])
    final = []
    for m in mesas:
        if all(math.hypot(m['centro'][0] - o['centro'][0], m['centro'][1] - o['centro'][1]) > 0.35
               for o in final):
            final.append(m)
    return final


# ---------------------------------------------------------------------------
# Acompanhamento ao longo do tempo (sem ROS)
# ---------------------------------------------------------------------------

def _dang(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class Rastreador:
    """Junta as deteccoes de mapas sucessivos em candidatas estaveis."""

    def __init__(self, confirmar=3, esquecer_s=60.0):
        self.areas = []      # dicts: rotulo, face, yaw, L, vistas, ultima, nome, altura
        self.n = 0
        self.confirmar = confirmar
        self.esquecer = esquecer_s

    def atualizar(self, deteccoes, agora):
        for d in deteccoes:
            alvo = None
            for a in self.areas:
                if (math.hypot(a['face'][0] - d['face'][0], a['face'][1] - d['face'][1]) < 0.25
                        and _dang(a['yaw'], d['yaw']) < math.radians(20)):
                    alvo = a
                    break
            if alvo is None:
                self.n += 1
                self.areas.append(dict(rotulo='C%d' % self.n, face=d['face'], yaw=d['yaw'],
                                       L=d['L'], vistas=1, ultima=agora, nome=None, altura=0.10))
                continue
            k = 0.5  # media movel: o SLAM mexe o mapa um pouco a cada atualizacao
            alvo['face'] = (alvo['face'][0] + k * (d['face'][0] - alvo['face'][0]),
                            alvo['face'][1] + k * (d['face'][1] - alvo['face'][1]))
            alvo['yaw'] = math.atan2(math.sin(alvo['yaw']) + k * (math.sin(d['yaw']) - math.sin(alvo['yaw'])),
                                     math.cos(alvo['yaw']) + k * (math.cos(d['yaw']) - math.cos(alvo['yaw'])))
            alvo['L'] = alvo['L'] + k * (d['L'] - alvo['L'])
            alvo['vistas'] += 1
            alvo['ultima'] = agora
        # esquece candidatas sem nome que sumiram (pedacos de parede que o SLAM corrigiu)
        self.areas = [a for a in self.areas
                      if a['nome'] or (agora - a['ultima'] < (self.esquecer if a['vistas'] >= self.confirmar else 10.0))]

    def confirmadas(self):
        return [a for a in self.areas if a['nome'] or a['vistas'] >= self.confirmar]

    def nomear(self, rotulo, nome, altura=None):
        for a in self.areas:
            if a['rotulo'] == rotulo.upper() or (a['nome'] and a['nome'] == rotulo.upper()):
                for b in self.areas:  # o nome e unico
                    if b is not a and b['nome'] == nome.upper():
                        b['nome'] = None
                a['nome'] = nome.upper()
                if altura is not None:
                    a['altura'] = altura
                return a
        return None

    def apagar(self, rotulo):
        antes = len(self.areas)
        self.areas = [a for a in self.areas if a['rotulo'] != rotulo.upper() and a['nome'] != rotulo.upper()]
        return len(self.areas) < antes


def tipo_do_nome(nome):
    for pref, tipo in (('WS', 'workstation'), ('SH', 'shelf'), ('PP', 'precision_placement'),
                       ('ST', 'stacking')):
        if nome.startswith(pref):
            return tipo
    return 'workstation'


def poses(a, distancia_aprox=0.60):
    """Face (centro, robo olhando a mesa) e pose de aproximacao a distancia_aprox da face."""
    fx, fy = a['face']
    yaw = a['yaw']
    ax = fx - math.cos(yaw) * distancia_aprox
    ay = fy - math.sin(yaw) * distancia_aprox
    return (fx, fy, yaw), (ax, ay, yaw)


def documento_yaml(areas, distancia_aprox=0.60):
    doc = {'frame': 'map', 'gerado_em': time.strftime('%Y-%m-%d %H:%M:%S'), 'service_areas': {}}
    for a in sorted(areas, key=lambda a: a['nome']):
        face, aprox = poses(a, distancia_aprox)
        doc['service_areas'][a['nome']] = {
            'type': tipo_do_nome(a['nome']),
            'table': {'width': 0.8, 'depth': 0.5, 'height': round(a['altura'], 2)},
            'face': {'x': round(face[0], 3), 'y': round(face[1], 3),
                     'yaw': round(face[2], 4), 'largura_medida': round(a['L'], 2)},
            'aproximacao': {'x': round(aprox[0], 3), 'y': round(aprox[1], 3),
                            'yaw': round(aprox[2], 4)},
        }
    return doc


# ---------------------------------------------------------------------------
# No ROS 2
# ---------------------------------------------------------------------------

def main():
    import rclpy
    import yaml
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    class ServiceAreas(Node):
        def __init__(self):
            super().__init__('service_areas')
            self.declare_parameter('arquivo', os.path.expanduser('~/mapas/service_areas.yaml'))
            self.declare_parameter('distancia_aproximacao', 0.60)
            self.declare_parameter('periodo', 2.0)
            self.rast = Rastreador()
            self.lock = threading.Lock()
            self.mapa = None
            self.mapa_novo = False
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, '/map', self.ao_mapa, qos)
            self.create_subscription(String, '/service_areas/nomear', self.ao_nomear, 10)
            qos_m = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
            self.pub = self.create_publisher(MarkerArray, '/service_areas/marcadores', qos_m)
            self.create_timer(float(self.get_parameter('periodo').value), self.ciclo)
            self.get_logger().info('Service Areas pronto. Marcadores em /service_areas/marcadores; '
                                   'arquivo: %s' % self.get_parameter('arquivo').value)

        def ao_mapa(self, msg):
            with self.lock:
                self.mapa = msg
                self.mapa_novo = True

        def ciclo(self):
            with self.lock:
                m, novo = self.mapa, self.mapa_novo
                self.mapa_novo = False
            if m is None or not novo:
                return
            i = m.info
            g = np.array(m.data, dtype=np.int8).reshape(i.height, i.width)
            t0 = time.monotonic()
            dets = detectar(g, i.resolution, i.origin.position.x, i.origin.position.y)
            antes = {a['rotulo'] for a in self.rast.confirmadas()}
            self.rast.atualizar(dets, time.monotonic())
            for a in self.rast.confirmadas():
                if a['rotulo'] not in antes:
                    self.get_logger().info('Nova candidata %s: face em (%.2f, %.2f), largura %.2f m' % (
                        a['rotulo'], a['face'][0], a['face'][1], a['L']))
            if time.monotonic() - t0 > 1.0:
                self.get_logger().warn('Deteccao lenta: %.1f s' % (time.monotonic() - t0))
            self.publicar()
            if any(a['nome'] for a in self.rast.areas):
                self.salvar()

        def ao_nomear(self, msg):
            partes = msg.data.split()
            if len(partes) == 2 and partes[0].lower() == 'apagar':
                ok = self.rast.apagar(partes[1])
                self.get_logger().info('%s %s' % ('Apagada' if ok else 'Nao achei', partes[1]))
            elif len(partes) in (2, 3):
                altura = None
                if len(partes) == 3:
                    try:
                        altura = float(partes[2]) / 100.0
                    except ValueError:
                        self.get_logger().warn('Altura invalida: %s (use 5, 10 ou 15)' % partes[2])
                        return
                a = self.rast.nomear(partes[0], partes[1], altura)
                if a is None:
                    self.get_logger().warn('Nao achei a candidata %s' % partes[0])
                    return
                self.get_logger().info('%s agora e %s (mesa de %.0f cm)' % (a['rotulo'], a['nome'],
                                                                            a['altura'] * 100))
            else:
                self.get_logger().warn('Use "C1 WS_1 10" ou "apagar C1"')
                return
            self.publicar()
            self.salvar()

        def salvar(self):
            caminho = self.get_parameter('arquivo').value
            nomeadas = [a for a in self.rast.areas if a['nome']]
            doc = documento_yaml(nomeadas, self.get_parameter('distancia_aproximacao').value)
            try:
                os.makedirs(os.path.dirname(caminho), exist_ok=True)
                tmp = caminho + '.tmp'
                with open(tmp, 'w') as f:
                    f.write('# Gerado por service_areas.py. Coordenadas no frame map desta sessao.\n')
                    yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
                os.replace(tmp, caminho)
            except OSError as exc:
                self.get_logger().error('Nao consegui salvar %s: %s' % (caminho, exc))

        def publicar(self):
            arr = MarkerArray()
            limpar = Marker()
            limpar.action = Marker.DELETEALL
            arr.markers.append(limpar)
            dist = self.get_parameter('distancia_aproximacao').value
            k = 0
            for a in self.rast.confirmadas():
                cor = (0.1, 0.8, 0.2) if a['nome'] else (1.0, 0.8, 0.0)
                rotulo = a['nome'] or a['rotulo']
                (fx, fy, yaw), (ax, ay, _) = poses(a, dist)
                c, s = math.cos(yaw), math.sin(yaw)  # direcao da face para dentro da mesa
                L, P = a['L'], 0.5
                cantos = [(0, -L / 2), (0, L / 2), (P, L / 2), (P, -L / 2), (0, -L / 2)]
                ret = Marker()
                ret.header.frame_id = 'map'
                ret.ns, ret.id, ret.type = 'mesa', k, Marker.LINE_STRIP
                ret.scale.x = 0.03
                ret.color.r, ret.color.g, ret.color.b, ret.color.a = cor[0], cor[1], cor[2], 1.0
                ret.pose.orientation.w = 1.0
                for u, v in cantos:
                    ret.points.append(Point(x=fx + c * u - s * v, y=fy + s * u + c * v, z=0.02))
                seta = Marker()
                seta.header.frame_id = 'map'
                seta.ns, seta.id, seta.type = 'aproximacao', k, Marker.ARROW
                seta.scale.x, seta.scale.y, seta.scale.z = 0.03, 0.06, 0.06
                seta.color.r, seta.color.g, seta.color.b, seta.color.a = cor[0], cor[1], cor[2], 0.9
                seta.pose.orientation.w = 1.0
                seta.points = [Point(x=ax, y=ay, z=0.02), Point(x=fx - c * 0.08, y=fy - s * 0.08, z=0.02)]
                txt = Marker()
                txt.header.frame_id = 'map'
                txt.ns, txt.id, txt.type = 'nome', k, Marker.TEXT_VIEW_FACING
                txt.scale.z = 0.15
                txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
                txt.pose.position.x = fx + c * P / 2
                txt.pose.position.y = fy + s * P / 2
                txt.pose.position.z = 0.3
                txt.pose.orientation.w = 1.0
                txt.text = rotulo
                arr.markers += [ret, seta, txt]
                k += 1
            self.pub.publish(arr)

    rclpy.init()
    no = ServiceAreas()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
