#!/usr/bin/env python3
"""BLOQUEIO_ROTA (24/11): memoria de obstaculos que barraram o robo, para o planner usar outra rota.

O obstacle_layer do global esquece um obstaculo fino quando os raios inf (31-38% no Delta-2D,
inf_is_valid) passam por ele, e o NavFn volta para o corredor curto bloqueado. Aqui:
  - /bloqueios/marcar (Trigger): pega o /scan_nav atual, os pontos ate 1,5 m do robo que NAO sao
    parede do mapa (> 0,15 m de celula ocupada) e grava um disco de 0,10 m em cada um, por 120 s;
  - /bloqueios/limpar (Trigger): apaga tudo;
  - publica /bloqueios_mapa (OccupancyGrid do tamanho do /map, 0 livre / 100 bloqueado, latched),
    lido por uma StaticLayer do global costmap (use_maximum: nunca apaga o mapa).
O mapa da prova continua sem obstaculos (regra): isto e memoria da corrida, expira sozinha.
"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

ALCANCE = 0.0   # DESLIGADA (26/11): com LIMITE_17 a memoria cercava o robo e o destino (era 3.0, depois 1.5)
LONGE_PAREDE = 0.15
RAIO = 0.10
VALIDADE = 300.0   # MEMORIA_LONGA (25/11): era 120


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pontos_novos(xs, ys, parede_perto, info, robo, alcance=ALCANCE):
    """Pontos (frame map) a <= alcance do robo e fora da faixa de parede do mapa."""
    if len(xs) == 0:
        return np.zeros((0, 2))
    r = info.resolution
    c = np.floor((xs - info.origin.position.x) / r).astype(int)
    l = np.floor((ys - info.origin.position.y) / r).astype(int)
    dentro = (c >= 0) & (c < info.width) & (l >= 0) & (l < info.height)
    parede = np.ones(len(xs), bool)
    parede[dentro] = parede_perto[l[dentro], c[dentro]]
    perto = np.hypot(xs - robo[0], ys - robo[1]) <= alcance
    sel = dentro & ~parede & perto
    return np.stack([xs[sel], ys[sel]], 1)


class Memoria:
    """Discos com validade (sem ROS, testavel)."""
    def __init__(self):
        self.discos = []   # (x, y, expira)

    def marcar(self, pts, agora, validade=VALIDADE):
        novos = 0
        for x, y in pts:
            if not any(math.hypot(x - a, y - b) < 0.05 for a, b, _ in self.discos):
                self.discos.append((float(x), float(y), agora + validade))
                novos += 1
            else:
                self.discos = [(a, b, agora + validade) if math.hypot(x - a, y - b) < 0.05 else (a, b, e)
                               for a, b, e in self.discos]
        return novos

    def expirar(self, agora):
        antes = len(self.discos)
        self.discos = [d for d in self.discos if d[2] > agora]
        return antes != len(self.discos)

    def grade(self, info):
        g = np.zeros((info.height, info.width), np.int8)
        r = info.resolution
        n = int(math.ceil(RAIO / r))
        for x, y, _ in self.discos:
            c0 = int((x - info.origin.position.x) / r)
            l0 = int((y - info.origin.position.y) / r)
            for dl in range(-n, n + 1):
                for dc in range(-n, n + 1):
                    if (dl * r) ** 2 + (dc * r) ** 2 <= RAIO ** 2:
                        l, c = l0 + dl, c0 + dc
                        if 0 <= l < info.height and 0 <= c < info.width:
                            g[l, c] = 100
        return g


class Bloqueios(Node):
    def __init__(self):
        super().__init__('bloqueios_rota')
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.mem = Memoria()
        self.info = None
        self.parede = None
        self.scan = None
        qm = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/bloqueios_mapa', qm)
        self.create_subscription(OccupancyGrid, '/map', self.ao_mapa, qm)
        self.create_subscription(LaserScan, '/scan_nav', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/bloqueios/marcar', self.marcar)
        self.create_service(Trigger, '/bloqueios/limpar', self.limpar)
        self.create_timer(2.0, self.vencer)
        self.get_logger().info('BLOQUEIO_ROTA pronto (/bloqueios/marcar, validade %.0f s)' % VALIDADE)

    def ao_mapa(self, m):
        from vigia_localizacao import dilatar
        g = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
        self.parede = dilatar(g >= 65, max(1, int(round(LONGE_PAREDE / m.info.resolution))))
        self.info = m.info
        self.publicar()

    def publicar(self):
        if self.info is None:
            return
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info = self.info
        m.data = self.mem.grade(self.info).ravel().tolist()
        self.pub.publish(m)

    def marcar(self, req, res):
        s = self.scan
        if s is None or self.info is None:
            res.success, res.message = False, 'sem scan ou sem mapa'
            return res
        try:
            t = self.tf.lookup_transform('map', s.header.frame_id, rclpy.time.Time())
            b = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            res.success, res.message = False, 'sem TF: %s' % e
            return res
        r = np.array(s.ranges, float)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > 0.05)
        tr, yaw = t.transform.translation, yaw_q(t.transform.rotation)
        lx, ly = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
        xs = tr.x + math.cos(yaw) * lx - math.sin(yaw) * ly
        ys = tr.y + math.sin(yaw) * lx + math.cos(yaw) * ly
        robo = (b.transform.translation.x, b.transform.translation.y)
        pts = pontos_novos(xs, ys, self.parede, self.info, robo)
        n = self.mem.marcar(pts, time.monotonic())
        self.publicar()
        res.success = True
        res.message = '%d ponto(s) novo(s), %d na memoria' % (n, len(self.mem.discos))
        self.get_logger().warn('BLOQUEIO_ROTA: marquei %s perto de (%.2f, %.2f)' % (res.message, *robo))
        return res

    def limpar(self, req, res):
        self.mem.discos = []
        self.publicar()
        res.success, res.message = True, 'memoria apagada'
        return res

    def vencer(self):
        if self.mem.expirar(time.monotonic()):
            self.get_logger().info('BLOQUEIO_ROTA: bloqueio(s) expirado(s), %d na memoria' % len(self.mem.discos))
            self.publicar()


def main():
    rclpy.init()
    no = Bloqueios()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
