#!/usr/bin/env python3
"""VIGIA_LOCALIZACAO (24/11): diz se o robo sabe onde esta no mapa (AMCL) e tenta se reencontrar.

Encaixe = fracao dos pontos validos do /scan_front que caem a <= TOL de uma celula ocupada do /map.
  ok      : encaixe >= 0,70 por 2 s
  duvida  : entre os limites, ou salto do AMCL > 0,30 m / 15 graus
  perdido : encaixe < 0,50 por 1 s
Perdido: espera 2 s parado; reinicia o AMCL em volta da ultima pose boa (/initialpose, 0,5 m);
ainda perdido depois de 6 s: relocalizacao global do AMCL. Nao mexe o robo: quem para a navegacao
e o piloto (le /localizacao/estado).

Publica /localizacao/estado (String JSON: estado, encaixe, ultima_boa) e /localizacao/encaixe (Float32).
"""
import json
import math
import os  # START_FORCADO (24/11)
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

OK, DUVIDA, PERDIDO = 'ok', 'duvida', 'perdido'
LIM_OK, LIM_PERDIDO = 0.70, 0.50
TOL = 0.10


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def dilatar(occ, n):
    out = occ.copy()
    for _ in range(n):
        d = out.copy()
        d[1:, :] |= out[:-1, :]
        d[:-1, :] |= out[1:, :]
        d[:, 1:] |= out[:, :-1]
        d[:, :-1] |= out[:, 1:]
        out = d
    return out


def encaixe(xs, ys, perto, info):
    """Fracao dos pontos (frame map) que caem numa celula 'perto' de parede. None sem pontos."""
    if len(xs) == 0:
        return None
    r = info.resolution
    c = np.floor((xs - info.origin.position.x) / r).astype(int)
    l = np.floor((ys - info.origin.position.y) / r).astype(int)
    dentro = (c >= 0) & (c < info.width) & (l >= 0) & (l < info.height)
    acertos = np.zeros(len(xs), bool)
    acertos[dentro] = perto[l[dentro], c[dentro]]
    return float(acertos.mean())


class Classificador:
    """Estado pelo encaixe com histerese temporal (sem ROS, testavel)."""
    def __init__(self):
        self.estado = DUVIDA
        self.desde_bom = None
        self.desde_ruim = None

    def passo(self, t, enc, salto=False):
        if enc is None:
            return self.estado
        bom, ruim = enc >= LIM_OK and not salto, enc < LIM_PERDIDO
        self.desde_bom = (t if self.desde_bom is None else self.desde_bom) if bom else None
        self.desde_ruim = (t if self.desde_ruim is None else self.desde_ruim) if ruim else None
        if self.desde_ruim is not None and t - self.desde_ruim >= 1.0:
            self.estado = PERDIDO
        elif self.desde_bom is not None and t - self.desde_bom >= 2.0:
            self.estado = OK
        elif salto or (self.estado == OK and enc < LIM_OK):
            self.estado = DUVIDA
        return self.estado


class Vigia(Node):
    def __init__(self):
        super().__init__('vigia_localizacao')
        self.declare_parameter('recuperar', True)
        self.declare_parameter('start_forcado', True)  # START_FORCADO (24/11)
        self.t_forcou = None
        self.odom0 = None
        self.odom_mexeu = False
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, '/odom', self.ao_odom_start, 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.perto = None
        self.info = None
        self.cls = Classificador()
        self.ultima_boa = None
        self.amcl_ant = None
        self.salto_ate = 0.0
        self.t_perdido = None
        self.etapa = 0
        qm = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.ao_mapa, qm)
        self.create_subscription(LaserScan, '/scan_front', self.ao_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.ao_amcl, 10)
        self.pub_estado = self.create_publisher(String, '/localizacao/estado', 10)
        self.pub_enc = self.create_publisher(Float32, '/localizacao/encaixe', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cli_global = self.create_client(Empty, '/reinitialize_global_localization')
        self.get_logger().info('VIGIA_LOCALIZACAO pronto (ok >= %.0f%%, perdido < %.0f%%)' % (
            LIM_OK * 100, LIM_PERDIDO * 100))

    def ao_mapa(self, m):
        g = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
        self.perto = dilatar(g >= 65, max(1, int(round(TOL / m.info.resolution))))
        self.info = m.info

    def ao_amcl(self, m):
        p = m.pose.pose
        a = (p.position.x, p.position.y, yaw_q(p.orientation))
        if self.amcl_ant is not None:
            d = math.hypot(a[0] - self.amcl_ant[0], a[1] - self.amcl_ant[1])
            dy = abs(math.atan2(math.sin(a[2] - self.amcl_ant[2]), math.cos(a[2] - self.amcl_ant[2])))
            if d > 0.30 or dy > math.radians(15):
                self.salto_ate = time.monotonic() + 2.0
                self.get_logger().warn('VIGIA: salto do AMCL %.2f m / %.0f graus' % (d, math.degrees(dy)))
        self.amcl_ant = a

    def ao_scan(self, s):
        if self.perto is None:
            return
        try:
            t = self.tf.lookup_transform('map', s.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        r = np.array(s.ranges, float)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > max(s.range_min, 0.15)) & (r < min(s.range_max, 6.0))
        if ok.sum() < 20:
            return
        tr, q = t.transform.translation, t.transform.rotation
        yaw = yaw_q(q)
        lx, ly = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
        xs = tr.x + math.cos(yaw) * lx - math.sin(yaw) * ly
        ys = tr.y + math.sin(yaw) * lx + math.cos(yaw) * ly
        enc = encaixe(xs, ys, self.perto, self.info)
        agora = time.monotonic()
        est = self.cls.passo(agora, enc, salto=agora < self.salto_ate)
        if est == OK:
            try:
                b = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                self.ultima_boa = (b.transform.translation.x, b.transform.translation.y,
                                   yaw_q(b.transform.rotation))
            except Exception:
                pass
            self.t_perdido, self.etapa = None, 0
        elif est == PERDIDO and self.get_parameter('recuperar').value:
            self.recuperar(agora)
        self.pub_enc.publish(Float32(data=float(enc)))
        self.pub_estado.publish(String(data=json.dumps(dict(
            estado=est, encaixe=round(enc, 3), etapa=self.etapa,
            ultima_boa=[round(v, 3) for v in self.ultima_boa] if self.ultima_boa else None))))

    def ao_odom_start(self, m):   # START_FORCADO: saiu do START = andou 10 cm ou girou 10 graus
        p = m.pose.pose
        a = (p.position.x, p.position.y, yaw_q(p.orientation))
        if self.odom0 is None:
            self.odom0 = a
        elif not self.odom_mexeu:
            dy = abs(math.atan2(math.sin(a[2] - self.odom0[2]), math.cos(a[2] - self.odom0[2])))
            if math.hypot(a[0] - self.odom0[0], a[1] - self.odom0[1]) > 0.10 or dy > math.radians(10):
                self.odom_mexeu = True
                self.get_logger().info('START_FORCADO: robo saiu do START (odom); nao forco mais o START')

    def start_salvo(self):   # START_FORCADO (24/11)
        try:
            import yaml
            p = yaml.safe_load(open(os.path.expanduser('~/mapas/pontos.yaml')))['pontos']['START']
            return float(p['x']), float(p['y']), float(p['yaw'])
        except Exception as e:  # noqa: BLE001
            self.get_logger().error('START_FORCADO: nao li o START de ~/mapas/pontos.yaml (%s)' % e)
            return None

    def forcar_start(self, agora):   # START_FORCADO (24/11)
        if self.t_forcou is not None and agora - self.t_forcou < 5.0:
            return
        st = self.start_salvo()
        if st is None:
            return
        self.t_forcou = agora
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = st[0], st[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(st[2] / 2), math.cos(st[2] / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(5) ** 2
        self.pub_init.publish(m)
        self.get_logger().warn('START_FORCADO: sem localizacao boa ainda; AMCL forcado no START (%.2f, %.2f, %.0f graus)'
                               % (st[0], st[1], math.degrees(st[2])))

    def recuperar(self, agora):
        if self.ultima_boa is None and self.get_parameter('start_forcado').value:   # START_FORCADO
            if not self.odom_mexeu:
                self.forcar_start(agora)
            elif self.t_forcou is None or agora - self.t_forcou > 10.0:
                self.t_forcou = agora
                self.get_logger().warn('START_FORCADO: perdido sem nunca ter localizado bem; NAO faco busca global')
            return
        if self.t_perdido is None:
            self.t_perdido = agora
            self.get_logger().warn('VIGIA: PERDIDO (encaixe baixo); espero 2 s parado')
        dt = agora - self.t_perdido
        if self.etapa == 0 and dt >= 2.0 and self.ultima_boa is not None:
            self.etapa = 1
            m = PoseWithCovarianceStamped()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            x, y, yaw = self.ultima_boa
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.25
            m.pose.covariance[35] = math.radians(20) ** 2
            self.pub_init.publish(m)
            self.get_logger().warn('VIGIA: AMCL reiniciado na ultima pose boa (%.2f, %.2f, %.0f graus)' % (
                x, y, math.degrees(yaw)))
        elif self.etapa <= 1 and dt >= 8.0:
            self.etapa = 2
            if self.cli_global.service_is_ready():
                self.cli_global.call_async(Empty.Request())
                self.get_logger().warn('VIGIA: relocalizacao GLOBAL do AMCL')
        elif self.etapa == 2 and dt >= 20.0:
            self.t_perdido, self.etapa = None, 0   # recomeca o ciclo


def main():
    rclpy.init()
    no = Vigia()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
