#!/usr/bin/env python3
"""MAPA_PARADAS (25/11/2026): mapa montado por PARADAS, so com pontos constantes.

Ideia do Marcos: para o robo, aperta um botao, ele junta ~5 s de scans e grava no mapa so o que
apareceu de forma constante; repete pela arena. Motivo: a pose do M2M2 treme 5-17 cm entre scans
(medido 25/11) e o mapa continuo sai com paredes grossas. Parado, a MEDIANA da pose em 5 s tira o
tremor e a constancia tira gente passando e ruido.

- Botao: clique do analogico DIREITO (RS) no Xbox (js botao 10; --botao muda). Le o /dev/input/js*
  em paralelo ao teleop (os dois recebem os eventos; o teleop nao usa esse botao).
- Captura: --tempo s de /scan_front + TF map->laser_raw por scan. Recusa se a pose andou
  (p90 da distancia ate a mediana > --max-mov m ou giro > --max-giro graus) ou se faltou scan.
- Ocupado: celula (5 cm) atingida em >= --constancia dos scans (com a pose MEDIANA).
- Livre: celulas no caminho do raio ate cada ponto constante. Livre visto numa parada apaga
  ocupado antigo so se a parada nova viu a celula livre em >= 80% dos scans.
- Publica /map_paradas (nav_msgs/OccupancyGrid, frame map, transient local) a cada captura.
- Servicos: /mapa_paradas/capturar (o mesmo que o botao), /mapa_paradas/salvar (pgm+yaml em
  --saida), /mapa_paradas/desfazer (volta a ultima parada), /mapa_paradas/limpar.
Nao mexe no /map do slam_toolbox nem no Nav2.
"""
import argparse
import array
import fcntl
import glob
import math
import os
import struct
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

JS_FMT = 'IhBB'
JS_SIZE = struct.calcsize(JS_FMT)
DESC, LIVRE, OCUP = -1, 0, 100


def yaw_de(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MapaParadas(Node):
    def __init__(self, a):
        super().__init__('mapa_paradas')
        self.a = a
        self.res = 0.05
        self.n = int(round(a.tamanho / self.res))
        self.ox = self.oy = -a.tamanho / 2.0
        self.grade = np.full((self.n, self.n), DESC, dtype=np.int8)
        self.historico = []
        self.paradas = 0
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.coletando = None
        self.lock = threading.Lock()
        self.create_subscription(LaserScan, '/scan_front', self.ao_scan, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/map_paradas', qos)
        self.create_service(Trigger, '/mapa_paradas/capturar', lambda q, r: self.srv(r, self.capturar))
        self.create_service(Trigger, '/mapa_paradas/salvar', lambda q, r: self.srv(r, self.salvar))
        self.create_service(Trigger, '/mapa_paradas/desfazer', lambda q, r: self.srv(r, self.desfazer))
        self.create_service(Trigger, '/mapa_paradas/limpar', lambda q, r: self.srv(r, self.limpar))
        self.publicar()
        threading.Thread(target=self.ler_controle, daemon=True).start()
        self.log('pronto: pare o robo e aperte RS (clique do analogico direito, botao %d). Saida: %s'
                 % (a.botao, a.saida))

    def log(self, msg):
        self.get_logger().info('MAPA_PARADAS: ' + msg)

    def srv(self, resp, fn):
        ok, msg = fn()
        resp.success, resp.message = ok, msg
        return resp

    # ---------------- controle ----------------
    def ler_controle(self):
        while rclpy.ok():
            devs = sorted(glob.glob('/dev/input/js*'))
            if not devs:
                time.sleep(1.0)
                continue
            try:
                fd = os.open(devs[0], os.O_RDONLY)
            except OSError:
                time.sleep(1.0)
                continue
            try:
                while rclpy.ok():
                    data = os.read(fd, JS_SIZE)
                    if len(data) < JS_SIZE:
                        break
                    _, valor, tipo, num = struct.unpack(JS_FMT, data)
                    if tipo == 0x01 and num == self.a.botao and valor == 1:   # botao, sem INIT
                        threading.Thread(target=self.capturar, daemon=True).start()
            except OSError:
                pass
            finally:
                os.close(fd)
            time.sleep(1.0)

    # ---------------- captura ----------------
    def ao_scan(self, m):
        with self.lock:
            if self.coletando is None:
                return
        try:
            t = self.buf.lookup_transform('map', m.header.frame_id, rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001
            return
        pose = (t.translation.x, t.translation.y, yaw_de(t.rotation))
        with self.lock:
            if self.coletando is not None:
                self.coletando.append((pose, m.angle_min, m.angle_increment, np.array(m.ranges, dtype=np.float32)))

    def capturar(self):
        with self.lock:
            if self.coletando is not None:
                return False, 'ja capturando'
            self.coletando = []
        self.log('capturando %.0f s... NAO mexa o robo' % self.a.tempo)
        time.sleep(self.a.tempo)
        with self.lock:
            scans, self.coletando = self.coletando, None
        if len(scans) < max(5, int(self.a.tempo * 3)):
            return self.recusa('so %d scans em %.0f s (LiDAR/TF?)' % (len(scans), self.a.tempo))
        P = np.array([s[0] for s in scans])
        mx, my = np.median(P[:, 0]), np.median(P[:, 1])
        mt = math.atan2(np.median(np.sin(P[:, 2])), np.median(np.cos(P[:, 2])))
        dist = np.hypot(P[:, 0] - mx, P[:, 1] - my)
        giro = np.degrees(np.abs(np.arctan2(np.sin(P[:, 2] - mt), np.cos(P[:, 2] - mt))))
        if np.percentile(dist, 90) > self.a.max_mov or np.percentile(giro, 90) > self.a.max_giro:
            return self.recusa('robo se mexeu (p90 %.0f cm / %.1f graus)' % (100 * np.percentile(dist, 90),
                                                                           np.percentile(giro, 90)))
        cont = {}
        for _, amin, ainc, r in scans:
            ang = amin + ainc * np.arange(len(r))
            ok = np.isfinite(r) & (r > 0.12) & (r < self.a.alcance)
            x = mx + r[ok] * np.cos(mt + ang[ok])
            y = my + r[ok] * np.sin(mt + ang[ok])
            cel = set(zip(((x - self.ox) / self.res).astype(int), ((y - self.oy) / self.res).astype(int)))
            for c in cel:
                cont[c] = cont.get(c, 0) + 1
        lim = self.a.constancia * len(scans)
        ocup = [c for c, k in cont.items() if k >= lim and 0 <= c[0] < self.n and 0 <= c[1] < self.n]
        if not ocup:
            return self.recusa('nenhum ponto constante')
        self.historico.append(self.grade.copy())
        cx0, cy0 = int((mx - self.ox) / self.res), int((my - self.oy) / self.res)
        oc = set(ocup)
        for cx, cy in ocup:
            for lx, ly in self.raio(cx0, cy0, cx, cy)[:-1]:
                if (lx, ly) in oc or not (0 <= lx < self.n and 0 <= ly < self.n):
                    continue
                if self.grade[ly, lx] != OCUP or cont.get((lx, ly), 0) <= 0.2 * len(scans):
                    self.grade[ly, lx] = LIVRE
        for cx, cy in ocup:
            self.grade[cy, cx] = OCUP
        self.paradas += 1
        self.publicar()
        msg = ('parada %d: %d scans, pose (%.2f, %.2f, %.0f graus), tremor p90 %.1f cm; %d celulas de parede'
               % (self.paradas, len(scans), mx, my, math.degrees(mt), 100 * np.percentile(dist, 90), len(ocup)))
        self.log(msg)
        return True, msg

    def recusa(self, motivo):
        self.log('RECUSADA: ' + motivo + '. Nada mudou.')
        return False, motivo

    @staticmethod
    def raio(x0, y0, x1, y1):
        pts = []
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        err = dx + dy
        while True:
            pts.append((x0, y0))
            if x0 == x1 and y0 == y1:
                return pts
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    # ---------------- saida ----------------
    def publicar(self):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = self.res
        g.info.width = g.info.height = self.n
        g.info.origin.position.x, g.info.origin.position.y = self.ox, self.oy
        g.info.origin.orientation.w = 1.0
        g.data = self.grade.flatten().tolist()
        self.pub.publish(g)

    def salvar(self):
        os.makedirs(self.a.saida, exist_ok=True)
        base = os.path.join(self.a.saida, 'mapa_paradas_' + time.strftime('%H%M%S'))
        img = np.full(self.grade.shape, 205, dtype=np.uint8)
        img[self.grade == LIVRE] = 254
        img[self.grade == OCUP] = 0
        img = np.flipud(img)
        with open(base + '.pgm', 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (self.n, self.n))
            f.write(img.tobytes())
        with open(base + '.yaml', 'w') as f:
            f.write('image: %s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
                    % (os.path.basename(base), self.res, self.ox, self.oy))
        msg = 'salvo %s.pgm/.yaml (%d paradas)' % (base, self.paradas)
        self.log(msg)
        return True, msg

    def desfazer(self):
        if not self.historico:
            return False, 'nada para desfazer'
        self.grade = self.historico.pop()
        self.paradas -= 1
        self.publicar()
        self.log('ultima parada desfeita')
        return True, 'desfeita'

    def limpar(self):
        self.historico.append(self.grade.copy())
        self.grade[:] = DESC
        self.paradas = 0
        self.publicar()
        self.log('mapa limpo')
        return True, 'limpo'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--botao', type=int, default=10, help='botao do js (Xbox xpad: 10 = RS)')
    ap.add_argument('--tempo', type=float, default=5.0)
    ap.add_argument('--constancia', type=float, default=0.6)
    ap.add_argument('--max-mov', type=float, default=0.10, help='m (p90 ate a mediana; o M2M2 treme)')
    ap.add_argument('--max-giro', type=float, default=3.0, help='graus (p90)')
    ap.add_argument('--alcance', type=float, default=6.0)
    ap.add_argument('--tamanho', type=float, default=30.0, help='m de lado da grade')
    ap.add_argument('--saida', default=os.path.expanduser('~/mapas/sessao_20261125_0949'))
    a, _ = ap.parse_known_args()
    rclpy.init()
    no = MapaParadas(a)
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == '__main__':
    main()
