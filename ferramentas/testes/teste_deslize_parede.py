#!/usr/bin/env python3
"""TESTE_DESLIZE v1 (06/11/2026): mede o k do deslize de frente para uma parede (H2 da missao22).

O robo desliza de lado e o LiDAR mede quanto ele chegou perto ou se afastou da parede da frente.
k = variacao da distancia / deslize lateral. k > 0: andou para a frente; k < 0: para tras.
A distancia e a da reta da parede ate o centro do robo, entao o giro quase nao entra nela.

Rode SO a ponte, o filtro do LiDAR e a localizacao: o teste publica direto em /cmd_vel, SEM o
Collision Monitor, e recusa se outro no publica nesse topico. Ele para sozinho com:
folga da frente < 15 cm, algo a menos de 15 cm do lado, atras < 10 cm, ou sensor parado > 0,5 s.

Montagem: parede reta de pelo menos 2 m, de 35 a 70 cm do para-choque; 1 m livre de cada lado.
Uso: python3 -P teste_deslize_parede.py --piso liso [--vels 0.08,0.12,0.16] [--dist 0.50]
"""
import argparse
import csv
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

X_FRENTE, X_TRAS, Y_LADO = 0.375, -0.215, 0.325
FOLGA_FRENTE, FOLGA_LADO, FOLGA_TRAS = 0.15, 0.15, 0.10
IDADE_MAX = 0.5
TAB_CMD = (0.02, 0.03, 0.05, 0.08, 0.12, 0.16, 0.20, 0.31)  # localization.yaml
TAB_Y = (0.0621, 0.0662, 0.0743, 0.0866, 0.1030, 0.1195, 0.1358, 0.1809)


def v_tabela(v):
    for i in range(len(TAB_CMD) - 1):
        if v <= TAB_CMD[i + 1]:
            f = (v - TAB_CMD[i]) / (TAB_CMD[i + 1] - TAB_CMD[i])
            return TAB_Y[i] + max(0.0, f) * (TAB_Y[i + 1] - TAB_Y[i])
    return TAB_Y[-1]


def ang_dif(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def yaw_de(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ajusta_parede(pts):
    """Reta x = a + b*y na parede da frente, com consenso: uma parede lateral no canto nao puxa a reta.
    Devolve (distancia ao centro, angulo, pontos, a, b) ou None."""
    p = [(x, y) for x, y in pts if X_FRENTE + 0.02 <= x <= 2.0 and abs(y) <= 0.70]
    if len(p) < 8:
        return None
    passo = max(1, len(p) // 40)
    amostra = p[::passo]
    melhor = []
    for i, (x1, y1) in enumerate(amostra):
        for x2, y2 in amostra[i + 1:]:
            if abs(y2 - y1) < 0.10:
                continue
            b = (x2 - x1) / (y2 - y1)
            if abs(b) > 0.45:  # mais de ~24 graus: nao e a parede da frente
                continue
            a = x1 - b * y1
            dentro = [q for q in p if abs(q[0] - (a + b * q[1])) <= 0.02]
            if len(dentro) > len(melhor):
                melhor = dentro
    if len(melhor) < 8 or max(y for _, y in melhor) - min(y for _, y in melhor) < 0.30:
        return None
    n = float(len(melhor))
    my, mx = sum(y for _, y in melhor) / n, sum(x for x, _ in melhor) / n
    b = sum((y - my) * (x - mx) for x, y in melhor) / sum((y - my) ** 2 for _, y in melhor)
    a = mx - b * my
    return a / math.sqrt(1 + b * b), math.atan(b), len(melhor), a, b


class Deslize(Node):
    def __init__(self):
        super().__init__('teste_deslize')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.tf_buf = Buffer()
        self.tf_l = TransformListener(self.tf_buf, self)
        self.laser = None
        self.lock = threading.Lock()
        self.scan, self.odom, self.t_imu = None, None, None  # scan: (t, pontos, reta)
        self.hist = []
        self.create_subscription(LaserScan, '/scan_front', self.ao_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.ao_odom, 20)
        self.create_subscription(Imu, '/imu/data', self.ao_imu, qos_profile_sensor_data)

    def ao_imu(self, _):
        self.t_imu = time.monotonic()

    def ao_odom(self, m):
        p = m.pose.pose
        self.odom = (time.monotonic(), p.position.x, p.position.y, yaw_de(p.orientation))

    def ao_scan(self, m):
        if self.laser is None:
            try:
                tr = self.tf_buf.lookup_transform('base_link', m.header.frame_id, rclpy.time.Time())
            except Exception:
                return
            self.laser = (tr.transform.translation.x, tr.transform.translation.y, yaw_de(tr.transform.rotation))
        lx, ly, lt = self.laser
        pts, ang = [], m.angle_min
        for r in m.ranges:
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                pts.append((lx + r * math.cos(ang + lt), ly + r * math.sin(ang + lt)))
            ang += m.angle_increment
        reta = ajusta_parede(pts)
        t = time.monotonic()
        with self.lock:
            self.scan = (t, pts, reta)
            self.hist.append((t, reta, self.odom))

    def vel(self, vy):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.y = float(vy)
        self.pub.publish(m)

    def parar(self):
        for _ in range(10):
            self.vel(0.0)
            time.sleep(0.05)

    def motivo_parar(self, lado):
        agora = time.monotonic()
        s, o, ti = self.scan, self.odom, self.t_imu
        if s is None or agora - s[0] > IDADE_MAX:
            return 'LiDAR parado'
        if o is None or agora - o[0] > IDADE_MAX:
            return 'odom parada'
        if ti is None or agora - ti > IDADE_MAX:
            return 'IMU parada'
        for x, y in s[1]:
            if X_TRAS <= x <= X_FRENTE and Y_LADO <= y * lado < Y_LADO + FOLGA_LADO:
                return 'algo a menos de %.0f cm do lado' % (100 * FOLGA_LADO)
            if X_TRAS - FOLGA_TRAS < x < X_TRAS - 0.02 and abs(y) <= Y_LADO:
                return 'algo a menos de %.0f cm atras' % (100 * FOLGA_TRAS)
        r = s[2]
        if r is None:
            return 'parede da frente sumiu'
        if min(r[3] + r[4] * Y_LADO, r[3] - r[4] * Y_LADO) - X_FRENTE < FOLGA_FRENTE:
            return 'frente a menos de %.0f cm' % (100 * FOLGA_FRENTE)
        return None

    def medir(self, seg=1.2):
        """Parado: mediana da distancia e do angulo por seg segundos."""
        t0 = time.monotonic()
        time.sleep(seg)
        with self.lock:
            q = [h[1] for h in self.hist if h[0] >= t0 and h[1] is not None]
        if len(q) < 3:
            return None
        return (sorted(x[0] for x in q)[len(q) // 2], sorted(x[1] for x in q)[len(q) // 2])

    def deslizar(self, lado, v, dist):
        """Devolve (motivo, deslize pela odom, tempo)."""
        o0 = self.odom
        c, s = math.cos(o0[3]), math.sin(o0[3])
        t0 = time.monotonic()
        t_max = 2.5 * dist / v_tabela(v) + 2.0
        motivo, lat = None, 0.0
        try:
            while True:
                motivo = self.motivo_parar(lado)
                if motivo:
                    break
                o = self.odom
                lat = abs(-s * (o[1] - o0[1]) + c * (o[2] - o0[2]))
                if lat >= dist:
                    break
                if time.monotonic() - t0 > t_max:
                    motivo = 'tempo maximo'
                    break
                self.vel(lado * v)
                time.sleep(0.05)
        finally:
            self.parar()
        return motivo, lat, time.monotonic() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--piso', required=True, help='nome do piso: liso, frisos...')
    ap.add_argument('--vels', default='0.08,0.12,0.16')
    ap.add_argument('--dist', type=float, default=0.50)
    ap.add_argument('--saida', default=os.path.expanduser('~/testes_h2'))
    a = ap.parse_args()
    vels = [float(x) for x in a.vels.split(',')]
    if not all(0.05 <= v <= 0.20 for v in vels) or not 0.10 <= a.dist <= 0.80:
        raise SystemExit('Use velocidades entre 0.05 e 0.20 e --dist entre 0.10 e 0.80.')
    rclpy.init()
    no = Deslize()
    ex = MultiThreadedExecutor()
    ex.add_node(no)
    giro = threading.Thread(target=ex.spin, daemon=True)
    giro.start()
    res = []
    try:
        time.sleep(3.0)
        if no.count_publishers('/cmd_vel') != 1:
            raise SystemExit('RECUSADO: outro no publica em /cmd_vel. Deixe so ponte, filtro do LiDAR e localizacao.')
        m = no.medir()
        if m is None or no.motivo_parar(1) == 'odom parada' or no.t_imu is None:
            raise SystemExit('RECUSADO: sem parede na frente ou sem sensores (LiDAR, odom, IMU).')
        folga = m[0] - X_FRENTE
        print('Parede a %.0f cm do para-choque, angulo %+.1f graus.' % (100 * folga, math.degrees(m[1])))
        if not 0.35 <= folga <= 0.70 or abs(math.degrees(m[1])) > 8:
            raise SystemExit('RECUSADO: coloque de 35 a 70 cm da parede e de frente (ate 8 graus).')
        print('Marque a posicao do robo no chao com fita antes de cada deslize, para medir o lateral real.')
        if input('Digite pronto para o robo se mover (Ctrl+C para na hora): ').strip().lower() != 'pronto':
            raise SystemExit('Cancelado: nada foi comandado.')
        for v in vels:
            for lado, nome in ((1, 'esquerda'), (-1, 'direita')):
                input('\nv=%.2f para a %s, %.0f cm. Enter para ir...' % (v, nome, 100 * a.dist))
                m0 = no.medir()
                if m0 is None:
                    raise SystemExit('Parei: sem parede medida antes do deslize.')
                o0 = no.odom
                motivo, lat, t = no.deslizar(lado, v, a.dist)
                if motivo:
                    print('    PAREI: %s' % motivo)
                if motivo and ('parado' in motivo or 'parada' in motivo):
                    raise SystemExit('Teste encerrado: sensor sem dados.')
                time.sleep(0.8)
                m1 = no.medir()
                if m1 is None:
                    raise SystemExit('Parei: sem parede medida depois do deslize.')
                dy = math.degrees(ang_dif(no.odom[3], o0[3]))
                txt = input('    Deslize real pela fita em cm (vazio = usar odom, %.0f cm): ' % (100 * lat))
                txt = txt.strip().replace(',', '.')
                real = float(txt) / 100.0 if txt else None
                base = real if real else lat
                avanco = m0[0] - m1[0]  # distancia caiu: andou para a frente
                k = avanco / base if base > 0.02 else float('nan')
                res.append((a.piso, nome, v, base, 'fita' if real else 'odom', avanco, k, dy,
                            math.degrees(m1[1] - m0[1]), t, motivo or ''))
                print('    frente %+.1f cm | k %+.2f | giro %+.1f graus%s' % (
                    100 * avanco, k, dy, ' | parou: ' + motivo if motivo else ''))
    finally:
        no.parar()
        if res:
            os.makedirs(a.saida, exist_ok=True)
            data = time.strftime('%Y%m%d_%H%M%S')
            arq = os.path.join(a.saida, 'deslize_%s_%s.csv' % (a.piso, data))
            with open(arq, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(('piso', 'lado', 'v_cmd', 'deslize_m', 'fonte', 'frente_m', 'k', 'giro_graus',
                            'parede_graus', 'tempo_s', 'parou_por'))
                w.writerows(res)
            print('\n=== Resultado, piso %s (aceite: frente variando ate 1-2 cm em 50 cm) ===' % a.piso)
            print('lado      v     deslize  frente    k     giro')
            for r in res:
                print('%-8s %.2f  %5.1f cm %+5.1f cm %+.2f %+5.1f gr %s' % (
                    r[1], r[2], 100 * r[3], 100 * r[5], r[6], r[7], r[10]))
            print('Arquivo: %s' % arq)
        ex.shutdown()
        giro.join(2.0)
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrompido: a ponte para sozinha em 0,25 s sem comando.')
