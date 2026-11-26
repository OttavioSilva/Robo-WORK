#!/usr/bin/env python3
"""TESTE_DESLIZE_ICP v1 (06/11/2026): k e velocidade real do deslize, sem parede reta nem fita.

Compara scans parados antes e depois de cada deslize (ICP): da o deslocamento real para o lado, para
a frente e o giro. k = frente / lado; k > 0 anda para a frente. Serve em qualquer lugar com paredes
ou objetos em mais de uma direcao.

Rode SO a ponte, o filtro do LiDAR e a localizacao: publica direto em /cmd_vel, SEM o Collision
Monitor, e recusa se outro no publica nesse topico. Para sozinho com: frente < 12 cm, lado < 15 cm,
atras < 10 cm, giro > 25 graus ou sensor parado > 0,5 s. A ponte para sozinha 0,25 s sem comando.

Uso:
    python3 -P teste_deslize_icp.py --so-medir            # nao move: confere o ICP parado
    python3 -P teste_deslize_icp.py --recuar 0.15 --dist 0.40 --vels 0.08,0.12,0.16
"""
import argparse
import csv
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

X_FRENTE, X_TRAS, Y_LADO = 0.375, -0.215, 0.325
X_LASER = 0.33
FOLGA_FRENTE, FOLGA_LADO, FOLGA_TRAS = 0.12, 0.15, 0.10
GIRO_MAX = math.radians(25.0)
IDADE_MAX = 0.5


def yaw_de(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang_dif(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def icp(a, b, chute=(0.0, 0.0, 0.0), voltas=40):
    """Acha (dx, dy, dth): a pose do scan b no referencial do scan a. Devolve tambem o residuo e os pares."""
    th, tx, ty = chute[2], chute[0], chute[1]
    raio = 0.30
    res, n = 9.0, 0
    for i in range(voltas):
        c, s = math.cos(th), math.sin(th)
        q = np.column_stack((c * b[:, 0] - s * b[:, 1] + tx, s * b[:, 0] + c * b[:, 1] + ty))
        d2 = ((q[:, None, :] - a[None, :, :]) ** 2).sum(axis=2)
        j = d2.argmin(axis=1)
        dmin = np.sqrt(d2[np.arange(len(q)), j])
        ok = dmin < raio
        if ok.sum() < 15:
            return None
        lim = np.percentile(dmin[ok], 75)
        ok &= dmin <= lim
        p, r = b[ok], a[j[ok]]
        mp, mr = p.mean(axis=0), r.mean(axis=0)
        h = (p - mp).T @ (r - mr)
        th = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        c, s = math.cos(th), math.sin(th)
        tx = mr[0] - (c * mp[0] - s * mp[1])
        ty = mr[1] - (s * mp[0] + c * mp[1])
        res, n = float(dmin[ok].mean()), int(ok.sum())
        raio = max(0.06, raio * 0.85)
    return tx, ty, th, res, n


def icp_varios(a, b, lat, giro):
    """ICP de varias partidas: pega o que casa melhor e devolve tambem a outra solucao boa mais distante,
    para saber se a medida e ambigua. (dx, dy, dth, residuo, pares, ambiguidade em m)"""
    sols = []
    for fx in (-0.30, -0.15, 0.0, 0.15, 0.30):
        for fy in (0.0, 0.5, 1.0):
            r = icp(a, b, chute=(fx, fy * lat, giro))
            if r is not None:
                sols.append(r)
    if not sols:
        return None
    mais = float(max(r[4] for r in sols))
    nota = lambda r: r[3] + 0.5 * (1.0 - r[4] / mais)
    sols.sort(key=nota)
    m = sols[0]
    amb = max((math.hypot(r[0] - m[0], r[1] - m[1]) for r in sols if nota(r) <= nota(m) + 0.004), default=0.0)
    return m + (amb,)


class Teste(Node):
    def __init__(self):
        super().__init__('teste_deslize_icp')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.lock = threading.Lock()
        self.scan, self.odom, self.t_imu = None, None, None
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
        ang = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        r = np.array(m.ranges, dtype=float)
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        pts = np.column_stack((X_LASER + r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])))
        t = time.monotonic()
        with self.lock:
            self.scan = (t, pts)
            self.hist.append((t, r.copy(), m.angle_min, m.angle_increment))
            self.hist = self.hist[-40:]

    def vel(self, vx, vy):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.linear.y = float(vx), float(vy)
        self.pub.publish(m)

    def parar(self):
        for _ in range(10):
            self.vel(0.0, 0.0)
            time.sleep(0.05)

    def motivo(self, ux, uy, yaw0):
        agora = time.monotonic()
        s, o, ti = self.scan, self.odom, self.t_imu
        if s is None or agora - s[0] > IDADE_MAX:
            return 'LiDAR parado'
        if o is None or agora - o[0] > IDADE_MAX:
            return 'odom parada'
        if ti is None or agora - ti > 1.0:   # falhas de 0,6 s acontecem; o rumo vem da ponte
            return 'IMU parada'
        if abs(ang_dif(o[3], yaw0)) > GIRO_MAX:
            return 'girou mais de 25 graus'
        x, y = s[1][:, 0], s[1][:, 1]
        fora = ~((x >= X_TRAS - 0.02) & (x <= X_FRENTE + 0.02) & (np.abs(y) <= Y_LADO + 0.02))
        x, y = x[fora], y[fora]
        larg = np.abs(y) <= Y_LADO + 0.03
        fr = x[larg & (x > X_FRENTE)]
        tr = x[larg & (x < X_TRAS)]
        faixa = (x >= X_TRAS) & (x <= X_FRENTE)
        if (ux > 0 or uy != 0) and fr.size and fr.min() - X_FRENTE < FOLGA_FRENTE:
            return 'frente a %.0f cm' % (100 * (fr.min() - X_FRENTE))
        if tr.size and X_TRAS - tr.max() < FOLGA_TRAS:
            return 'atras a %.0f cm' % (100 * (X_TRAS - tr.max()))
        if uy > 0 and (y[faixa] > 0).any() and y[faixa & (y > 0)].min() - Y_LADO < FOLGA_LADO:
            return 'esquerda a %.0f cm' % (100 * (y[faixa & (y > 0)].min() - Y_LADO))
        if uy < 0 and (y[faixa] < 0).any() and -y[faixa & (y < 0)].max() - Y_LADO < FOLGA_LADO:
            return 'direita a %.0f cm' % (100 * (-y[faixa & (y < 0)].max() - Y_LADO))
        return None

    def foto(self, seg=2.0):
        """Parado: mediana de cada raio. Devolve os pontos sem o corpo."""
        t0 = time.monotonic()
        time.sleep(seg)
        with self.lock:
            q = [h for h in self.hist if h[0] >= t0]
        if len(q) < 4:
            return None
        m = np.array([h[1] for h in q])
        m[~np.isfinite(m)] = np.nan
        boas = np.sum(np.isfinite(m), axis=0) >= max(3, len(q) // 2)
        with np.errstate(all='ignore'):
            r = np.nanmedian(m[:, boas], axis=0)
        ang = q[0][2] + np.nonzero(boas)[0] * q[0][3]
        p = np.column_stack((X_LASER + r * np.cos(ang), r * np.sin(ang)))
        corpo = (p[:, 0] >= X_TRAS - 0.03) & (p[:, 0] <= X_FRENTE + 0.03) & (np.abs(p[:, 1]) <= Y_LADO + 0.03)
        return p[~corpo]

    def mover(self, vx, vy, dist, t_max):
        """Anda dist pela odom. Devolve (motivo, tempo)."""
        o0 = self.odom
        c, s = math.cos(o0[3]), math.sin(o0[3])
        ux, uy = (vx > 0) - (vx < 0), (vy > 0) - (vy < 0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0 and (self.scan is None or t0 - self.scan[0] > 0.0):
            time.sleep(0.02)   # o calculo do ICP atrasa as leituras: parte com um scan novo
        t0 = time.monotonic()
        mot = None
        try:
            while True:
                mot = self.motivo(ux, uy, o0[3])
                if mot:
                    break
                o = self.odom
                dx, dy = o[1] - o0[1], o[2] - o0[2]
                if math.hypot(c * dx + s * dy, -s * dx + c * dy) >= dist:
                    break
                if time.monotonic() - t0 > t_max:
                    mot = 'tempo maximo'
                    break
                self.vel(vx, vy)
                time.sleep(0.05)
        finally:
            self.parar()
        return mot, time.monotonic() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--so-medir', action='store_true')
    ap.add_argument('--recuar', type=float, default=0.0)
    ap.add_argument('--frente', default='', help='comandos para a frente: ex. 0.08,0.16')
    ap.add_argument('--vels', default='0.08,0.12,0.16')
    ap.add_argument('--dist', type=float, default=0.40)
    ap.add_argument('--saida', default=os.path.expanduser('~/testes_h2'))
    a = ap.parse_args()
    vels = [float(x) for x in a.vels.split(',')]
    if not all(0.05 <= v <= 0.20 for v in vels) or not 0.10 <= a.dist <= 0.60 or not 0.0 <= a.recuar <= 0.25:
        raise SystemExit('Use velocidades entre 0.05 e 0.20, --dist entre 0.10 e 0.60 e --recuar ate 0.25.')
    rclpy.init()
    no = Teste()
    ex = MultiThreadedExecutor()
    ex.add_node(no)
    giro = threading.Thread(target=ex.spin, daemon=True)
    giro.start()
    res = []
    try:
        time.sleep(2.5)
        f0 = no.foto()
        f1 = no.foto()
        if f0 is None or f1 is None or len(f0) < 30:
            raise SystemExit('RECUSADO: poucos pontos do LiDAR.')
        r = icp(f0, f1)
        if r is None:
            raise SystemExit('RECUSADO: o ICP nao casou dois scans parados.')
        print('Parado: %d pontos; ICP dx %+.1f cm, dy %+.1f cm, giro %+.2f graus, residuo %.1f cm' % (
            len(f0), 100 * r[0], 100 * r[1], math.degrees(r[2]), 100 * r[3]), flush=True)
        for nome, d in () if not a.so_medir else (('+5 cm frente', (0.05, 0.0, 0.0)), ('+40 cm lado', (0.0, 0.40, 0.0)),
                        ('20 cm lado, 8 frente, 5 graus', (0.08, 0.20, math.radians(5.0)))):
            c, s = math.cos(d[2]), math.sin(d[2])
            # pontos do scan como vistos depois de andar d
            q = f1 - np.array(d[:2])
            q = np.column_stack((c * q[:, 0] + s * q[:, 1], -s * q[:, 0] + c * q[:, 1]))
            r = icp(f0, q, chute=(0.0, d[1], 0.0))
            print('  Prova %s: dx %+.1f, dy %+.1f cm, giro %+.1f' % (
                nome, 100 * r[0], 100 * r[1], math.degrees(r[2])) if r else '  Prova %s: falhou' % nome, flush=True)
        if a.so_medir:
            return
        if no.count_publishers('/cmd_vel') != 1:
            raise SystemExit('RECUSADO: outro no publica em /cmd_vel.')
        if a.frente:
            for v in [float(x) for x in a.frente.split(',')]:
                fa = no.foto()
                oa = no.odom
                mot, t = no.mover(v, 0.0, 0.25, 8.0)
                time.sleep(0.8)
                fb = no.foto()
                ob = no.odom
                r = icp_varios(fa, fb, 0.0, ang_dif(ob[3], oa[3]))
                dod = math.hypot(ob[1] - oa[1], ob[2] - oa[2])
                print('frente v=%.2f: real %+.1f cm, lado %+.1f (odom %.1f) | %.1f s | media %.3f m/s | giro %+.1f%s' % (
                    v, 100 * r[0], 100 * r[1], 100 * dod, t, r[0] / t, math.degrees(r[2]),
                    ' | parou: ' + mot if mot else ''), flush=True)
                time.sleep(0.5)
                mot, t = no.mover(-v, 0.0, 0.25, 8.0)
                print('  voltou%s' % (': parou ' + mot if mot else ''), flush=True)
                time.sleep(0.8)
            return
        if a.recuar > 0:
            mot, _ = no.mover(-0.03, 0.0, a.recuar, 6.0)
            print('Recuo de %.0f cm%s' % (100 * a.recuar, ': parei por ' + mot if mot else ''), flush=True)
            if mot and ('parad' in mot):
                raise SystemExit('Encerrado: sensor sem dados.')
            time.sleep(0.8)
        for v in vels:
            for lado, nome in ((1, 'esquerda'), (-1, 'direita')):
                fa = no.foto()
                if fa is None:
                    raise SystemExit('Encerrado: sem scan antes do deslize.')
                oa = no.odom
                mot, t = no.mover(0.0, lado * v, a.dist, 2.5 * a.dist / 0.05 + 2.0)
                if mot:
                    print('  PAREI: %s' % mot, flush=True)
                if mot and 'parad' in mot:
                    raise SystemExit('Encerrado: sensor sem dados.')
                time.sleep(0.8)
                fb = no.foto()
                ob = no.odom
                c, s = math.cos(oa[3]), math.sin(oa[3])
                lat_od = -s * (ob[1] - oa[1]) + c * (ob[2] - oa[2])
                giro_od = ang_dif(ob[3], oa[3])
                r = icp_varios(fa, fb, lat_od, giro_od) if fb is not None else None
                if fb is not None:
                    os.makedirs(a.saida, exist_ok=True)
                    np.savez(os.path.join(a.saida, 'fotos_%s_%s_%.2f.npz' % (time.strftime('%H%M%S'), nome, v)),
                             antes=fa, depois=fb, lat_odom=lat_od, giro_imu=giro_od, tempo=t)
                if r is None:
                    print('  ICP nao casou para a %s: sem medida' % nome, flush=True)
                    continue
                dx, dy, dth, resid, n, amb = r
                k = dx / abs(dy) if abs(dy) > 0.03 else float('nan')
                res.append((nome, v, lat_od, dy, dx, k, math.degrees(dth), math.degrees(giro_od), t,
                            abs(dy) / t if t > 0 else 0.0, resid, n, mot or ''))
                print('%-8s v=%.2f: lado %+5.1f cm (odom %+5.1f) | frente %+5.1f cm | k %+.2f | giro %+5.1f (IMU %+5.1f) | %.1f s | residuo %.1f cm%s' % (
                    nome, v, 100 * dy, 100 * lat_od, 100 * dx, k, math.degrees(dth), math.degrees(giro_od),
                    t, 100 * resid, (' | AMBIGUO %.0f cm' % (100 * amb) if amb > 0.03 else '') + (' | parou: ' + mot if mot else '')), flush=True)
    finally:
        no.parar()
        if res:
            os.makedirs(a.saida, exist_ok=True)
            arq = os.path.join(a.saida, 'deslize_icp_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
            with open(arq, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(('lado', 'v_cmd', 'lat_odom_m', 'lat_icp_m', 'frente_icp_m', 'k', 'giro_icp_graus',
                            'giro_imu_graus', 'tempo_s', 'v_real_media', 'residuo_m', 'pares', 'parou_por'))
                w.writerows(res)
            print('Arquivo: %s' % arq, flush=True)
        ex.shutdown()
        giro.join(2.0)
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrompido: a ponte para sozinha em 0,25 s sem comando.')
