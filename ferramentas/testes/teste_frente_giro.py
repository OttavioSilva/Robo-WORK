#!/usr/bin/env python3
"""TESTE_FRENTE_GIRO v1 (12/11/2026): efeito do GANHO_RODA ao andar para frente/tras e girar no lugar.

Para cada conjunto de ganhos (fl,fr,rl,rr): posiciona a ~48 cm da parede, anda dist para a frente e volta,
gira +ang e -ang. Mede pelo ICP entre fotos paradas (e pela parede da frente no andar):
  frente: distancia real, deriva lateral, giro;   giro: angulo real (ICP x IMU), deslocamento do centro.
Comandos em /cmd_vel_nav (velocity_smoother + Collision Monitor reais; stack completa no ar, sem goals).
Para na hora com as folgas do teste_deslize_icp; o giro so comeca com circulo de 0,58 m livre.
Ganhos voltam a 1,0 no fim.

Uso: python3 -P teste_frente_giro.py --ganhos "1,1,1,1;1.6,0.6,0.7,1.6;1,1,1,1;1.6,0.6,0.7,1.6"
"""
import argparse
import csv
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
from teste_k_lateral import TesteK, parede_frente  # noqa: E402
from teste_deslize_icp import icp_varios, ang_dif  # noqa: E402

RAIO_GIRO_LIVRE = 0.58   # canto do corpo a ~0,50 m do centro + 8 cm


class TesteFG(TesteK):
    def vel3(self, vx, vy, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(m)

    def circulo_livre(self):
        s = self.scan
        if s is None or time.monotonic() - s[0] > 0.5:
            return None
        p = s[1]
        corpo = (p[:, 0] >= -0.235) & (p[:, 0] <= 0.395) & (np.abs(p[:, 1]) <= 0.345)
        d = np.hypot(p[~corpo, 0], p[~corpo, 1])
        return float(d.min()) if d.size else 9.0

    def girar(self, ang, wz=0.30, t_max=8.0):
        """Gira ang (rad) pela odom/IMU. Para por sensor parado, circulo ocupado ou tempo."""
        o0 = self.odom
        alvo = abs(ang)
        t0 = time.monotonic()
        mot = None
        try:
            while True:
                agora = time.monotonic()
                s, o = self.scan, self.odom
                if s is None or agora - s[0] > 0.5:
                    mot = 'LiDAR parado'
                    break
                if o is None or agora - o[0] > 0.5:
                    mot = 'odom parada'
                    break
                c = self.circulo_livre()
                if c is not None and c < RAIO_GIRO_LIVRE - 0.05:
                    mot = 'circulo de giro ocupado (%.0f cm)' % (100 * c)
                    break
                feito = abs(ang_dif(o[3], o0[3]))
                if feito >= alvo - math.radians(4.0):   # o robo embala ~4 graus depois do zero
                    break
                if agora - t0 > t_max:
                    mot = 'tempo maximo'
                    break
                self.vel3(0.0, 0.0, math.copysign(wz, ang))
                time.sleep(0.05)
        finally:
            self.parar()
        return mot, time.monotonic() - t0


def posicionar(no, alvo=0.48):
    f = parede_frente(no.foto(1.0))
    if f is None:
        return 'sem parede'
    d = max(-0.30, min(0.30, alvo - f))
    if abs(d) > 0.03:
        mot, _ = no.mover(-0.03 if d > 0 else 0.03, 0.0, abs(d), 10.0)
        time.sleep(0.8)
        return mot
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ganhos', default='1,1,1,1;1.6,0.6,0.7,1.6;1,1,1,1;1.6,0.6,0.7,1.6')
    ap.add_argument('--dist', type=float, default=0.15)
    ap.add_argument('--vx', type=float, default=0.05)
    ap.add_argument('--ang', type=float, default=45.0)
    ap.add_argument('--saida', default=os.path.expanduser('~/testes_k_lateral'))
    a = ap.parse_args()
    gsets = [tuple(float(v) for v in g.split(',')) for g in a.ganhos.split(';')]
    if not all(len(g) == 4 and all(0.5 <= v <= 1.6 for v in g) for g in gsets) or not 0.05 <= a.dist <= 0.25 \
            or not 0.03 <= a.vx <= 0.10 or not 10 <= a.ang <= 90:
        raise SystemExit('Limites: ganhos 0,5..1,6; --dist 0,05..0,25; --vx 0,03..0,10; --ang 10..90')
    rclpy.init()
    no = TesteFG()
    no.configurar('/cmd_vel_nav')
    no.get_logger().set_level(40)
    ex = MultiThreadedExecutor()
    ex.add_node(no)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    res = []
    arq = os.path.join(a.saida, 'frente_giro_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
    try:
        time.sleep(2.5)
        for g in gsets:
            tag = '/'.join('%.2f' % v for v in g)
            if not no.ganhos(g):
                raise SystemExit('RECUSADO: a ponte nao aceitou os ganhos %s' % tag)
            m = posicionar(no)
            if m:
                print('posicionar: %s' % m, flush=True)
            # ---- frente e volta
            for nome, v in (('frente', a.vx), ('tras', -a.vx)):
                fa = no.foto(1.5)
                oa = no.odom
                mot, t = no.mover(v, 0.0, a.dist, 8.0)
                time.sleep(0.8)
                fb = no.foto(1.5)
                ob = no.odom
                gi = ang_dif(ob[3], oa[3])
                r = icp_varios(fa, fb, 0.0, gi)
                pa, pb = parede_frente(fa), parede_frente(fb)
                dxp = (pa - pb) if pa is not None and pb is not None else float('nan')
                dx, dy, dth = (r[0], r[1], math.degrees(r[2])) if r else (float('nan'),) * 3
                lin = dict(ganhos=tag, teste=nome, cmd=v, real_parede_cm=100 * dxp, real_icp_cm=100 * dx,
                           lateral_icp_cm=100 * dy, giro_icp=dth, giro_imu=math.degrees(gi), tempo=t, parou=mot or '',
                           deriva_por_m=dy / dx if r and abs(dx) > 0.03 else float('nan'))
                res.append(lin)
                print('ganhos %s | %-6s: real %+5.1f cm (ICP %+5.1f) | LATERAL %+5.1f cm (%.2f cm/10 cm) | giro %+5.1f (IMU %+5.1f) | %.1f s%s' % (
                    tag, nome, lin['real_parede_cm'], lin['real_icp_cm'], lin['lateral_icp_cm'],
                    10 * abs(lin['deriva_por_m']) if math.isfinite(lin['deriva_por_m']) else float('nan'),
                    dth, lin['giro_imu'], t, ' | PAROU: ' + mot if mot else ''), flush=True)
            # ---- giro no lugar
            c = None
            for _ in range(10):
                c = no.circulo_livre()
                if c is not None:
                    break
                time.sleep(0.1)
            if c is None or c < RAIO_GIRO_LIVRE:
                print('ganhos %s | giro PULADO: circulo livre %s cm (precisa %.0f)' % (
                    tag, '-' if c is None else '%.0f' % (100 * c), 100 * RAIO_GIRO_LIVRE), flush=True)
                continue
            for sinal in (1, -1):
                fa = no.foto(1.5)
                oa = no.odom
                mot, t = no.girar(math.radians(sinal * a.ang))
                time.sleep(0.8)
                fb = no.foto(1.5)
                ob = no.odom
                gi = ang_dif(ob[3], oa[3])
                r = icp_varios(fa, fb, 0.0, gi)
                dx, dy, dth = (r[0], r[1], math.degrees(r[2])) if r else (float('nan'),) * 3
                desl = 100 * math.hypot(dx, dy) if r else float('nan')
                lin = dict(ganhos=tag, teste='giro%+d' % (sinal * a.ang), cmd=0.30 * sinal, real_parede_cm=float('nan'),
                           real_icp_cm=100 * dx, lateral_icp_cm=100 * dy, giro_icp=dth, giro_imu=math.degrees(gi),
                           tempo=t, parou=mot or '', deriva_por_m=float('nan'))
                res.append(lin)
                print('ganhos %s | giro %+3.0f: real ICP %+5.1f graus (IMU %+5.1f) | centro saiu %.1f cm (x %+.1f, y %+.1f) | %.1f s | %.0f graus/s%s' % (
                    tag, sinal * a.ang, dth, math.degrees(gi), desl, 100 * dx, 100 * dy, t,
                    abs(dth) / t if t > 0 else 0, ' | PAROU: ' + mot if mot else ''), flush=True)
    finally:
        no.parar()
        print('Ganhos de volta a 1,0: %s' % ('ok' if no.ganhos((1.0, 1.0, 1.0, 1.0)) else 'FALHOU - confira'), flush=True)
        if res:
            os.makedirs(a.saida, exist_ok=True)
            with open(arq, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(res[0].keys()))
                w.writeheader()
                w.writerows(res)
            print('Arquivo: %s' % arq, flush=True)
        ex.shutdown()
        th.join(2.0)
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrompido: a ponte para sozinha em 0,25 s sem comando.')
