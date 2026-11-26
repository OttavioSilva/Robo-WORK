#!/usr/bin/env python3
"""TESTE_K_LATERAL v1 (12/11/2026): qual k do feedforward (vx = -k*vy) amansa o avanco do deslize lateral.

Robo DE FRENTE PARA A PAREDE com espaco dos lados. Para cada k: desliza para a esquerda e volta para a
direita (o robo termina perto de onde comecou), em modo CONTINUO (uma distancia) ou PULSOS (n pulsos curtos
com parada entre eles, como a saida lateral do alinhador na missao34).

Mede o avanco real de dois jeitos independentes:
  - parede da frente (mediana dos pontos |y| < 0,20 m a frente): dx direto, nao depende de ICP;
  - ICP entre fotos paradas (teste_deslize_icp.icp_varios): dx, dy e giro.
O comando passa pelo modelo do velocity_smoother validado na missao34 (ferramentas/analise/partida_lateral/
modelo.py), entao a partida (rampa, deadband que segura o vx) e a mesma do alinhador real.

Rode SO a ponte, o filtro do LiDAR e a localizacao (sem Nav2/CM): publica direto em /cmd_vel e recusa se
outro no publica. Para na hora (zero direto, sem rampa) com: frente < 12 cm, lado < 15 cm, atras < 10 cm,
giro > 25 graus, sensor parado. A ponte para sozinha 0,25 s sem comando. Ctrl+C para.

Uso:
  python3 -P teste_k_lateral.py --so-medir
  python3 -P teste_k_lateral.py --modo continuo --ks 0,0.2,0.44 --dist 0.25
  python3 -P teste_k_lateral.py --modo pulso --ks 0,0.2,0.44 --pulsos 4 --pulso 0.8
"""
import argparse
import csv
import math
import os
import sys
import time
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import MultiThreadedExecutor

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, '..', 'analise', 'partida_lateral'))
from teste_deslize_icp import Teste, icp, icp_varios, ang_dif, X_FRENTE  # noqa: E402
import modelo  # noqa: E402

FRENTE_MIN_INICIO = 0.18   # recusa comecar com o para-choque a menos disso da parede
V_LAT = 0.08


def parede_frente(fotos):
    """Distancia (m) do para-choque a parede da frente: mediana dos pontos a frente com |y| < 0,20."""
    p = fotos[(fotos[:, 0] > X_FRENTE) & (np.abs(fotos[:, 1]) < 0.20) & (fotos[:, 0] < X_FRENTE + 1.5)]
    return float(np.median(p[:, 0]) - X_FRENTE) if len(p) >= 5 else None


class TesteK(Teste):
    def configurar(self, topico):
        """/cmd_vel_nav: passa pelo velocity_smoother e Collision Monitor reais (stack completa no ar)."""
        self.topico = topico
        if topico != '/cmd_vel':
            self.destroy_publisher(self.pub)
            self.pub = self.create_publisher(TwistStamped, topico, 10)
        from rcl_interfaces.srv import SetParameters
        self.cli_par = self.create_client(SetParameters, '/robo_hardware/set_parameters')

    def ganhos(self, g):
        """GANHO_RODA: (fl, fr, rl, rr) na ponte. Devolve True se a ponte aceitou."""
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        if not self.cli_par.wait_for_service(timeout_sec=3.0):
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter('lateral_wheel_gain_' + n, value=float(v)).to_parameter_msg()
                          for n, v in zip(('fl', 'fr', 'rl', 'rr'), g)]
        f = self.cli_par.call_async(req)
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < 3.0:
            time.sleep(0.02)
        return f.done() and all(r.successful for r in f.result().results)

    def esperar_scan(self, seg=4.0):
        """Parado: espera o LiDAR voltar (falha de USB com os motores girando)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seg:
            s = self.scan
            if s is not None and time.monotonic() - s[0] < 0.25:
                time.sleep(0.4)
                return True
            time.sleep(0.05)
        return False

    def deslizar_total(self, lado, k, dur, partida=False):
        """Desliza dur s de motor; se o LiDAR falhar (>0,5 s), para na hora, espera voltar e retoma.
        Devolve (motivo de seguranca ou None, tempo de motor, retomadas)."""
        feito, ret = 0.0, 0
        while dur - feito > 0.15:
            mot, t = self.deslizar(lado, k, dur - feito, partida)
            feito += t
            if mot != 'LiDAR parado':
                return mot, feito, ret
            ret += 1
            if ret > 6 or not self.esperar_scan(12.0):
                return 'LiDAR parado (%d vezes)' % ret, feito, ret
        return None, feito, ret

    def deslizar(self, lado, k, dur, partida=False):
        """Um deslize de dur s (comando pelo modelo do smoother) e desaceleracao como o real.
        Devolve (motivo da parada de seguranca ou None, tempo)."""
        sm = modelo.Smoother()
        o0 = self.odom
        uy = lado
        t0 = time.monotonic()
        mot = None
        try:
            while True:
                t = time.monotonic() - t0
                mot = self.motivo(0, uy, o0[3])
                if mot:
                    return mot, t   # finally: zero direto (parada imediata)
                if t >= dur:
                    break
                vy = lado * V_LAT
                vx = -k * vy
                if partida and t < 0.5:   # PARTIDA_LATERAL (padrao do patch)
                    v = min(V_LAT, 0.03 + 0.16 * max(0.0, t - 0.15))
                    vy = lado * v
                    lim = v - 0.012
                    if abs(vx) > lim:
                        vx = math.copysign(lim, vx) if lim >= 0.016 else 0.0
                if vx != 0.0 and abs(vx) < 0.016:
                    vx = math.copysign(0.016, vx)   # mesmo piso do alinhador
                o = sm.step([vx, vy, 0.0]) if self.topico == '/cmd_vel' else [vx, vy, 0.0]
                self.vel(o[0], o[1])
                time.sleep(0.05)
            for _ in range(6 if self.topico == '/cmd_vel' else 0):   # desaceleracao (o smoother real ja faz)
                o = sm.step([0.0, 0.0, 0.0])
                self.vel(o[0], o[1])
                time.sleep(0.05)
            return None, time.monotonic() - t0
        finally:
            self.parar()


def foto_ok(no, tent=4):
    """Foto parada; se o LiDAR caiu (motores), espera voltar e tenta de novo."""
    for _ in range(tent):
        f = no.foto(1.5)
        if f is not None and len(f) >= 30:
            return f
        no.esperar_scan(6.0)
    raise SystemExit('Encerrado: LiDAR sem dados para a foto parada.')


def medir(no, fa, fb, oa, ob):
    c, s = math.cos(oa[3]), math.sin(oa[3])
    lat_od = -s * (ob[1] - oa[1]) + c * (ob[2] - oa[2])
    giro_od = ang_dif(ob[3], oa[3])
    r = icp_varios(fa, fb, lat_od, giro_od)
    pa, pb = parede_frente(fa), parede_frente(fb)
    dx_par = (pa - pb) if pa is not None and pb is not None else float('nan')
    return dict(lat_odom=lat_od, giro_imu=math.degrees(giro_od), dx_parede=dx_par,
                frente_depois=pb if pb is not None else float('nan'),
                dx_icp=r[0] if r else float('nan'), dy_icp=r[1] if r else float('nan'),
                giro_icp=math.degrees(r[2]) if r else float('nan'), resid=r[3] if r else float('nan'),
                amb=r[5] if r else float('nan'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--so-medir', action='store_true')
    ap.add_argument('--modo', choices=('continuo', 'pulso'), default='continuo')
    ap.add_argument('--ks', default='0,0.2,0.44')
    ap.add_argument('--dist', type=float, default=0.25, help='continuo: tempo = dist / 0,055 m/s real')
    ap.add_argument('--pulsos', type=int, default=4)
    ap.add_argument('--pulso', type=float, default=0.8, help='s por pulso')
    ap.add_argument('--partida', action='store_true', help='usa a PARTIDA_LATERAL do patch')
    ap.add_argument('--lados', default='esq,dir')
    ap.add_argument('--saida', default=os.path.expanduser('~/testes_k_lateral'))
    ap.add_argument('--topico', default='/cmd_vel', choices=('/cmd_vel', '/cmd_vel_nav'))
    ap.add_argument('--ganhos', default='1,1,1,1', help='GANHO_RODA: conjuntos fl,fr,rl,rr separados por ;')
    ap.add_argument('--pares', default='', help='ganhos por lado: "fl,fr,rl,rr|fl,fr,rl,rr" (esq|dir) separados por ;')
    a = ap.parse_args()
    ks = [float(x) for x in a.ks.split(',')]
    if not all(0.0 <= k <= 1.0 for k in ks) or not 0.05 <= a.dist <= 0.40 or not 1 <= a.pulsos <= 6 \
            or not 0.3 <= a.pulso <= 1.5:
        raise SystemExit('Limites: k 0..1, --dist 0,05..0,40, --pulsos 1..6, --pulso 0,3..1,5 s')
    lados = [{'esq': 1, 'dir': -1}[x] for x in a.lados.split(',')]
    gsets = [tuple(float(v) for v in g.split(',')) for g in a.ganhos.split(';')]
    if a.pares:
        gsets = [tuple(tuple(float(v) for v in x.split(',')) for x in par.split('|')) for par in a.pares.split(';')]
        planos = [g for par in gsets for g in par]
    else:
        gsets = [(g, g) for g in gsets]
        planos = [g for par in gsets for g in par]
    if not all(len(g) == 4 and all(0.5 <= v <= 1.6 for v in g) for g in planos):
        raise SystemExit('Ganhos: 4 valores entre 0,5 e 1,6 por conjunto')
    rclpy.init()
    no = TesteK()
    no.configurar(a.topico)
    no.get_logger().set_level(40)
    ex = MultiThreadedExecutor()
    ex.add_node(no)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    res = []
    os.makedirs(a.saida, exist_ok=True)
    arq = os.path.join(a.saida, 'k_lateral_%s_%s.csv' % (a.modo, time.strftime('%Y%m%d_%H%M%S')))
    try:
        time.sleep(2.5)
        f0 = no.foto()
        if f0 is None or len(f0) < 30:
            raise SystemExit('RECUSADO: poucos pontos do LiDAR.')
        f1 = no.foto()
        r = icp(f0, f1)
        pf = parede_frente(f0)
        print('Parado: %d pontos | parede da frente a %s cm do para-choque | ICP parado dx %+.1f dy %+.1f cm' % (
            len(f0), '%.1f' % (100 * pf) if pf is not None else '-', 100 * r[0] if r else float('nan'),
            100 * r[1] if r else float('nan')), flush=True)
        for nome, d in (('+3 cm frente', (0.03, 0.0)), ('+20 cm lado', (0.0, 0.20))):
            q = f1 - np.array(d)
            rr = icp_varios(f0, q, d[1], 0.0)
            print('  Prova ICP %s: dx %+.1f dy %+.1f cm, ambiguidade %.0f cm' % (
                nome, 100 * rr[0], 100 * rr[1], 100 * rr[5]) if rr else '  Prova ICP %s: falhou' % nome, flush=True)
        if a.so_medir:
            return
        if pf is None:
            raise SystemExit('RECUSADO: nao vejo a parede da frente.')
        if pf < FRENTE_MIN_INICIO:
            raise SystemExit('RECUSADO: parede a %.0f cm (minimo %.0f).' % (100 * pf, 100 * FRENTE_MIN_INICIO))
        if a.topico == '/cmd_vel' and no.count_publishers('/cmd_vel') != 1:
            raise SystemExit('RECUSADO: outro no publica em /cmd_vel.')
        dur_cont = a.dist / 0.055
        for par, k in [(g, k) for g in gsets for k in ks]:
            for lado in lados:
                g = par[0] if lado > 0 else par[1]
                if not no.ganhos(g):
                    raise SystemExit('RECUSADO: a ponte nao aceitou os ganhos %s (GANHO_RODA instalado?)' % (g,))
                nome = 'esq' if lado > 0 else 'dir'
                fa = foto_ok(no)
                oa = no.odom
                pf = parede_frente(fa)
                if pf is None or pf < 0.14:
                    print('PAREI a serie: parede da frente a %s cm' % ('-' if pf is None else '%.0f' % (100 * pf)))
                    return
                mot, tt, nret = None, 0.0, 0
                n = 1 if a.modo == 'continuo' else a.pulsos
                for i in range(n):
                    m_, t_, r_ = no.deslizar_total(lado, k, dur_cont if a.modo == 'continuo' else a.pulso, a.partida)
                    tt += t_
                    nret += r_
                    if m_:
                        mot = m_
                        break
                    if a.modo == 'pulso':
                        time.sleep(0.8)   # parado entre pulsos (assentar + medir do alinhador)
                time.sleep(0.8)
                fb = foto_ok(no)
                ob = no.odom
                m = medir(no, fa, fb, oa, ob)
                lat = m['dy_icp'] if math.isfinite(m['dy_icp']) and m['amb'] < 0.03 else m['lat_odom']
                kreal = m['dx_parede'] / abs(lat) if math.isfinite(m['dx_parede']) and abs(lat) > 0.02 else float('nan')
                linha = dict(modo=a.modo, partida=a.partida, ganhos='/'.join('%.2f' % v for v in g), k=k, lado=nome, n_pulsos=n, tempo=tt, retomadas=nret,
                             parou=mot or '', **m, dx_por_dy=kreal, dx_cm_s=100 * m['dx_parede'] / tt if tt > 0 else float('nan'))
                res.append(linha)
                print('ganhos FL %.2f FR %.2f RL %.2f RR %.2f | ' % g + 'k %.2f %-3s %-8s: AVANCO parede %+5.1f cm (ICP %+5.1f) | lado ICP %+5.1f (odom %+5.1f) | '
                      'dx/dy %+.2f | %+.1f cm/s | giro %+.1f | %.1f s motor, %d retomadas | frente agora %.0f cm%s' % (
                          k, nome, a.modo, 100 * m['dx_parede'], 100 * m['dx_icp'], 100 * m['dy_icp'],
                          100 * m['lat_odom'], kreal, linha['dx_cm_s'], m['giro_imu'], tt, nret, 100 * m['frente_depois'],
                          (' | ICP ambiguo %.0f cm' % (100 * m['amb']) if m['amb'] >= 0.03 else '') +
                          (' | PAROU: ' + mot if mot else '')), flush=True)
                if mot and 'parad' in mot:
                    raise SystemExit('Encerrado: sensor sem dados.')
    finally:
        no.parar()
        if (a.ganhos != '1,1,1,1' or a.pares) and not a.so_medir:
            print('Ganhos de volta a 1,0: %s' % ('ok' if no.ganhos((1.0, 1.0, 1.0, 1.0)) else 'FALHOU - confira com ros2 param get'),
                  flush=True)
        if res:
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
