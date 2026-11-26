#!/usr/bin/env python3
"""CURVA_M2M2 (26/11/2026): mede a curva distancia real x leitura BRUTA do M2M2 a frente.

Motivo: a correcao fixa real = (bruta - 0,30) * 0,85 acertou 38 cm mas errou 30 cm (leitura 50).
Uso (robo PARADO, barreira plana na frente, distancia medida na trena a partir do CENTRO do LiDAR):
    python3 curva_m2m2.py
  digite a distancia real em cm + Enter -> grava 5 s -> mostra a leitura; 'f' + Enter termina.
Le o /scan_raw e desfaz a correcao em uso na ponte (range_offset/range_scale) -> leitura bruta.
Por scan: menor leitura em +-10 graus da frente; resultado = mediana dos scans (e min/max).
Salva CSV em ~/mapas/curva_m2m2_<data>.csv (real_cm, bruta_mediana_cm, bruta_min, bruta_max, n,
corrigida_atual_cm).
"""
import csv
import math
import os
import statistics
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SETOR = math.radians(10.0)


def main():
    rclpy.init()
    no = Node('curva_m2m2')
    cli = no.create_client(GetParameters, '/m2m2_bridge/get_parameters')
    off, esc = 0.0, 1.0
    if cli.wait_for_service(timeout_sec=5.0):
        req = GetParameters.Request()
        req.names = ['range_offset', 'range_scale']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(no, fut, timeout_sec=5.0)
        if fut.result() and len(fut.result().values) == 2:
            off, esc = fut.result().values[0].double_value, fut.result().values[1].double_value
    print('Correcao em uso na ponte: offset %.3f m, escala %.3f (desfeita para obter a bruta)' % (off, esc))
    st = {'gravando': False, 'v': []}

    def ao_scan(m):
        if not st['gravando']:
            return
        melhor = None
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r):
                continue
            a = math.atan2(math.sin(m.angle_min + i * m.angle_increment), math.cos(m.angle_min + i * m.angle_increment))
            if abs(a) <= SETOR and (melhor is None or r < melhor):
                melhor = r
        if melhor is not None:
            st['v'].append(melhor)

    no.create_subscription(LaserScan, '/scan_raw', ao_scan, qos_profile_sensor_data)
    arq = os.path.expanduser('~/mapas/curva_m2m2_%s.csv' % time.strftime('%Y%m%d_%H%M'))
    linhas = []
    while True:
        txt = input('\nDistancia REAL do centro do LiDAR ate a barreira (cm), ou f para terminar: ').strip()
        if txt.lower().startswith('f'):
            break
        try:
            real = float(txt.replace(',', '.'))
        except ValueError:
            print('numero invalido')
            continue
        st['v'] = []
        st['gravando'] = True
        t = time.time()
        while time.time() - t < 5.0:
            rclpy.spin_once(no, timeout_sec=0.05)
        st['gravando'] = False
        if len(st['v']) < 5:
            print('poucos scans (%d): LiDAR parado? repita' % len(st['v']))
            continue
        corr = st['v']
        bruta = [r / esc + off for r in corr]
        lin = [real, 100 * statistics.median(bruta), 100 * min(bruta), 100 * max(bruta), len(bruta),
               100 * statistics.median(corr)]
        linhas.append(lin)
        print('  real %.1f cm -> BRUTA %.1f cm (min %.1f, max %.1f, %d scans); com a correcao atual: %.1f cm'
              % tuple(lin))
    if linhas:
        with open(arq, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['real_cm', 'bruta_mediana_cm', 'bruta_min_cm', 'bruta_max_cm', 'n', 'corrigida_atual_cm'])
            w.writerows(sorted(linhas))
        print('\nSalvo: %s' % arq)
        for lin in sorted(linhas):
            print('  real %6.1f  bruta %6.1f  (atual corrigida %6.1f)' % (lin[0], lin[1], lin[5]))


if __name__ == '__main__':
    main()
