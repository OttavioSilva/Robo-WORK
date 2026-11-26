#!/usr/bin/env python3
"""NAV_INTEGRADA: SO LEITURA. Com o robo PARADO, mostra o que o destravar v4 autorizaria aqui e por que.

Nao cria publicador, nao chama acao, nao muda parametro. Assina /scan_nav e mostra, para re 10/15 cm,
lateral esq/dir 10-40 cm e giro +-40 graus: pontos na area varrida, % da area observada pelo LiDAR,
se o Collision Monitor barraria e (re) se o ICP conseguiria confirmar o deslocamento.
Serve para validar a cobertura traseira ANTES de liberar a re (ros2 param set /destravar permitir_re true).

    python3 ~/robo_jazzy_ws/ferramentas/testes/conferir_manobras_parado.py [--scans 6]
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                'src', 'robo_navigation', 'scripts'))
import recuperacao_avancada as ra  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scans', type=int, default=6)
    ap.add_argument('--cobertura-re', type=float, default=0.90)
    ap.add_argument('--cobertura-lateral', type=float, default=0.50)
    a = ap.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rclpy.init()
    no = rclpy.create_node('conferir_manobras_parado')
    lidos = []
    no.create_subscription(LaserScan, '/scan_nav', lambda m: lidos.append(m), qos_profile_sensor_data)
    t1 = time.monotonic() + 10
    while len(lidos) < a.scans and time.monotonic() < t1:
        rclpy.spin_once(no, timeout_sec=0.2)
    no.destroy_node()
    rclpy.try_shutdown()
    if not lidos:
        raise SystemExit('sem /scan_nav em 10 s')
    for k, m in enumerate(lidos[-3:]):
        r = ra.Raios(m.angle_min, m.angle_increment, tuple(m.ranges), max(m.range_min, 0.146), m.range_max, ra.LASER)
        pts = r.pontos()
        inf = sum(1 for x in m.ranges if not math.isfinite(x)) / max(1, len(m.ranges))
        print('\n== scan %d: %d raios, %.0f%% sem retorno, frente %s' % (
            k + 1, len(m.ranges), 100 * inf, '-' if ra.folga_frente(pts) >= 9 else '%.0f cm' % (100 * ra.folga_frente(pts))))
        for d in (0.10, 0.15):
            av = ra.avaliar(r, [(-d, 0.0, 0.0)], cobertura_min=a.cobertura_re)
            ver, why = ra.icp_verificavel(pts, -1.0, 0.0, d)
            cm = ra.cm_barraria(pts, 'backward')
            print('  re %2.0f cm : %s | pontos %d | observada %3.0f%% | CM %s | ICP %s' % (
                100 * d, 'AUTORIZARIA' if av.livre and ver and not cm else 'recusaria', av.obstaculos,
                100 * av.cobertura, 'barraria' if cm else 'ok', 'ok' if ver else why))
        for lado, nome in ((1, 'esq'), (-1, 'dir')):
            for d in (0.10, 0.25, 0.40):
                av = ra.avaliar(r, ra._lateral_movs(lado * d, 0.4), cobertura_min=a.cobertura_lateral)
                cm = ra.cm_barraria(pts, 'left' if lado > 0 else 'right')
                print('  %s %2.0f cm: %s | pontos %d | observada %3.0f%% | CM %s' % (
                    nome, 100 * d, 'livre' if av.livre and not cm else 'recusaria', av.obstaculos,
                    100 * av.cobertura, 'barraria' if cm else 'ok'))
        for g in (0.70, -0.70):
            av = ra.avaliar(r, [(0.0, 0.0, g)], margem=0.045, cobertura_min=0.5)
            print('  giro %+3.0f graus: %s | pontos %d | observada %3.0f%%' % (
                math.degrees(g), 'livre' if av.livre else 'recusaria', av.obstaculos, 100 * av.cobertura))


if __name__ == '__main__':
    main()
