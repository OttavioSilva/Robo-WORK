#!/usr/bin/env python3
"""ARENA_FUSION (25/11): missao simples na arena de teste: manda as areas pelo /ir_para_area (como o executor_prova),
uma por vez, espera o /ir_para_area/resultado e cronometra. Termina sempre tentando o FINISH.
Uso: python3 missao_fusion.py [AREA ...]      (padrao: MESA1 FINISH)
     python3 missao_fusion.py MESA2 SHELF MESA1 FINISH
Ctrl+C cancela (o alinhador para com o proximo goal ou cancelamento manual no RViz)."""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TIMEOUT = 240.0


class Missao(Node):
    def __init__(self):
        super().__init__('missao_fusion')
        self.pub = self.create_publisher(String, '/ir_para_area', 10)
        self.res = []
        self.create_subscription(String, '/ir_para_area/resultado', lambda m: self.res.append(m.data), 10)


def main():
    rota = [a.upper() for a in sys.argv[1:]] or ['MESA1', 'FINISH']
    if rota[-1] != 'FINISH':
        rota.append('FINISH')
    rclpy.init()
    no = Missao()
    t_ini = time.monotonic()
    while no.pub.get_subscription_count() == 0 and time.monotonic() - t_ini < 15:
        rclpy.spin_once(no, timeout_sec=0.2)
    if no.pub.get_subscription_count() == 0:
        print('!!! Ninguem escuta /ir_para_area (alinhador nao subiu?)')
        return 1
    print('Rota: ' + ' -> '.join(rota))
    t0 = time.monotonic()
    tabela = []
    for area in rota:
        no.res.clear()
        m = String(); m.data = area
        no.pub.publish(m)
        ta = time.monotonic()
        print('>>> %s ...' % area, flush=True)
        r = None
        while time.monotonic() - ta < TIMEOUT:
            rclpy.spin_once(no, timeout_sec=0.2)
            r = next((x for x in no.res if x.split()[0] == area), None)
            if r:
                break
        dt = time.monotonic() - ta
        r = r or '%s SEM RESPOSTA em %.0f s' % (area, TIMEOUT)
        print('    %s (%.0f s)' % (r, dt), flush=True)
        tabela.append((area, r, dt))
        if 'SUCESSO' not in r and area != 'FINISH':
            print('    falhou: sigo para o FINISH')
            # pula o resto e vai ao FINISH
            no.res.clear(); m = String(); m.data = 'FINISH'; no.pub.publish(m); ta = time.monotonic()
            print('>>> FINISH ...', flush=True)
            r = None
            while time.monotonic() - ta < TIMEOUT:
                rclpy.spin_once(no, timeout_sec=0.2)
                r = next((x for x in no.res if x.split()[0] == 'FINISH'), None)
                if r:
                    break
            dt = time.monotonic() - ta
            r = r or 'FINISH SEM RESPOSTA'
            print('    %s (%.0f s)' % (r, dt)); tabela.append(('FINISH', r, dt))
            break
    print('\nTotal %.0f s' % (time.monotonic() - t0))
    for a, r, dt in tabela:
        print('  %-7s %5.0f s  %s' % (a, dt, r))
    no.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\nCancelado.')
