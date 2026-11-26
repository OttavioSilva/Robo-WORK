#!/usr/bin/env python3
"""Missao simples entre Service Areas usando o alinhador (v3.8 ou mais novo).

Para cada area da lista: manda o nome em /ir_para_area, espera a resposta do alinhador
em /ir_para_area/resultado ("WS_1 SUCESSO ..." ou "WS_1 FALHOU <motivo>"), espera alguns
segundos parado na mesa e segue para a proxima. Nao depende de ler logs.

Uso:
    python3 missao_ws.py WS_1 WS_2 WS_1
    python3 missao_ws.py WS_1 WS_2 WS_1 --espera 10 --tentativas 2 --timeout 240

Ctrl+C para so a missao: o alinhador termina o que estiver fazendo. Para parar o robo,
de Ctrl+C no alinhador.
"""
import argparse
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from std_msgs.msg import String


class Missao(Node):
    def __init__(self):
        super().__init__('missao_ws')
        self.pub = self.create_publisher(String, '/ir_para_area', 10)
        self.create_subscription(String, '/ir_para_area/resultado', self.ao_resultado, 10)
        self.resultados = []

    def ao_resultado(self, msg):
        self.resultados.append(msg.data)

    def esperar(self, segundos, condicao=lambda: False):
        fim = time.monotonic() + segundos
        while rclpy.ok() and time.monotonic() < fim:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condicao():
                return True
        return condicao()

    def ir(self, area, timeout):
        """Devolve (ok, texto) ou (None, motivo) se nao houve resposta."""
        if not self.esperar(10.0, lambda: self.pub.get_subscription_count() > 0):
            return None, 'ninguem escutando /ir_para_area (o alinhador v3.8 esta rodando?)'
        self.resultados.clear()
        msg = String()
        msg.data = area
        self.pub.publish(msg)
        self.get_logger().info('-> %s' % area)

        def chegou():
            return any(r.split()[0].upper() == area.upper() for r in self.resultados if r.split())

        if not self.esperar(timeout, chegou):
            return None, 'sem resposta em %.0f s' % timeout
        r = [r for r in self.resultados if r.split() and r.split()[0].upper() == area.upper()][-1]
        partes = r.split(None, 2)
        ok = len(partes) > 1 and partes[1] == 'SUCESSO'
        return ok, (partes[2] if len(partes) > 2 else '')


def main():
    ap = argparse.ArgumentParser(description='Visita Service Areas em sequencia.')
    ap.add_argument('areas', nargs='+', help='ex.: WS_1 WS_2 WS_1')
    ap.add_argument('--espera', type=float, default=10.0, help='segundos parado em cada mesa')
    ap.add_argument('--tentativas', type=int, default=2, help='tentativas por mesa')
    ap.add_argument('--timeout', type=float, default=240.0, help='segundos maximos por mesa')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    no = Missao()
    log = no.get_logger()
    resumo = []
    codigo = 0
    try:
        for i, area in enumerate(args.areas, 1):
            area = area.upper()
            ok = None
            for tentativa in range(1, args.tentativas + 1):
                log.info('Etapa %d/%d: %s (tentativa %d)' % (i, len(args.areas), area, tentativa))
                ok, texto = no.ir(area, args.timeout)
                if ok:
                    log.info('%s: SUCESSO %s' % (area, texto))
                    break
                log.warning('%s: %s %s' % (area, 'FALHOU' if ok is False else 'SEM RESPOSTA', texto))
                if ok is None:
                    break  # sem resposta: nao adianta repetir
            resumo.append((area, ok, texto))
            if not ok:
                log.error('Missao interrompida em %s.' % area)
                codigo = 1
                break
            if i < len(args.areas):
                log.info('Parado em %s por %.0f s' % (area, args.espera))
                no.esperar(args.espera)
    except KeyboardInterrupt:
        log.warning('Missao interrompida pelo usuario (o alinhador continua o que estava fazendo).')
        codigo = 2
    finally:
        print('\n===== RESUMO DA MISSAO =====')
        for area, ok, texto in resumo:
            print('%-6s %-12s %s' % (area, {True: 'SUCESSO', False: 'FALHOU', None: 'SEM RESPOSTA'}[ok], texto))
        faltou = args.areas[len(resumo):]
        if faltou:
            print('nao executadas: %s' % ' '.join(faltou))
        no.destroy_node()
        rclpy.try_shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
