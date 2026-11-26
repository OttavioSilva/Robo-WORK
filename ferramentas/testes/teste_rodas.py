#!/usr/bin/env python3
"""TESTE_RODAS v1 (06/11/2026): compara os pares diagonais com o robo SUSPENSO (H2 da missao22).

Por que: indo para a esquerda FR+RL giram para a frente e FL+RR para tras; indo para a direita,
o contrario. Se FR+RL for mais forte, o deslize sai torto exatamente como o bag mostra.

Mistura da ponte (hardware_bridge_node.cpp): FL=vx-vy, FR=vx+vy, RL=vx+vy, RR=vx-vy.
Com vx=vy=0,04 giram so FR e RL a 0,08, o mesmo PWM de um deslize com vy=0,08.

Rode SO a ponte: o teste recusa se outro no publica em /cmd_vel. Ctrl+C para na hora.
Uso: python3 -P teste_rodas.py [--v 0.04] [--tempo 20] [--saida ~/testes_h2]
"""
import argparse
import csv
import os
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

RODAS = ('FL', 'FR', 'RL', 'RR')
PASSOS = (  # nome, sinal de vx, sinal de vy, rodas que giram
    ('FR+RL para a frente', 1, 1, ('FR', 'RL')),
    ('FR+RL para tras', -1, -1, ('FR', 'RL')),
    ('FL+RR para a frente', 1, -1, ('FL', 'RR')),
    ('FL+RR para tras', -1, 1, ('FL', 'RR')),
)


class Rodas(Node):
    def __init__(self):
        super().__init__('teste_rodas')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.diag = {}
        self.t_diag = None
        self.create_subscription(DiagnosticArray, '/diagnostics', self.ao_diag, 10)

    def ao_diag(self, msg):
        for st in msg.status:
            vals = {kv.key: kv.value for kv in st.values}
            if 'target_pwm_FL' in vals:
                self.diag, self.t_diag = vals, time.monotonic()

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


def pede_voltas(roda):
    while True:
        txt = input('    Voltas da %s (ex.: 8.5; vazio = nao contei): ' % roda).strip().replace(',', '.')
        if not txt:
            return None
        try:
            return float(txt)
        except ValueError:
            print('    Numero invalido.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--v', type=float, default=0.04, help='vx e vy (m/s); a roda recebe o dobro')
    ap.add_argument('--tempo', type=float, default=20.0)
    ap.add_argument('--saida', default=os.path.expanduser('~/testes_h2'))
    a = ap.parse_args()
    if not 0.02 <= a.v <= 0.08 or not 5.0 <= a.tempo <= 60.0:
        raise SystemExit('Use --v entre 0.02 e 0.08 e --tempo entre 5 e 60.')
    rclpy.init()
    no = Rodas()
    ex = SingleThreadedExecutor()
    ex.add_node(no)
    giro = threading.Thread(target=ex.spin, daemon=True)
    giro.start()
    linhas = []
    try:
        time.sleep(3.0)  # descoberta e pelo menos um /diagnostics
        if no.count_publishers('/cmd_vel') != 1:
            raise SystemExit('RECUSADO: outro no publica em /cmd_vel. Pare o Nav2 e deixe so a ponte.')
        if no.t_diag is None or time.monotonic() - no.t_diag > 3.0:
            raise SystemExit('RECUSADO: sem /diagnostics da ponte. A ponte esta rodando?')
        print('As 4 rodas precisam estar FORA do chao. Marque cada roda com fita para contar as voltas.')
        if input('Digite suspenso para continuar: ').strip().lower() != 'suspenso':
            raise SystemExit('Cancelado: nada foi comandado.')
        for nome, sx, sy, giram in PASSOS:
            input('\n%s, %.0f s. Comece a filmar e aperte Enter (Ctrl+C para)...' % (nome, a.tempo))
            t0 = time.monotonic()
            alvo, atual = {r: [] for r in RODAS}, {r: [] for r in RODAS}
            try:
                while time.monotonic() - t0 < a.tempo:
                    no.vel(sx * a.v, sy * a.v)
                    d = no.diag
                    if time.monotonic() - t0 > 2.0 and d:
                        for r in RODAS:
                            alvo[r].append(int(d['target_pwm_' + r]))
                            atual[r].append(int(d['current_pwm_' + r]))
                    time.sleep(0.05)
            finally:
                no.parar()
            med = {r: sorted(alvo[r])[len(alvo[r]) // 2] if alvo[r] else None for r in RODAS}
            med2 = {r: sorted(atual[r])[len(atual[r]) // 2] if atual[r] else None for r in RODAS}
            print('    PWM alvo %s | atual %s' % (med, med2))
            parada = [r for r in RODAS if r not in giram and med[r]]
            if parada:
                print('    AVISO: %s deveria ficar parada (correcao de rumo atuando?)' % parada)
            for r in giram:
                v = pede_voltas(r)
                linhas.append((nome, r, med[r], med2[r], a.tempo, v))
    finally:
        no.parar()
        ex.shutdown()
        giro.join(2.0)
        rclpy.shutdown()
    if linhas:
        os.makedirs(a.saida, exist_ok=True)
        arq = os.path.join(a.saida, 'rodas_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
        with open(arq, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(('passo', 'roda', 'pwm_alvo', 'pwm_atual', 'tempo_s', 'voltas'))
            w.writerows(linhas)
        print('\n=== Resultado (voltas por minuto) ===')
        for nome, r, _, _, t, v in linhas:
            print('%-22s %s: %s' % (nome, r, '-' if v is None else '%.1f' % (60.0 * v / t)))
        par = {}
        for nome, r, _, _, _, v in linhas:
            if v is not None:
                par.setdefault(nome.split()[0], []).append(v)
        if len(par) == 2:
            m = {k: sum(x) / len(x) for k, x in par.items()}
            print('Media FR+RL / FL+RR = %.2f (acima de 1: FR+RL mais forte, como o bag sugere)' % (
                m['FR+RL'] / m['FL+RR']))
        print('Arquivo: %s' % arq)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrompido: a ponte para sozinha em 0,25 s sem comando.')
