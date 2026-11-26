#!/usr/bin/env python3
"""Testa o CONTORNAR sem o robo: destravar_node.py de verdade no dominio ROS 87, num mundo simulado em
que o scan muda conforme o robo anda. Nada chega na ponte, que esta em outro dominio.

Uso: python3 -P testar_contornar.py [caminho do destravar_node.py]
"""
import math
import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ['ROS_DOMAIN_ID'] = '87'
import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
import testar_bloqueio_local as T  # noqa: E402

CORPO = (-0.215, 0.375, 0.325)


class MundoObst(T.Mundo):
    def __init__(self, obst=None, parede_x=None, fim_caminho=1.5):
        self.obst, self.parede_x, self.fim_caminho = obst, parede_x, fim_caminho
        self.toques = 0
        self.cmds = []
        super().__init__([])

    def caminho(self):
        p = Path()
        p.header.frame_id = 'map'
        for i in range(int(self.fim_caminho / 0.05) + 1):
            s = PoseStamped()
            s.header.frame_id = 'map'
            s.pose.position.x = 0.05 * i
            s.pose.orientation.w = 1.0
            p.poses.append(s)
        return p

    def ao_cmd(self, m):
        super().ao_cmd(m)
        if abs(m.twist.linear.x) > 1e-3 or abs(m.twist.linear.y) > 1e-3:
            self.cmds.append((time.monotonic(), m.twist.linear.x, m.twist.linear.y))

    def corpo_rel(self):
        c, s = math.cos(self.a), math.sin(self.a)
        dx, dy = self.obst[0] - self.x, self.obst[1] - self.y
        return c * dx + s * dy, -s * dx + c * dy

    def scan(self):
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser_raw'
        n = 180
        m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / n
        m.range_min, m.range_max = 0.02, 6.0
        lx = self.x + 0.33 * math.cos(self.a)
        ly = self.y + 0.33 * math.sin(self.a)
        m.ranges = []
        for i in range(n):
            a = m.angle_min + i * m.angle_increment + self.a
            ca, sa = math.cos(a), math.sin(a)
            r = float('inf')
            if self.parede_x is not None and ca > 1e-3:
                r = min(r, (self.parede_x - lx) / ca)
            if self.obst is not None:
                dx, dy = self.obst[0] - lx, self.obst[1] - ly
                t = dx * ca + dy * sa
                if t > 0 and math.hypot(dx - t * ca, dy - t * sa) < 0.025:
                    r = min(r, t)
            m.ranges.append(r if 0.146 <= r <= 5.0 else float('inf'))
        self.pub_scan.publish(m)
        if self.obst is not None:
            rx, ry = self.corpo_rel()
            if CORPO[0] < rx < CORPO[1] and abs(ry) < CORPO[2]:
                self.toques += 1


def cenario(nome, verificar, **kw):
    print('\n--- %s' % nome)
    log = open(os.path.join(os.environ.get('TMPDIR', '/tmp'), 'destravar_teste.log'), 'w')
    proc = subprocess.Popen([sys.executable, '-P', T.NODE], stdout=log, stderr=log, env=dict(os.environ))
    no = MundoObst(**kw)
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(no)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        time.sleep(2.0)
        no.pub_plano.publish(no.caminho())
        time.sleep(1.0)
        n0 = len(no.cmds)
        assert no.chamar(), 'o destravar nao terminou'
        log.flush()
        texto = open(log.name, errors='replace').read()
        print('    pose final (%.2f, %.2f); toques no corpo %d' % (no.x, no.y, no.toques))
        for linha in texto.splitlines():
            if 'CONTORNAR' in linha or 'DESTRAVAR' in linha:
                print('    ' + linha.split(']: ')[-1][:110])
        verificar(no, texto, no.cmds[n0:], list(no.eventos))
        print('    OK')
        return True
    except AssertionError as e:
        print('    FALHOU: %s' % e)
        return False
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        ex.shutdown()
        no.destroy_node()


def v_vence(lado):
    def f(no, texto, cmds, ev):
        assert 'CONTORNAR: venci' in texto, 'nao venceu'
        rx, ry = no.corpo_rel()
        assert abs(ry) >= CORPO[2] + 0.03, 'o corpo nao livrou o obstaculo: y %.3f' % ry
        assert rx < CORPO[1] - 0.10, 'nao avancou: obstaculo ainda a frente, x %.3f' % rx
        assert no.toques == 0, 'o obstaculo entrou no corpo %d vezes' % no.toques
        assert (no.y > 0) == (lado > 0), 'deslizou para o lado errado'
        assert not any(n == 'nuvem_cheia' for _, n in ev), 'marcou bloqueio ao contornar'
    return f


def v_nao_contorna(no, texto, cmds, ev):
    assert 'CONTORNAR' not in texto, 'contornou onde nao devia'


def main():
    rclpy.init()
    print('Testando %s' % T.NODE)
    r = [
        cenario('obstaculo na quina direita (missao22)', v_vence(+1), obst=(0.40, -0.28)),
        cenario('obstaculo na quina esquerda', v_vence(-1), obst=(0.40, 0.28)),
        cenario('obstaculo a 2 cm, fora do ponto cego', v_vence(+1), obst=(0.42, -0.20)),
        cenario('parede larga na frente', v_nao_contorna, parede_x=0.45),
        cenario('obstaculo perto do destino', v_nao_contorna, obst=(0.40, -0.28), fim_caminho=0.40),
    ]
    rclpy.shutdown()
    print('\n%d de %d cenarios OK' % (sum(r), len(r)))
    return 0 if all(r) else 1


if __name__ == '__main__':
    sys.exit(main())
