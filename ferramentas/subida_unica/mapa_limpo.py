#!/usr/bin/env python3
"""MAPA_LIMPO (21/11/2026): /map -> /map_nav, apagando falhas de mapeamento conhecidas.

O mapa arena_21-11_1727 ficou com pontos de "obstaculo" embaixo do robo no START (gente perto
do robo no inicio/fim do mapeamento). Com o centro do robo dentro de um obstaculo o planner nao
planeja nada. Este no republica o mapa do SLAM para o costmap GLOBAL do Nav2 com as zonas do
arquivo <mapa>_limpar.yaml marcadas como livres. A localizacao e o /map original nao mudam; o
LiDAR ao vivo (obstacle_layer) continua vendo qualquer obstaculo real.

Zonas: retangulos no referencial de uma pose (x, y, yaw do mapa):
  limpar:
    - {nome: corpo_no_start, x: 0.082, y: -0.123, yaw: -0.0547, frente: 0.415, tras: 0.205, lado: 0.365}
Sem arquivo (ou --zonas vazio) o mapa passa sem mudanca.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml


def carregar_zonas(caminho):
    if not caminho or not Path(caminho).expanduser().exists():
        return []
    dados = yaml.safe_load(Path(caminho).expanduser().read_text()) or {}
    zonas = []
    for z in dados.get('limpar', []) or []:
        vals = [float(z[k]) for k in ('x', 'y', 'yaw', 'frente', 'tras', 'lado')]
        if not all(math.isfinite(v) for v in vals) or min(vals[3:]) < 0 or max(vals[3:]) > 1.5:
            raise ValueError('zona invalida: %r' % z)
        zonas.append(dict(z, x=vals[0], y=vals[1], yaw=vals[2], frente=vals[3], tras=vals[4], lado=vals[5]))
    return zonas


def limpar(dados, largura, altura, resolucao, origem_x, origem_y, zonas):
    """Devolve (lista nova, celulas limpas). Celulas com centro dentro de uma zona viram 0 (livre).

    So mexe em celulas ocupadas ou desconhecidas dentro das zonas; o resto e copiado igual.
    Origem do OccupancyGrid sem rotacao (o slam_toolbox publica assim).
    """
    grade = np.asarray(dados, dtype=np.int16).reshape(altura, largura).copy()
    total = 0
    if zonas:
        xs = origem_x + (np.arange(largura) + 0.5) * resolucao
        ys = origem_y + (np.arange(altura) + 0.5) * resolucao
        gx, gy = np.meshgrid(xs, ys)
        for z in zonas:
            c, s = math.cos(z['yaw']), math.sin(z['yaw'])
            dx, dy = gx - z['x'], gy - z['y']
            frente = dx * c + dy * s
            lado = -dx * s + dy * c
            dentro = (frente >= -z['tras']) & (frente <= z['frente']) & (np.abs(lado) <= z['lado'])
            mudar = dentro & (grade != 0)
            total += int(mudar.sum())
            grade[mudar] = 0
    return grade.ravel().astype(np.int8).tolist(), total


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--zonas', default='')
    a = p.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    zonas = carregar_zonas(a.zonas)
    rclpy.init()
    no = Node('mapa_limpo')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    pub = no.create_publisher(OccupancyGrid, '/map_nav', qos)
    estado = dict(ultimo=None)
    if zonas:
        no.get_logger().info('MAPA_LIMPO: %d zona(s) de %s: %s' % (len(zonas), a.zonas, [z.get('nome') for z in zonas]))
    else:
        no.get_logger().info('MAPA_LIMPO: sem zonas (%s); /map_nav = /map' % (a.zonas or 'nenhum arquivo'))

    def recebeu(m):
        o = m.info.origin
        if abs(o.orientation.z) > 1e-6 or abs(o.orientation.x) > 1e-6 or abs(o.orientation.y) > 1e-6:
            no.get_logger().error('MAPA_LIMPO: mapa com origem rodada; repassando sem limpar', throttle_duration_sec=10.)
            pub.publish(m)
            return
        dados, total = limpar(m.data, m.info.width, m.info.height, m.info.resolution,
                              o.position.x, o.position.y, zonas)
        novo = OccupancyGrid(header=m.header, info=m.info, data=dados)
        pub.publish(novo)
        if total != estado['ultimo']:
            no.get_logger().info('MAPA_LIMPO: %d celula(s) apagadas nas zonas' % total)
            estado['ultimo'] = total

    no.create_subscription(OccupancyGrid, '/map', recebeu, qos)
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
