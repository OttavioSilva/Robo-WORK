#!/usr/bin/env python3
"""PASSAGEM_IMPOSSIVEL (25/11/2026): "passagem" mais estreita que o possivel = mapa e costmap discordam: replaneja.

Bags 24/11 (ws10_mesa_contorno, ws08_alinhar): o alinhador achou "passagem" com largura no mapa 0,00 m e 0,35 m
(o caminho do planner atravessava algo ocupado no /map: costmap com mapa velho / prateleira oca no contorno),
fez preparo e travessia ("nao vejo as paredes da passagem"), ANTI_CISCADO e falhou. Corpo 0,65 m: passagem
< passagem_largura_minima (0,40) nao existe. Agora, com goal: limpa os costmaps e replaneja ate
passagem_replanejar_max vezes (1,5 s entre elas); sumiu -> segue normal; continua -> NAO faz preparo/travessia,
segue com o Nav2 (o costmap e o CM continuam protegendo) e loga. Vigia (sem goal): so ignora, nao interrompe.
Uso: aplicar_passagem_impossivel.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    ("""            if mapa is None or pontos is None:
                return [], 'sem mapa ou sem caminho'
            return planejar_etapas(mapa, pontos, (gx, gy), self.p('largura_estreita'),
                                   self.p('distancia_preparo'), self.p('distancia_saida'),
                                   ja_tratadas=tratadas,
                                   preparo_lateral_max=float(self.p('preparo_lateral_max')))  # PREPARO_LATERAL (19/11)
""",
     """            if mapa is None or pontos is None:
                return [], 'sem mapa ou sem caminho'
            etapas, info = planejar_etapas(mapa, pontos, (gx, gy), self.p('largura_estreita'),
                                           self.p('distancia_preparo'), self.p('distancia_saida'),
                                           ja_tratadas=tratadas,
                                           preparo_lateral_max=float(self.p('preparo_lateral_max')))  # PREPARO_LATERAL
            lmin = float(self.p('passagem_largura_minima'))   # PASSAGEM_IMPOSSIVEL (25/11)
            if not etapas or lmin <= 0 or info['largura_mapa'] >= lmin:
                return etapas, info
            if goal is None:
                return [], 'PASSAGEM_IMPOSSIVEL: largura %.2f m no mapa (vigia ignora)' % info['largura_mapa']
            for k in range(int(self.p('passagem_replanejar_max'))):
                self.get_logger().warn('PASSAGEM_IMPOSSIVEL %d: passagem de %.2f m no mapa em (%.2f, %.2f) (o robo tem '
                                       '0,65 m): mapa e costmap discordam; limpo os costmaps e replanejo' % (
                                           k + 1, info['largura_mapa'], info['entrada'][0], info['entrada'][1]))
                self._limpar_costmaps()
                time.sleep(1.5)
                if self._novo_goal_chegou():
                    return [], 'PASSAGEM_IMPOSSIVEL: goal novo'
                novo = self._caminho(goal)
                if not novo:
                    continue
                etapas, info = planejar_etapas(mapa, novo, (gx, gy), self.p('largura_estreita'),
                                               self.p('distancia_preparo'), self.p('distancia_saida'),
                                               ja_tratadas=tratadas,
                                               preparo_lateral_max=float(self.p('preparo_lateral_max')))
                if not etapas or info['largura_mapa'] >= lmin:
                    self.get_logger().info('PASSAGEM_IMPOSSIVEL: resolvido no replanejamento %d' % (k + 1))
                    return etapas, info
            self.get_logger().warn('PASSAGEM_IMPOSSIVEL: continua depois de %d replanejamentos: sem preparo/travessia, '
                                   'sigo com o Nav2' % int(self.p('passagem_replanejar_max')))
            return [], 'PASSAGEM_IMPOSSIVEL: sigo com o Nav2'
"""),
    ("""            self.declare_parameter('max_passagens', 4)
""",
     """            self.declare_parameter('max_passagens', 4)
            self.declare_parameter('passagem_largura_minima', 0.40)   # PASSAGEM_IMPOSSIVEL (25/11); 0 = como antes
            self.declare_parameter('passagem_replanejar_max', 5)      # PASSAGEM_IMPOSSIVEL
"""),
]


def conferir(velho, novo):
    assert novo.count('PASSAGEM_IMPOSSIVEL') >= 7
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('passagem_impossivel', 'PASSAGEM_IMPOSSIVEL', PADRAO, TROCAS, conferir, doc=__doc__)
