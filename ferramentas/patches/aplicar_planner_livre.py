#!/usr/bin/env python3
"""PLANNER_LIVRE (25/11/2026): o planner pode usar a arena inteira; a rota do mapa so quando ele nao tem caminho.

Antes (ROTA_MAPA, missao23): nas 2 primeiras tentativas para o mesmo destino, se o caminho do planner fosse
> 1,3x a rota mais curta do mapa + 0,5 m, o alinhador TROCAVA pelo mais curto do mapa (contar com contornar o
obstaculo la). So na 3a aceitava o desvio. Pedido do usuario 25/11: com obstaculo na arena o robo tem que ter a
liberdade de planejar por toda a arena. Agora: caminho do planner existe -> fica com ele (log PLANNER_LIVRE);
rota do mapa so quando o planner nao tem caminho (ou o caminho dele cruza um bloqueio marcado, como antes).
Parametro 'planner_livre' (false = como antes). Uso: aplicar_planner_livre.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    ("""                if rota is not None and (pontos is None or _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5):
""",
     """                if rota is not None and pontos is not None and self.p('planner_livre') and \\
                        _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5:   # PLANNER_LIVRE (25/11)
                    self.get_logger().info('PLANNER_LIVRE: fico com o caminho do planner (%.1f m; o mais curto do mapa '
                                           'tem %.1f m): ele ve o que barra o curto' % (_comp_rota(pontos), _comp_rota(rota)))
                if rota is not None and (pontos is None or (not self.p('planner_livre') and   # PLANNER_LIVRE
                                                            _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5)):
"""),
    ("""            self.declare_parameter('rota_mapa', True)           # ROTA_MAPA
""",
     """            self.declare_parameter('rota_mapa', True)           # ROTA_MAPA
            self.declare_parameter('planner_livre', True)       # PLANNER_LIVRE (25/11)
"""),
]


def conferir(velho, novo):
    assert novo.count('PLANNER_LIVRE') >= 4
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('planner_livre', 'PLANNER_LIVRE', PADRAO, TROCAS, conferir, doc=__doc__)
