#!/usr/bin/env python3
"""CAIXA_LATERAL (25/11/2026): entrar na caixa do FINISH sem tratar a boca como corredor.

Sim da arena Fusion (caixa 0,67 x 0,69; robo 0,59 x 0,65): a analise de passagens via a boca da caixa (0,67 m)
como corredor; a travessia pelo LiDAR entra de FRENTE e aborta "passagem estreita demais (0,68 m)" (2x).
De lado o corpo tem 0,59 m: cabe com 4 cm de cada lado. Agora (param 'caixa_lateral', false = como antes):
  - goal START*/FINISH* (ponto sem mesa) com ENTRADA_TRANSLADA ligada: passagem cuja entrada fica a <= 1,0 m
    do ponto e ignorada (a boca e da caixa; a ENTRADA_TRANSLADA cuida);
  - ENTRADA_TRANSLADA: depois do giro, acerta primeiro o eixo CRUZADO (o menor deslocamento) fora da caixa e so
    depois faz o deslocamento longo; antes era sempre lateral primeiro (entrava de lado com x errado).
Log 'CAIXA_LATERAL:'.
Uso: aplicar_caixa_lateral.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    ("""            self.declare_parameter('saida_caixa', 0.70)""",
     """            self.declare_parameter('caixa_lateral', True)       # CAIXA_LATERAL (25/11)
            self.declare_parameter('saida_caixa', 0.70)"""),
    # loop principal e vigia: ignora a boca da caixa
    ("""                else:
                    etapas, info = self._analisar(gx, gy, tratadas, goal=goal)
                if etapas:
                    tratadas.append(info['entrada'])
""",
     """                else:
                    etapas, info = self._analisar(gx, gy, tratadas, goal=goal)
                if etapas and self._boca_da_caixa(area, info, gx, gy):   # CAIXA_LATERAL (25/11)
                    etapas, info = [], 'CAIXA_LATERAL: a passagem e a boca da caixa do %s' % area
                if etapas:
                    tratadas.append(info['entrada'])
"""),
    ("""                etapas, info = self._analisar(gx, gy, tratadas, pontos=pontos)
                if etapas:
                    alerta['etapas'], alerta['info'] = etapas, info
""",
     """                etapas, info = self._analisar(gx, gy, tratadas, pontos=pontos)
                if etapas and self._boca_da_caixa(area, info, gx, gy):   # CAIXA_LATERAL (25/11)
                    return False
                if etapas:
                    alerta['etapas'], alerta['info'] = etapas, info
"""),
    # eixo cruzado primeiro
    ("""            self._girar_seguro(gyaw, n, tempo_max=30.0, movs_max=4)
            d = None
            for _ in range(4):
""",
     """            self._girar_seguro(gyaw, n, tempo_max=30.0, movs_max=4)
            if self.p('caixa_lateral'):   # CAIXA_LATERAL (25/11): eixo cruzado primeiro, ainda fora da caixa
                pm = self._pose_mapa_yaw()
                if pm is not None:
                    dx, dy = gx - pm[0], gy - pm[1]
                    c, s_ = math.cos(pm[2]), math.sin(pm[2])
                    fx, ly = c * dx + s_ * dy, -s_ * dx + c * dy
                    if abs(ly) > abs(fx) and abs(fx) > 0.03:
                        self.get_logger().info('CAIXA_LATERAL: %s: acerto %+.0f cm de frente antes de entrar de lado '
                                               '(%.0f cm)' % (area, fx * 100, abs(ly) * 100))
                        self._andar_reto('frente' if fx > 0 else 'tras', min(abs(fx), 0.5), n)
            d = None
            for _ in range(4):
"""),
    ("""        def _saida_caixa(self, area):
""",
     """        def _boca_da_caixa(self, area, info, gx, gy):
            \"\"\"CAIXA_LATERAL (25/11): passagem cuja entrada e a boca da caixa do START/FINISH.\"\"\"
            if not (self.p('caixa_lateral') and self.p('entrada_translada') and area in self.sem_encosto
                    and str(area).startswith(('START', 'FINISH')) and isinstance(info, dict)):
                return False
            ex, ey = info['entrada'][0], info['entrada'][1]
            if math.hypot(ex - gx, ey - gy) > 1.0:
                return False
            self.get_logger().info('CAIXA_LATERAL: passagem em (%.2f, %.2f) e a boca da caixa do %s: '
                                   'entro pela ENTRADA_TRANSLADA' % (ex, ey, area))
            return True

        def _saida_caixa(self, area):
"""),
]


def conferir(velho, novo):
    assert novo.count('CAIXA_LATERAL') >= 8
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('caixa_lateral', 'CAIXA_LATERAL', PADRAO, TROCAS, conferir, doc=__doc__)
