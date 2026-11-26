#!/usr/bin/env python3
"""ROTA_RETAS (25/11/2026): o alinhador leva o robo por RETAS nos eixos do mapa (frente/re e lateral), girando
parado so onde o giro cabe; o Nav2 fica de reserva.

Teste real na arena Fusion (25/11 18:00): o Nav2 fez uma curva ate a MESA1, derivou para a Shelf e nao voltou
ao FINISH. Ideia do operador: mecanum anda reto e de lado; so gira parado (ex.: Shelf = reto, lateral, gira 45).
Planejador: scripts/rota_retas.py (espaco de configuracao por rumo de 90 graus, giro com o circulo do corpo
livre, ate 4 retas, custo lateral 1,8x, re 1,3x). Execucao: _girar_seguro e _andar_reto (para com o LiDAR a
< 7 cm), corrigindo pelo AMCL ao fim de cada reta (ate 3 vezes). START/FINISH terminam ali; mesa segue o fluxo
de sempre (encosto). Sem rota ou reta falhou: segue como antes (Nav2).
Param 'rota_retas' (false = como antes). Log 'ROTA_RETAS:'.
Uso: aplicar_rota_retas.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    ("""            self.declare_parameter('caixa_lateral', True)       # CAIXA_LATERAL (25/11)
""",
     """            self.declare_parameter('caixa_lateral', True)       # CAIXA_LATERAL (25/11)
            self.declare_parameter('rota_retas', True)          # ROTA_RETAS (25/11): retas nos eixos; Nav2 de reserva
"""),
    ("""                    self._saida_caixa(area)   # SAIDA_CAIXA (25/11)
                    self.executar(goal, area)
""",
     """                    self._saida_caixa(area)   # SAIDA_CAIXA (25/11)
                    if (self._rota_retas(goal, area) == 'ok' and area is not None
                            and area in self.sem_encosto):   # ROTA_RETAS (25/11): ponto sem mesa: chegou
                        self._publicar_resultado(area, True, 'chegou (ROTA_RETAS)')
                    else:
                        self.executar(goal, area)
"""),
    ("""        def _boca_da_caixa(self, area, info, gx, gy):
""",
     """        def _rota_retas(self, goal, area):
            \"\"\"ROTA_RETAS (25/11): retas nos eixos do mapa + giro parado onde cabe. 'ok' | 'falha' | None.\"\"\"
            if not self.p('rota_retas'):
                return None
            import rota_retas as RR
            mapa, pm = self._mapa(), self._pose_mapa_yaw()
            if mapa is None or pm is None:
                return None
            gx, gy, gyaw = goal.pose.position.x, goal.pose.position.y, yaw_de(goal.pose.orientation)
            if math.hypot(gx - pm[0], gy - pm[1]) < 0.10:
                return None
            n = 'ROTA_RETAS %s' % (area or 'goal')
            try:
                t0 = time.monotonic()
                plano = RR.plano_retas(mapa, pm, (gx, gy, gyaw), pose_livre=pose_livre)
            except Exception as exc:
                self.get_logger().warn('ROTA_RETAS: planejador falhou (%s): sigo como antes' % exc)
                return None
            if not plano:
                self.get_logger().info('ROTA_RETAS: sem rota de retas ate %s: sigo como antes (Nav2)' % area)
                return None
            self.get_logger().info('ROTA_RETAS: %s (%.1f s): %s' % (area, time.monotonic() - t0, RR.descrever(plano)))
            for k, passo in enumerate(plano):
                if passo[0] == 'girar':
                    self._girar_seguro(passo[1], '%s passo %d' % (n, k + 1), tempo_max=30.0, movs_max=4)
                    continue
                x, y = passo[1], passo[2]
                for _ in range(3):
                    pm = self._pose_mapa_yaw()
                    if pm is None:
                        return 'falha'
                    dx, dy = x - pm[0], y - pm[1]
                    c, s_ = math.cos(pm[2]), math.sin(pm[2])
                    fx, ly = c * dx + s_ * dy, -s_ * dx + c * dy
                    if math.hypot(fx, ly) <= 0.04:
                        break
                    # o eixo maior primeiro (a reta), depois o resto
                    for eixo in sorted(('f', 'l'), key=lambda e: -abs(fx if e == 'f' else ly)):
                        v = fx if eixo == 'f' else ly
                        if abs(v) > 0.03:
                            d = ('frente' if v > 0 else 'tras') if eixo == 'f' else ('esq' if v > 0 else 'dir')
                            self._andar_reto(d, abs(v), '%s passo %d' % (n, k + 1), margem=0.0)
                pm = self._pose_mapa_yaw()
                erro = None if pm is None else math.hypot(x - pm[0], y - pm[1])
                if erro is None or erro > 0.10:
                    self.get_logger().warn('ROTA_RETAS: passo %d parou a %s de (%.2f, %.2f): sigo como antes (Nav2)' % (
                        k + 1, '?' if erro is None else '%.0f cm' % (erro * 100), x, y))
                    return 'falha'
            pm = self._pose_mapa_yaw()
            erro = None if pm is None else math.hypot(gx - pm[0], gy - pm[1])
            self.get_logger().info('ROTA_RETAS: %s: cheguei a %s do ponto' % (area, '?' if erro is None else '%.0f cm' % (erro * 100)))
            return 'ok' if erro is not None and erro <= 0.10 else 'falha'

        def _boca_da_caixa(self, area, info, gx, gy):
"""),
]


def conferir(velho, novo):
    assert novo.count('ROTA_RETAS') >= 8
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('rota_retas', 'ROTA_RETAS', PADRAO, TROCAS, conferir, doc=__doc__)
