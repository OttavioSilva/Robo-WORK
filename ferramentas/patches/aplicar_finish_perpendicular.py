#!/usr/bin/env python3
"""FINISH_PERPENDICULAR (25/11/2026, complemento do FINISH_CHEGADA): entra na caixa reto, de frente para ela.

Sim 25/11 02:49 (FINISH_CHEGADA + INFLACAO_SUAVE): vindo de leste a volta ficou limpa (s01 88 -> 76 s, s03 de
FALHOU para OK), mas vindo do norte (s02, da SH03) o angulo de chegada pelo caminho era diagonal (-135 graus) e
a quina do toco norte do START barrou o bico (frente 7-9 cm, 3 destravamentos, 55 -> 83 s). A caixa do
START/FINISH e funda so 0,6 m e a boca tem 1,09 m: so cabe entrar perpendicular. Agora (param
'finish_perpendicular', false = como antes) o FINISH usa o yaw do START + 180 graus (de frente para a caixa),
venha de onde vier; o goal checker continua sem cobrar angulo (GIRO_SO_ALINHADOR).
Uso: aplicar_finish_perpendicular.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""            self.declare_parameter('finish_chegada', True)     # FINISH_CHEGADA (25/11): sem mesa entra de frente
""",
     """            self.declare_parameter('finish_chegada', True)     # FINISH_CHEGADA (25/11): sem mesa entra de frente
            self.declare_parameter('finish_perpendicular', True)   # FINISH_PERPENDICULAR (25/11)
"""),
    ("""                    yaw = yaw_chegada(self._caminho(goal), (gx, gy), self._pose_mapa_yaw())
                    if yaw is not None:
                        self.get_logger().info('%s: angulo livre, usando o de chegada (%.0f graus)' % (
                            area, math.degrees(yaw)))
                        goal = self._pose(gx, gy, yaw)
""",
     """                    yaw = yaw_chegada(self._caminho(goal), (gx, gy), self._pose_mapa_yaw())
                    if str(area).startswith('FINISH') and self.p('finish_perpendicular'):   # FINISH_PERPENDICULAR
                        yaw = yaw_de(goal.pose.orientation) + math.pi
                        self.get_logger().info('FINISH_PERPENDICULAR: %s: entro de frente para a caixa (%.0f graus)' % (
                            area, math.degrees(math.atan2(math.sin(yaw), math.cos(yaw)))))
                    if yaw is not None:
                        self.get_logger().info('%s: angulo livre, usando o de chegada (%.0f graus)' % (
                            area, math.degrees(yaw)))
                        goal = self._pose(gx, gy, yaw)
"""),
]


def conferir(velho, novo):
    assert novo.count('FINISH_PERPENDICULAR') == 3
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('finish_perpendicular', 'FINISH_PERPENDICULAR', PADRAO, TROCAS, conferir, doc=__doc__)
