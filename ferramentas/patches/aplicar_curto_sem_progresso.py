#!/usr/bin/env python3
"""CURTO_SEM_PROGRESSO (13/11/2026) - alinhador, TRAJETO CURTO: nao repetir tentativa que nao aproximou.

Bag arena_start_20261113_112926, 11:46:56-11:47:13: o TRAJETO CURTO fez 4 tentativas de _deslocar_ate; as duas
ultimas foram "andei 0 cm para dir / andei 0 cm para frente" (frente barrada a ~4 cm) e so depois disse "fiquei a
31 cm do ponto: sigo pelo Nav2". ~9 s repetindo o que ja nao andava.
Agora (com o parametro ligado): da 2a tentativa em diante, se a distancia ao ponto nao caiu pelo menos 2 cm na
tentativa, para de repetir e segue exatamente como antes (Nav2 se ficou a mais de 10 cm). Nada mais muda.
Parametro do alinhador: curto_sem_progresso (padrao FALSE: 1 mudanca por teste fisico)
  ros2 param set /alinhador_passagem curto_sem_progresso true
Uso: python3 aplicar_curto_sem_progresso.py [--raiz <copia do workspace>] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_multi import rodar  # noqa: E402

MARCA = 'CURTO_SEM_PROGRESSO (13/11)'
AL = 'src/robo_navigation/scripts/alinhador_passagem.py'

TROCAS = [
    ("            err = 9.0\n"
     "            for _ in range(4):\n"
     "                self._deslocar_ate(gx, gy, nome, tol=0.03, passo_max=1.0)\n"
     "                pm = self._pose_mapa_yaw()\n"
     "                if pm is None:\n"
     "                    return None\n"
     "                err = math.hypot(gx - pm[0], gy - pm[1])\n"
     "                if err <= 0.05:\n"
     "                    break\n",
     "            err = 9.0\n"
     "            for _ in range(4):\n"
     "                ant = err   # CURTO_SEM_PROGRESSO (13/11)\n"
     "                self._deslocar_ate(gx, gy, nome, tol=0.03, passo_max=1.0)\n"
     "                pm = self._pose_mapa_yaw()\n"
     "                if pm is None:\n"
     "                    return None\n"
     "                err = math.hypot(gx - pm[0], gy - pm[1])\n"
     "                if err <= 0.05:\n"
     "                    break\n"
     "                if ant < 9.0 and ant - err < 0.02 and self.p('curto_sem_progresso'):\n"
     "                    self.get_logger().warn('%s: CURTO_SEM_PROGRESSO: a tentativa nao aproximou (%.0f -> %.0f cm): '\n"
     "                                           'paro de repetir' % (nome, ant * 100, err * 100))\n"
     "                    break\n"),
    ("            self.declare_parameter('curto_max', 1.0)           # v3.23: trajeto curto sem Nav2 (m; 0 desliga)\n",
     "            self.declare_parameter('curto_max', 1.0)           # v3.23: trajeto curto sem Nav2 (m; 0 desliga)\n"
     "            # CURTO_SEM_PROGRESSO (13/11): tentativa que nao aproximou 2 cm nao se repete (padrao false: 1 mudanca/teste)\n"
     "            self.declare_parameter('curto_sem_progresso', False)\n"),
]

if __name__ == '__main__':
    rodar('curto_sem_progresso', MARCA, [(AL, TROCAS)], doc=__doc__)
