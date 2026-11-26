#!/usr/bin/env python3
"""CHEGADA_PERTO (13/11/2026) - alinhador: o Nav2 falhou JA no goal (perto de obstaculo): o alinhador termina.

Teste 13/11 13:18:34-13:19:03 (sem bag; logs do controller/bt/destravar): goal (2,19; 3,78) perto de um obstaculo.
O DWB chegou a (2,10; 3,76) — 9 cm do ponto, tolerancia 6 cm — e repetiu "Could not find a legal trajectory: No
valid trajectories out of 400" (toda trajetoria colidia com o obstaculo perto) -> "Controller patience exceeded" ->
destravar com caminho de 3 pontos (sem direcao) -> "sem alternativa segura" -> GOAL FINAL: FALHOU. O robo ficou
parado no proprio goal.
Agora: quando o Nav2 termina com falha (nao cancelamento) e o robo esta a <= chegada_perto_raio (0,15 m) do goal, o
alinhador faz o resto com as manobras dele, guardadas pelo LiDAR: _deslocar_ate (como no TRAJETO CURTO) e, se o
angulo pedido estiver a mais de 10 graus, _girar_seguro (varre o retangulo do CM). So declara SUCESSO dentro de
10 cm e 10 graus (a mesma tolerancia do TRAJETO CURTO); senao FALHOU como antes. O angulo final do goal e mantido.
Parametro do alinhador: chegada_perto (padrao true; false = como antes), chegada_perto_raio (0,15 m)
  ros2 param set /alinhador_passagem chegada_perto false
Uso: python3 aplicar_chegada_perto.py [--raiz <copia do workspace>] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_multi import rodar  # noqa: E402

MARCA = 'CHEGADA_PERTO (13/11)'
AL = 'src/robo_navigation/scripts/alinhador_passagem.py'

TROCAS = [
    ("            ok = r.status == GoalStatus.STATUS_SUCCEEDED\n"
     "            self.get_logger().info('%s: %s' % (nome, 'SUCESSO' if ok else 'FALHOU'))\n"
     "            return 'ok' if ok else 'falha'\n",
     "            ok = r.status == GoalStatus.STATUS_SUCCEEDED\n"
     "            # CHEGADA_PERTO (13/11): o Nav2 falhou ja no goal (DWB sem trajetoria perto de obstaculo)\n"
     "            if not ok and r.status != GoalStatus.STATUS_CANCELED and self.p('chegada_perto') \\\n"
     "                    and self._chegada_perto(alvo, nome):\n"
     "                return 'ok'\n"
     "            self.get_logger().info('%s: %s' % (nome, 'SUCESSO' if ok else 'FALHOU'))\n"
     "            return 'ok' if ok else 'falha'\n"),
    ("        def _publicar_vel(self, vx, vy, wz):\n",
     "        def _chegada_perto(self, alvo, nome):\n"
     "            \"\"\"CHEGADA_PERTO (13/11): termina os ultimos cm com manobras guardadas pelo LiDAR. True = chegou.\"\"\"\n"
     "            pm = self._pose_mapa_yaw()\n"
     "            if pm is None:\n"
     "                return False\n"
     "            gx, gy = alvo.pose.position.x, alvo.pose.position.y\n"
     "            gyaw = yaw_de(alvo.pose.orientation)\n"
     "            d = math.hypot(gx - pm[0], gy - pm[1])\n"
     "            if d > float(self.p('chegada_perto_raio')):\n"
     "                return False\n"
     "            n = nome + ' (CHEGADA_PERTO)'\n"
     "            self.get_logger().warn('%s: o Nav2 falhou a %.0f cm do ponto (%.0f graus do angulo): termino eu' % (\n"
     "                n, d * 100, math.degrees(abs(_ang(gyaw - pm[2])))))\n"
     "            self._esperar_sensores(n)\n"
     "            if d > 0.05:\n"
     "                self._deslocar_ate(gx, gy, n, tol=0.03, passo_max=0.20)\n"
     "            pm = self._pose_mapa_yaw()\n"
     "            if pm is None:\n"
     "                return False\n"
     "            if abs(_ang(gyaw - pm[2])) > math.radians(10.0):\n"
     "                self._girar_seguro(gyaw, n, tempo_max=20.0, movs_max=1)\n"
     "                pm = self._pose_mapa_yaw()\n"
     "                if pm is None:\n"
     "                    return False\n"
     "            d, e = math.hypot(gx - pm[0], gy - pm[1]), abs(_ang(gyaw - pm[2]))\n"
     "            if d <= 0.10 and e <= math.radians(10.0):\n"
     "                self.get_logger().info('%s: SUCESSO (%.0f cm e %.0f graus do goal)' % (n, d * 100, math.degrees(e)))\n"
     "                return True\n"
     "            self.get_logger().warn('%s: nao cheguei (%.0f cm e %.0f graus do goal)' % (n, d * 100, math.degrees(e)))\n"
     "            return False\n"
     "\n"
     "        def _publicar_vel(self, vx, vy, wz):\n"),
    ("            self.declare_parameter('raio_chegada', 0.10)\n",
     "            self.declare_parameter('raio_chegada', 0.10)\n"
     "            # CHEGADA_PERTO (13/11): Nav2 falhou a <= chegada_perto_raio do goal -> o alinhador termina\n"
     "            self.declare_parameter('chegada_perto', True)\n"
     "            self.declare_parameter('chegada_perto_raio', 0.15)\n"),
]

if __name__ == '__main__':
    rodar('chegada_perto', MARCA, [(AL, TROCAS)], doc=__doc__)
