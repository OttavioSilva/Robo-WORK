#!/usr/bin/env python3
"""VIRAR_REPLANEJA (13/11/2026) - alinhador: o pre-giro (VIRAR_CAMINHO) tambem acontece quando o planner falha.

Teste 13/11 13:17:04 (sem bag; logs do no): goal (2,06; 1,94) atras do robo, em (2,13; 4,92) virado para +90. O
alinhador consultou o planner: "Planner: sem caminho; sigo a rota do mapa" -> GOAL FINAL direto no Nav2 SEM o
pre-giro (o VIRAR_CAMINHO so gira com caminho do planner). O DWB (sem re, sem giro no lugar) oscilou e caiu no
destravar, que girou 8 graus, tentou orientar e falhou. Logo depois (13:18:25), com caminho do planner, o mesmo
VIRAR_CAMINHO girou +157 graus e deu SUCESSO: quem deve fazer o giro grande e o alinhador, ANTES do Nav2.
Agora, no VIRAR_CAMINHO, sem caminho do planner:
  1. limpa os costmaps, espera 1,5 s e pede o caminho de novo (o destravar achou caminho assim as 13:17:32);
  2. ainda sem caminho: usa a rota do MAPA so se o 1o metro dela estiver livre no costmap ATUAL (nenhum objeto do
     momento a < 35 cm, obstaculo_novo_na_rota). Assim a direcao do giro e validada por sensor (skill: nada de
     pre-giro por rota estatica NAO validada). Sem costmap de obstaculos, nao vira.
O giro continua sendo o _girar_seguro de sempre (varre o retangulo do CM, anda o minimo se a quina pega).
Parametro do alinhador: virar_replaneja (padrao true; false = como antes)
  ros2 param set /alinhador_passagem virar_replaneja false
Uso: python3 aplicar_virar_replaneja.py [--raiz <copia do workspace>] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_multi import rodar  # noqa: E402

MARCA = 'VIRAR_REPLANEJA (13/11)'
AL = 'src/robo_navigation/scripts/alinhador_passagem.py'

TROCAS = [
    ("            caminho = self._caminho(alvo)\n"
     "            if not caminho or len(caminho) < 3:\n"
     "                return\n",
     "            caminho = self._caminho(alvo)\n"
     "            if (not caminho or len(caminho) < 3) and self.p('virar_replaneja'):   # VIRAR_REPLANEJA (13/11)\n"
     "                caminho = self._caminho_validado(alvo, nome)\n"
     "            if not caminho or len(caminho) < 3:\n"
     "                return\n"),
    ("        def _chamar_destravar(self, nome):\n",
     "        def _caminho_validado(self, alvo, nome):\n"
     "            \"\"\"VIRAR_REPLANEJA (13/11): planner sem caminho -> limpa costmaps e replaneja 1 vez; ainda sem caminho ->\n"
     "            rota do mapa SO se o 1o metro estiver livre no costmap de obstaculos atual. None = nao vira.\"\"\"\n"
     "            self._limpar_costmaps()\n"
     "            time.sleep(1.5)\n"
     "            c = self._caminho(alvo)\n"
     "            if c and len(c) >= 3:\n"
     "                self.get_logger().info('%s: VIRAR_REPLANEJA: planner estava sem caminho; limpei os costmaps e ele '\n"
     "                                       'achou (%d pontos)' % (nome, len(c)))\n"
     "                return c\n"
     "            mapa, pm, obs = self._mapa(), self._pose_mapa(), self._obs_global()\n"
     "            if mapa is None or pm is None or obs is None:\n"
     "                self.get_logger().warn('%s: VIRAR_REPLANEJA: planner sem caminho e sem mapa/costmap para validar a '\n"
     "                                       'rota: nao viro' % nome)\n"
     "                return None\n"
     "            rota = rota_no_mapa(mapa, pm, (alvo.pose.position.x, alvo.pose.position.y))\n"
     "            if not rota or len(rota) < 3:\n"
     "                self.get_logger().warn('%s: VIRAR_REPLANEJA: planner e mapa sem rota: nao viro' % nome)\n"
     "                return None\n"
     "            ini, comp = [rota[0]], 0.0\n"
     "            for a, b in zip(rota, rota[1:]):\n"
     "                comp += math.hypot(b[0] - a[0], b[1] - a[1])\n"
     "                ini.append(b)\n"
     "                if comp >= 1.0:\n"
     "                    break\n"
     "            novos = obstaculo_novo_na_rota(mapa, obs, ini)\n"
     "            if novos:\n"
     "                self.get_logger().warn('%s: VIRAR_REPLANEJA: rota do mapa com objeto do momento a < 35 cm no 1o metro '\n"
     "                                       '(%.2f, %.2f): nao viro por ela' % (nome, novos[0][0], novos[0][1]))\n"
     "                return None\n"
     "            self.get_logger().info('%s: VIRAR_REPLANEJA: planner continua sem caminho; rota do mapa com o 1o metro '\n"
     "                                   'livre no costmap atual: viro por ela' % nome)\n"
     "            return rota\n"
     "\n"
     "        def _chamar_destravar(self, nome):\n"),
    # ancora propria (a linha do virar_caminho_graus e do PRE_GIRO_UNICO: o --desfazer dele precisa dela intacta)
    ("            self.declare_parameter('planner_id', 'GridBased')\n",
     "            self.declare_parameter('planner_id', 'GridBased')\n"
     "            # VIRAR_REPLANEJA (13/11): pre-giro tambem sem caminho do planner (replaneja / rota do mapa validada)\n"
     "            self.declare_parameter('virar_replaneja', True)\n"),
]

if __name__ == '__main__':
    rodar('virar_replaneja', MARCA, [(AL, TROCAS)], doc=__doc__)
