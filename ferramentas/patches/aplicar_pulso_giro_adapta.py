#!/usr/bin/env python3
"""PULSO_GIRO_ADAPTA (13/11/2026) - alinhador, saida lateral (TravessiaLateral): pulso de correcao de rumo que nao gira.

Bag arena_start_20261113_112926, CURVA 1, 11:38:52-11:39:15 (23 s) e 11:45:35-11:46:12 (37 s): "Rumo saiu -5 graus:
corrigindo parado" -> "Paredes acabaram perto da saida" a cada ~3 s. Medido no bag (11:45:35-11:45:42): o pulso de
giro foi wz -0,020 por ~0,3 s e o yaw da IMU NAO mudou (-108,4 -> -107,4 -> -108,4 graus: ruido). O pulso e
calculado como erro/w_real - 0,06 com w_real 0,26 rad/s (teste de 26/10, giro com PWM 80); hoje minimum_rotation_pwm
e 50 (~0,15 rad/s) e o motor nem arranca em 0,3 s. O laco so terminava porque o deslize reseta o contador.

Agora (com o parametro ligado): depois de cada pulso de giro, o yaw da IMU e comparado com o pedido. Pulso que girou
< 35% do pedido -> o proximo pulso fica 1,6x mais longo (ate 3x e no maximo 1,2 s). Nao muda a VELOCIDADE do giro
(o mesmo wz de antes), so o tempo do pulso; o giro continua parando ao chegar (antecipacao de 2 graus).
Depois de 5 pulsos fracos seguidos, para as correcoes finas de rumo nessa saida (como quando acabam as correcoes:
acima de 7 graus a seguranca continua corrigindo e abortando). O DESLIZE lateral nao e tocado (GANHO_LATERAL e k_ff
iguais).
Parametro do alinhador: pulso_giro_adapta (padrao FALSE: 1 mudanca por teste fisico; vale na proxima saida lateral)
  ros2 param set /alinhador_passagem pulso_giro_adapta true
Uso: python3 aplicar_pulso_giro_adapta.py [--raiz <copia do workspace>] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_multi import rodar  # noqa: E402

MARCA = 'PULSO_GIRO_ADAPTA (13/11)'
AL = 'src/robo_navigation/scripts/alinhador_passagem.py'

TROCAS = [
    ("        self.recuo_curva = False   # RECUO_CURVA (12/11): ligado pelo no\n"
     "        self.n_recuo_curva = 0\n",
     "        self.recuo_curva = False   # RECUO_CURVA (12/11): ligado pelo no\n"
     "        self.n_recuo_curva = 0\n"
     "        self.pulso_adapta = False  # PULSO_GIRO_ADAPTA (13/11): ligado pelo no (parametro pulso_giro_adapta)\n"
     "        self.pulso_ganho = 1.0\n"
     "        self.pulsos_fracos = 0\n"
     "        self._giro_pend = None     # (yaw no inicio do pulso, giro pedido) ate medir depois do assentar\n"),
    ("    def _mudar(self, e, t, pose):\n"
     "        if self.estado == 'deslizar' and e != 'deslizar' and self.p_desl is not None:\n",
     "    def _avaliar_pulso(self, pose):\n"
     "        \"\"\"PULSO_GIRO_ADAPTA (13/11): o pulso de giro girou de verdade (yaw da IMU)? Se nao, alonga o proximo.\"\"\"\n"
     "        yaw0, pedido = self._giro_pend\n"
     "        self._giro_pend = None\n"
     "        if abs(pedido) < math.radians(1.5):\n"
     "            return\n"
     "        feito = _ang(pose[2] - yaw0)\n"
     "        if feito * pedido >= 0.35 * pedido * pedido:\n"
     "            self.pulsos_fracos = 0\n"
     "            return\n"
     "        self.pulsos_fracos += 1\n"
     "        if self.pulso_ganho < 3.0:\n"
     "            self.pulso_ganho = min(3.0, self.pulso_ganho * 1.6)\n"
     "            self.log('PULSO_GIRO_ADAPTA: o pulso girou %+.1f de %+.1f graus: proximo pulso %.1fx mais longo' % (\n"
     "                math.degrees(feito), math.degrees(pedido), self.pulso_ganho))\n"
     "        elif self.pulsos_fracos >= 5 and self.correcoes < self.correcoes_max:\n"
     "            self.correcoes = self.correcoes_max\n"
     "            self.log('PULSO_GIRO_ADAPTA: %d pulsos seguidos sem girar: paro as correcoes finas de rumo nesta '\n"
     "                     'saida (acima de 7 graus a seguranca continua)' % self.pulsos_fracos)\n"
     "\n"
     "    def _mudar(self, e, t, pose):\n"
     "        if self.pulso_adapta:   # PULSO_GIRO_ADAPTA (13/11)\n"
     "            if self.estado == 'girar' and e != 'girar' and self.inicio_estado is not None \\\n"
     "                    and self.alvo is not None:\n"
     "                self._giro_pend = (self.inicio_estado[2], _ang(self.alvo - self.inicio_estado[2]))\n"
     "            elif self._giro_pend is not None and self.estado != 'girar':\n"
     "                self._avaliar_pulso(pose)   # mede depois do assentar (o motor ainda embala no fim do pulso)\n"
     "            if e == 'girar':\n"
     "                self.pulso = min(1.2, self.pulso * self.pulso_ganho)\n"
     "        if self.estado == 'deslizar' and e != 'deslizar' and self.p_desl is not None:\n"),
    ("            try:   # RECUO_CURVA (12/11)\n"
     "                tl.recuo_curva = bool(self.p('recuo_curva'))\n"
     "            except Exception:\n"
     "                pass\n",
     "            try:   # RECUO_CURVA (12/11)\n"
     "                tl.recuo_curva = bool(self.p('recuo_curva'))\n"
     "            except Exception:\n"
     "                pass\n"
     "            try:   # PULSO_GIRO_ADAPTA (13/11)\n"
     "                tl.pulso_adapta = bool(self.p('pulso_giro_adapta'))\n"
     "            except Exception:\n"
     "                pass\n"),
    ("            self.declare_parameter('recuo_curva', True)   # RECUO_CURVA (12/11): false = como antes\n",
     "            self.declare_parameter('recuo_curva', True)   # RECUO_CURVA (12/11): false = como antes\n"
     "            # PULSO_GIRO_ADAPTA (13/11): pulso de giro que nao girou fica mais longo (padrao false: 1 mudanca/teste)\n"
     "            self.declare_parameter('pulso_giro_adapta', False)\n"),
]

if __name__ == '__main__':
    rodar('pulso_giro_adapta', MARCA, [(AL, TROCAS)], doc=__doc__)
