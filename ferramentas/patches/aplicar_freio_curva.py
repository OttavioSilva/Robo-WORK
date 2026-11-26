#!/usr/bin/env python3
"""FREIO_CURVA (12/11/2026) - alinhador, classe Travessia (so com curva planejada no fim: parada_lidar).

Teste no corredor 12/11 10:43-10:48 (log ~/logs/12-11_103941/nav2.log): corredor reto com curva para a esquerda
no fim; objeto logo apos a curva, na continuacao do reto (marcado em (2,24; -0,11)). O alinhador planejou certo
("curva para a ESQUERDA: modo lateral"), mas:
  10:44:02 - "Collision Monitor bloqueando dentro da passagem" -> abortou, re de 1,91 m;
  10:46:10 - "algo a 10 cm do para-choque" -> abortou, re de 1,95 m, PASSAGEM_BLOQUEADA, rota "alternativa"
             pelo mesmo cruzamento, preparo a 3 cm do objeto, destravar sem saida.
O CM para a frente com algo a 11 cm do para-choque (poligono forward_rapido 0,485 m) e o FREIO_FRENTE em 10 cm:
o robo nao chega mais perto do objeto que isso. O FREIO_CRUZAMENTO (09/11) so aceitava parar e seguir a saida com
o robo JA ALEM do ponto do cruzamento (restante <= 0); aqui ele parou ~5-6 cm ANTES do ponto, com o corpo ja
dentro da faixa do corredor novo (mapa: centro 1,74..1,85 cabe; parado a 10-11 cm do objeto: ~1,76).

O que muda: com curva planejada, se o FREIO_FRENTE (< 10 cm) ou o Collision Monitor (bloqueio) parar a travessia
com restante <= tolerancia, a travessia termina ali e segue para a saida lateral (que tem as protecoes dela:
folgas, PARADA DE SEGURANCA, CM). Tolerancia automatica = (largura do corredor novo + 0,06 [paredes do mapa ~6 cm
mais grossas] - 0,59 [corpo]) / 2, limitada a 0,03..0,10 m. Parametro freio_curva_tol: -1 automatico (padrao),
0 = desligado (como antes), > 0 = valor fixo em m.
Nao muda: travessia sem curva, aborto com restante maior que a tolerancia, FREIO_CRUZAMENTO (restante <= 0),
parada pela parede do T, PAREDE_MAPA, saida lateral, CM.

Uso: python3 aplicar_freio_curva.py [--arquivo copia.py] [--desfazer]
Testes: src/robo_navigation/test/test_freio_curva.py"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
MARCA = 'FREIO_CURVA (12/11)'

TROCAS = [
    # 1) atributos
    ("        self.f_corpo = None           # FREIO_FRENTE: (t, folga a frente na largura do corpo)\n",
     "        self.f_corpo = None           # FREIO_FRENTE: (t, folga a frente na largura do corpo)\n"
     "        # FREIO_CURVA (12/11): -1 automatico, 0 desligado, > 0 tolerancia fixa (m) antes do ponto do cruzamento\n"
     "        self.freio_curva_tol = -1.0\n"
     "        self.ult_restante = None\n"),
    # 2) guarda o restante
    ("        if t - self.t0 > self.tempo_max:\n"
     "            self.abortar('tempo maximo')\n"
     "            return 0.0, 0.0, 0.0\n"
     "        e = self.estado\n\n"
     "        if e == 'aproximar':\n",
     "        if t - self.t0 > self.tempo_max:\n"
     "            self.abortar('tempo maximo')\n"
     "            return 0.0, 0.0, 0.0\n"
     "        e = self.estado\n"
     "        self.ult_restante = restante   # FREIO_CURVA\n\n"
     "        if e == 'aproximar':\n"),
    # 3) tolerancia
    ("    def gap_alvo(self):\n",
     "    def tol_curva(self):\n"
     "        \"\"\"FREIO_CURVA (12/11): quanto antes do ponto do cruzamento o corpo ja cabe no corredor novo (m), ou None.\"\"\"\n"
     "        if not self.parada_lidar or self.freio_curva_tol == 0.0:\n"
     "            return None\n"
     "        if self.freio_curva_tol > 0.0:\n"
     "            return min(0.15, self.freio_curva_tol)\n"
     "        larg = self.largura_b or self.largura or self.largura_padrao\n"
     "        return min(0.10, max(0.03, (larg + 0.06 - 0.59) / 2.0))\n\n"
     "    def gap_alvo(self):\n"),
    # 4) FREIO_FRENTE com tolerancia
    ("                if self.parada_lidar and restante <= 0.0:\n"
     "                    self.estado = 'fim'\n",
     "                tol = self.tol_curva()   # FREIO_CURVA (12/11)\n"
     "                if tol is not None and 0.0 < restante <= tol:\n"
     "                    self.estado = 'fim'\n"
     "                    self.log('FREIO_CURVA: algo a %.0f cm do para-choque, %.0f cm antes do ponto do cruzamento '\n"
     "                             '(tolerancia %.0f cm: o corpo ja cabe no corredor novo): paro aqui e sigo a saida' % (\n"
     "                                 fc[1] * 100, restante * 100, tol * 100))\n"
     "                    return 0.0, 0.0, 0.0\n"
     "                if self.parada_lidar and restante <= 0.0:\n"
     "                    self.estado = 'fim'\n"),
    # 5) CM segurando perto do ponto
    ("        if self.estado == 'atravessar':\n"
     "            # dentro da passagem, seguir empurrando so piora (pivota na quina):\n",
     "        tol = self.tol_curva() if self.estado == 'atravessar' else None   # FREIO_CURVA (12/11)\n"
     "        if tol is not None and self.ult_restante is not None and self.ult_restante <= tol:\n"
     "            self.estado = 'fim'\n"
     "            self.log('FREIO_CURVA: Collision Monitor segurando %.0f cm %s do ponto do cruzamento (tolerancia %.0f cm): '\n"
     "                     'paro aqui e sigo a saida' % (abs(self.ult_restante) * 100,\n"
     "                                                  'antes' if self.ult_restante > 0 else 'alem', tol * 100))\n"
     "            return\n"
     "        if self.estado == 'atravessar':\n"
     "            # dentro da passagem, seguir empurrando so piora (pivota na quina):\n"),
    # 6) parametro do no
    ("            self.declare_parameter('fundido_k_ff', 0.2)\n",
     "            self.declare_parameter('fundido_k_ff', 0.2)\n"
     "            self.declare_parameter('freio_curva_tol', -1.0)   # FREIO_CURVA (12/11): -1 auto, 0 desligado\n"),
    # 7) passa para a Travessia
    ("            if largura_b:\n"
     "                trav.largura_b = largura_b\n",
     "            if largura_b:\n"
     "                trav.largura_b = largura_b\n"
     "            try:   # FREIO_CURVA (12/11)\n"
     "                trav.freio_curva_tol = float(self.p('freio_curva_tol'))\n"
     "            except Exception:\n"
     "                pass\n"),
]

if __name__ == '__main__':
    rodar('freio_curva', MARCA, ARQ, TROCAS, doc=__doc__)
