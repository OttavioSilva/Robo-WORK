#!/usr/bin/env python3
"""PARTIDA_LATERAL (12/11/2026) - alinhador, so a saida lateral com ANGULO_FUNDIDO (TravessiaLateral._deslizar_fundido).

Missao34, CURVA 1 (11:03:20-11:04:08, saida para a ESQUERDA): 10 deslizes, mediana de 4,6 cm de AVANCO real por
deslize (paredes da frente e de tras pelo LiDAR concordam), com vx comandado sempre NEGATIVO (-0,016 a -0,035).
O avanco e deriva mecanica, nao comando. Cada deslize durava 0,5-1,3 s ate a folga da frente cair a 4 cm
(PARADA DE SEGURANCA), recuar 7 cm, medir e repetir; abortou com "Collision Monitor bloqueou 4 vezes".
Partes do avanco (mediana): rampa 1,4 cm (primeiros ~0,35 s), regime 2,1 cm, inercia depois do zero 0,4 cm.

Na rampa o feedforward nao chega as rodas: o velocity_smoother (scale_velocities, deadband 0,015) escala vx
junto com vy e zera |vx| < 0,015; com k_ff 0,2 o vx so aparece quando vy ja passou de ~0,06 (0,15-0,3 s depois
da 1a roda girar). A ponte e o firmware nao sao a causa: 1o quadro simetrico (-79, 79, 79, -79), o firmware
parte direto no PWM pedido e a rampa dele (200 PWM/s) nunca limita (a ponte sobe ~100 PWM/s).

O que muda (tudo DESLIGADO por padrao = comportamento da missao24/34 identico):
  - fundido_k_ff_esq / fundido_k_ff_dir (padrao -1 = usa fundido_k_ff, como hoje): k do feedforward por lado;
    so ligar depois da calibracao fisica (o modelo simetrico vx = -k*vy nao precisa servir para os dois lados);
  - partida_lateral_s (padrao 0 = desligado; sugerido 0,5 quando for testar): nos primeiros N s de cada deslize
    o alinhador segura vy em partida_lateral_v0 (0,03) por partida_lateral_espera (0,15 s, o smoother parte de 0 e
    alcanca 0,03 com o vx junto) e depois sobe ate vel_lateral a partida_lateral_acel (0,16 m/s2, abaixo dos 0,2 do smoother) e mantem o vx do regime. Assim a razao vx/vy na partida e maior e o
    vx passa do deadband ja no 3o passo do smoother (com k 0,2: antes so no 7o). Limite: |vx| <= |vy| - 0,012
    durante a partida, para nenhuma roda do par mais fraco ficar abaixo do PWM minimo (>= ~76).
Nao muda: parada de seguranca, Collision Monitor, correcoes paradas, giro, abortos, preparo, travessia, encosto,
destravar, Nav2, ponte (C++) e firmware. A parada continua imediata (o patch so mexe no comando de movimento).

Uso: python3 aplicar_partida_lateral.py [--arquivo copia.py] [--desfazer]
Testes: src/robo_navigation/test/test_partida_lateral.py"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
MARCA = 'PARTIDA_LATERAL (12/11)'

TROCAS = [
    # 1) construtor: parametros novos, desligados por padrao
    ("                 progresso_reset=0.06, seguranca_max=6, log=print, fundido=False, k_ff=0.2):\n"
     "        self.fundido, self.k_ff = fundido, k_ff   # ANGULO_FUNDIDO\n",
     "                 progresso_reset=0.06, seguranca_max=6, log=print, fundido=False, k_ff=0.2,\n"
     "                 k_ff_esq=-1.0, k_ff_dir=-1.0, partida_s=0.0, partida_v0=0.03, partida_acel=0.16,\n"
     "                 partida_espera=0.15):\n"
     "        self.fundido, self.k_ff = fundido, k_ff   # ANGULO_FUNDIDO\n"
     "        # PARTIDA_LATERAL (12/11): k do feedforward por lado (< 0 = usa k_ff, como antes) e rampa propria na\n"
     "        # partida com o vx do regime desde o inicio (partida_s 0 = como antes). Desligados ate calibrar.\n"
     "        k_lado = k_ff_esq if lado > 0 else k_ff_dir\n"
     "        self.k_ff_lado = k_lado if k_lado is not None and k_lado >= 0.0 else k_ff\n"
     "        self.partida_s, self.partida_v0, self.partida_acel = max(0.0, partida_s), partida_v0, partida_acel\n"
     "        self.partida_espera = partida_espera   # s em v0: o smoother (parte de 0) alcanca v0 com o vx junto\n"
     "        self.partida_folga_roda = 0.012   # |vy| - |vx| minimo na partida: roda do par fraco >= ~PWM 76\n"),
    # 2) feedforward pelo k do lado (igual a k_ff quando desligado)
    ("        vx = -self.k_ff * vy   # deslize torto medido",
     "        vx = -self.k_ff_lado * vy   # PARTIDA_LATERAL: k do lado (= k_ff se desligado). Deslize torto medido"),
    # 3) partida: rampa de vy propria com o vx do regime
    ("            vx = math.copysign(0.016, vx)   # o velocity_smoother zera abaixo de 0,015\n"
     "        return vx, vy, 0.0\n",
     "            vx = math.copysign(0.016, vx)   # o velocity_smoother zera abaixo de 0,015\n"
     "        if self.partida_s > 0.0 and t - self.t_estado < self.partida_s:\n"
     "            # PARTIDA_LATERAL (12/11): o smoother escala vx com vy e zera |vx| < 0,015; com a rampa aqui o vx\n"
     "            # do regime chega as rodas no 3o passo. Nenhuma roda abaixo do PWM minimo: |vx| <= |vy| - folga.\n"
     "            v = min(self.v_lat, self.partida_v0 + self.partida_acel * max(0.0, t - self.t_estado - self.partida_espera))\n"
     "            vy = self.lado * v\n"
     "            lim = v - self.partida_folga_roda\n"
     "            if abs(vx) > lim:\n"
     "                vx = math.copysign(lim, vx) if lim >= 0.016 else 0.0\n"
     "        return vx, vy, 0.0\n"),
    # 4) parametros do no (desligados)
    ("            self.declare_parameter('fundido_k_ff', 0.2)\n",
     "            self.declare_parameter('fundido_k_ff', 0.2)\n"
     "            # PARTIDA_LATERAL (12/11): desligados = como a missao24; ligar so depois da calibracao fisica\n"
     "            self.declare_parameter('fundido_k_ff_esq', -1.0)   # < 0 = usa fundido_k_ff\n"
     "            self.declare_parameter('fundido_k_ff_dir', -1.0)\n"
     "            self.declare_parameter('partida_lateral_s', 0.0)   # 0 = desligado (sugerido 0,5 para testar)\n"
     "            self.declare_parameter('partida_lateral_v0', 0.03)\n"
     "            self.declare_parameter('partida_lateral_acel', 0.16)\n"
     "            self.declare_parameter('partida_lateral_espera', 0.15)\n"),
    # 5) passa para a TravessiaLateral (parametro ausente = padrao desligado)
    ("            tl = TravessiaLateral(curva['lado'], fundido=self.p('angulo_fundido'), k_ff=self.p('fundido_k_ff'),\n",
     "            extra = {}   # PARTIDA_LATERAL (12/11): parametro ausente = padrao desligado\n"
     "            for chave, nome in (('k_ff_esq', 'fundido_k_ff_esq'), ('k_ff_dir', 'fundido_k_ff_dir'),\n"
     "                                ('partida_s', 'partida_lateral_s'), ('partida_v0', 'partida_lateral_v0'),\n"
     "                                ('partida_acel', 'partida_lateral_acel'), ('partida_espera', 'partida_lateral_espera')):\n"
     "                try:\n"
     "                    extra[chave] = float(self.p(nome))\n"
     "                except Exception:\n"
     "                    pass\n"
     "            tl = TravessiaLateral(curva['lado'], fundido=self.p('angulo_fundido'), k_ff=self.p('fundido_k_ff'),\n"
     "                                  **extra,\n"),
]


def conferir(velho, novo):
    """So a TravessiaLateral, o bloco de parametros e a chamada em _travessia_lateral mudam."""
    import difflib
    mud = [l for l in difflib.unified_diff(velho.splitlines(), novo.splitlines(), lineterm='', n=0)
           if l[:1] in '+-' and not l.startswith(('+++', '---'))]
    removidas = [l for l in mud if l.startswith('-')]
    esperadas = {
        "-                 progresso_reset=0.06, seguranca_max=6, log=print, fundido=False, k_ff=0.2):",
        "-        vx = -self.k_ff * vy   # deslize torto medido: k 0,05-0,37, para a frente indo para a esquerda",
    }
    if set(removidas) != esperadas:
        sys.exit('ERRO: linhas removidas inesperadas: %r' % removidas)


if __name__ == '__main__':
    rodar('partida_lateral', MARCA, ARQ, TROCAS, conferir=conferir, doc=__doc__)
