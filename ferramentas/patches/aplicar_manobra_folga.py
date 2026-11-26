#!/usr/bin/env python3
"""MANOBRA_FOLGA (12/11/2026) - alinhador, escolher_manobra (giro com a quina pegando: anda antes de girar).

Bag corredor_re_freio_curva_20261112_113352 11:44:31: "girando 56 graus a quina pegaria: primeiro 10 cm para tras",
depois "primeiro 4 cm para tras" (andou 3): ele anda a MENOR distancia que libera o giro e o robo anda menos que o
pedido, entao repete em passos curtos. O usuario pediu re mais longa. Agora anda a menor distancia + manobra_margem
(0,06 m padrao), limitada pela folga real da direcao (mesma folga minima de antes). manobra_margem 0 = como antes.
Uso: python3 aplicar_manobra_folga.py [--arquivo copia.py] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("def escolher_manobra(pts, ang, passos=(0.04, 0.07, 0.10, 0.14, 0.20, 0.28, 0.36)):\n",
     "def escolher_manobra(pts, ang, passos=(0.04, 0.07, 0.10, 0.14, 0.20, 0.28, 0.36), margem=0.0):\n"),
    ("    if melhor is not None:\n"
     "        return ('mover', melhor[1], melhor[2])\n",
     "    if melhor is not None:\n"
     "        # MANOBRA_FOLGA (12/11): anda um pouco alem do minimo (o robo anda menos que o pedido), se houver folga\n"
     "        d = melhor[2]\n"
     "        if margem > 0.0:\n"
     "            d = max(d, min(d + margem, folgas[melhor[1]] - FOLGA_MOV))\n"
     "        return ('mover', melhor[1], d)\n"),
    ("                    m = escolher_manobra(sc[1], ang) if (abs(ang) > math.radians(10.0) or travou) else ('girar', ang)\n",
     "                    try:   # MANOBRA_FOLGA (12/11)\n"
     "                        margem_m = max(0.0, min(0.15, float(self.p('manobra_margem'))))\n"
     "                    except Exception:\n"
     "                        margem_m = 0.0\n"
     "                    m = escolher_manobra(sc[1], ang, margem=margem_m) if (abs(ang) > math.radians(10.0) or travou) \\\n"
     "                        else ('girar', ang)\n"),
    ("            self.declare_parameter('uma_parede', True)          # UMA_PAREDE\n",
     "            self.declare_parameter('uma_parede', True)          # UMA_PAREDE\n"
     "            self.declare_parameter('manobra_margem', 0.06)   # MANOBRA_FOLGA (12/11): 0 = como antes\n"),
]
if __name__ == '__main__':
    rodar('manobra_folga', 'MANOBRA_FOLGA (12/11)', ARQ, TROCAS, doc=__doc__)
