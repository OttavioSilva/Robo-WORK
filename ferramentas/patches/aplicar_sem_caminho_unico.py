#!/usr/bin/env python3
"""SEM_CAMINHO_UNICO (13/11/2026) - destravar: a espera "SEM CAMINHO: limpo costmaps, 2 s parado" vale 1 vez por lugar.

Bag arena_start_20261113_112926: 3 chamadas do destravar (11:41:19, 11:41:34, 11:43:44) repetiram "SEM CAMINHO:
limpei os costmaps; espero 2 s parado e planejo de novo (1/2, 2/2)" — ~6 s parado cada, e o planner continuou sem
caminho. A 2a chamada foi no mesmo lugar (6 cm) da 1a: a espera repetida nao traz informacao nova e come o tempo
que a manobra (ex.: ORIENTAR_CAMINHO) precisaria.
Agora: na 1a chamada num lugar, igual a antes (ate 2 esperas). Voltando ao mesmo lugar (Historico, raio 15 cm,
120 s), vai direto ao diagnostico e as manobras. O replanejamento inicial continua acontecendo sempre.
Em tempo de execucao: Padrao DESLIGADO (1 mudanca por teste fisico). Ligar: ros2 param set /destravar sem_caminho_unico true
Uso: python3 aplicar_sem_caminho_unico.py [--raiz <copia do workspace>] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_multi import rodar  # noqa: E402

MARCA = 'SEM_CAMINHO_UNICO (13/11)'
RA = 'src/robo_navigation/scripts/recuperacao_avancada.py'
NO = 'src/robo_navigation/scripts/destravar_node.py'

TROCAS_RA = [
    ("    re_max: float = 0.15\n",
     "    re_max: float = 0.15\n"
     "    sem_caminho_unico: bool = False    # SEM_CAMINHO_UNICO (13/11): espera de 2x2 s so na 1a chamada no lugar\n"),
    ("        tent = 0\n"
     "        while planner is False and tent < 2 and io.agora() < t_fim - 6.0:\n",
     "        tent = 0\n"
     "        # SEM_CAMINHO_UNICO (13/11): voltando ao mesmo lugar, a espera ja foi feita e nao trouxe caminho\n"
     "        tent_max = 0 if (cfg.sem_caminho_unico and retornos >= 1) else 2\n"
     "        if planner is False and tent_max == 0:\n"
     "            io.log('info', 'SEM CAMINHO de novo no mesmo lugar (%da chamada aqui): nao repito a espera de '\n"
     "                           '2 x 2 s; vou direto as manobras' % (retornos + 1))\n"
     "        while planner is False and tent < tent_max and io.agora() < t_fim - 6.0:\n"),
]

TROCAS_NO = [
    # Ancoras proprias: NAO tocar nas linhas de RE_LIBERADA/RE_LONGA (o --desfazer deles precisa achar o texto)
    ("            self.raios = None\n"
     "            self.odom = None\n",
     "            self.raios = None\n"
     "            self.declare_parameter('sem_caminho_unico', False)   # SEM_CAMINHO_UNICO (13/11): opt-in\n"
     "            self.odom = None\n"),
    ("            res = DriveOnHeading.Result()\n"
     "            try:\n",
     "            res = DriveOnHeading.Result()\n"
     "            cfg.sem_caminho_unico = bool(self.get_parameter('sem_caminho_unico').value)   # SEM_CAMINHO_UNICO (13/11)\n"
     "            try:\n"),
]

if __name__ == '__main__':
    rodar('sem_caminho_unico', MARCA, [(RA, TROCAS_RA), (NO, TROCAS_NO)], doc=__doc__)
