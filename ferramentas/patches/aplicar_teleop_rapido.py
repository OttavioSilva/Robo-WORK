#!/usr/bin/env python3
"""TELEOP_RAPIDO (24/11/2026): teleop mais rapido (pedido do usuario).

Normal: frente 0,10 -> 0,15 m/s; lado 0,08 -> 0,12; giro 0,30 -> 0,35 rad/s. RB (x2) continua.
O velocity_smoother do Nav2 corta em 0,20 / 0,16 / 0,40 (nav2_params, nao mudado): com RB o teto real
e esse. Para mapear, devagar (sem RB) continua melhor: a odom e pelo comando (sem encoder).
Uso: aplicar_teleop_rapido.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/ferramentas/teleop/teleop_mapeamento_joy.py')

TROCAS = [
    ("""    p.add_argument('--linear', type=float, default=0.10, help='m/s maximo frente/re (normal)')
    p.add_argument('--lateral', type=float, default=0.08, help='m/s maximo lateral (normal)')
    p.add_argument('--angular', type=float, default=0.30, help='rad/s maximo giro (normal)')
""",
     """    # TELEOP_RAPIDO (24/11): era 0.10 / 0.08 / 0.30 (teto do velocity_smoother: 0.20 / 0.16 / 0.40)
    p.add_argument('--linear', type=float, default=0.15, help='m/s maximo frente/re (normal)')
    p.add_argument('--lateral', type=float, default=0.12, help='m/s maximo lateral (normal)')
    p.add_argument('--angular', type=float, default=0.35, help='rad/s maximo giro (normal)')
"""),
]


def conferir(velho, novo):
    assert novo.count('TELEOP_RAPIDO') == 1
    compile(novo, 'teleop', 'exec')


if __name__ == '__main__':
    rodar('teleop_rapido', 'TELEOP_RAPIDO', PADRAO, TROCAS, conferir, doc=__doc__)
