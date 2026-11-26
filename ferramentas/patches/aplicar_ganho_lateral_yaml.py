#!/usr/bin/env python3
"""GANHO_LATERAL_YAML (12/11/2026) - hardware.yaml: valores medidos do GANHO_LATERAL (requer aplicar_ganho_lateral.py
na ponte). Teste 12/11, robo de frente para a parede: FL 1,6 / FR 0,6 / RL 0,7 / RR 1,6 (lateral puro).
Uso: python3 aplicar_ganho_lateral_yaml.py [--arquivo copia.yaml] [--desfazer]
O hardware.yaml instalado e link para o src: vale no proximo inicio da ponte."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_hardware/config/hardware.yaml')
MARCA = 'GANHO_LATERAL_YAML (12/11)'
NOVAS = {'lateral_wheel_gain_fl': 1.6, 'lateral_wheel_gain_fr': 0.6,
         'lateral_wheel_gain_rl': 0.7, 'lateral_wheel_gain_rr': 1.6}
TROCAS = [
    ('    heading_hold_imu_timeout_s: 0.20\n',
     '    heading_hold_imu_timeout_s: 0.20\n'
     '\n'
     '    # GANHO_LATERAL_YAML (12/11): ganho de PWM por roda so no movimento lateral (peso |vy|/(|vx|+|vy|)).\n'
     '    # Teste 12/11: com 1,0 o deslize saia na diagonal (dx/dy ~3); frente/tras e giro continuam com 1,0.\n'
     '    lateral_wheel_gain_fl: 1.6\n'
     '    lateral_wheel_gain_fr: 0.6\n'
     '    lateral_wheel_gain_rl: 0.7\n'
     '    lateral_wheel_gain_rr: 1.6\n'),
]


def conferir(velho, novo):
    import yaml
    a = yaml.safe_load(velho)['robo_hardware']['ros__parameters']
    b = yaml.safe_load(novo)['robo_hardware']['ros__parameters']
    if {k: v for k, v in b.items() if k not in NOVAS} != a or {k: b[k] for k in NOVAS} != NOVAS:
        sys.exit('ERRO: o yaml mudou alem das 4 chaves lateral_wheel_gain_*')


if __name__ == '__main__':
    rodar('ganho_lateral_yaml', MARCA, ARQ, TROCAS, conferir=conferir, doc=__doc__)
