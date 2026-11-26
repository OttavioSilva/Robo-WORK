#!/usr/bin/env python3
"""GANHO_LATERAL (12/11/2026) - ponte (hardware_bridge_node.cpp): ganho de PWM por roda SO no movimento lateral.

Teste 12/11 (robo de frente para a parede, ~/testes_k_lateral/): com ganhos 1,0 o deslize lateral sai na diagonal
(dx/dy +3 a +3,7 para a esquerda, -1 a -4 para a direita). Com FL 1,6 / FR 0,6 / RL 0,7 / RR 1,6 o deslize ficou
em dx/dy +0,05 a +1,07 (pulsos ~+0,25), mas o mesmo ganho FIXO entortou o andar para a frente (2,7-3,1 cm de
deriva lateral a cada 10 cm, contra 0,2-0,6). Por isso o ganho entra so na proporcao do movimento lateral:

  peso = |vy| / (|vx| + |vy|) do comando (0 parado, so frente/tras ou so giro; 1 lateral puro)
  ganho efetivo da roda = 1 + peso * (lateral_wheel_gain_<roda> - 1)
  |pwm| * ganho efetivo, limitado a [PWM minimo efetivo, maximum_pwm]; pwm 0 continua 0.

Parametros (padrao 1,0 no codigo = PWM identico ao de antes; ros2 param set muda na hora):
  lateral_wheel_gain_fl, _fr, _rl, _rr (0,5 .. 1,6). Valores medidos: aplicar_ganho_lateral_yaml.py.
Nao muda: cinematica, heading hold, PWM minimo, parada, firmware, andar para frente/tras, giro.

Uso: python3 aplicar_ganho_lateral.py [--arquivo copia.cpp] [--desfazer]
Depois: cd ~/robo_jazzy_ws && colcon build --symlink-install --packages-select robo_hardware; reiniciar a ponte."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_hardware/src/hardware_bridge_node.cpp')
MARCA = 'GANHO_LATERAL (12/11)'

TROCAS = [
    ('    maximum_pwm_ = declare_parameter<int>("maximum_pwm", 215);\n',
     '    maximum_pwm_ = declare_parameter<int>("maximum_pwm", 215);\n'
     '    // GANHO_LATERAL (12/11): ganho de PWM por roda so no movimento lateral (1.0 = como antes)\n'
     '    lateral_wheel_gain_[0] = declare_parameter<double>("lateral_wheel_gain_fl", 1.0);\n'
     '    lateral_wheel_gain_[1] = declare_parameter<double>("lateral_wheel_gain_fr", 1.0);\n'
     '    lateral_wheel_gain_[2] = declare_parameter<double>("lateral_wheel_gain_rl", 1.0);\n'
     '    lateral_wheel_gain_[3] = declare_parameter<double>("lateral_wheel_gain_rr", 1.0);\n'
     '    for (const double g : lateral_wheel_gain_) {\n'
     '      if (!(g >= 0.5 && g <= 1.6)) {\n'
     '        throw std::invalid_argument("lateral_wheel_gain_* must be between 0.5 and 1.6");\n'
     '      }\n'
     '    }\n'),
    ('        } else if (name == "heading_hold_imu_timeout_s") {\n',
     '        } else if (name.rfind("lateral_wheel_gain_", 0) == 0) {   // GANHO_LATERAL (12/11)\n'
     '          const double g = parameter.as_double();\n'
     '          if (!(g >= 0.5 && g <= 1.6)) {\n'
     '            result.reason = "lateral_wheel_gain_* must be between 0.5 and 1.6";\n'
     '            return result;\n'
     '          }\n'
     '          const std::size_t i = name == "lateral_wheel_gain_fl" ? 0U : name == "lateral_wheel_gain_fr" ? 1U :\n'
     '            name == "lateral_wheel_gain_rl" ? 2U : name == "lateral_wheel_gain_rr" ? 3U : 4U;\n'
     '          if (i > 3U) {\n'
     '            result.reason = "unknown lateral wheel gain";\n'
     '            return result;\n'
     '          }\n'
     '          lateral_gains[i] = g;\n'
     '        } else if (name == "heading_hold_imu_timeout_s") {\n'),
    ('    double imu_timeout = heading_hold_imu_timeout_s_;\n',
     '    double imu_timeout = heading_hold_imu_timeout_s_;\n'
     '    std::array<double, 4> lateral_gains = lateral_wheel_gain_;   // GANHO_LATERAL (12/11)\n'),
    ('    heading_hold_imu_timeout_s_ = imu_timeout;\n',
     '    heading_hold_imu_timeout_s_ = imu_timeout;\n'
     '    lateral_wheel_gain_ = lateral_gains;   // GANHO_LATERAL (12/11)\n'),
    ('    std::array<int16_t, 4> pwm{{0, 0, 0, 0}};\n'
     '    for (std::size_t index = 0; index < pwm.size(); ++index) {\n'
     '      pwm[index] = wheel_speed_to_pwm(wheel_speed[index], effective_minimum_pwm);\n',
     '    // GANHO_LATERAL (12/11): peso do movimento lateral no comando (0 = frente/tras/giro/parado)\n'
     '    const double linear_sum = std::abs(commanded_vx_mps_) + std::abs(commanded_vy_mps_);\n'
     '    const double lateral_weight = linear_sum > 1e-9 ? std::abs(commanded_vy_mps_) / linear_sum : 0.0;\n'
     '    std::array<int16_t, 4> pwm{{0, 0, 0, 0}};\n'
     '    for (std::size_t index = 0; index < pwm.size(); ++index) {\n'
     '      pwm[index] = wheel_speed_to_pwm(wheel_speed[index], effective_minimum_pwm);\n'
     '      const double gain = 1.0 + lateral_weight * (lateral_wheel_gain_[index] - 1.0);\n'
     '      if (pwm[index] != 0 && gain != 1.0) {\n'
     '        const long scaled = std::lround(std::abs(static_cast<double>(pwm[index])) * gain);\n'
     '        const long limited = std::clamp<long>(scaled, effective_minimum_pwm, maximum_pwm_);\n'
     '        pwm[index] = static_cast<int16_t>(pwm[index] < 0 ? -limited : limited);\n'
     '      }\n'),
    ('  int maximum_pwm_{215};\n',
     '  int maximum_pwm_{215};\n'
     '  std::array<double, 4> lateral_wheel_gain_{{1.0, 1.0, 1.0, 1.0}};   // GANHO_LATERAL (12/11): FL FR RL RR\n'),
]

if __name__ == '__main__':
    rodar('ganho_lateral', MARCA, ARQ, TROCAS, doc=__doc__)
