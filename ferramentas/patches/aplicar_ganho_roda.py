#!/usr/bin/env python3
"""GANHO_RODA (12/11/2026) - ponte (hardware_bridge_node.cpp): ganho de PWM por roda, para achar o equilibrio
dos motores no deslize lateral (o robo anda na diagonal em vez de lado: teste 12/11, dx/dy 3-5).

O comando (vx, vy, wz) so controla 3 dos 4 modos das rodas: equilibrio frente x tras nao da para obter por
comando. Parametros novos (padrao 1,0 = PWM identico ao de hoje), mudam na hora com ros2 param set:
  wheel_gain_fl, wheel_gain_fr, wheel_gain_rl, wheel_gain_rr  (0,5 .. 1,6)
Aplicado no PWM ja calculado: |pwm| * ganho, limitado a [PWM minimo efetivo, maximum_pwm]; roda parada continua 0.
Nao muda: cinematica, heading hold, PWM minimo, parada (pwm 0 continua 0), firmware.

Uso: python3 aplicar_ganho_roda.py [--arquivo copia.cpp] [--desfazer]
Depois de aplicar: cd ~/robo_jazzy_ws && colcon build --symlink-install --packages-select robo_hardware
e reiniciar so a ponte."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_hardware/src/hardware_bridge_node.cpp')
MARCA = 'GANHO_RODA (12/11)'

TROCAS = [
    ('    maximum_pwm_ = declare_parameter<int>("maximum_pwm", 215);\n',
     '    maximum_pwm_ = declare_parameter<int>("maximum_pwm", 215);\n'
     '    // GANHO_RODA (12/11): ganho de PWM por roda (1.0 = como antes)\n'
     '    wheel_gain_[0] = declare_parameter<double>("wheel_gain_fl", 1.0);\n'
     '    wheel_gain_[1] = declare_parameter<double>("wheel_gain_fr", 1.0);\n'
     '    wheel_gain_[2] = declare_parameter<double>("wheel_gain_rl", 1.0);\n'
     '    wheel_gain_[3] = declare_parameter<double>("wheel_gain_rr", 1.0);\n'
     '    for (const double g : wheel_gain_) {\n'
     '      if (!(g >= 0.5 && g <= 1.6)) {\n'
     '        throw std::invalid_argument("wheel_gain_* must be between 0.5 and 1.6");\n'
     '      }\n'
     '    }\n'),
    ('        } else if (name == "heading_hold_imu_timeout_s") {\n',
     '        } else if (name.rfind("wheel_gain_", 0) == 0) {   // GANHO_RODA (12/11)\n'
     '          const double g = parameter.as_double();\n'
     '          if (!(g >= 0.5 && g <= 1.6)) {\n'
     '            result.reason = "wheel_gain_* must be between 0.5 and 1.6";\n'
     '            return result;\n'
     '          }\n'
     '          const std::size_t i = name == "wheel_gain_fl" ? 0U : name == "wheel_gain_fr" ? 1U :\n'
     '            name == "wheel_gain_rl" ? 2U : name == "wheel_gain_rr" ? 3U : 4U;\n'
     '          if (i > 3U) {\n'
     '            result.reason = "unknown wheel gain";\n'
     '            return result;\n'
     '          }\n'
     '          gains[i] = g;\n'
     '        } else if (name == "heading_hold_imu_timeout_s") {\n'),
    ('    double imu_timeout = heading_hold_imu_timeout_s_;\n',
     '    double imu_timeout = heading_hold_imu_timeout_s_;\n'
     '    std::array<double, 4> gains = wheel_gain_;   // GANHO_RODA (12/11)\n'),
    ('    heading_hold_imu_timeout_s_ = imu_timeout;\n',
     '    heading_hold_imu_timeout_s_ = imu_timeout;\n'
     '    wheel_gain_ = gains;   // GANHO_RODA (12/11)\n'),
    ('      pwm[index] = wheel_speed_to_pwm(wheel_speed[index], effective_minimum_pwm);\n',
     '      pwm[index] = wheel_speed_to_pwm(wheel_speed[index], effective_minimum_pwm);\n'
     '      if (pwm[index] != 0 && wheel_gain_[index] != 1.0) {   // GANHO_RODA (12/11)\n'
     '        const long scaled = std::lround(std::abs(static_cast<double>(pwm[index])) * wheel_gain_[index]);\n'
     '        const long limited = std::clamp<long>(scaled, effective_minimum_pwm, maximum_pwm_);\n'
     '        pwm[index] = static_cast<int16_t>(pwm[index] < 0 ? -limited : limited);\n'
     '      }\n'),
    ('  int maximum_pwm_{215};\n',
     '  int maximum_pwm_{215};\n'
     '  std::array<double, 4> wheel_gain_{{1.0, 1.0, 1.0, 1.0}};   // GANHO_RODA (12/11): FL FR RL RR\n'),
]

if __name__ == '__main__':
    rodar('ganho_roda', MARCA, ARQ, TROCAS, doc=__doc__)
