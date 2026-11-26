#!/usr/bin/env python3
"""Suavizador da saida do controller do Nav2 (SUAVIZADOR_NAV v1, 03/11/2026).

Por que (missao7): em area aberta o DWB troca de direcao 4-5 vezes por segundo entre
(0,08; 0), (0,053; 0,053), (0; 0,08), (-0,053; 0,053)... Como o robo anda 0,15-0,26 m/s com
qualquer comando (PWM minimo), cada troca vira um tranco: 2 m levaram 25-29 s, com 2,5-3,7 m
percorridos.

O que faz: /cmd_vel_ctrl (saida do controller_server) -> /cmd_vel_nav (entrada do
velocity_smoother, a mesma do alinhador).
  - direcao (vx, vy): media exponencial com constante 'tau' (0,5 s): as trocas de um ciclo
    se misturam numa direcao media;
  - modulo: media do modulo pedido (a mistura nao cai abaixo da zona morta);
  - wz: media com 'tau_giro' (0,25 s) quando ha movimento linear; giro puro (RotationShim,
    vx = vy = 0) passa direto;
  - zero (parar) passa na hora (a direcao media fica guardada por 'timeout' s); sem
    comando por 'timeout' s o filtro zera
    e nada e publicado (o alinhador usa o mesmo /cmd_vel_nav nessa hora).
O Collision Monitor continua depois de tudo (seguranca nao muda). As recuperacoes do
behavior_server continuam indo direto para /cmd_vel_nav.

Uso: sobe pelo navigation.launch.py (aplicar_suavizador.py) ou, para testar:
    python3 suavizador_nav.py
"""
import math
import time

TAU = 0.25  # SUAV_V2 (05/11): era 0.5
TAU_GIRO = 0.12  # SUAV_V2: era 0.25
TIMEOUT = 0.3


def comando_corredor(vx, vy, wz, fator):
    """Escala a translação continuamente, inclusive nas correções diagonais."""
    if fator == 0.0:
        return 0.0, 0.0, 0.0
    # O giro puro precisa vencer a zona morta angular. Durante translação,
    # a partir de 0,02 rad/s a ponte distingue giro pedido de retenção de rumo.
    # Um ajuste angular não deve acelerar a translação.
    piso_giro = 0.0201 if (vx or vy) and abs(wz) >= 0.02 else 0.0151
    giro = math.copysign(min(abs(wz), max(abs(wz) * fator, piso_giro)), wz)
    return vx * fator, vy * fator, giro


class Filtro:
    """Sem ROS (testavel): passo(t, vx, vy, wz) -> (vx, vy, wz)."""

    def __init__(self, tau=TAU, tau_giro=TAU_GIRO, timeout=TIMEOUT, preservar_eixos_zero=False):
        self.tau, self.tau_giro, self.timeout = tau, tau_giro, timeout
        self.preservar_eixos_zero = preservar_eixos_zero
        self.t = None
        self.fx = self.fy = self.fw = self.fm = 0.0
        self.ativo = False

    def zerar(self):
        self.fx = self.fy = self.fw = self.fm = 0.0
        self.ativo = False

    def passo(self, t, vx, vy, wz):
        dt = None if self.t is None else t - self.t
        self.t = t
        if dt is None or dt > self.timeout or dt < 0:
            self.zerar()
        m = math.hypot(vx, vy)
        if m < 1e-4:
            # parar ou giro puro: na hora. A direcao fica guardada por 'timeout' s (um zero
            # isolado do DWB no meio do trecho nao apaga a media); giro puro zera.
            if abs(wz) > 1e-4:
                self.zerar()
            return 0.0, 0.0, wz
        if not self.ativo:
            self.fx, self.fy, self.fw, self.fm = vx, vy, wz, m
            self.ativo = True
            return vx, vy, wz
        a = 1.0 - math.exp(-dt / self.tau)
        ag = 1.0 - math.exp(-dt / self.tau_giro)
        self.fx += a * (vx - self.fx)
        self.fy += a * (vy - self.fy)
        # LATERAL_SEM_PULSO_M35: nao recriar vx quando Nav2 pediu lateral pura.
        # Diagonal intencional permanece diagonal; nao altera motores/feedforward.
        if self.preservar_eixos_zero:
            if abs(vx) < 1e-4:
                self.fx = 0.0
            if abs(vy) < 1e-4:
                self.fy = 0.0
            if abs(vx) < 1e-4 or abs(vy) < 1e-4:
                self.fm = min(self.fm, m)  # nao transferir velocidade frontal acumulada para o lado
        self.fm += a * (m - self.fm)
        self.fw += ag * (wz - self.fw)
        fm = math.hypot(self.fx, self.fy)
        if fm < 1e-4:
            return 0.0, 0.0, self.fw
        k = self.fm / fm
        return self.fx * k, self.fy * k, self.fw


def main():
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
    from geometry_msgs.msg import TwistStamped

    class Suavizador(Node):
        def __init__(self):
            super().__init__('suavizador_nav')
            self.declare_parameter('tau', TAU)
            self.declare_parameter('tau_giro', TAU_GIRO)
            self.declare_parameter('preservar_eixos_zero', False)
            self.declare_parameter('fator_comando_corredor', 1.0, ParameterDescriptor(
                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))
            self.f = Filtro(float(self.get_parameter('tau').value),
                            float(self.get_parameter('tau_giro').value),
                            preservar_eixos_zero=bool(self.get_parameter('preservar_eixos_zero').value))
            self.pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
            self.create_subscription(TwistStamped, '/cmd_vel_ctrl', self.ao_cmd, 10)
            self.get_logger().info('Suavizador pronto: /cmd_vel_ctrl -> /cmd_vel_nav (tau %.2f s)' % self.f.tau)

        def ao_cmd(self, msg):
            tw = msg.twist
            agora = time.monotonic()
            cmd = self.f.passo(agora, tw.linear.x, tw.linear.y, tw.angular.z)
            vx, vy, wz = comando_corredor(*cmd, self.get_parameter('fator_comando_corredor').value)
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = msg.header.frame_id or 'base_link'
            out.twist.linear.x, out.twist.linear.y, out.twist.angular.z = vx, vy, wz
            self.pub.publish(out)

    rclpy.init()
    no = Suavizador()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
