#!/usr/bin/env python3
"""Intertravamento na entrada do velocity_smoother, antes do Collision Monitor."""
import time
import json
from giro_frontal import ArbitragemT, ZERO, fonte_exclusiva


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool, Trigger

    class Arbitro(Node):
        def __init__(self):
            super().__init__('arbitro_t')
            self.a = ArbitragemT()
            self.heartbeat_parado = self.declare_parameter('heartbeat_parado', False).value
            self.ultimo_input_normal = time.monotonic()
            self.pub = self.create_publisher(TwistStamped, '/cmd_vel_nav_seguro', 10)
            self.falha = self.create_publisher(Bool, '/alinhador/conflito_t', 10)
            self.create_subscription(TwistStamped, '/cmd_vel_nav', self.normal, 10)
            self.create_subscription(TwistStamped, '/cmd_vel_nav_t', self.experimental, 10)
            self.create_service(SetBool, '/alinhador/controle_t', self.controle)
            self.create_service(Trigger, '/alinhador/reset_controle_t', self.resetar)
            self.pub_estado = self.create_publisher(String, '/arbitro_t/estado', 20)
            self.create_service(Trigger, '/alinhador/parar_t', self.parar)
            self.create_timer(0.05, self.tick)

        def enviar(self, cmd):
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = cmd
            self.pub.publish(m)

        @staticmethod
        def cmd(m):
            return m.twist.linear.x, m.twist.linear.y, m.twist.angular.z

        def normal(self, m):
            self.ultimo_input_normal = time.monotonic()
            cmd = self.a.normal(time.monotonic(), self.cmd(m))
            # Zeros tardios do Nav2 nao interrompem a realimentacao experimental.
            if not self.a.ativo and not self.a.falha and not self.a.estacionado:
                self.pub.publish(m)
            elif self.a.falha or self.a.estacionado:
                self.enviar(cmd)

        def experimental(self, m, info):
            if not fonte_exclusiva(self.get_publishers_info_by_topic('/cmd_vel_nav_t'), info.publisher_gid):
                self.a.ativo, self.a.falha, self.a.estacionado = False, True, True
                self.enviar(ZERO)
                return
            if self.a.ativo:
                self.enviar(self.a.experimental(time.monotonic(), self.cmd(m)))

        def controle(self, req, res):
            if req.data:
                res.success = self.a.adquirir(time.monotonic())
            else:
                self.enviar(ZERO)
                res.success = self.a.liberar(time.monotonic())
            res.message = (('controle exclusivo' if req.data else 'parado_confirmado')
                           if res.success else 'entrada ocupada/falha')
            return res

        def resetar(self, req, res):
            self.enviar(ZERO)
            res.success = self.a.resetar(time.monotonic())
            res.message = 'reset manual' if res.success else 'cancele as outras fontes primeiro'
            return res

        def parar(self, req, res):
            self.a.ativo, self.a.falha, self.a.estacionado = False, True, True
            self.enviar(ZERO)
            res.success, res.message = True, 'parada travada; retomada exige reset'
            return res

        def tick(self):
            self.a.vigiar(time.monotonic())
            if (self.a.falha or self.a.estacionado or
                    (self.heartbeat_parado and not self.a.ativo and
                     time.monotonic()-self.ultimo_input_normal > 0.25)):
                self.enviar(ZERO)
            m = Bool()
            m.data = self.a.falha
            self.falha.publish(m)
            estado = String()
            estado.data = json.dumps(dict(ativo=self.a.ativo, falha=self.a.falha,
                estacionado=self.a.estacionado, stamp_ns=self.get_clock().now().nanoseconds))
            self.pub_estado.publish(estado)

    rclpy.init()
    n = Arbitro()
    try:
        rclpy.spin(n)
    finally:
        n.enviar(ZERO)
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
