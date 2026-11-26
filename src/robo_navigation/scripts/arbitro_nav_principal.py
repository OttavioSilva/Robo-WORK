#!/usr/bin/env python3
"""NAV2_PRINCIPAL: unica saida para velocity_smoother, antes do CM; inicia bloqueado."""
from controle_nav_principal import Autoridade


def main():
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import TwistStamped
    from std_srvs.srv import SetBool

    class Arbitro(Node):
        def __init__(self):
            super().__init__('arbitro_nav_principal')
            self.a = Autoridade()
            self.pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
            self.create_subscription(TwistStamped, '/cmd_vel_nav_base', lambda m:self.cmd('nav2',m),10)
            self.create_subscription(TwistStamped, '/cmd_vel_nav_especial', lambda m:self.cmd('especial',m),10)
            self.create_subscription(TwistStamped, '/navegacao/heartbeat', self.heartbeat,1)
            self.create_service(SetBool, '/navegacao/especial', self.trocar)
            self.create_timer(.05,self.vigiar)
            self.get_logger().info('ARBITRO_PRONTO: saida exclusiva /cmd_vel_nav; autoridade bloqueada ate lease')

        def heartbeat(self, m):
            stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            agora=self.get_clock().now().nanoseconds/1e9
            if stamp >= self.a.desde_ros and 0 <= agora-stamp <= .3:
                self.a.renovar(m.header.frame_id,time.monotonic())

        def zero(self):
            m=TwistStamped(); m.header.stamp=self.get_clock().now().to_msg(); self.pub.publish(m)

        def trocar(self, req, res):
            self.zero()
            self.a.trocar('especial' if req.data else 'nav2',time.monotonic())
            self.a.desde_ros=self.get_clock().now().nanoseconds/1e9
            res.success=True; res.message=self.a.dono
            return res

        def cmd(self,dono,m):
            stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            if self.a.permite(dono,time.monotonic(),stamp,self.get_clock().now().nanoseconds/1e9):
                self.pub.publish(m)

        def vigiar(self):
            if time.monotonic()-self.a.heartbeat > self.a.timeout:
                self.zero()

    rclpy.init(); n=Arbitro()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # SIGINT pode invalidar o contexto antes de sair do spin.
        if n.context.ok():
            n.zero()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__': main()
