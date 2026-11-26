#!/usr/bin/env python3
"""M2M2_ODOM (25/11/2026): pose do M2M2 vira a odometria (TF odom->base_link) do slam_toolbox.

Medido 25/11 (60 s andando): com a odometria por comando o slam_toolbox derivava (paredes dobradas,
2-3% do scan em parede do mapa). A pose do M2M2 nao deriva, mas treme 5-17 cm entre scans; o
slam_toolbox usa ela como chute inicial e o casamento de scans tira o tremor.

- /m2m2/pose = pose do SENSOR (laser_raw) no mapa interno do M2M2.
  base = sensor * inv(base_link->laser_raw)  (TF estatico do URDF).
- Publica TF odom->base_link (o command_imu_odometry roda com publish_tf=false nesta subida).
- Servico /reset_odometry (Trigger, o mesmo nome que o teleop Y 2x chama): a pose atual vira a
  origem do odom (0,0,0). O do command_imu_odometry fica em /reset_odometry_comando.
Usado so no base_m2m2_slam.launch.py.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def compor(a, b):
    ax, ay, at = a
    bx, by, bt = b
    return (ax + bx * math.cos(at) - by * math.sin(at), ay + bx * math.sin(at) + by * math.cos(at), at + bt)


def inverter(a):
    x, y, t = a
    return (-x * math.cos(t) - y * math.sin(t), x * math.sin(t) - y * math.cos(t), -t)


class M2m2Odom(Node):
    def __init__(self):
        super().__init__('m2m2_odom')
        self.espelhar = bool(self.declare_parameter('espelhar', True).value)  # M2M2_ESPELHO (25/11)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.tf = TransformBroadcaster(self)
        self.base_laser = None
        self.origem = None      # pose do base no mapa do M2M2 que vira (0,0,0)
        self.ultima = None
        self.create_subscription(PoseStamped, '/m2m2/pose', self.ao_pose, 10)
        self.create_service(Trigger, '/reset_odometry', self.zerar)
        # M2M2_ODOM_CONTINUA (25/11): a pose do M2M2 chegou a falhar 1,1-1,5 s (o costmap tolera 0,3 s).
        # Entre poses do M2M2, soma o deslocamento do /odom (comando + IMU) e publica a 30 Hz fixos.
        self.odom_agora = None      # pose do /odom (comando) mais recente
        self.odom_na_m2 = None      # pose do /odom no instante da ultima pose do M2M2
        self.base_m2 = None         # base no frame odom (M2M2) da ultima pose
        self.create_subscription(Odometry, '/odom', self.ao_odom, 20)
        self.create_timer(1.0 / 30.0, self.publicar)
        self.get_logger().info('M2M2_ODOM: /m2m2/pose -> TF odom->base_link')

    def ao_pose(self, m):
        if self.base_laser is None:
            try:
                t = self.buf.lookup_transform('base_link', 'laser_raw', rclpy.time.Time()).transform
            except Exception:  # noqa: BLE001
                return
            q = t.rotation
            self.base_laser = (t.translation.x, t.translation.y,
                               math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            self.get_logger().info('M2M2_ODOM: base_link->laser_raw = %.3f %.3f %.1f graus'
                                   % (self.base_laser[0], self.base_laser[1], math.degrees(self.base_laser[2])))
        q = m.pose.orientation
        sensor = (m.pose.position.x, m.pose.position.y,
                  math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        if self.espelhar:
            # M2M2_ESPELHO (25/11): medido em 60 s andando: IMU -315 graus, M2M2 +302; lateral p/ direita
            # na IMU = esquerda no M2M2. O M2M2 esta espelhado: y -> -y, yaw -> -yaw (scan: angle_sign -1).
            sensor = (sensor[0], -sensor[1], -sensor[2])
        base = compor(sensor, inverter(self.base_laser))
        self.ultima = base
        if self.origem is None:
            self.origem = base
        medida = compor(inverter(self.origem), base)
        # M2M2_SALTO (25/11): M2M2 reiniciado (emergencia corta a alimentacao) ou relocalizado = pose
        # em outro referencial. Aplicar o salto girou o robo no mapa (rota que batia). Se a medida
        # difere do previsto (ultima pose + deslocamento do /odom) > 0,30 m ou > 15 graus,
        # re-ancora a origem: o odom continua de onde estava e o slam_toolbox corrige.
        if self.base_m2 is not None:
            prev = self.base_m2
            if self.odom_na_m2 is not None and self.odom_agora is not None:
                prev = compor(self.base_m2, compor(inverter(self.odom_na_m2), self.odom_agora))
            d = math.hypot(medida[0] - prev[0], medida[1] - prev[1])
            dt = abs(math.degrees(math.atan2(math.sin(medida[2] - prev[2]), math.cos(medida[2] - prev[2]))))
            if d > 0.30 or dt > 15.0:
                self.origem = compor(base, inverter(prev))
                medida = prev
                self.get_logger().warn('M2M2_SALTO: pose do M2M2 saltou %.2f m / %.0f graus: re-ancorado '
                                       '(odom continua, SLAM corrige)' % (d, dt))
        self.base_m2 = medida
        self.odom_na_m2 = self.odom_agora

    def ao_odom(self, m):
        q = m.pose.pose.orientation
        self.odom_agora = (m.pose.pose.position.x, m.pose.pose.position.y,
                           math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def publicar(self):
        if self.base_m2 is None:
            return
        x, y, th = self.base_m2
        if self.odom_na_m2 is not None and self.odom_agora is not None:
            x, y, th = compor(self.base_m2, compor(inverter(self.odom_na_m2), self.odom_agora))
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self.tf.sendTransform(t)

    def zerar(self, req, resp):
        if self.ultima is None:
            resp.success, resp.message = False, 'sem pose do M2M2 ainda'
            return resp
        self.origem = self.ultima
        self.base_m2, self.odom_na_m2 = (0.0, 0.0, 0.0), self.odom_agora
        resp.success, resp.message = True, 'odom M2M2 zerada na pose atual'
        self.get_logger().info('M2M2_ODOM: odometria zerada aqui')
        return resp


def main():
    rclpy.init()
    no = M2m2Odom()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, Exception):
        pass


if __name__ == '__main__':
    main()
