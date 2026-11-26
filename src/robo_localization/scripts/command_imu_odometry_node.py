#!/usr/bin/env python3
# ODOM_TABELA v1 (04/11/2026): velocidade pela tabela medida (comando -> velocidade real).
#
# Antes: velocidade = comando x escala fixa (linear_scale_x 5,0 / linear_scale_y 6,0). Com o PWM
# minimo 150 o robo anda ~0,21-0,25 m/s (frente) e ~0,12-0,16 m/s (lado) com QUALQUER comando,
# entao a escala fixa errava ate 4x (0,08 de comando virava 0,40 m/s na odom; real 0,23).
# Agora (velocity_model: tabela): o modulo do comando de translacao entra na tabela medida pelo
# teste_velocidade_v2 (PWM 150, distancia andada / 1,6 s) e a direcao continua a do comando.
# Frente e lado tem tabelas proprias; numa diagonal cada eixo usa a sua. Abaixo de
# speed_deadband (zona morta das rodas) a translacao e zero. O giro continua da IMU.
# velocity_model: escala volta exatamente ao comportamento antigo.
# Replay das missoes 7-12 (janelas de 1 s, contra a pose do SLAM): erro mediano 5,8 -> 4,2 cm,
# p90 22,5 -> 12,1 cm; frente odom/mapa 0,93 -> 1,02.
# Ao trocar o PWM: rodar o teste_velocidade_v2 de novo e trocar as 3 listas no localization.yaml.

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


class CommandImuOdometry(Node):
    def __init__(self) -> None:
        super().__init__("command_imu_odometry")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("imu_timeout_s", 0.20)
        self.declare_parameter("maximum_integration_step_s", 0.10)
        self.declare_parameter("linear_scale_x", 1.0)
        self.declare_parameter("linear_scale_y", 1.0)
        self.declare_parameter("velocity_model", "escala")  # escala | tabela (ODOM_TABELA)
        self.declare_parameter("speed_table_cmd", [0.02, 0.20])
        self.declare_parameter("speed_table_x", [0.10, 1.00])
        self.declare_parameter("speed_table_y", [0.10, 1.00])
        self.declare_parameter("speed_deadband", 0.005)
        self.declare_parameter("position_variance", 0.25)
        self.declare_parameter("yaw_variance", 0.02)
        self.declare_parameter("linear_velocity_variance", 0.10)
        self.declare_parameter("angular_velocity_variance", 0.02)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_timeout_s = float(
            self.get_parameter("command_timeout_s").value
        )
        self.imu_timeout_s = float(self.get_parameter("imu_timeout_s").value)
        self.maximum_integration_step_s = float(
            self.get_parameter("maximum_integration_step_s").value
        )
        self.linear_scale_x = float(self.get_parameter("linear_scale_x").value)
        self.linear_scale_y = float(self.get_parameter("linear_scale_y").value)
        self.velocity_model = str(self.get_parameter("velocity_model").value).strip().lower()
        self.table_cmd = [float(v) for v in self.get_parameter("speed_table_cmd").value]
        self.table_x = [float(v) for v in self.get_parameter("speed_table_x").value]
        self.table_y = [float(v) for v in self.get_parameter("speed_table_y").value]
        self.speed_deadband = float(self.get_parameter("speed_deadband").value)
        if self.velocity_model not in ("escala", "tabela"):
            raise ValueError("velocity_model deve ser 'escala' ou 'tabela'")
        if self.velocity_model == "tabela":
            n = len(self.table_cmd)
            if n < 1 or len(self.table_x) != n or len(self.table_y) != n:
                raise ValueError("speed_table_cmd, speed_table_x e speed_table_y precisam ter o mesmo tamanho")
            if any(b <= a for a, b in zip(self.table_cmd, self.table_cmd[1:])):
                raise ValueError("speed_table_cmd precisa ser crescente")
            if min(self.table_x + self.table_y) < 0.0 or self.speed_deadband < 0.0:
                raise ValueError("tabela e speed_deadband nao podem ser negativos")
        self.position_variance = float(
            self.get_parameter("position_variance").value
        )
        self.yaw_variance = float(self.get_parameter("yaw_variance").value)
        self.linear_velocity_variance = float(
            self.get_parameter("linear_velocity_variance").value
        )
        self.angular_velocity_variance = float(
            self.get_parameter("angular_velocity_variance").value
        )

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz deve ser positivo")
        if self.command_timeout_s <= 0.0 or self.imu_timeout_s <= 0.0:
            raise ValueError("timeouts devem ser positivos")
        if self.maximum_integration_step_s <= 0.0:
            raise ValueError("maximum_integration_step_s deve ser positivo")

        self.x = 0.0
        self.y = 0.0
        self.raw_imu_yaw = 0.0
        self.imu_janela_fim = 0.0  # IMU_CONTINUA
        self.yaw_origin: Optional[float] = None
        self.gyro_z = 0.0
        self.command_vx = 0.0
        self.command_vy = 0.0
        self.command_wz = 0.0

        monotonic_now = time.monotonic()
        self.last_timer_time = monotonic_now
        self.last_command_time: Optional[float] = None
        self.last_imu_time: Optional[float] = None
        self.last_warning_time = 0.0

        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.command_subscription = self.create_subscription(
            TwistStamped, self.cmd_vel_topic, self.command_callback, 10
        )
        self.imu_subscription = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data
        )
        self.reset_service = self.create_service(
            Trigger, "reset_odometry", self.reset_odometry
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

        if self.velocity_model == "tabela":
            self.get_logger().info(
                "ODOM_TABELA: velocidade pela tabela medida; comando %s -> frente %s / lado %s m/s, "
                "zona morta %.3f" % (self.table_cmd, self.table_x, self.table_y, self.speed_deadband)
            )
        else:
            self.get_logger().info(
                "Velocidade = comando x escala (%.2f, %.2f)" % (self.linear_scale_x, self.linear_scale_y)
            )
        self.get_logger().info(
            "Odometria comando+BNO055 pronta: "
            f"{self.cmd_vel_topic} + {self.imu_topic} -> {odom_topic}; "
            f"TF {self.odom_frame} -> {self.base_frame}"
        )

    def command_callback(self, message: TwistStamped) -> None:
        vx = float(message.twist.linear.x)
        vy = float(message.twist.linear.y)
        wz = float(message.twist.angular.z)
        if not all(math.isfinite(value) for value in (vx, vy, wz)):
            self.get_logger().warning("Comando nao finito ignorado")
            return
        self.command_vx = vx
        self.command_vy = vy
        self.command_wz = wz
        self.last_command_time = time.monotonic()

    def imu_callback(self, message: Imu) -> None:
        quaternion = message.orientation
        yaw = yaw_from_quaternion(
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        )
        if not math.isfinite(yaw):
            self.get_logger().warning("Orientacao IMU nao finita ignorada")
            return

        # IMU_CONTINUA (06/11): a placa reconectou e zerou a IMU (missao22: 30,8 -> 0 graus). Mantem o
        # yaw publicado; o SLAM corrige o que girou durante a falha. Olha 3 s, se o zero vier depois.
        agora_imu = time.monotonic()
        if self.yaw_origin is not None and self.last_imu_time is not None:
            if agora_imu - self.last_imu_time > 1.0:
                self.imu_janela_fim = agora_imu + 3.0
                self.yaw_origin = normalize_angle(yaw - self.current_yaw())
                self.get_logger().warning(
                    'IMU voltou depois de %.1f s: yaw da odom mantido'
                    % (agora_imu - self.last_imu_time))
            elif agora_imu < self.imu_janela_fim and \
                    abs(normalize_angle(yaw - self.raw_imu_yaw)) > math.radians(5.0):
                self.yaw_origin = normalize_angle(yaw - self.current_yaw())
                self.get_logger().warning('IMU zerada depois de voltar: yaw da odom mantido')
        self.raw_imu_yaw = yaw
        if self.yaw_origin is None:
            self.yaw_origin = yaw
            self.get_logger().info("Referencia inicial de yaw capturada")

        if math.isfinite(message.angular_velocity.z):
            self.gyro_z = float(message.angular_velocity.z)
        self.last_imu_time = time.monotonic()

    @staticmethod
    def interpolate(table_cmd, table_value, value: float) -> float:
        if value <= table_cmd[0]:
            return table_value[0]
        for k in range(1, len(table_cmd)):
            if value <= table_cmd[k]:
                a, b = table_cmd[k - 1], table_cmd[k]
                return table_value[k - 1] + (table_value[k] - table_value[k - 1]) * (value - a) / (b - a)
        return table_value[-1]

    def body_velocity(self, command_vx: float, command_vy: float):
        """Velocidade estimada do corpo (base_link) para o comando de translacao."""
        if self.velocity_model == "escala":
            return command_vx * self.linear_scale_x, command_vy * self.linear_scale_y
        speed = math.hypot(command_vx, command_vy)
        if speed <= self.speed_deadband:
            return 0.0, 0.0
        real_x = self.interpolate(self.table_cmd, self.table_x, speed)
        real_y = self.interpolate(self.table_cmd, self.table_y, speed)
        return real_x * command_vx / speed, real_y * command_vy / speed

    def current_yaw(self) -> float:
        if self.yaw_origin is None:
            return 0.0
        return normalize_angle(self.raw_imu_yaw - self.yaw_origin)

    def reset_odometry(self, _request, response):
        self.x = 0.0
        self.y = 0.0
        self.yaw_origin = self.raw_imu_yaw if self.last_imu_time is not None else None
        self.last_timer_time = time.monotonic()
        response.success = True
        response.message = "Odometria zerada na pose atual"
        self.get_logger().info(response.message)
        return response

    def timer_callback(self) -> None:
        monotonic_now = time.monotonic()
        elapsed = monotonic_now - self.last_timer_time
        self.last_timer_time = monotonic_now
        dt = min(max(elapsed, 0.0), self.maximum_integration_step_s)

        have_imu = self.last_imu_time is not None and self.yaw_origin is not None
        imu_fresh = have_imu and (
            monotonic_now - self.last_imu_time <= self.imu_timeout_s
        )
        command_fresh = self.last_command_time is not None and (
            monotonic_now - self.last_command_time <= self.command_timeout_s
        )

        yaw = self.current_yaw()
        vx = 0.0
        vy = 0.0
        if imu_fresh and command_fresh:
            vx, vy = self.body_velocity(self.command_vx, self.command_vy)
            cosine = math.cos(yaw)
            sine = math.sin(yaw)
            self.x += (cosine * vx - sine * vy) * dt
            self.y += (sine * vx + cosine * vy) * dt

        if not have_imu:
            if monotonic_now - self.last_warning_time >= 5.0:
                self.get_logger().warning("Aguardando a primeira mensagem da IMU")
                self.last_warning_time = monotonic_now
            return

        angular_velocity = self.gyro_z if imu_fresh else 0.0
        self.publish_odometry(vx, vy, angular_velocity, yaw)

    def publish_odometry(
        self, vx: float, vy: float, angular_velocity: float, yaw: float
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw

        message.pose.covariance[0] = self.position_variance
        message.pose.covariance[7] = self.position_variance
        message.pose.covariance[14] = 1.0e6
        message.pose.covariance[21] = 1.0e6
        message.pose.covariance[28] = 1.0e6
        message.pose.covariance[35] = self.yaw_variance

        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = angular_velocity
        message.twist.covariance[0] = self.linear_velocity_variance
        message.twist.covariance[7] = self.linear_velocity_variance
        message.twist.covariance[14] = 1.0e6
        message.twist.covariance[21] = 1.0e6
        message.twist.covariance[28] = 1.0e6
        message.twist.covariance[35] = self.angular_velocity_variance
        self.odom_publisher.publish(message)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandImuOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
