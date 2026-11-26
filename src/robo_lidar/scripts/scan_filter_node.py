#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def normalize_angle(angle: float) -> float:
    """Normaliza um angulo para o intervalo [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def parse_intervals(text: str) -> List[Tuple[float, float]]:
    """Converte 'inicio:fim,inicio:fim' de graus para radianos."""
    intervals: List[Tuple[float, float]] = []
    if not text.strip():
        return intervals

    for item in text.split(","):
        parts = item.strip().split(":")
        if len(parts) != 2:
            raise ValueError(
                "blocked_intervals_deg deve usar o formato inicio:fim,inicio:fim"
            )
        start_deg = float(parts[0])
        end_deg = float(parts[1])
        intervals.append(
            (
                normalize_angle(math.radians(start_deg)),
                normalize_angle(math.radians(end_deg)),
            )
        )
    return intervals


def angle_is_blocked(angle: float, intervals: List[Tuple[float, float]]) -> bool:
    angle = normalize_angle(angle)
    for start, end in intervals:
        if start <= end:
            if start <= angle <= end:
                return True
        elif angle >= start or angle <= end:
            return True
    return False


def ponto_do_corpo(distance, angle, laser=(.330,.005,0.0)):
    """CORPO_REAL_M35: mascara geometrica limitada ao corpo; nao cega o setor inteiro."""
    if not math.isfinite(distance) or distance <= 0:
        return False
    x=laser[0]+distance*math.cos(angle+laser[2])
    y=laser[1]+distance*math.sin(angle+laser[2])
    return -.215 < x < .375 and abs(y) < .325


def ponto_do_chao(distance, angle, setor_min_deg, alcance_m, lado_min_deg=None, lado_alcance_m=None):
    """CHAO_TRASEIRO (19/11): o robo parado fica inclinado e o feixe traseiro bate no chao a ~1,13-1,35 m.
    No setor traseiro (|angulo| >= setor_min_deg), retorno a partir de alcance_m (ou sem retorno) e chao/cego:
    vira NaN (nao marca obstaculo nem limpa o costmap). Retorno mais perto que alcance_m continua obstaculo."""
    a = abs(math.degrees(normalize_angle(angle)))
    if a < setor_min_deg:
        # CHAO_LADOS (19/11 noite): o chao tambem aparece nos lados; |angulo| >= lado_min_deg corta em lado_alcance_m
        if lado_min_deg is None or a < lado_min_deg:
            return False
        alcance_m = lado_alcance_m
    return not math.isfinite(distance) or distance >= alcance_m


def ponto_do_chao_frente(distance, angle, setor_max_deg, alcance_m):
    """CHAO_FRENTE (26/11): no setor frontal (|angulo| < setor_max_deg), retorno a partir de alcance_m e chao.
    Sem retorno (inf/nan) fica como esta."""
    if not math.isfinite(distance) or distance < alcance_m:
        return False
    return abs(math.degrees(normalize_angle(angle))) < setor_max_deg


class ScanFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_filter")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan_front")
        self.declare_parameter("invert_angles", True)
        self.declare_parameter("blocked_intervals_deg", "")
        self.declare_parameter("force_out_of_range_to_inf", True)
        self.declare_parameter("body_filter_enabled", False)
        self.declare_parameter("body_laser_x", .330)
        self.declare_parameter("body_laser_y", .005)
        self.declare_parameter("body_laser_yaw", 0.0)
        self.declare_parameter("floor_filter_enabled", False)  # CHAO_TRASEIRO
        self.declare_parameter("floor_sector_min_abs_deg", 130.0)
        self.declare_parameter("floor_range_m", 1.05)
        self.floor_enabled = bool(self.get_parameter("floor_filter_enabled").value)
        self.floor_sector = float(self.get_parameter("floor_sector_min_abs_deg").value)
        self.floor_range = float(self.get_parameter("floor_range_m").value)
        self.declare_parameter("floor_side_enabled", False)  # CHAO_LADOS
        self.declare_parameter("floor_side_min_abs_deg", 60.0)
        self.declare_parameter("floor_side_range_m", 1.5)
        self.floor_side = (float(self.get_parameter("floor_side_min_abs_deg").value)
                           if bool(self.get_parameter("floor_side_enabled").value) else None)
        self.floor_side_range = float(self.get_parameter("floor_side_range_m").value)
        self.declare_parameter("floor_front_enabled", False)  # CHAO_FRENTE
        self.declare_parameter("floor_front_max_abs_deg", 60.0)
        self.declare_parameter("floor_front_range_m", 1.7)
        self.floor_front = (float(self.get_parameter("floor_front_max_abs_deg").value)
                            if bool(self.get_parameter("floor_front_enabled").value) else None)
        self.floor_front_range = float(self.get_parameter("floor_front_range_m").value)
        self.body_filter_enabled = bool(self.get_parameter("body_filter_enabled").value)
        self.body_laser = tuple(float(self.get_parameter(k).value) for k in
                                ("body_laser_x", "body_laser_y", "body_laser_yaw"))

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.invert_angles = bool(self.get_parameter("invert_angles").value)
        blocked_text = str(self.get_parameter("blocked_intervals_deg").value)
        self.force_out_of_range = bool(
            self.get_parameter("force_out_of_range_to_inf").value
        )

        try:
            self.blocked_intervals = parse_intervals(blocked_text)
        except ValueError as error:
            self.get_logger().fatal(str(error))
            raise

        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Filtro LiDAR pronto: {input_topic} -> {output_topic}; "
            f"inversao angular={'ativa' if self.invert_angles else 'inativa'}; "
            f"intervalos bloqueados={len(self.blocked_intervals)}"
        )
        if self.floor_front is not None:
            self.get_logger().info(
                f"CHAO_FRENTE: |angulo| < {self.floor_front:.0f} graus, retorno >= {self.floor_front_range:.2f} m vira NaN (chao)")
        if self.floor_enabled:
            self.get_logger().info(
                f"CHAO_TRASEIRO: |angulo| >= {self.floor_sector:.0f} graus, retorno >= {self.floor_range:.2f} m vira NaN (chao)")
            if self.floor_side is not None:
                self.get_logger().info(
                    f"CHAO_LADOS: {self.floor_side:.0f} <= |angulo| < {self.floor_sector:.0f} graus, "
                    f"retorno >= {self.floor_side_range:.2f} m vira NaN (chao)")

    def scan_callback(self, source: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = source.header
        filtered.time_increment = source.time_increment
        filtered.scan_time = source.scan_time
        filtered.range_min = source.range_min
        filtered.range_max = source.range_max

        if self.invert_angles:
            # Correcao lateral validada em 21/09, conforme o codigo
            # utilizado antes dos testes de quina de 30/09.
            filtered.angle_min = -source.angle_max
            filtered.angle_max = -source.angle_min
            filtered.angle_increment = abs(source.angle_increment)
            filtered.ranges = list(reversed(source.ranges))
            filtered.intensities = list(reversed(source.intensities))
        else:
            filtered.angle_min = source.angle_min
            filtered.angle_max = source.angle_max
            filtered.angle_increment = source.angle_increment
            filtered.ranges = list(source.ranges)
            filtered.intensities = list(source.intensities)

        for index, distance in enumerate(filtered.ranges):
            angle = filtered.angle_min + index * filtered.angle_increment
            invalid_range = (
                self.force_out_of_range
                and math.isfinite(distance)
                and (distance < filtered.range_min or distance > filtered.range_max)
            )
            if self.body_filter_enabled and ponto_do_corpo(distance,angle,self.body_laser):
                filtered.ranges[index] = math.nan  # nao cria raio livre ate alem do motor
                if index < len(filtered.intensities):
                    filtered.intensities[index] = 0.0
            elif getattr(self, 'floor_enabled', False) and ponto_do_chao(
                    distance, angle, self.floor_sector, self.floor_range,
                    getattr(self, 'floor_side', None), getattr(self, 'floor_side_range', None)):
                filtered.ranges[index] = math.nan  # CHAO_TRASEIRO: chao/cego, sem marcar nem limpar
                if index < len(filtered.intensities):
                    filtered.intensities[index] = 0.0
            elif getattr(self, 'floor_front', None) is not None and ponto_do_chao_frente(
                    distance, angle, self.floor_front, self.floor_front_range):
                filtered.ranges[index] = math.nan  # CHAO_FRENTE
                if index < len(filtered.intensities):
                    filtered.intensities[index] = 0.0
            elif invalid_range or angle_is_blocked(angle, self.blocked_intervals):
                filtered.ranges[index] = math.inf
                if index < len(filtered.intensities):
                    filtered.intensities[index] = 0.0

        self.publisher.publish(filtered)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanFilterNode()
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
