#!/usr/bin/env python3
"""Mostra uma leitura do feixe de 0° do M2M2, em centímetros."""

import math

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import LaserScan


def leitura_frontal(scan):
    passo = scan.angle_increment
    if not math.isfinite(passo) or passo == 0 or not math.isfinite(scan.angle_min):
        return "Grade angular inválida."
    indice = round(-scan.angle_min / passo)
    if not 0 <= indice < len(scan.ranges):
        return "0° está fora da varredura."
    # Os ângulos de LaserScan são float32. Aceita o arredondamento do feixe central.
    if abs(scan.angle_min + indice * passo) > 1e-6:
        return "Esta varredura não tem feixe centrado em 0°."
    metros = scan.ranges[indice]
    if not math.isfinite(metros) or not scan.range_min <= metros <= scan.range_max:
        return "0° — sem retorno válido nesta varredura."
    return f"0° — {100 * metros:.1f} cm"


def main():
    rclpy.init()
    node = rclpy.create_node("medir_zero_m2m2")
    try:
        recebeu, scan = wait_for_message(
            LaserScan, node, "/scan_raw", qos_profile=qos_profile_sensor_data,
            time_to_wait=8,
        )
        if not recebeu:
            raise SystemExit("Nenhuma mensagem de /scan_raw em 8 segundos.")
        print(leitura_frontal(scan))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
