#!/usr/bin/env python3
"""Mostra a mediana de cada feixe cardinal do M2M2 no último segundo."""

import math
import statistics
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


DIRECOES = (("Frente", 0), ("Esquerda", 90), ("Direita", -90), ("Trás", -180))


def main():
    rclpy.init()
    node = rclpy.create_node("medir_direcoes_m2m2")
    valores = {nome: [] for nome, _ in DIRECOES}
    scans = 0

    def receber(msg):
        nonlocal scans
        scans += 1
        passo = msg.angle_increment
        if not math.isfinite(passo) or passo == 0 or not math.isfinite(msg.angle_min):
            return
        for nome, graus in DIRECOES:
            alvo = math.radians(graus)
            for angulo in (alvo, alvo + math.tau, alvo - math.tau):
                i = round((angulo - msg.angle_min) / passo)
                if not 0 <= i < len(msg.ranges):
                    continue
                if abs(msg.angle_min + i * passo - angulo) > 1e-6:
                    continue
                distancia = msg.ranges[i]
                if math.isfinite(distancia) and msg.range_min <= distancia <= msg.range_max:
                    valores[nome].append(distancia * 100)
                break

    def mostrar():
        nonlocal scans
        campos = []
        for nome, _ in DIRECOES:
            pontos = valores[nome]
            leitura = f"{statistics.median(pontos):.1f} cm" if pontos else "—"
            campos.append(f"{nome} {leitura} ({len(pontos)}/{scans})")
            pontos.clear()
        linha = " | ".join(campos) if scans else "Aguardando /scan_raw…"
        if sys.stdout.isatty():
            print("\r\033[K" + linha, end="", flush=True)
        else:
            print(linha, flush=True)
        scans = 0

    node.create_subscription(LaserScan, "/scan_raw", receber, qos_profile_sensor_data)
    node.create_timer(1.0, mostrar)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        if sys.stdout.isatty():
            print()


if __name__ == "__main__":
    main()
