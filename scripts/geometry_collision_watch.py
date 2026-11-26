#!/usr/bin/env python3
"""Registra estados reais do Collision Monitor e o comando associado.

Nao envia /cmd_vel, nao altera parametros ou a Behavior Tree.
"""
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PolygonStamped
from nav2_msgs.msg import CollisionMonitorState


class Watch(Node):
    def __init__(self, csv_path):
        super().__init__("geometry_collision_watch")
        self.out = open(csv_path, "w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.out)
        self.csv.writerow([
            "wall_clock", "action", "polygon_name", "vx_in", "vy_in",
            "wz_in", "input_age_s", "vx_out", "vy_out", "wz_out",
            "output_age_s", "polygon_vertices", "polygon_age_s"
        ])
        self.last_in = None
        self.last_out = None
        self.last_poly = None
        self.sub_in = self.create_subscription(
            TwistStamped, "/cmd_vel_smoothed", self.on_in, 10)
        self.sub_out = self.create_subscription(
            TwistStamped, "/cmd_vel", self.on_out, 10)
        self.sub_poly = self.create_subscription(
            PolygonStamped, "/collision_monitor_velocity_polygon",
            self.on_poly, 10)
        self.sub_state = self.create_subscription(
            CollisionMonitorState, "/collision_monitor_state",
            self.on_state, 10)
        self.events = 0
        self.get_logger().info("Observando CollisionMonitorState sem comandar motores")

    def on_in(self, msg):
        self.last_in = (time.monotonic(), msg.twist)

    def on_out(self, msg):
        self.last_out = (time.monotonic(), msg.twist)

    def on_poly(self, msg):
        self.last_poly = (time.monotonic(), len(msg.polygon.points))

    def velocity(self, item, now):
        if item is None:
            return ["", "", "", ""]
        moment, twist = item
        age = now - moment
        if age > 0.5:
            return ["", "", "", round(age, 3)]
        return [
            round(twist.linear.x, 5), round(twist.linear.y, 5),
            round(twist.angular.z, 5), round(age, 3)
        ]

    def on_state(self, msg):
        now = time.monotonic()
        vin = self.velocity(self.last_in, now)
        vout = self.velocity(self.last_out, now)
        poly_n = self.last_poly[1] if self.last_poly else ""
        poly_age = round(now - self.last_poly[0], 3) if self.last_poly else ""
        kind = {
            0: "DO_NOTHING", 1: "STOP", 2: "SLOWDOWN",
            3: "APPROACH", 4: "LIMIT"
        }.get(int(msg.action_type), str(msg.action_type))
        stamp = datetime.now().isoformat(timespec="milliseconds")
        self.csv.writerow([stamp, kind, msg.polygon_name] +
                          vin + vout + [poly_n, poly_age])
        self.out.flush()
        self.events += 1
        print(
            f"{stamp} | {kind} | {msg.polygon_name} | "
            f"entrada vx={vin[0]} vy={vin[1]} wz={vin[2]} | "
            f"saida vx={vout[0]} vy={vout[1]} wz={vout[2]} | "
            f"vertices={poly_n}",
            flush=True
        )

    def close(self):
        self.out.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", default="")
    args = p.parse_args()
    destination = args.csv or (
        str(Path.home() / "robo_jazzy_ws/logs") +
        "/collision_watch_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    )
    Path(destination).parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Watch(destination)
    print("CSV:", destination)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("Eventos de estado registrados:", node.events)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
