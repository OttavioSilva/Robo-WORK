#!/usr/bin/env python3
"""Grava ROS serializado sem o CLI ros2 bag; relê cada mensagem ao terminar.

Não publica, não cancela ações e não movimenta o robô.
"""
import argparse
from collections import Counter
import datetime
import gc
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import rosbag2_py

TOPICS = set('''/scan_raw /scan_front /scan_nav /imu/data /odom /tf /tf_static
/map /map_updates /global_costmap/costmap_raw /local_costmap/costmap_raw
/global_costmap/costmap_raw_updates /local_costmap/costmap_raw_updates
/cmd_vel /cmd_vel_ctrl /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_hardware
/collision_monitor_state /collision_monitor_velocity_polygon
/collision_monitor_swept_turn /collision_monitor/collision_points_marker
/diagnostics /hardware/sensor_timing /rosout /plan /local_plan
/destravar/estado /alinhador/estado_t /ir_para_area/resultado
/navigate_to_pose/_action/status /follow_path/_action/status
/destravar/_action/status /virtual_wall/keepout_mask /virtual_wall/filter_info
'''.split())
REQUIRED = {'/scan_front', '/scan_nav', '/imu/data', '/odom', '/tf', '/tf_static', '/map'}


def verify(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    counts = Counter()
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        deserialize_message(data, types[topic])
        counts[topic] += 1
    return dict(counts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=180)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--verify', type=Path)
    args = parser.parse_args()
    if args.verify:
        print(json.dumps(verify(args.verify), indent=2))
        return 0
    if args.seconds <= 0:
        parser.error('--seconds deve ser positivo')
    path = args.output or Path.home() / 'bags' / ('curva_diag_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    if path.exists():
        parser.error('Destino já existe; não será sobrescrito')
    path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('gravador_diagnostico_curva')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    subscriptions = {}
    counts = Counter()
    last_received = {}
    last_header = {}
    started = time.monotonic()

    def discover():
        for topic, types in node.get_topic_names_and_types():
            if topic not in TOPICS or topic in subscriptions or len(types) != 1:
                continue
            endpoints = node.get_publishers_info_by_topic(topic)
            if not endpoints:
                continue
            # Adapta reliability aos publicadores; reliable recupera dados latched. Retém dados
            # latched só quando há publicadores transient local (inclui /tf_static).
            durability = (DurabilityPolicy.TRANSIENT_LOCAL if any(
                e.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for e in endpoints)
                else DurabilityPolicy.VOLATILE)
            reliability = (ReliabilityPolicy.RELIABLE if all(
                e.qos_profile.reliability == ReliabilityPolicy.RELIABLE for e in endpoints)
                else ReliabilityPolicy.BEST_EFFORT)
            qos = QoSProfile(depth=100, reliability=reliability,
                             durability=durability)
            cls = get_message(types[0])
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=len(subscriptions)+1, name=topic, type=types[0], serialization_format='cdr',
                offered_qos_profiles=[
                    (rosbag2_py._storage.QoS(100).best_effort()
                     if e.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                     else rosbag2_py._storage.QoS(100).reliable()).transient_local()
                    if e.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                    else (rosbag2_py._storage.QoS(100).best_effort()
                          if e.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                          else rosbag2_py._storage.QoS(100).reliable()).durability_volatile()
                    for e in endpoints]))

            def receive(data, topic=topic, cls=cls):
                now = node.get_clock().now().nanoseconds
                writer.write(topic, data, now)
                counts[topic] += 1
                last_received[topic] = time.monotonic()
                msg = deserialize_message(data, cls)
                if hasattr(msg, 'header'):
                    last_header[topic] = now / 1e9 - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9

            subscriptions[topic] = node.create_subscription(cls, topic, receive, qos, raw=True)

    error = None
    try:
        next_discovery = next_report = 0
        print('Gravando:', path, flush=True)
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            now = time.monotonic()
            if now >= next_discovery:
                discover()
                next_discovery = now + 1
            rclpy.spin_once(node, timeout_sec=.05)
            if now >= next_report:
                missing = sorted(t for t in REQUIRED if t not in counts or (t not in {'/map', '/tf_static'} and now - last_received.get(t, 0) > 1))
                print('Mensagens:', sum(counts.values()), '| sem dados recentes:', missing, flush=True)
                next_report = now + 5
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        error = repr(exc)
    finally:
        recording_end = time.monotonic()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # SequentialWriter fecha e cria metadata.yaml no destrutor nesta versão.
        del writer
        gc.collect()
    disk = verify(path)
    missing = sorted(REQUIRED - disk.keys())
    stale = sorted(t for t in REQUIRED - {'/tf_static', '/map'} if recording_end - last_received.get(t, 0) > 1 or last_header.get(t, 0) > 1)
    summary = dict(path=str(path), duration_s=recording_end-started, verification_s=time.monotonic()-recording_end,
                   counts=dict(counts), reread_counts=disk, missing_required=missing,
                   header_age_s=last_header, stale_required=stale, error=error,
                   integrity_ok=disk == dict(counts),
                   complete_for_curve=not missing and not stale and disk == dict(counts) and error is None)
    path.joinpath('verificacao.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary['complete_for_curve'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
