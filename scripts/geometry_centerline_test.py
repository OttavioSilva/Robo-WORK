#!/usr/bin/env python3
"""Teste local, reversivel, do planejador Smac2D priorizando folga lateral.

Uso:
  python3 geometry_centerline_test.py --apply
  python3 geometry_centerline_test.py --status
  python3 geometry_centerline_test.py --restore
  python3 geometry_centerline_test.py --compare

Nao envia comandos ao robo; apenas altera YAML e BT, com backups.
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:
    sys.exit("PyYAML indisponivel. Instale python3-yaml antes de continuar.")

ROOT = Path.home() / "robo_jazzy_ws"
CONFIG = ROOT / "src/robo_navigation/config/nav2_params.yaml"
BT = ROOT / "src/robo_navigation/behavior_trees/navigate_to_pose_corridor.xml"
LOGS = ROOT / "logs"
MANIFEST = LOGS / "geometry_centerline_last_backup.txt"

CENTERLINE = """    Centerline:
      plugin: nav2_smac_planner::SmacPlanner2D
      tolerance: 0.10
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_planning_time: 2.0
      cost_travel_multiplier: 3.0
      use_final_approach_orientation: false
"""


def read_config():
    source = CONFIG.read_text(encoding="utf-8")
    data = yaml.safe_load(source)
    if not isinstance(data, dict):
        raise RuntimeError("YAML inesperado ou vazio")
    return source, data


def params_of(data):
    return data["planner_server"]["ros__parameters"]


def bt_default(xml):
    root = ET.fromstring(xml)
    elems = root.findall(".//PlannerSelector")
    if len(elems) != 1:
        raise RuntimeError("Esperado exatamente um PlannerSelector na arvore")
    return elems[0].get("default_planner")


def planner_block(text):
    match = re.search(r"(?m)^planner_server:\s*$", text)
    if not match:
        raise RuntimeError("Secao planner_server nao encontrada")
    next_top = re.search(
        r"(?m)^[A-Za-z_][\w]*:\s*$", text[match.end():]
    )
    end = match.end() + next_top.start() if next_top else len(text)
    return match.start(), end, text[match.start():end]


def atomic_write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".geometry_center_", dir=path.parent)
    try:
        os.fchmod(fd, path.stat().st_mode & 0o777 if path.exists() else 0o644)
        with os.fdopen(fd, "w", encoding="utf-8") as file:
            file.write(content)
            file.flush()
            os.fsync(file.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def plugin_available():
    if shutil.which("ros2") is None:
        return False
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "nav2_smac_planner"],
            capture_output=True, text=True, timeout=10, check=False,
        )
        return result.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def status():
    text, data = read_config()
    xml = BT.read_text(encoding="utf-8")
    planners = params_of(data).get("planner_plugins", [])
    print("Planner plugins:", planners)
    for name in planners:
        p = params_of(data).get(name, {})
        print("  %s: %s" % (name, p.get("plugin", "DESCONHECIDO")))
        if "cost_travel_multiplier" in p:
            print("    cost_travel_multiplier:", p["cost_travel_multiplier"])
    print("BT default_planner:", bt_default(xml))
    print("Pacote nav2_smac_planner disponivel:", plugin_available())
    print("Backups:", MANIFEST.read_text().strip() if MANIFEST.exists() else "ainda nao gerados")


def apply():
    if not plugin_available():
        raise RuntimeError(
            "nav2_smac_planner nao encontrado neste ambiente ROS. "
            "Verifique: source /opt/ros/jazzy/setup.bash; "
            "ros2 pkg prefix nav2_smac_planner. Nenhum arquivo alterado."
        )
    original, data = read_config()
    original_xml = BT.read_text(encoding="utf-8")
    params = params_of(data)
    if params.get("planner_plugins") != ["GridBased"]:
        raise RuntimeError("Planner configurado diferente do esperado. Nada alterado.")
    if params.get("GridBased", {}).get("plugin") != "nav2_navfn_planner::NavfnPlanner":
        raise RuntimeError("GridBased nao esta usando NavFn. Nada alterado.")
    if bt_default(original_xml) != "GridBased":
        raise RuntimeError("BT ja nao usa GridBased como default. Nada alterado.")
    start, end, segment = planner_block(original)
    updated_segment, count = re.subn(
        r"(?m)^([ \t]*)- GridBased[ \t]*$",
        lambda m: m.group(1) + "- GridBased\n" + m.group(1) + "- Centerline",
        segment, count=1,
    )
    if count != 1:
        raise RuntimeError("Nao foi possivel localizar a lista de planners")
    updated_segment = updated_segment.rstrip() + "\n" + CENTERLINE
    new_config = original[:start] + updated_segment + original[end:]
    new_data = yaml.safe_load(new_config)
    new_params = params_of(new_data)
    if (new_params.get("planner_plugins") != ["GridBased", "Centerline"]
            or new_params["Centerline"]["cost_travel_multiplier"] != 3.0
            or new_params["GridBased"] != params["GridBased"]):
        raise RuntimeError("YAML de teste nao validou; nada alterado")

    new_xml, count = re.subn(
        r'(<PlannerSelector\b[^>]*\bdefault_planner=")GridBased(")',
        r'\g<1>Centerline\2', original_xml, count=1,
    )
    if count != 1 or bt_default(new_xml) != "Centerline":
        raise RuntimeError("XML de teste nao validou; nada alterado")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dest = LOGS / ("geometry_centerline_backup_" + stamp)
    dest.mkdir(parents=True, exist_ok=False)
    (dest / CONFIG.name).write_text(original, encoding="utf-8")
    (dest / BT.name).write_text(original_xml, encoding="utf-8")
    try:
        atomic_write(CONFIG, new_config)
        atomic_write(BT, new_xml)
        atomic_write(MANIFEST, str(dest) + "\n")
    except Exception:
        atomic_write(CONFIG, original)
        atomic_write(BT, original_xml)
        raise
    print("TESTE CENTERLINE APLICADO — sem mover o robo.")
    print("Backup:", dest)
    print("NavFn preservado; Centerline selecionado na BT.")
    print("Reinicie SOMENTE o Nav2 para carregar os plugins.")
    print("Para desfazer: python3 geometry_centerline_test.py --restore")


def restore():
    if not MANIFEST.exists():
        raise RuntimeError("Nao existe backup registrado; nenhum arquivo modificado")
    dest = Path(MANIFEST.read_text(encoding="utf-8").strip())
    original = (dest / CONFIG.name).read_text(encoding="utf-8")
    original_xml = (dest / BT.name).read_text(encoding="utf-8")
    yaml.safe_load(original)
    bt_default(original_xml)
    now, data = read_config()
    now_xml = BT.read_text(encoding="utf-8")
    if ("Centerline" not in params_of(data).get("planner_plugins", [])
            or bt_default(now_xml) != "Centerline"):
        raise RuntimeError(
            "Configuracao atual divergiu do teste Centerline. "
            "Restaure manualmente os backups em %s" % dest
        )
    atomic_write(CONFIG, original)
    atomic_write(BT, original_xml)
    print("ORIGINAIS RESTAURADOS de:", dest)
    print("Reinicie SOMENTE o Nav2 para voltar ao NavFn original.")


def compare():
    """Calcula NavFn e Smac2D ao mesmo destino; publica Path sem navegar."""
    import math
    import time

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from geometry_msgs.msg import PointStamped, PoseStamped
    from nav_msgs.msg import Path as RosPath
    from tf2_ros import Buffer, TransformListener
    from nav2_simple_commander.robot_navigator import BasicNavigator

    _source, data = read_config()
    if "Centerline" not in params_of(data).get("planner_plugins", []):
        raise RuntimeError("Execute --apply e reinicie Nav2 antes do comparativo")

    rclpy.init()
    nav = BasicNavigator()
    buffer = Buffer()
    listener = TransformListener(buffer, nav)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    original_pub = nav.create_publisher(RosPath, "/geometry_plan_navfn", qos)
    centered_pub = nav.create_publisher(RosPath, "/geometry_plan_centerline", qos)
    clicked = [None]

    def on_click(msg):
        if clicked[0] is None:
            clicked[0] = msg

    sub = nav.create_subscription(PointStamped, "/clicked_point", on_click, 10)

    def make_pose(x, y, orientation):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = nav.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = orientation
        return pose

    def stats(path):
        dist = sum(math.hypot(b.pose.position.x - a.pose.position.x,
                              b.pose.position.y - a.pose.position.y)
                   for a, b in zip(path.poses[:-1], path.poses[1:]))
        return len(path.poses), dist

    try:
        print("RViz: use Publish Point e clique em um destino mapeado apos a quina.",
              flush=True)
        print("Nao use Nav2 Goal neste comparativo: somente Publish Point.",
              flush=True)
        deadline = time.monotonic() + 90
        while clicked[0] is None and time.monotonic() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.2)
        if clicked[0] is None:
            raise RuntimeError("Nenhum ponto recebido em /clicked_point em 90 s")
        if clicked[0].header.frame_id != "map":
            raise RuntimeError("O RViz precisa publicar ponto em frame 'map'")
        deadline = time.monotonic() + 12
        tf = None
        while time.monotonic() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.1)
            try:
                tf = buffer.lookup_transform("map", "base_link", Time())
                break
            except Exception:
                pass
        if tf is None:
            raise RuntimeError("Sem TF map -> base_link: nenhum planejamento")

        origin = make_pose(tf.transform.translation.x,
                           tf.transform.translation.y, tf.transform.rotation)
        goal = make_pose(clicked[0].point.x, clicked[0].point.y,
                         tf.transform.rotation)
        print("Inicio: (%.3f, %.3f)  Meta: (%.3f, %.3f)" % (
            origin.pose.position.x, origin.pose.position.y,
            goal.pose.position.x, goal.pose.position.y), flush=True)
        print("Calculando NavFn original...", flush=True)
        old = nav.getPath(origin, goal, planner_id="GridBased", use_start=True)
        print("Calculando Smac Centerline...", flush=True)
        new = nav.getPath(origin, goal, planner_id="Centerline", use_start=True)
        if old is None or not old.poses:
            print("NavFn: SEM CAMINHO valido para esta meta.")
        else:
            n, d = stats(old)
            print("NavFn: %d poses, percurso %.2f m" % (n, d))
            original_pub.publish(old)
        if new is None or not new.poses:
            print("Centerline: SEM CAMINHO valido para esta meta.")
        else:
            n, d = stats(new)
            print("Centerline: %d poses, percurso %.2f m" % (n, d))
            centered_pub.publish(new)
        if (old is None or not old.poses) and (new is None or not new.poses):
            print("Ambos recusaram: investigar meta, area desconhecida e costmap.")
            return
        print("RViz > Add > By topic > Path: /geometry_plan_navfn e "
              "/geometry_plan_centerline.")
        print("Deixe o processo ligado para visualizar. Ctrl+C para encerrar.")
        while rclpy.ok():
            rclpy.spin_once(nav, timeout_sec=0.5)
    except KeyboardInterrupt:
        print("Comparacao finalizada; sem comando de movimento.")
    finally:
        nav.destroy_subscription(sub)
        nav.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--apply", action="store_true")
    group.add_argument("--status", action="store_true")
    group.add_argument("--restore", action="store_true")
    group.add_argument("--compare", action="store_true")
    args = parser.parse_args()
    try:
        if args.apply:
            apply()
        elif args.restore:
            restore()
        elif args.compare:
            compare()
        else:
            status()
    except (KeyError, ValueError, OSError, RuntimeError, ET.ParseError) as exc:
        sys.exit("ERRO (nenhuma mudanca intencional): " + str(exc))


if __name__ == "__main__":
    main()
