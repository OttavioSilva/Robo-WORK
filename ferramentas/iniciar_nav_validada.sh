#!/usr/bin/env bash
# Partida manual, sem goals. Encerrar a instancia anterior do Nav2 antes de executar.
set -eo pipefail
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS/install_nav2_principal/setup.bash"
ESPERADO="$WS/install_nav2_principal/robo_navigation"
RESOLVIDO=$(ros2 pkg prefix robo_navigation)
if [ "$(realpath "$RESOLVIDO")" != "$(realpath "$ESPERADO")" ]; then
    echo "RECUSADO: robo_navigation veio de $RESOLVIDO, esperado $ESPERADO" >&2
    exit 1
fi
SHARE="$RESOLVIDO/share/robo_navigation"
CFG="$SHARE/config/nav2_params_principal.yaml"
BT="$SHARE/behavior_trees/navigate_to_pose_principal.xml"
python3 - "$CFG" "$BT" <<'PY'
import sys,yaml,xml.etree.ElementTree as ET
p=yaml.safe_load(open(sys.argv[1]))
assert p['controller_server']['ros__parameters']['FollowPath']['plugin']=='dwb_core::DWBLocalPlanner'
assert p['controller_server']['ros__parameters']['FollowPath']['min_vel_x']==-.10
assert 'forward_veloz' not in p['collision_monitor']['ros__parameters']['VelocityPolygonStop']
t=ET.parse(sys.argv[2]);assert t.find('.//CaminhoSemCarimbo') is not None
assert len(list(t.iter('SmoothPath')))==2
PY
printf 'Nav2 validado: %s\nParametros: %s\nArvore: %s\n' "$RESOLVIDO" "$CFG" "$BT"
exec ros2 launch "$SHARE/launch/navigation.launch.py" nav2_principal:=true "params_file:=$CFG" "bt_xml:=$BT"
