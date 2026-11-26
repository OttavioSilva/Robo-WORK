#!/usr/bin/env bash
# CAMINHO_ORIGINAL (11/11/2026): sobe SO um bt_navigator (+ servidores falsos que rejeitam tudo) num dominio isolado (166, so localhost),
# configura e ativa, e confere se o plugin robo_bt_plugins e a arvore carregaram. Sem controller/planner,
# sem goal, sem cmd_vel: nada se move. Uso: bash ferramentas/testes/conferir_bt_navigator_isolado.sh
# (sem set -u: o setup.bash do ROS usa variaveis nao definidas)
WS=$(cd "$(dirname "$0")/../.." && pwd)
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
XML=${BT_XML:-$WS/install/robo_navigation/share/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml}
PARAMS=${BT_PARAMS:-$WS/src/robo_navigation/config/nav2_params.yaml}
LOG=$(mktemp)
python3 "$WS/ferramentas/testes/servidores_falsos_bt.py" > /dev/null 2>&1 &
FALSOS=$!
/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator --ros-args --params-file "$PARAMS" \
  -p default_nav_to_pose_bt_xml:="$XML" > "$LOG" 2>&1 &
PID=$!
sleep 6
for tr in configure activate; do
  for k in 1 2 3 4 5; do
    timeout 20 ros2 lifecycle set /bt_navigator $tr 2>&1 | grep -q "successful" && break
    sleep 2
  done
  sleep 2
done
ESTADO=$(timeout 15 ros2 lifecycle get /bt_navigator 2>&1)
P=$(timeout 15 ros2 param get /bt_navigator plugin_lib_names 2>&1)
timeout 10 ros2 topic pub --once --qos-durability transient_local /caminho_original/modo std_msgs/msg/String "{data: aplicar}" > /dev/null 2>&1
ros2 daemon stop > /dev/null 2>&1
kill -INT $PID $FALSOS; sleep 2; kill -9 $FALSOS 2>/dev/null; kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null
echo "estado: $ESTADO"; echo "plugin_lib_names: $P"; 
grep -iE "error|fail|exception|CaminhoOriginal|not recognized|not available|timed out" "$LOG" | head -20; cp "$LOG" "${CONF_LOG:-/dev/null}"
case "$ESTADO" in active*) echo "RESULTADO: OK (arvore e plugin carregados)"; exit 0;; *) echo "RESULTADO: FALHOU"; cat "$LOG" | tail -30; exit 1;; esac
