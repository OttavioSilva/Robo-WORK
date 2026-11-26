#!/usr/bin/env bash
# Inicio manual; nenhum goal. Usar somente apos encerrar a instancia Nav2 anterior.
set -eo pipefail
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MODO=${1:-observar}
VALIDAR=${2:-}
case "$MODO" in observar|aplicar) ;; *) echo 'Use observar ou aplicar'; exit 1;; esac
source /opt/ros/jazzy/setup.bash
source "$WS/install_nav2_principal/setup.bash"
# SUBIDA_SUPERVISOR_20261111: validar overlay e arquivos antes de qualquer node.
ESPERADO="$WS/install_nav2_principal/robo_navigation"
RESOLVIDO=$(ros2 pkg prefix robo_navigation)
[ "$(realpath "$RESOLVIDO")" = "$(realpath "$ESPERADO")" ] || { echo "Overlay incorreto: $RESOLVIDO" >&2; exit 1; }
SHARE="$ESPERADO/share/robo_navigation"
python3 - "$SHARE/config/nav2_params_principal.yaml" "$SHARE/behavior_trees/navigate_to_pose_principal.xml" <<'PYCFG'
import sys,yaml,xml.etree.ElementTree as ET
cfg=yaml.safe_load(open(sys.argv[1]))
assert cfg['controller_server']['ros__parameters']['FollowPath']['plugin']=='dwb_core::DWBLocalPlanner'
ET.parse(sys.argv[2])
PYCFG
if [ "$MODO" = aplicar ]; then
 python3 - "$WS/src/robo_navigation/config/regioes_passagem.yaml" <<'PY'
import sys,yaml
r=yaml.safe_load(open(sys.argv[1]))['regioes']
assert r, 'Cadastrar e conferir regioes no mapa correto antes de aplicar'
for v in r:
 assert v['id'] and len(v['entrada'])==len(v['saida'])==2 and .67 <= v['largura'] <= .9
PY
fi
printf 'SUBIDA_SUPERVISOR_20261111: nav2_principal=true supervisor_modo=%s overlay=%s\n' "$MODO" "$RESOLVIDO"
if [ "$VALIDAR" = --validar ]; then exit 0; fi
[ -z "$VALIDAR" ] || { echo 'Segundo argumento deve ser --validar'; exit 1; }
NOS=$(timeout 8 ros2 node list --no-daemon) || { echo 'Descoberta ROS falhou; recusando iniciar'; exit 1; }
if echo "$NOS" | grep -Eq '^/(bt_navigator|alinhador_passagem|arbitro_nav_principal)$'; then
 echo 'Instancia de navegacao ja existe; recusando duplicar'; exit 1
fi
exec ros2 launch "$WS/src/robo_navigation/launch/navigation.launch.py" nav2_principal:=true "supervisor_modo:=$MODO" "params_file:=$SHARE/config/nav2_params_principal.yaml" "bt_xml:=$SHARE/behavior_trees/navigate_to_pose_principal.xml"
