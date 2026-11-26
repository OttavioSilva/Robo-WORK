#!/usr/bin/env bash
# Nao inicia navegacao nem habilita motor. Envia um goal apenas apos preflight.
set -euo pipefail
if [[ $# != 3 ]]; then
  echo 'Uso: iniciar_teste_giro_t.sh X_DESTINO Y_DESTINO YAW_RAD' >&2
  exit 2
fi
if [[ ${SUPERVISAO_GIRO_T:-} != CONFIRMADA ]]; then
  echo 'Exige operador no E-stop e SUPERVISAO_GIRO_T=CONFIRMADA; nao habilita motores.' >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/robo_jazzy_ws/install/setup.bash"
set -u
python3 "$HOME/robo_jazzy_ws/ferramentas/verificar_giro_t.py" --seconds 5
payload=$(python3 - "$@" <<'PY'
import json,math,sys
x,y,yaw=map(float,sys.argv[1:])
assert all(math.isfinite(v) for v in (x,y,yaw))
print(json.dumps(dict(header=dict(frame_id='map'),pose=dict(position=dict(x=x,y=y,z=0.0),
    orientation=dict(x=0.0,y=0.0,z=math.sin(yaw/2),w=math.cos(yaw/2))))))
PY
)
ros2 topic pub --once /goal_pose_alinhado geometry_msgs/msg/PoseStamped "$payload"
