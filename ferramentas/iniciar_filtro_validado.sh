#!/usr/bin/env bash
# Somente para partida manual: verificar prefixo/binario antes de iniciar filtros.
# Encerrar Nav2 e a instancia anterior do filtro antes de executar.
set -eo pipefail
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS/install_nav2_principal/setup.bash"
ESPERADO="$WS/install_nav2_principal/robo_lidar"
RESOLVIDO=$(ros2 pkg prefix robo_lidar)
if [ "$(realpath "$RESOLVIDO")" != "$(realpath "$ESPERADO")" ]; then
    echo "RECUSADO: robo_lidar veio de $RESOLVIDO, esperado $ESPERADO" >&2
    exit 1
fi
BINARIO="$RESOLVIDO/lib/robo_lidar/scan_filter_node"
if ! rg -q 'declare_parameter\("body_filter_enabled"' "$BINARIO"; then
    echo 'RECUSADO: executavel sem filtro corporal novo' >&2
    exit 1
fi
python3 - "$RESOLVIDO/share/robo_lidar/config/lidar.yaml" <<'PY'
import sys,yaml
p=yaml.safe_load(open(sys.argv[1]))['scan_filter']['ros__parameters']
assert p.get('body_filter_enabled') is True, 'RECUSADO: filtro corporal desligado'
assert p.get('blocked_intervals_deg') == '', 'RECUSADO: perfil diferente do validado'
PY
printf 'Filtro validado: %s\n' "$BINARIO"
exec ros2 launch "$RESOLVIDO/share/robo_lidar/launch/lidar_filter.launch.py"
