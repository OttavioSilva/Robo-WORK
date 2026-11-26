#!/usr/bin/env bash
# NAV_INTEGRADA (10/11/2026): validacao OFFLINE completa. Nao publica no dominio do robo
# (ROS_DOMAIN_ID=166, descoberta desligada), nao manda goal nem comando de movimento.
#   bash ~/robo_jazzy_ws/ferramentas/testes/validar_nav_integrada.sh
set -eo pipefail
WS=$(cd "$(dirname "$0")/../.." && pwd)
NAV=$WS/src/robo_navigation
B=$WS/build/sondas_nav_integrada
OUT=$WS/entregas/nav_integrada_20261110/resultados
mkdir -p "$B" "$OUT"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
for s in dwb_probe native_navigation truncate_probe setblackboard; do
  echo "== compilando sonda $s"
  cmake -S "$NAV/test/sondas_nav2/$s" -B "$B/$s" -DCMAKE_BUILD_TYPE=Release > "$OUT/build_$s.log" 2>&1
  cmake --build "$B/$s" -j4 >> "$OUT/build_$s.log" 2>&1
done
echo "== pytest (robo_navigation/test)"
cd "$NAV"
SONDAS_NAV="$B" python3 -m pytest -p no:cacheprovider -q test --junitxml="$OUT/pytest.xml" 2>&1 | tee "$OUT/pytest.log"
