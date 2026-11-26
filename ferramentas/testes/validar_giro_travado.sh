#!/usr/bin/env bash
# GIRO_TRAVADO (11/11/2026): compila as sondas com o codigo real do Nav2 e roda a reproducao offline.
# Dominio isolado, sem servidores, sem goal, sem cmd_vel.   bash ferramentas/testes/validar_giro_travado.sh
WS=$(cd "$(dirname "$0")/../.." && pwd)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
B=$WS/build/sondas_nav_integrada/giro_travado
cmake -S "$WS/src/robo_navigation/test/sondas_nav2/giro_travado" -B "$B" -DCMAKE_BUILD_TYPE=Release > /dev/null && cmake --build "$B" -j4 > /dev/null || exit 1
cd "$WS/src/robo_navigation" && SONDAS_NAV=$WS/build/sondas_nav_integrada python3 -m pytest -p no:cacheprovider -v test/test_giro_travado.py
