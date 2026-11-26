#!/usr/bin/env bash
# SUBIR_ROBO v1.1 (05/11/2026): sobe o robo inteiro num terminal so, na ordem certa.
#
#   ponte (robo_hardware) -> filtro do LiDAR (robo_lidar) -> localizacao -> SLAM ->
#   carrega o mapa no START -> Nav2 (com suavizador, filtro do corpo, destravar e alinhador,
#   pelo NAV_TUDO)
#
# Cada parte espera a anterior ficar pronta. Os logs completos vao para ~/logs/<hora>/; aqui
# aparecem so as linhas importantes (alinhador, executor, destravar, avisos e erros).
# Ctrl+C para tudo, na ordem inversa.
#
# Uso (com o robo parado no START, virado como quando o START foi marcado):
#   ~/subir_robo.sh                 # mapa mais recente (~/mapas/arena_*.posegraph)
#   ~/subir_robo.sh arena_04-11_1235   # um mapa especifico (nome sem extensao)
#   ~/subir_robo.sh --novo          # sem mapa: SLAM do zero (para mapear)
set +u
WS=~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
LOG=~/logs/$(date +%d-%m_%H%M%S); mkdir -p "$LOG"
PIDS=()

parar_tudo() {
  trap - INT TERM EXIT
  echo; echo ">>> Parando tudo..."
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do kill -INT -- -"${PIDS[$i]}" 2>/dev/null; done
  sleep 4
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do kill -TERM -- -"${PIDS[$i]}" 2>/dev/null; done
  sleep 1
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do kill -KILL -- -"${PIDS[$i]}" 2>/dev/null; done
  echo ">>> Parado. Logs em $LOG"
  exit 0
}
trap parar_tudo INT TERM EXIT

sobe() {   # nome, comando...
  local nome=$1; shift
  echo ">>> $nome: $*"
  setsid "$@" > "$LOG/$nome.log" 2>&1 &
  PIDS+=($!)
}

espera_topico() {   # topico, segundos, nome
  local fim=$((SECONDS + $2))
  while (( SECONDS < fim )); do
    if timeout 3 ros2 topic echo --once "$1" > /dev/null 2>&1; then echo "    ok: $1"; return 0; fi
  done
  echo "!!! $3: nao vi $1 em $2 s. Veja $LOG/$3.log"; tail -5 "$LOG/$3.log"; return 1
}

espera_servico() {   # servico, segundos, nome
  local fim=$((SECONDS + $2))
  while (( SECONDS < fim )); do
    ros2 service list 2>/dev/null | grep -qx "$1" && { echo "    ok: $1"; return 0; }
    sleep 1
  done
  echo "!!! $3: nao vi o servico $1 em $2 s. Veja $LOG/$3.log"; return 1
}

MAPA=""
if [ "${1:-}" != "--novo" ]; then
  if [ -n "${1:-}" ]; then MAPA=~/mapas/$1; else MAPA=$(ls -t ~/mapas/arena_*.posegraph 2>/dev/null | head -1 | sed 's/\.posegraph$//'); fi
  [ -f "$MAPA.posegraph" ] || { echo "!!! Nao achei $MAPA.posegraph"; exit 1; }
  [ -f "${MAPA}_pontos.yaml" ] || { echo "!!! Nao achei ${MAPA}_pontos.yaml (START do mapa)"; exit 1; }
  echo ">>> Mapa: $MAPA ($(du -h "$MAPA.posegraph" | cut -f1))"
fi

sobe ponte ros2 launch robo_hardware hardware.launch.py
espera_topico /imu/data 40 ponte || exit 1
sobe lidar ros2 launch robo_lidar lidar_filter.launch.py
espera_topico /scan_front 30 lidar || exit 1
sobe localizacao ros2 launch robo_localization localization.launch.py
espera_topico /odom 30 localizacao || exit 1
# SLAM: se a localizacao ja sobe o slam_toolbox, usa ele; senao sobe o mapping.launch.py
if espera_servico /slam_toolbox/deserialize_map 8 localizacao 2>/dev/null; then
  echo "    (o slam_toolbox ja subiu junto com a localizacao)"
else
  ML=$(find $WS/src -path "*/launch/mapping.launch.py" 2>/dev/null | head -1)
  [ -n "$ML" ] || { echo "!!! Nao achei o mapping.launch.py em $WS/src"; exit 1; }
  PKG=$(basename "$(dirname "$(dirname "$ML")")")
  sobe slam ros2 launch "$PKG" mapping.launch.py
  espera_servico /slam_toolbox/deserialize_map 40 slam || exit 1
fi

if [ -n "$MAPA" ]; then
  cp "${MAPA}_service_areas.yaml" ~/mapas/service_areas.yaml 2>/dev/null && echo "    mesas: ${MAPA}_service_areas.yaml"
  cp "${MAPA}_pontos.yaml" ~/mapas/pontos.yaml
  read X Y TH < <(python3 -c "import yaml;p=yaml.safe_load(open('${MAPA}_pontos.yaml'))['pontos']['START'];print(p['x'],p['y'],p['yaw'])")
  echo ">>> Carregando o mapa no START ($X, $Y, $TH rad)"
  ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
    "{filename: '$MAPA', match_type: 2, initial_pose: {x: $X, y: $Y, theta: $TH}}" > "$LOG/mapa.log" 2>&1 \
    || { echo "!!! Falhou carregar o mapa (veja $LOG/mapa.log)"; exit 1; }
  sleep 3
fi
espera_topico /map 30 slam || exit 1

sobe nav2 ros2 launch robo_navigation navigation.launch.py use_sim_time:=false autostart:=true
fim=$((SECONDS + 90)); ok=0
while (( SECONDS < fim )); do
  ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active" && { ok=1; break; }
  sleep 2
done
(( ok )) || { echo "!!! O Nav2 nao ficou ativo em 90 s. Veja $LOG/nav2.log"; tail -15 "$LOG/nav2.log"; exit 1; }
grep -q "NAV_TUDO" $WS/src/robo_navigation/launch/navigation.launch.py \
  || echo "    (sem NAV_TUDO: rode o filtro, o destravar e o alinhador a mao)"
sleep 3
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1

echo
echo "================ ROBO PRONTO ================"
echo " Confira no RViz se o scan bate com o mapa."
echo " Missao noutro terminal:  ~/missao.sh missao17 \"descricao\""
echo " Ctrl+C aqui para tudo. Logs: $LOG"
echo "============================================="
tail -n0 -F "$LOG/nav2.log" 2>/dev/null | grep --line-buffered -E "alinhador|executor|destravar|DESTRAVAR|filtro_corpo|ERROR|FATAL|process has died"
