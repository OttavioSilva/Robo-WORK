#!/usr/bin/env bash
# SUBIR_GARRA (23/11/2026): sobe a garra (repo Arthur-Demetrio/garra em ~/robo_work_teste/garra) NUM TERMINAL SO,
# ja ligada a navegacao LEGADA do robo. Rodar DEPOIS do subir_tudo.sh / abrir_arena.sh (robo ja de pe).
#   garra_real.launch.py (MoveIt + driver serial da garra, sem RViz)
#   pegar_cubo      (escuta /pegar_cubo e /entregar_cubo)
#   camera_cubo     (webcam + AprilTag, --apoio = altura do tampo das mesas)
#   plataforma_y    (eixo Y; centraliza em y=+0,110 ao subir)
#   alcance_cubo    (--plataforma; so age com /coleta/pedido do executor_prova.py --garra)
#   ajuste_base     (--sem-arbitro em /cmd_vel_nav: o legado nao tem arbitro; passa pelo suavizador e pelo CM)
# Depois: python3 ~/robo_jazzy_ws/src/robo_navigation/scripts/executor_prova.py <tarefa.yaml> --garra
# Variaveis: GARRA_PORTA (serial do Mega da garra), APOIO (altura da mesa, padrao 0.10), VALIDADE (s, padrao 3).
# Logs completos em ~/logs/garra_<data>/. Ctrl+C para tudo.
set +u
GARRA_WS=~/robo_work_teste/garra
LOG=~/logs/garra_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"
APOIO=${APOIO:-0.10}
VALIDADE=${VALIDADE:-3}
# CAM_RES (23/11): 1080p deixava o camera_cubo em ~160% de CPU (load 38) e travava tudo
export GARRA_CAM_RES=${GARRA_CAM_RES:-1280x720}
source /opt/ros/jazzy/setup.bash
source "$GARRA_WS/install/setup.bash"
export ROS_DOMAIN_ID=0 PYTHONUNBUFFERED=1
export GARRA_PORTA=${GARRA_PORTA:-/dev/serial/by-path/pci-0000:00:14.0-usb-0:3.3:1.0-port0}
PIDS=()
parar_tudo() {
  trap - INT TERM EXIT
  echo; echo ">>> Parando a garra..."
  # SIGTERM: processos em segundo plano (&) num bash nao interativo IGNORAM SIGINT (23/11)
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do kill -TERM -- -"${PIDS[$i]}" 2>/dev/null; done
  sleep 4
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do kill -KILL -- -"${PIDS[$i]}" 2>/dev/null; done
  echo ">>> Parado. Logs em $LOG"
}
trap parar_tudo INT TERM EXIT
sobe() { local nome=$1; shift; echo ">>> $nome: $*"; setsid "$@" > "$LOG/$nome.log" 2>&1 & PIDS+=($!); }

if pgrep -f "[m]ove_group|garra_coleta/(pegar_cubo|alcance_cubo|ajuste_base|camera_cubo)" >/dev/null; then
  echo "!!! Ja tem garra rodando. Feche antes:"; pgrep -af "[m]ove_group|garra_coleta/" | cut -c1-120
  trap - EXIT; exit 1
fi
if [ ! -e "$GARRA_PORTA" ]; then
  echo "!!! Porta da garra nao existe: $GARRA_PORTA"
  echo "    Portas vistas:"; ls -l /dev/serial/by-path/ 2>/dev/null | sed 's/^/      /' || echo "      (nenhuma)"
  echo "    Rode de novo com GARRA_PORTA=<porta do Mega da garra> (NAO a do Mega da base)."
  trap - EXIT; exit 1
fi
pgrep -f "[a]linhador_passagem" >/dev/null || echo "!!! AVISO: alinhador nao esta rodando (suba o robo antes; a coleta so age com /coleta/pedido)."

sobe garra ros2 launch garra_moveit_config garra_real.launch.py use_rviz:=false
echo ">>> Esperando o move_group (ate 90 s)..."
fim=$((SECONDS + 90)); ok=0
while (( SECONDS < fim )); do
  grep -q "You can start planning now" "$LOG/garra.log" 2>/dev/null && { ok=1; break; }
  sleep 2
done
(( ok )) || { echo "!!! move_group nao ficou pronto. Veja $LOG/garra.log"; tail -15 "$LOG/garra.log"; exit 1; }

sobe pegar ros2 launch garra_coleta pegar_cubo.launch.py
sobe camera ros2 launch garra_coleta camera_cubo.launch.py apoio:="$APOIO"
# CENTRALIZAR="" (vazio) = plataforma Y NAO se mexe ao subir (padrao 0.110 = junto da esquerda)
if [ -n "${CENTRALIZAR-0.110}" ]; then sobe plataforma ros2 run garra_coleta plataforma_y --centralizar "${CENTRALIZAR-0.110}"
else sobe plataforma ros2 run garra_coleta plataforma_y; fi
sobe alcance ros2 run garra_coleta alcance_cubo --plataforma --validade-cubo "$VALIDADE"
sobe base ros2 launch garra_coleta ajuste_base.launch.py sem_arbitro:=true topico_cmd:=/cmd_vel_nav
sleep 8
echo; echo "---- Conferencia ----"
for n in move_group pegar_cubo camera_cubo plataforma_y alcance_cubo ajuste_base; do
  ros2 node list 2>/dev/null | grep -qx "/$n" && echo "    ok: /$n" || echo "    !!! falta /$n"
done
echo
echo "================ GARRA PRONTA ================"
echo " Mesa (apoio) = $APOIO m. Agora, em outro terminal:"
echo "   python3 ~/robo_jazzy_ws/src/robo_navigation/scripts/executor_prova.py <tarefa.yaml> --garra"
echo " Ctrl+C aqui para a garra. Logs: $LOG"
echo "=============================================="
tail -n0 -F "$LOG"/*.log 2>/dev/null | grep --line-buffered -E "Coleta|Pedido|Cubo em|Garra:|Base:|Plataforma|Recebido|ERRO|AVISO|falhou|Pronto|ERROR|FATAL|process has died"
