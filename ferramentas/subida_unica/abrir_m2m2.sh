#!/usr/bin/env bash
# ABRIR_M2M2 (24/11): PONTE (base + M2M2) | NAV2 | RVIZ | TELEOP em 4 terminais.
# Mapa do M2M2 do zero (como o subir_tudo). Para localizar num mapa salvo:
#   M2M2_ARGS="m2m2_mapa:=$HOME/mapas/NOME.stcm m2m2_carregar:=true m2m2_limpar:=false m2m2_mapeando:=false" abrir_m2m2.sh
# TELEOP fica esperando o controle (/dev/input/js*) e reabre sozinho se ele cair ou o script sair.
LOG=~/logs/m2m2_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"
# bash -c nao le o ~/.bashrc: o ambiente vai explicito (SO_LOCALHOST depois dos source)
AMB="source /opt/ros/jazzy/setup.bash; source ~/robo_jazzy_ws/install/setup.bash; source ~/m2m2_ws/install/setup.bash; export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1"
if pgrep -f "[s]lam_toolbox|[h]ardware_bridge_node|[b]t_navigator|[m]2m2_bridge" >/dev/null; then echo "!!! Ja tem robo rodando. Feche antes."; exit 1; fi
timeout 2 bash -c "echo > /dev/tcp/192.168.11.1/1445" 2>/dev/null || { echo "!!! M2M2 nao responde em 192.168.11.1:1445 (desligue/religue o M2M2)"; exit 1; }
gnome-terminal --title="PONTE + M2M2" -- bash -c "$AMB; ros2 launch ~/robo_jazzy_ws/ferramentas/subida_unica/base_m2m2.launch.py ${M2M2_ARGS:-} 2>&1 | tee $LOG/base.log; exec bash"
sleep 8
gnome-terminal --title="NAV2" -- bash -c "$AMB; bash ~/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.sh 2>&1 | tee $LOG/nav2.log; exec bash"
sleep 3
gnome-terminal --title="RVIZ" -- bash -c "$AMB; bash ~/logs/reabertura_20261116_100105/rviz.sh 2>&1 | tee $LOG/rviz.log; exec bash"
gnome-terminal --title="TELEOP" -- bash -c "$AMB; while true; do
  if ! ls /dev/input/js* >/dev/null 2>&1; then echo '>>> Esperando o controle (USB/Bluetooth)...'; until ls /dev/input/js* >/dev/null 2>&1; do sleep 1; done; sleep 1; fi
  echo '>>> Controle conectado: abrindo o teleop'; python3 -u ~/robo_jazzy_ws/ferramentas/teleop/teleop_mapeamento_joy.py 2>&1 | tee -a $LOG/teleop.log
  echo '>>> teleop saiu; reabrindo em 2 s (Ctrl+C aqui para parar de vez)'; sleep 2
done; exec bash"
echo "Logs: $LOG"
