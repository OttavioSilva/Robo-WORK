#!/usr/bin/env bash
# ABRIR_RESUMIDO (23/11): 3 terminais - NAVEGACAO (base+arena em localizacao + Nav2 legado com fita) | GARRA | RVIZ.
# Nao manda goal nem velocidade. Garra: plataforma Y sem centralizar (CENTRALIZAR vazio).
LOG=~/logs/resumido_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"
AMB="source /opt/ros/jazzy/setup.bash; source ~/robo_jazzy_ws/install/setup.bash; export ROS_DOMAIN_ID=0 PYTHONUNBUFFERED=1"
if pgrep -f "[s]lam_toolbox|[h]ardware_bridge_node|[b]t_navigator|[m]ove_group" >/dev/null; then echo "!!! Ja tem robo/garra rodando"; exit 1; fi
gnome-terminal --title="NAVEGACAO" -- bash -c "$AMB; ros2 launch ~/robo_jazzy_ws/ferramentas/subida_unica/base_arena_localizacao.launch.py > $LOG/base.log 2>&1 & sleep 8; bash ~/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.sh 2>&1 | tee $LOG/nav2.log; exec bash"
echo ">>> Esperando o alinhador (ate 240 s)..."
for i in $(seq 120); do pgrep -f "[a]linhador_passagem" >/dev/null && break; sleep 2; done
gnome-terminal --title="GARRA" -- bash -c "CENTRALIZAR= GARRA_PORTA=/dev/serial/by-path/pci-0000:00:14.0-usb-0:3.3:1.0-port0 bash ~/robo_jazzy_ws/ferramentas/subida_unica/subir_garra.sh; exec bash"
sleep 3
gnome-terminal --title="RVIZ" -- bash -c "bash ~/logs/reabertura_20261116_100105/rviz.sh 2>&1 | tee $LOG/rviz.log; exec bash"
echo "$LOG"
