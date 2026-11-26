#!/usr/bin/env bash
# ARENA_FUSION (25/11): arena de teste (arena_cotada-3.pdf) com o mapa DA PLANTA + AMCL, LiDAR M2M2.
# Terminais: PONTE+M2M2 (sem mapa/TF do Mapper) | AMCL (map_server + vigia + bloqueios) | NAV2 | RVIZ.
# Robo no START: dentro da caixa do canto de baixo a direita, centrado, de frente para CIMA (para dentro da arena).
# Troca ~/mapas/service_areas.yaml e pontos.yaml pelos da Fusion (backup .bak_fusion_<hora>).
set -e
F=~/mapas/arena_fusion
LOG=~/logs/fusion_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"
AMB="source /opt/ros/jazzy/setup.bash; source ~/robo_jazzy_ws/install/setup.bash; source ~/m2m2_ws/install/setup.bash; export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1"
if pgrep -f "[s]lam_toolbox|[h]ardware_bridge_node|[b]t_navigator|[m]2m2_bridge|[a]mcl " >/dev/null; then echo "!!! Ja tem robo rodando. Feche antes."; exit 1; fi
if pgrep -f "[s]imular.py|[g]z sim" >/dev/null; then echo "!!! Simulacao rodando. Feche antes."; exit 1; fi
timeout 2 bash -c "echo > /dev/tcp/192.168.11.1/1445" 2>/dev/null || { echo "!!! M2M2 nao responde em 192.168.11.1:1445"; exit 1; }
for f in $F/arena.yaml $F/arena_fusion_pontos.yaml $F/arena_fusion_service_areas.yaml; do [ -f $f ] || { echo "!!! falta $f"; exit 1; }; done
H=$(date +%H%M%S)
for n in service_areas pontos; do
  cp ~/mapas/$n.yaml ~/mapas/$n.yaml.bak_fusion_$H
  cp $F/arena_fusion_$n.yaml ~/mapas/$n.yaml
done
read X Y TH < <(python3 -c "import yaml;p=yaml.safe_load(open('$F/arena_fusion_pontos.yaml'))['pontos']['START'];print(p['x'],p['y'],p['yaw'])")
echo ">>> START ($X, $Y, $TH rad). Areas: $(python3 -c "import yaml;print(', '.join(yaml.safe_load(open('$F/arena_fusion_service_areas.yaml'))['service_areas']))")"
gnome-terminal --title="PONTE + M2M2 (fusion)" -- bash -c "$AMB; ros2 launch ~/robo_jazzy_ws/ferramentas/subida_unica/base_m2m2_fusion.launch.py 2>&1 | tee $LOG/base.log; exec bash"
sleep 10
gnome-terminal --title="AMCL (fusion)" -- bash -c "$AMB; ros2 launch robo_navigation localizacao_amcl.launch.py mapa:=$F/arena.yaml x:=$X y:=$Y yaw:=$TH 2>&1 | tee $LOG/amcl.log; exec bash"
sleep 6
gnome-terminal --title="NAV2" -- bash -c "$AMB; bash ~/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.sh 2>&1 | tee $LOG/nav2.log; exec bash"
sleep 3
gnome-terminal --title="RVIZ" -- bash -c "$AMB; bash ~/logs/reabertura_20261116_100105/rviz.sh 2>&1 | tee $LOG/rviz.log; exec bash"
echo "Logs: $LOG"
echo "Para voltar o ~/mapas: cp ~/mapas/service_areas.yaml.bak_fusion_$H ~/mapas/service_areas.yaml; cp ~/mapas/pontos.yaml.bak_fusion_$H ~/mapas/pontos.yaml"
