#!/usr/bin/env bash
# ABRIR_ARENA (19/11): 3 terminais - base unificada COM arena_11-11_1938 em LOCALIZACAO (mapa fixo) (ponte, LiDAR, localizacao, SLAM, parede, mesas, mapa) | Nav2 legado com fita | RViz.
LOG=~/logs/arena_$(date +%Y%m%d_%H%M%S); mkdir -p "$LOG"
# PAREDES_SALVAS (21/11): --gravar-fitas comeca sem paredes e grava ~/mapas/<mapa>_fitas.json;
# sem a opcao, as fitas gravadas sao carregadas e o arquivo nao e alterado.
MODO_FITAS=load; [[ "${1:-}" == --gravar-fitas ]] && MODO_FITAS=record
echo ">>> Fitas: $MODO_FITAS (record = gravando do zero; load = usando as gravadas)"
AMB="source /opt/ros/jazzy/setup.bash; source ~/robo_jazzy_ws/install/setup.bash; export ROS_DOMAIN_ID=0 PYTHONUNBUFFERED=1 PAREDES_FITA_MODO=$MODO_FITAS"
if pgrep -f "[s]lam_toolbox|[h]ardware_bridge_node|[b]t_navigator" >/dev/null; then echo "!!! Ja tem ponte/SLAM/Nav2 rodando"; exit 1; fi
gnome-terminal --title="BASE + ARENA" -- bash -c "$AMB; ros2 launch ~/robo_jazzy_ws/ferramentas/subida_unica/base_arena_localizacao.launch.py 2>&1 | tee $LOG/base.log; exec bash"
sleep 8
gnome-terminal --title="NAV2" -- bash -c "$AMB; bash ~/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.sh 2>&1 | tee $LOG/nav2.log; exec bash"
sleep 5
gnome-terminal --title="RVIZ" -- bash -c "bash ~/logs/reabertura_20261116_100105/rviz.sh 2>&1 | tee $LOG/rviz.log; exec bash"
echo "$LOG"
