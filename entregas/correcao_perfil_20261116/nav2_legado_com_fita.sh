#!/usr/bin/env bash
# PERFIL_LEGADO_FITA_20261116: restaurar alinhador operacional. SEM_PAREDE_VIRTUAL 23/11: sem filtros de fita.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/marcos/robo_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
# SEM_PAREDE_VIRTUAL (23/11): sem filtros de fita (tape_keepout_filter/tape_inflation) nos costmaps;
# o nome do arquivo ficou "com_fita" so porque varios scripts apontam para ele.
python3 - <<'CONFIG'
from pathlib import Path
import yaml,xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
share=Path(get_package_share_directory('robo_navigation'))
assert share.resolve()==Path('/home/marcos/robo_jazzy_ws/install/robo_navigation/share/robo_navigation').resolve(),share
base=yaml.safe_load((share/'config/nav2_params.yaml').read_text())
for name in ('local_costmap','global_costmap'):
 assert not any('tape' in f for f in base[name][name]['ros__parameters'].get('filters',[])),name
assert base['controller_server']['ros__parameters']['FollowPath']['plugin']=='dwb_core::DWBLocalPlanner'
ET.parse(share/'behavior_trees/navigate_to_pose_smooth.xml')
Path('/home/marcos/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.yaml').write_text(yaml.safe_dump(base,sort_keys=False))
print('Perfil operacional original confirmado; SEM parede virtual (sem filtros de fita).')
CONFIG
python3 /home/marcos/robo_jazzy_ws/entregas/antecipacao_frontal_20261116/patch_horizonte_dwb.py --arquivo /home/marcos/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.yaml
python3 /home/marcos/robo_jazzy_ws/entregas/cm_lateral65_20261119/patch_cm_lateral65.py --arquivo /home/marcos/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.yaml  # CM_LATERAL65 19/11 noite
python3 /home/marcos/robo_jazzy_ws/ferramentas/patches/aplicar_mapa_limpo.py --nav2-yaml /home/marcos/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.yaml  # MAPA_LIMPO 21/11
[[ ${1:-} != --validar ]] || exit 0
echo 'Alinhador operacional: nav2_principal=false. Aguardando base/TF recentes.'
until python3 /home/marcos/logs/reabertura_20261116_100105/aguardar_base.py; do sleep 5; done
python3 /home/marcos/logs/reabertura_20261116_100105/ativar_giro.py &
exec ros2 launch /home/marcos/robo_jazzy_ws/src/robo_navigation/launch/navigation.launch.py nav2_principal:=false use_sim_time:=false autostart:=true params_file:=/home/marcos/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.yaml bt_xml:=/home/marcos/robo_jazzy_ws/install/robo_navigation/share/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml
