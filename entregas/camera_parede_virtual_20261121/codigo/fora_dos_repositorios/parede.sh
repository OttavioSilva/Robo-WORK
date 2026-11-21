source /opt/ros/jazzy/setup.bash
source /home/marcos/robo_jazzy_ws/install_nav2_principal/setup.bash
source /home/marcos/Robo-WORK/install_retirada_fita_20261113/robo_virtual_wall/share/robo_virtual_wall/package.bash
export ROS_DOMAIN_ID=0
v4l2-ctl -d /dev/v4l/by-id/usb-XIFT_Web_Camera_20250904.1549-video-index0 --set-ctrl=focus_automatic_continuous=0,focus_absolute=150 || exit 1
# PAREDES_SALVAS (21/11): so o abrir_arena.sh (mapa fixo) define PAREDES_FITA_ARQUIVO/MODO;
# sem elas (subir_tudo, SLAM do zero) o tape_keepout fica em walls_mode off, como antes.
PAREDES_ARGS=()
if [[ -n "${PAREDES_FITA_ARQUIVO:-}" ]]; then
  PAREDES_ARGS=(walls_file:="$PAREDES_FITA_ARQUIVO" walls_mode:="${PAREDES_FITA_MODO:-load}")
fi
exec ros2 launch robo_virtual_wall observation_tape.launch.py device:=/dev/v4l/by-id/usb-XIFT_Web_Camera_20250904.1549-video-index0 homography_file:=/home/marcos/Robo-WORK/src/robo_virtual_wall/calibration/front_base_link_charuco_20261121_153538.json preview_homography_file:=/home/marcos/Robo-WORK/src/robo_virtual_wall/calibration/front_charuco_20261121_153538.json "${PAREDES_ARGS[@]}"
