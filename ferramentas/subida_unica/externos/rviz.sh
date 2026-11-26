source /opt/ros/jazzy/setup.bash
source /home/marcos/robo_jazzy_ws/install_nav2_principal/setup.bash
source /home/marcos/Robo-WORK/install_retirada_fita_20261113/robo_virtual_wall/share/robo_virtual_wall/package.bash
export ROS_DOMAIN_ID=0
exec rviz2 -d /home/marcos/logs/parede_virtual_20261116/parede_virtual.rviz
