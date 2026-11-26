#!/usr/bin/env bash
# Compatibilidade: ~/gravar_teste.sh nome "descricao" [--duration segundos]
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/robo_jazzy_ws/install/setup.bash"
exec python3 "$HOME/robo_jazzy_ws/ferramentas/gravar_teste.py" "$@"
