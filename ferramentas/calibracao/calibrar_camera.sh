#!/usr/bin/env bash
# CALIBRACAO_CHARUCO_20261121: abre a ferramenta de calibração da XIFT (sem ROS).
if pgrep -f "robo_virtual_wall/virtual_wall" >/dev/null; then
  echo "A câmera está em uso pelo robô. Feche o subir_tudo (Ctrl+C) e rode de novo."; exit 1
fi
exec python3 "$(dirname "$(readlink -f "$0")")/calibrar_camera.py"
