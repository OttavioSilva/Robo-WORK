#!/usr/bin/env bash
# Terminal fica aberto para diagnostico se a subida falhar; nenhum goal enviado.
set -e
if [ "$#" -eq 0 ]; then set -- --localization arena_04-11_1235; fi
unset GNOME_TERMINAL_SERVICE GNOME_TERMINAL_SCREEN
exec gnome-terminal --window --title='Robô — supervisor Nav2' -- bash -c '
  bash "$@"
  estado=$?
  printf "\nSubida encerrada (codigo %s). Terminal mantido aberto.\n" "$estado"
  exec bash
' bash "$HOME/subir_robo.sh" "$@"
