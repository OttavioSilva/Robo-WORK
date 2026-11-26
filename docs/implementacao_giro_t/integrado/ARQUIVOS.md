# Arquivos da integracao (contra o snapshot pre-integracao)

| Estado | Arquivo |
|---|---|
| Modificado | `ferramentas/gravar_teste.sh` |
| Novo | `ferramentas/gravar_teste.py` |
| Novo | `ferramentas/gravacao_qos.yaml` |
| Novo | `ferramentas/trajetoria_teste.py` |
| Novo | `ferramentas/verificar_giro_t.py` |
| Novo | `ferramentas/iniciar_teste_giro_t.sh` |
| Novo | `ferramentas/compilar_firmware_temporal.py` |
| Novo | `ferramentas/restaurar_arquivos_integrado.py` |
| Novo | `firmware/mega_mecanum_temporal/mega_mecanum_temporal.ino` |
| Novo | `firmware/mega_mecanum_temporal/sliding_scan.h` |
| Modificado | `src/robo_hardware/CMakeLists.txt` |
| Modificado | `src/robo_hardware/launch/hardware.launch.py` |
| Modificado | `src/robo_hardware/src/hardware_bridge_node.cpp` |
| Novo | `src/robo_hardware/include/robo_hardware/sensor_clock.hpp` |
| Novo | `src/robo_hardware/test/test_sensor_clock.cpp` |
| Modificado | `src/robo_lidar/launch/lidar_filter.launch.py` |
| Modificado | `src/robo_lidar/package.xml` |
| Modificado | `src/robo_navigation/CMakeLists.txt` |
| Modificado | `src/robo_navigation/package.xml` |
| Modificado | `src/robo_navigation/config/nav2_params.yaml` |
| Modificado | `src/robo_navigation/launch/navigation.launch.py` |
| Modificado | `src/robo_navigation/scripts/alinhador_passagem.py` |
| Modificado | `src/robo_navigation/scripts/giro_frontal.py` |
| Modificado | `src/robo_navigation/scripts/arbitro_t.py` |
| Modificado | `src/robo_navigation/test/test_giro_frontal.py` |
| Novo | `src/robo_navigation/test/test_integrado_t.py` |

Externo ao workspace: `~/gravar_teste.sh` modificado e preservado no backup.

Arquivos nao commitados das rodadas anteriores foram preservados. Nenhum commit foi criado.

```text
 M ferramentas/gravar_teste.sh
 M src/robo_hardware/CMakeLists.txt
 M src/robo_hardware/launch/hardware.launch.py
 M src/robo_hardware/src/hardware_bridge_node.cpp
 M src/robo_lidar/launch/lidar_filter.launch.py
 M src/robo_lidar/package.xml
 M src/robo_navigation/CMakeLists.txt
 M src/robo_navigation/config/nav2_params.yaml
 M src/robo_navigation/launch/navigation.launch.py
 M src/robo_navigation/package.xml
 M src/robo_navigation/scripts/alinhador_passagem.py
?? backups/giro_frontal_20261109/
?? backups/giro_frontal_revisao2_20261109/
?? backups/giro_integrado_20261109/
?? docs/
?? ferramentas/compilar_firmware_temporal.py
?? ferramentas/gravacao_qos.yaml
?? ferramentas/gravar_teste.py
?? ferramentas/iniciar_teste_giro_t.sh
?? ferramentas/restaurar_arquivos_integrado.py
?? ferramentas/trajetoria_teste.py
?? ferramentas/verificar_giro_t.py
?? firmware/mega_mecanum_temporal/
?? src/robo_hardware/include/
?? src/robo_hardware/test/
?? src/robo_navigation/scripts/arbitro_t.py
?? src/robo_navigation/scripts/giro_frontal.py
?? src/robo_navigation/test/
```

Documentos novos: OPERACAO.md, PROTOCOLO.md, RESULTADOS.md, verificacoes.txt, arquivos.json, integracao.patch e os artefatos/manifesto de build. Os documentos historicos anteriores receberam apenas um aviso para esta versao.
