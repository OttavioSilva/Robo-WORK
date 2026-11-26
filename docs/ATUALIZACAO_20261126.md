# Atualização de 26/11/2026

## Estado do robô
- LiDAR em uso: Delta-2D (`subir_tudo.sh`, padrão `LIDAR=delta`). `LIDAR=m2m2` usa o Slamtec Mapper M2M2.
- Missão `missao_ws.py WS_1 WS_2` completa em 26/11 (folga 9,7/9,8 cm; centro +4,2/+4,6 cm), após reposicionar o robô no mapa.

## Mudanças do dia
- `slam_mapping.yaml`: `max_laser_range` 5.8 -> 1.7; costmaps `obstacle_max_range` do scan -> 1.7 (chão visto à frente).
- `bloqueios_rota.py`: memória de barreiras desligada (`ALCANCE = 0.0`), pois cercava o robô e o destino.
- SLAM sem a traseira (`scan_slam_sem_tras.py`) e candidatas a mesa no RViz (`marcar_mesa.py --mostrar`).
- Teleop `scripts/teleop_geometry_wasd_qe.py`: tecla T salva a mesa à frente como WS.
- `ferramentas/testes/pose_estimate_mapa.py`: 2D Pose Estimate recarrega o mapa salvo (SLAM em mapping).
- `hardware.yaml`: porta serial da base em `usb-0:3` (depende da entrada USB do notebook).

## Pendências conhecidas
- Nivelar o Delta-2D e rever o limite de 1,7 m (aumenta o risco de deriva do SLAM) e a memória de barreiras.
- O Delta-2D parou de enviar scan uma vez durante uma missão.
- M2M2: excesso de distância não resolvido e quedas da porta 1445.
- Testes automatizados não foram executados nesta publicação (robô ligado).

## Dependências fora do repositório
- Há caminhos fixos em `/home/marcos`. `subir_tudo.sh` usa arquivos de `~/logs/...`; cópias em `ferramentas/subida_unica/externos/`.
- `m2m2_ws/src/m2m2_bridge` precisa do SDK Slamware (não incluído), de https://www.slamtec.com.
- Mapas, logs, `build/` e `install/` não são versionados.
