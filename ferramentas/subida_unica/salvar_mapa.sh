#!/usr/bin/env bash
# SALVAR_MAPA (21/11): salva o mapa do SLAM que esta rodando (subir_tudo, SLAM do zero) em ~/mapas/<nome>,
# sem parar nada: .posegraph/.data (para o modo arena/localizacao) + .pgm/.yaml + <nome>_pontos.yaml
# (SEM_PAREDE_VIRTUAL 23/11: nao salva mais <nome>_fitas.json.)
# START/FINISH = origem do grafo (0,0,0): onde o robo estava quando o SLAM subiu.
# Uso: salvar_mapa.sh [nome]   (padrao arena_DD-MM_HHMM; recusa sobrescrever)
source /opt/ros/jazzy/setup.bash  # antes do set -u: o setup.bash usa variaveis nao definidas
set -euo pipefail
export ROS_DOMAIN_ID=0
NOME=${1:-arena_$(date +%d-%m_%H%M)}
BASE=~/mapas/$NOME
for ext in posegraph data pgm yaml; do
  [[ -e "$BASE.$ext" ]] && { echo "!!! $BASE.$ext ja existe; escolha outro nome."; exit 1; }
done
echo ">>> Pose atual do robo no mapa (se ele voltou ao START, deve estar perto de 0,0):"
timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -m2 -E "Translation|RPY \(degree" || echo "   (TF map->base_link indisponivel)"
# LIDAR_M2M2 (24/11): com a ponte do M2M2 rodando, salva o mapa interno dele em $BASE.stcm
# (para carregar depois: M2M2_ARGS="m2m2_mapa:=$BASE.stcm m2m2_carregar:=true m2m2_limpar:=false m2m2_mapeando:=false")
if ros2 node list 2>/dev/null | grep -qx /m2m2_bridge; then
  [[ -e "$BASE.stcm" ]] && { echo "!!! $BASE.stcm ja existe; escolha outro nome."; exit 1; }
  echo ">>> Salvando o mapa do M2M2 em $BASE.stcm ..."
  ros2 param set /m2m2_bridge arquivo_mapa "$BASE.stcm" >/dev/null
  timeout 60 ros2 service call /m2m2_bridge/salvar_mapa std_srvs/srv/Trigger | tail -2
  [[ -s "$BASE.stcm" ]] || echo "!!! .stcm nao foi gravado"
else
echo ">>> Salvando o grafo do SLAM em $BASE.posegraph/.data ..."
  R1=$(timeout 60 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$BASE'}")
  echo "$R1" | tail -2
  echo ">>> Salvando a imagem do mapa em $BASE.pgm/.yaml ..."
  R2=$(timeout 60 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$BASE'}}" || true)
  echo "$R2" | tail -2
fi
if [[ ! -s "$BASE.pgm" || ! -s "$BASE.yaml" ]]; then
  # save_map do slam_toolbox falhou (result 255 em 21/11): grava a imagem direto do /map.
  echo ">>> save_map falhou; gravando $BASE.pgm/.yaml a partir do /map ..."
  timeout 30 python3 - "$BASE" <<'PY'
import sys, time, numpy as np, rclpy
from pathlib import Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
base = Path(sys.argv[1]); rclpy.init(); node = rclpy.create_node('salvar_mapa_pgm'); got = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(OccupancyGrid, '/map', got.append, qos)
end = time.monotonic() + 20
while not got and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.3)
if not got:
    sys.exit('sem /map')
m = got[-1]; w, h = m.info.width, m.info.height
g = np.array(m.data, dtype=np.int16).reshape(h, w)[::-1]
img = np.full((h, w), 205, np.uint8); img[(g >= 0) & (g < 25)] = 254; img[g >= 65] = 0
base.with_suffix('.pgm').write_bytes(b'P5\n%d %d\n255\n' % (w, h) + img.tobytes())
o = m.info.origin.position
base.with_suffix('.yaml').write_text('image: %s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0]\n'
                                     'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
                                     % (base.name, m.info.resolution, o.x, o.y))
node.destroy_node(); rclpy.shutdown()
PY
fi
# SALVAR_PONTOS (24/11): usa os pontos marcados pelo teleop (START/FINISH e caixa 2), se houver
if python3 - "$BASE" <<'PY'
import sys, os, time, yaml
src = os.path.expanduser('~/mapas/pontos.yaml')
p = (yaml.safe_load(open(src)) or {}).get('pontos') or {}
if 'START' not in p:
    sys.exit(1)
p.setdefault('FINISH', dict(p['START']))
with open(sys.argv[1] + '_pontos.yaml', 'w') as f:
    f.write('# Pontos marcados (teleop) copiados de ~/mapas/pontos.yaml pelo salvar_mapa.sh em %s\n'
            % time.strftime('%d/%m %H:%M'))
    yaml.safe_dump({'pontos': p}, f, sort_keys=True)
print('>>> pontos marcados: ' + ', '.join(sorted(p)))
PY
then :; else
echo ">>> sem START marcado em ~/mapas/pontos.yaml: START = FINISH = (0,0,0)"
cat > "$BASE"_pontos.yaml <<EOF
# START e FINISH (frame map do mapa $NOME). Gravado em $(date '+%d/%m %H:%M') pelo salvar_mapa.sh:
# origem do grafo do SLAM = pose do robo quando o subir_tudo subiu (robo no START).
pontos:
  START:
    x: 0.0
    y: 0.0
    yaw: 0.0
  FINISH:
    x: 0.0
    y: 0.0
    yaw: 0.0
EOF
fi  # SALVAR_PONTOS
FALTA=0
for f in "$BASE.posegraph" "$BASE.data" "$BASE.pgm" "$BASE.yaml" "${BASE}_pontos.yaml"; do
  if [[ -s "$f" ]]; then echo "   ok  $f ($(stat -c %s "$f") bytes)"; else echo "   !!! FALTA $f"; FALTA=1; fi
done
(( FALTA == 0 )) && echo ">>> MAPA SALVO: $NOME" || { echo "!!! mapa incompleto"; exit 1; }
