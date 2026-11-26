#!/usr/bin/env python3
"""LOCALIZACAO_FIXA (09/11/2026): com mapa, o slam_toolbox roda em modo LOCALIZACAO.

Por que: em modo mapping o slam_toolbox continua corrigindo o mapa e a pose durante a missao.
Na missao27 o map->odom saltou +8,6 graus e 30 cm com o robo parado em frente a WS_2; na
missao28 o robo estava na pose da aproximacao pelo mapa e o LiDAR via a mesa girada 18-25 graus.

O que muda:
  - cria config/slam_localizacao.yaml: copia do slam_mapping.yaml, so com mode: localization;
  - cria launch/localizacao_slam.launch.py: localization_slam_toolbox_node (lifecycle, autostart);
  - ~/subir_robo.sh: com mapa sobe a localizacao e carrega o mapa com match_type 3
    (LOCALIZE_AT_POSE) no START. --novo (mapear) nao muda. --mapeando volta ao modo antigo.
Depois: colcon build --packages-select robo_navigation (a launch e o yaml vao para o install).

Uso:
    python3 aplicar_localizacao_fixa.py               # aplica (backup do subir_robo.sh antes)
    python3 aplicar_localizacao_fixa.py --desfazer    # volta o backup e apaga os 2 arquivos novos
    python3 aplicar_localizacao_fixa.py --raiz <ws_copia> --script <subir_copia>   # teste numa copia
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

import yaml

MARCA = 'LOCALIZACAO_FIXA'
SUF = '.bak_localizacao_'

LAUNCH = '''"""LOCALIZACAO_FIXA (09/11): slam_toolbox em modo localizacao (o mapa nao muda na missao)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    params = os.path.join(get_package_share_directory("robo_navigation"),
                          "config", "slam_localizacao.yaml")
    loc = os.path.join(get_package_share_directory("slam_toolbox"),
                       "launch", "localization_launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(loc),
            launch_arguments={"slam_params_file": params, "use_sim_time": "false",
                              "autostart": "true"}.items()),
    ])
'''

ARGS_ANTES = 'MAPA=""\nif [ "${1:-}" != "--novo" ]; then\n'
ARGS_DEPOIS = '''# LOCALIZACAO_FIXA (09/11): com mapa o slam_toolbox localiza (nao muda o mapa); --mapeando = modo antigo
MAPEANDO=0; ARGS=()
for a in "$@"; do if [ "$a" = "--mapeando" ]; then MAPEANDO=1; else ARGS+=("$a"); fi; done
set -- "${ARGS[@]}"

MAPA=""
if [ "${1:-}" != "--novo" ]; then
'''
SLAM_ANTES = '  sobe slam ros2 launch "$PKG" mapping.launch.py\n'
SLAM_DEPOIS = '''  if [ -n "$MAPA" ] && [ "$MAPEANDO" != 1 ]; then   # LOCALIZACAO_FIXA
    echo "    slam_toolbox em LOCALIZACAO (mapa fixo; --mapeando para o modo antigo)"
    sobe slam ros2 launch "$PKG" localizacao_slam.launch.py
  else
    sobe slam ros2 launch "$PKG" mapping.launch.py
  fi
'''
MATCH_ANTES = '''"{filename: '$MAPA', match_type: 2, initial_pose: {x: $X, y: $Y, theta: $TH}}"'''
MATCH_DEPOIS = '''"{filename: '$MAPA', match_type: $MATCH, initial_pose: {x: $X, y: $Y, theta: $TH}}"'''
MATCH_DEF_ANTES = '  echo ">>> Carregando o mapa no START ($X, $Y, $TH rad)"\n'
MATCH_DEF_DEPOIS = ('  MATCH=3; [ "$MAPEANDO" = 1 ] && MATCH=2   # LOCALIZACAO_FIXA: 3 = LOCALIZE_AT_POSE\n'
                    '  echo ">>> Carregando o mapa no START ($X, $Y, $TH rad, match_type $MATCH)"\n')


def caminhos(raiz, script):
    nav = os.path.join(raiz, 'src', 'robo_navigation')
    return (os.path.join(nav, 'config', 'slam_mapping.yaml'),
            os.path.join(nav, 'config', 'slam_localizacao.yaml'),
            os.path.join(nav, 'launch', 'localizacao_slam.launch.py'),
            script)


def desfazer(raiz, script):
    _, yml, lch, sh = caminhos(raiz, script)
    baks = sorted(glob.glob(sh + SUF + '*'))
    if not baks:
        print('Backup %s%s* nao encontrado. Nada alterado.' % (sh, SUF))
        return 1
    shutil.copy2(baks[-1], sh)
    print('restaurado: %s <- %s' % (sh, baks[-1]))
    for p in (yml, lch):
        if os.path.exists(p) and MARCA in open(p).read():
            os.remove(p)
            print('apagado: %s' % p)
    print('Agora: colcon build --packages-select robo_navigation e reiniciar o subir_robo.sh')
    return 0


def aplicar(raiz, script):
    mapping, yml, lch, sh = caminhos(raiz, script)
    texto_sh = open(sh).read()
    if MARCA in texto_sh:
        print('%s ja aplicado em %s. Nada alterado.' % (MARCA, sh))
        return 0
    for trecho in (ARGS_ANTES, SLAM_ANTES, MATCH_ANTES, MATCH_DEF_ANTES):
        if texto_sh.count(trecho) != 1:
            print('!!! trecho esperado nao achado (ou repetido) em %s:\n%s\nNada alterado.' % (sh, trecho))
            return 1
    for p in (yml, lch):
        if os.path.exists(p):
            print('!!! %s ja existe. Nada alterado.' % p)
            return 1

    # yaml: so a chave mode muda
    orig = open(mapping).read()
    if orig.count('    mode: mapping\n') != 1:
        print('!!! "mode: mapping" nao achado uma vez em %s. Nada alterado.' % mapping)
        return 1
    novo_yml = ('# LOCALIZACAO_FIXA (09/11): gerado de slam_mapping.yaml; so muda mode.\n' +
                orig.replace('    mode: mapping\n', '    mode: localization   # LOCALIZACAO_FIXA\n'))
    a, b = yaml.safe_load(orig), yaml.safe_load(novo_yml)
    pa, pb = a['slam_toolbox']['ros__parameters'], b['slam_toolbox']['ros__parameters']
    dif = sorted(k for k in set(pa) | set(pb) if pa.get(k) != pb.get(k))
    if dif != ['mode'] or pb['mode'] != 'localization':
        print('!!! conferencia do yaml falhou (diferencas: %s). Nada alterado.' % dif)
        return 1

    novo_sh = (texto_sh.replace(ARGS_ANTES, ARGS_DEPOIS).replace(SLAM_ANTES, SLAM_DEPOIS)
               .replace(MATCH_ANTES, MATCH_DEPOIS).replace(MATCH_DEF_ANTES, MATCH_DEF_DEPOIS))
    tmp = sh + '.tmp_localizacao'
    open(tmp, 'w').write(novo_sh)
    r = subprocess.run(['bash', '-n', tmp], capture_output=True, text=True)
    if r.returncode != 0:
        os.remove(tmp)
        print('!!! bash -n falhou no subir_robo.sh novo:\n%s\nNada alterado.' % r.stderr)
        return 1
    compile(LAUNCH, lch, 'exec')

    bak = sh + SUF + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(sh, bak)
    shutil.copymode(sh, tmp)
    os.replace(tmp, sh)
    open(yml, 'w').write(novo_yml)
    open(lch, 'w').write(LAUNCH)
    print('backup: %s' % bak)
    print('criado: %s' % yml)
    print('criado: %s' % lch)
    print('alterado: %s' % sh)
    print('Agora: cd %s && colcon build --packages-select robo_navigation && source install/setup.bash' % raiz)
    return 0


def main():
    ap = argparse.ArgumentParser(description=MARCA)
    ap.add_argument('--desfazer', action='store_true')
    ap.add_argument('--raiz', default=os.path.expanduser('~/robo_jazzy_ws'))
    ap.add_argument('--script', default=os.path.expanduser('~/subir_robo.sh'))
    a = ap.parse_args()
    return desfazer(a.raiz, a.script) if a.desfazer else aplicar(a.raiz, a.script)


if __name__ == '__main__':
    sys.exit(main())
