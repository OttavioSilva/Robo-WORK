#!/usr/bin/env python3
"""MAPA_LIMPO (21/11/2026): costmap GLOBAL do Nav2 le /map_nav (mapa do SLAM com falhas apagadas).

Falha vista: missao 17:41 no mapa arena_21-11_1727 nao saiu do START: pontos de "obstaculo"
embaixo do robo (mapeamento com gente perto) deixam o centro do robo letal e o planner recusa.
O no ferramentas/subida_unica/mapa_limpo.py republica /map em /map_nav apagando as zonas de
~/mapas/<mapa>_limpar.yaml (sem arquivo: copia igual). Localizacao, alinhador e /map nao mudam.

Arquivos (marca MAPA_LIMPO, backup .bak_mapa_limpo_<data>):
  ferramentas/subida_unica/base.launch.py                  (subir_tudo: no sem zonas)
  ferramentas/subida_unica/base_arena_localizacao.launch.py (arena: zonas do mapa)
  entregas/correcao_perfil_20261116/nav2_legado_com_fita.sh (chama --nav2-yaml a cada subida)
Uso: aplicar_mapa_limpo.py | --desfazer | --raiz DIR (teste em copias) | --nav2-yaml ARQ (usado pelo .sh)
"""
import argparse
import py_compile
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

MARCA = 'MAPA_LIMPO'
WS = Path.home() / 'robo_jazzy_ws'
ALVOS = {
    'base': WS / 'ferramentas/subida_unica/base.launch.py',
    'arena': WS / 'ferramentas/subida_unica/base_arena_localizacao.launch.py',
    'nav2': WS / 'entregas/correcao_perfil_20261116/nav2_legado_com_fita.sh',
}
SCRIPT = WS / 'ferramentas/subida_unica/mapa_limpo.py'


def trocar(texto, antigo, novo, nome):
    if texto.count(antigo) != 1:
        raise SystemExit('%s: trecho nao encontrado exatamente 1 vez:\n%s' % (nome, antigo))
    return texto.replace(antigo, novo)


def editar_base(t):
    t = trocar(t, '        ("parede_virtual", ["bash", "-c", camera_command, "camera", device, wall]),\n    ]:\n'
                  '        actions.append(ExecuteProcess(name=name, cmd=command, output="screen"))\n',
               '        ("parede_virtual", ["bash", "-c", camera_command, "camera", device, wall]),\n'
               '        # MAPA_LIMPO (21/11): /map -> /map_nav para o costmap global (sem zonas no mapa novo).\n'
               '        ("mapa_limpo", ["python3", "-u", "%s"]),\n    ]:\n'
               '        extra = dict(respawn=True, respawn_delay=2.0) if name == "mapa_limpo" else {}  # MAPA_LIMPO\n'
               '        actions.append(ExecuteProcess(name=name, cmd=command, output="screen", **extra))\n' % SCRIPT,
               'base')
    return t


def editar_arena(t):
    t = trocar(t, '        ("mapa", ["ros2", "service", "call", "/slam_toolbox/deserialize_map",\n',
               '        # MAPA_LIMPO (21/11): /map -> /map_nav com as zonas de <mapa>_limpar.yaml apagadas.\n'
               '        ("mapa_limpo", ["python3", "-u", "%s", "--zonas", f"{map_base}_limpar.yaml"]),\n'
               '        ("mapa", ["ros2", "service", "call", "/slam_toolbox/deserialize_map",\n' % SCRIPT, 'arena')
    t = trocar(t, '        extra = {}\n',
               '        extra = dict(respawn=True, respawn_delay=2.0) if name == "mapa_limpo" else {}  # MAPA_LIMPO\n',
               'arena')
    return t


def editar_nav2(t):
    linha = 'patch_cm_lateral65.py --arquivo /home/marcos/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.yaml  # CM_LATERAL65 19/11 noite\n'
    return trocar(t, linha, linha + 'python3 %s --nav2-yaml /home/marcos/robo_jazzy_ws/entregas/correcao_perfil_20261116/nav2_legado_com_fita.yaml  # MAPA_LIMPO 21/11\n'
                  % (WS / 'ferramentas/patches/aplicar_mapa_limpo.py'), 'nav2')


EDICOES = dict(base=editar_base, arena=editar_arena, nav2=editar_nav2)


def nav2_yaml(caminho):
    """Costmap global: static_layer.map_topic = /map_nav. So essa chave muda; repetir nao muda nada."""
    caminho = Path(caminho)
    antes = yaml.safe_load(caminho.read_text())
    depois = yaml.safe_load(caminho.read_text())
    camada = depois['global_costmap']['global_costmap']['ros__parameters'].setdefault('static_layer', {})
    if camada.get('plugin') != 'nav2_costmap_2d::StaticLayer':
        raise SystemExit('static_layer do costmap global nao encontrado')
    camada['map_topic'] = '/map_nav'
    a = antes['global_costmap']['global_costmap']['ros__parameters']['static_layer']
    if {k: v for k, v in a.items() if k != 'map_topic'} != {k: v for k, v in camada.items() if k != 'map_topic'}:
        raise SystemExit('mudaria mais do que map_topic')
    antes['global_costmap']['global_costmap']['ros__parameters']['static_layer'] = camada
    if antes != depois:
        raise SystemExit('mudaria mais do que static_layer.map_topic')
    caminho.write_text(yaml.safe_dump(depois, sort_keys=False))
    print('MAPA_LIMPO: costmap global le /map_nav (%s)' % caminho)
    return 0


def conferir(nome, caminho):
    if caminho.suffix == '.sh':
        r = subprocess.run(['bash', '-n', str(caminho)], capture_output=True, text=True)
        if r.returncode:
            raise SystemExit('%s: bash -n: %s' % (caminho, r.stderr))
    else:
        py_compile.compile(str(caminho), doraise=True)


def aplicar(alvos):
    for c in alvos.values():
        if MARCA in c.read_text():
            print('%s: ja tem a marca %s; nada feito' % (c, MARCA)); return 1
    data = time.strftime('%Y%m%d_%H%M%S'); feitos = []
    try:
        for nome, c in alvos.items():
            novo = EDICOES[nome](c.read_text())
            b = c.with_name(c.name + '.bak_mapa_limpo_' + data); shutil.copy2(c, b); feitos.append((c, b))
            c.write_text(novo); conferir(nome, c)
            print('aplicado: %s (backup %s)' % (c, b.name))
    except BaseException:
        for c, b in feitos:
            shutil.copy2(b, c)
        print('ERRO: arquivos devolvidos'); raise
    return 0


def desfazer(alvos):
    for c in alvos.values():
        if MARCA not in c.read_text():
            print('%s: sem a marca' % c); continue
        bs = sorted(c.parent.glob(c.name + '.bak_mapa_limpo_*'))
        if not bs:
            raise SystemExit('%s: sem backup' % c)
        shutil.copy2(bs[-1], c); print('desfeito: %s <- %s' % (c, bs[-1].name))
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--desfazer', action='store_true')
    p.add_argument('--raiz', type=Path)
    p.add_argument('--nav2-yaml', type=Path)
    a = p.parse_args()
    if a.nav2_yaml:
        return nav2_yaml(a.nav2_yaml)
    alvos = {k: a.raiz / v.name for k, v in ALVOS.items()} if a.raiz else dict(ALVOS)
    return desfazer(alvos) if a.desfazer else aplicar(alvos)


if __name__ == '__main__':
    sys.exit(main())
