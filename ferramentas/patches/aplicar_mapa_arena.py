#!/usr/bin/env python3
"""MAPA_ARENA (21/11/2026): o abrir_arena.sh passa a usar o mapa novo (padrao arena_21-11_1727).

O mapa e escolhido pela variavel ARENA_MAPA (padrao gravado no launch); o perfil de localizacao
e ferramentas/subida_unica/slam_localizacao_<mapa>.yaml (gerado por preparar_localization.py).
Voltar ao mapa antigo sem desfazer: ARENA_MAPA=arena_11-11_1938 abrir_arena.sh
ATENCAO: as mesas (~/mapas/service_areas.yaml) sao do mapa em uso; cada mapa tem a sua copia
~/mapas/<mapa>_service_areas.yaml.

Uso: python3 aplicar_mapa_arena.py [--mapa arena_21-11_1727] [--arquivo <copia>] [--desfazer]
"""
import argparse
import py_compile
import shutil
import sys
import time
from pathlib import Path

MARCA = 'MAPA_ARENA'
ALVO = Path.home() / 'robo_jazzy_ws/ferramentas/subida_unica/base_arena_localizacao.launch.py'


def editar(texto, mapa):
    trocas = [
        ('import os  # PAREDES_SALVAS (21/11)\n',
         'import os  # PAREDES_SALVAS (21/11)\n'),
        ('        args = {"slam_params_file": str(Path(__file__).resolve().parent / "slam_localizacao_arena_11-11_1938.yaml")} if "localizacao" in filename else {}\n',
         '        args = {"slam_params_file": str(Path(__file__).resolve().parent / f"slam_localizacao_{MAPA_ARENA}.yaml")} if "localizacao" in filename else {}\n'),
        ('    map_base = home / "mapas/arena_11-11_1938"\n',
         '    map_base = home / "mapas" / MAPA_ARENA  # MAPA_ARENA (21/11): era "mapas/arena_11-11_1938"\n'),
    ]
    for antigo, novo in trocas:
        if texto.count(antigo) != 1:
            raise SystemExit('trecho nao encontrado exatamente 1 vez:\n' + antigo)
        texto = texto.replace(antigo, novo)
    ancora = 'def generate_launch_description():\n'
    if texto.count(ancora) != 1:
        raise SystemExit('generate_launch_description nao encontrado')
    bloco = ('# MAPA_ARENA (21/11): mapa da arena escolhido por ARENA_MAPA; padrao = mapa remapeado em 21/11.\n'
             'MAPA_ARENA = os.environ.get("ARENA_MAPA", "%s")\n\n\n' % mapa)
    return texto.replace(ancora, bloco + ancora)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--mapa', default='arena_21-11_1727')
    p.add_argument('--arquivo', type=Path, default=ALVO)
    p.add_argument('--desfazer', action='store_true')
    a = p.parse_args()
    alvo = a.arquivo
    if a.desfazer:
        if MARCA not in alvo.read_text():
            print('sem a marca; nada a desfazer'); return 0
        backups = sorted(alvo.parent.glob(alvo.name + '.bak_mapa_arena_*'))
        if not backups:
            raise SystemExit('sem backup .bak_mapa_arena_*')
        shutil.copy2(backups[-1], alvo); print('desfeito <-', backups[-1].name); return 0
    antes = alvo.read_text()
    if MARCA in antes:
        print('ja tem a marca %s; nada feito' % MARCA); return 1
    home = Path.home()
    for f in (home / 'mapas' / (a.mapa + '.posegraph'), home / 'mapas' / (a.mapa + '.data'),
              home / 'mapas' / (a.mapa + '_pontos.yaml'),
              ALVO.parent / ('slam_localizacao_%s.yaml' % a.mapa)):
        if not f.exists():
            raise SystemExit('falta %s' % f)
    depois = editar(antes, a.mapa)
    backup = alvo.with_name(alvo.name + '.bak_mapa_arena_' + time.strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(alvo, backup)
    alvo.write_text(depois)
    try:
        py_compile.compile(str(alvo), doraise=True)
        removidas = [l for l in antes.splitlines() if l not in depois.splitlines()]
        if len(removidas) != 2:
            raise SystemExit('mudou mais do que o previsto: %s' % removidas)
    except BaseException:
        shutil.copy2(backup, alvo); print('ERRO: devolvido o original'); raise
    print('aplicado: %s (backup %s), mapa padrao %s' % (alvo, backup.name, a.mapa))
    return 0


if __name__ == '__main__':
    sys.exit(main())
