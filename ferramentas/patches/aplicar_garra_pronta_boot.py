#!/usr/bin/env python3
"""GARRA_PRONTA_BOOT (25/11/2026): traz o commit d359d3c (+ PRONTA do eb530a9) da branch
cajumakers/teste/j1-fora-de-zero para a garra usada pelo subir_garra.sh (~/robo_work_teste/garra).

Boot vai para home e depois, na rampa, para PRONTA [0.35, -0.63, 1.05]; antes ia para q=0 (vertical),
onde o J1 treme na folga e sacode a camera. Arquivos:
  garra_moveit_config/config/initial_positions.yaml
  garra_coleta/garra_coleta/cliente_garra.py
  firmware/garra_servos/garra_servos.ino   (PRECISA GRAVAR NO ARDUINO)

Uso:  python3 aplicar_garra_pronta_boot.py [--garra DIR] [--desfazer]
Marca: GARRA_PRONTA_BOOT em cada arquivo. So aplica se o arquivo local for igual a base 970f746.
"""
import argparse, ast, datetime, glob, hashlib, os, shutil, subprocess, sys

import yaml

REPO = os.path.expanduser('~/robo_jazzy_ws')
COMMIT, BASE = 'd359d3c', '970f746'
MARCA = 'GARRA_PRONTA_BOOT'
ARQS = {
    'garra_moveit_config/config/initial_positions.yaml': '# ' + MARCA + ' (25/11): de d359d3c\n',
    'garra_coleta/garra_coleta/cliente_garra.py': '# ' + MARCA + ' (25/11): PRONTA de d359d3c\n',
    'firmware/garra_servos/garra_servos.ino': '// ' + MARCA + ' (25/11): de d359d3c\n',
}


def git_show(rev, rel):
    return subprocess.run(['git', '-C', REPO, 'show', f'{rev}:garra/{rel}'],
                          check=True, capture_output=True, text=True).stdout


def conferir(rel, txt):
    if rel.endswith('.yaml'):
        ini = yaml.safe_load(txt)['initial_positions']
        assert [ini['Revolucionar_46'], ini['Revolucionar_49'], ini['Revolucionar_72']] == [0.35, -0.63, 1.05]
    elif rel.endswith('.py'):
        ast.parse(txt)
        assert 'PRONTA = [0.35, -0.63, 1.05]' in txt
    else:
        assert 'PULSO_PRONTA[N_SERVOS] = {1681, 1886, 654, PULSO_INICIAL}' in txt
        assert 'agora >= ESPERA_HOME_MS' in txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--garra', default=os.path.expanduser('~/robo_work_teste/garra'))
    ap.add_argument('--desfazer', action='store_true')
    a = ap.parse_args()

    if a.desfazer:
        for rel in ARQS:
            p = os.path.join(a.garra, rel)
            baks = sorted(glob.glob(p + '.bak_garra_pronta_boot_*'))
            if not baks:
                print(f'sem backup: {p}'); continue
            shutil.copy2(baks[-1], p); print(f'voltou {p} <- {baks[-1]}')
        return

    novos = {}
    for rel, cab in ARQS.items():
        p = os.path.join(a.garra, rel)
        atual = open(p).read()
        if MARCA in atual:
            print(f'ja aplicado ({MARCA}): {p}. Nada feito.'); return
        if atual != git_show(BASE, rel):
            sys.exit(f'!!! {p} difere da base {BASE}: nao aplico (alguem mexeu). Nada gravado.')
        novo = git_show(COMMIT, rel)
        # marca na 2a linha do .py (depois de eventual shebang/docstring nao importa: comentario no topo)
        novo = cab + novo
        conferir(rel, novo)
        novos[p] = novo

    data = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    for p, novo in novos.items():
        shutil.copy2(p, f'{p}.bak_garra_pronta_boot_{data}')
        with open(p, 'w') as f:
            f.write(novo)
        print(f'gravado {p}  (backup .bak_garra_pronta_boot_{data})')
    print('\nAgora: colcon build da garra E gravar o firmware no Arduino da garra.')


if __name__ == '__main__':
    main()
