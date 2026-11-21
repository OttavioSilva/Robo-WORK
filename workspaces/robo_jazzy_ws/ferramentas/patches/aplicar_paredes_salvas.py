#!/usr/bin/env python3
"""PAREDES_SALVAS (21/11/2026): paredes de fita salvas junto com o mapa fixo da arena.

  abrir_arena.sh                -> carrega ~/mapas/<mapa>_fitas.json (nao altera o arquivo)
  abrir_arena.sh --gravar-fitas -> comeca vazio e grava cada parede confirmada no arquivo
  subir_tudo.sh (SLAM do zero)  -> nada muda (sem arquivo: as paredes nao valeriam no mapa novo)

Arquivos alterados (cada um com a marca PAREDES_SALVAS e backup .bak_paredes_salvas_<data>):
  ~/logs/reabertura_20261116_100105/parede.sh
  ~/robo_jazzy_ws/ferramentas/subida_unica/base_arena_localizacao.launch.py
  ~/robo_jazzy_ws/ferramentas/subida_unica/abrir_arena.sh

Uso:  python3 aplicar_paredes_salvas.py            aplica
      python3 aplicar_paredes_salvas.py --desfazer volta os backups mais recentes
      python3 aplicar_paredes_salvas.py --raiz DIR testa em copias (DIR/<nome do arquivo>)
"""
import argparse
import py_compile
import shutil
import subprocess
import sys
import time
from pathlib import Path

MARCA = 'PAREDES_SALVAS'
HOME = Path.home()
ALVOS = {
    'parede': HOME / 'logs/reabertura_20261116_100105/parede.sh',
    'launch': HOME / 'robo_jazzy_ws/ferramentas/subida_unica/base_arena_localizacao.launch.py',
    'arena': HOME / 'robo_jazzy_ws/ferramentas/subida_unica/abrir_arena.sh',
}


def trocar(texto, antigo, novo, nome):
    if texto.count(antigo) != 1:
        raise SystemExit('%s: trecho esperado nao encontrado exatamente 1 vez:\n%s' % (nome, antigo))
    return texto.replace(antigo, novo)


def editar_parede(texto):
    linhas = texto.splitlines(keepends=True)
    alvo = [i for i, l in enumerate(linhas) if l.startswith('exec ros2 launch robo_virtual_wall observation_tape.launch.py')]
    if len(alvo) != 1:
        raise SystemExit('parede.sh: linha exec ros2 launch nao encontrada exatamente 1 vez')
    i = alvo[0]
    bloco = ('# PAREDES_SALVAS (21/11): so o abrir_arena.sh (mapa fixo) define PAREDES_FITA_ARQUIVO/MODO;\n'
             '# sem elas (subir_tudo, SLAM do zero) o tape_keepout fica em walls_mode off, como antes.\n'
             'PAREDES_ARGS=()\n'
             'if [[ -n "${PAREDES_FITA_ARQUIVO:-}" ]]; then\n'
             '  PAREDES_ARGS=(walls_file:="$PAREDES_FITA_ARQUIVO" walls_mode:="${PAREDES_FITA_MODO:-load}")\n'
             'fi\n')
    linhas[i] = linhas[i].rstrip('\n') + ' "${PAREDES_ARGS[@]}"\n'
    return ''.join(linhas[:i]) + bloco + ''.join(linhas[i:])


def editar_launch(texto):
    texto = trocar(texto, 'from pathlib import Path\n', 'import os  # PAREDES_SALVAS (21/11)\nfrom pathlib import Path\n',
                   'launch')
    return trocar(texto, '        actions.append(ExecuteProcess(name=name, cmd=command, output="screen"))\n',
                  '        # PAREDES_SALVAS (21/11): fitas salvas ao lado do mapa; modo vem do abrir_arena.sh.\n'
                  '        extra = {}\n'
                  '        if name == "parede_virtual":\n'
                  '            extra = dict(additional_env={"PAREDES_FITA_ARQUIVO": f"{map_base}_fitas.json",\n'
                  '                                         "PAREDES_FITA_MODO": os.environ.get("PAREDES_FITA_MODO", "load")})\n'
                  '        actions.append(ExecuteProcess(name=name, cmd=command, output="screen", **extra))\n', 'launch')


def editar_arena(texto):
    texto = trocar(texto, "AMB='source /opt/ros/jazzy/setup.bash; source ~/robo_jazzy_ws/install/setup.bash; "
                          "export ROS_DOMAIN_ID=0 PYTHONUNBUFFERED=1'\n",
                   "# PAREDES_SALVAS (21/11): --gravar-fitas comeca sem paredes e grava ~/mapas/<mapa>_fitas.json;\n"
                   "# sem a opcao, as fitas gravadas sao carregadas e o arquivo nao e alterado.\n"
                   "MODO_FITAS=load; [[ \"${1:-}\" == --gravar-fitas ]] && MODO_FITAS=record\n"
                   "echo \">>> Fitas: $MODO_FITAS (record = gravando do zero; load = usando as gravadas)\"\n"
                   "AMB=\"source /opt/ros/jazzy/setup.bash; source ~/robo_jazzy_ws/install/setup.bash; "
                   "export ROS_DOMAIN_ID=0 PYTHONUNBUFFERED=1 PAREDES_FITA_MODO=$MODO_FITAS\"\n", 'abrir_arena')
    return texto


EDICOES = dict(parede=editar_parede, launch=editar_launch, arena=editar_arena)


def conferir(nome, caminho, antes, depois):
    if caminho.suffix == '.sh':
        r = subprocess.run(['bash', '-n', str(caminho)], capture_output=True, text=True)
        if r.returncode:
            raise SystemExit('%s: bash -n falhou: %s' % (caminho, r.stderr))
    else:
        py_compile.compile(str(caminho), doraise=True)
    removidas = [l for l in antes.splitlines() if l not in depois.splitlines()]
    permitidas = {'parede': 1, 'launch': 2, 'arena': 1}[nome]
    if len(removidas) > permitidas:
        raise SystemExit('%s: mudou mais do que o previsto: %s' % (caminho, removidas))


def aplicar(alvos):
    for nome, caminho in alvos.items():
        if MARCA in caminho.read_text():
            print('%s: ja tem a marca %s; nada feito.' % (caminho, MARCA))
            return 1
    data = time.strftime('%Y%m%d_%H%M%S')
    feitos = []
    try:
        for nome, caminho in alvos.items():
            antes = caminho.read_text()
            depois = EDICOES[nome](antes)
            backup = caminho.with_name(caminho.name + '.bak_paredes_salvas_' + data)
            shutil.copy2(caminho, backup)
            feitos.append((caminho, backup))
            caminho.write_text(depois)
            conferir(nome, caminho, antes, depois)
            print('aplicado: %s (backup %s)' % (caminho, backup.name))
    except BaseException:
        for caminho, backup in feitos:
            shutil.copy2(backup, caminho)
        print('ERRO: arquivos devolvidos ao estado anterior.')
        raise
    return 0


def desfazer(alvos):
    for nome, caminho in alvos.items():
        if MARCA not in caminho.read_text():
            print('%s: sem a marca; nada a desfazer.' % caminho)
            continue
        backups = sorted(caminho.parent.glob(caminho.name + '.bak_paredes_salvas_*'))
        if not backups:
            raise SystemExit('%s: sem backup .bak_paredes_salvas_*' % caminho)
        shutil.copy2(backups[-1], caminho)
        print('desfeito: %s <- %s' % (caminho, backups[-1].name))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--desfazer', action='store_true')
    parser.add_argument('--raiz', type=Path, help='aplica em copias DIR/<nome do arquivo> (teste)')
    args = parser.parse_args()
    alvos = ({k: args.raiz / v.name for k, v in ALVOS.items()} if args.raiz else dict(ALVOS))
    return desfazer(alvos) if args.desfazer else aplicar(alvos)


if __name__ == '__main__':
    sys.exit(main())
