#!/usr/bin/env python3
"""Reversao explicita, sem Git reset/clean, preservando alteracoes posteriores."""
import datetime as dt
import hashlib
import json
from pathlib import Path
import shutil
import stat
import tarfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def safe_path(root, name):
    p = root/name
    if Path(name).is_absolute() or Path(name).parts[0] not in ('src','firmware','ferramentas','docs'):
        raise ValueError('caminho nao permitido: '+name)
    if not p.resolve().is_relative_to(root.resolve()) or p.is_symlink():
        raise ValueError('caminho externo/symlink: '+name)
    return p


def restore(root, manifest, snapshot, wrapper, original_wrapper):
    entries = manifest['modificados_existentes']+manifest['novos_codigo']
    # Nao sobrescrever trabalho feito depois da entrega, nem mesmo no wrapper home.
    for item in entries:
        p = safe_path(root,item['path'])
        if (not p.is_file() or digest(p)!=item['sha256'] or
                ('mode' in item and stat.S_IMODE(p.stat().st_mode)!=item['mode'])):
            raise RuntimeError('alteracao posterior detectada; revisar antes de restaurar: '+item['path'])
    if wrapper.is_symlink() or digest(wrapper)!=manifest['wrapper_home_sha256']:
        raise RuntimeError('wrapper home mudou depois da entrega; revisar antes de restaurar')
    backup = root/'backups'/('reversao_integrado_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.mkdir(parents=True,exist_ok=False)
    with tarfile.open(backup/'estado_antes_reversao.tar.gz','w:gz') as tar:
        for item in entries:
            tar.add(root/item['path'],arcname=item['path'])
        tar.add(wrapper,arcname='gravar_teste_home.sh')
    # Ler e validar o snapshot integralmente antes de tocar em fontes.
    contents=[]
    with tarfile.open(snapshot) as tar:
        for item in manifest['modificados_existentes']:
            m = tar.getmember(item['path'])
            if not m.isfile(): raise ValueError('snapshot sem arquivo regular')
            contents.append((safe_path(root,item['path']),tar.extractfile(m).read(),m.mode))
    for p, content, mode in contents:
        p.write_bytes(content)
        p.chmod(mode & 0o777)
    for item in manifest['novos_codigo']:
        src=safe_path(root,item['path'])
        dest=backup/'novos'/item['path']
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.move(src,dest)  # preserva, nao apaga arquivos novos
    shutil.copy2(original_wrapper,wrapper)
    (backup/'resultado.json').write_text(json.dumps(manifest,indent=2))
    return backup


def main():
    root=Path(__file__).resolve().parents[1]
    manifest=json.loads((root/'docs/implementacao_giro_t/integrado/arquivos.json').read_text())
    backup=restore(root,manifest,root/'backups/giro_integrado_20261109/antes.tar.gz',
                   Path.home()/'gravar_teste.sh',root/'backups/giro_integrado_20261109/gravar_teste_home.sh')
    print('Restaurado; todo estado posterior preservado em',backup)


if __name__=='__main__':
    main()
