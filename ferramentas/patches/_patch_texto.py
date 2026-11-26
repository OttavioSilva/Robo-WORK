"""Base comum dos patches de texto de 11/11 (marca, recusa dupla, backup, --desfazer, --arquivo)."""
import argparse, ast, datetime, glob, os, shutil, sys


def rodar(nome, marca, padrao, trocas, conferir=None, doc=''):
    ap = argparse.ArgumentParser(description=doc)
    ap.add_argument('--arquivo', default=padrao)
    ap.add_argument('--desfazer', action='store_true')
    a = ap.parse_args()
    if a.desfazer:
        # desfaz pela troca reversa (independe da ordem em que outros patches foram aplicados)
        txt = open(a.arquivo).read()
        if marca not in txt:
            print('%s nao esta aplicado: nada feito' % marca)
            return
        novo = txt
        for velho, rep in reversed(trocas):
            if novo.count(rep) != 1:
                sys.exit('ERRO: trecho do patch alterado depois de aplicado; volte o backup .bak_%s_* a mao' % nome)
            novo = novo.replace(rep, velho)
        if a.arquivo.endswith('.py'):
            ast.parse(novo)
        bak = '%s.bak_desfazer_%s_%s' % (a.arquivo, nome, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        shutil.copy2(a.arquivo, bak)
        open(a.arquivo, 'w').write(novo)
        print('desfeito', marca, 'em', a.arquivo, '(estado anterior em %s)' % bak)
        return
    txt = open(a.arquivo).read()
    if marca in txt:
        print('%s ja aplicado (marca encontrada): nada feito' % marca)
        return
    novo = txt
    for velho, rep in trocas:
        if novo.count(velho) != 1:
            sys.exit('ERRO: trecho esperado aparece %d vezes (versao diferente?):\n%s' % (novo.count(velho), velho[:160]))
        novo = novo.replace(velho, rep)
    if a.arquivo.endswith('.py'):
        ast.parse(novo)   # conferencia: compila
    if conferir:
        conferir(txt, novo)
    bak = '%s.bak_%s_%s' % (a.arquivo, nome, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(a.arquivo, bak)
    open(a.arquivo, 'w').write(novo)
    print('backup:', bak)
    print('gravado:', a.arquivo)
