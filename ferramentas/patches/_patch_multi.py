"""Base dos patches de 13/11 que mexem em MAIS DE UM arquivo (mesmo padrao do _patch_texto.py).

- marca unica em cada arquivo; achou a marca -> nao faz nada (recusa dupla);
- confere ANTES de gravar: cada trecho esperado aparece 1 vez em cada arquivo e o .py compila;
- tudo ou nada: se um arquivo falha na conferencia, nenhum e gravado;
- backup <arquivo>.bak_<nome>_<data> de cada arquivo;
- --desfazer: troca reversa (independe da ordem em que outros patches foram aplicados);
- --raiz <pasta>: aplica numa copia do workspace (mesmos caminhos relativos), para testar antes.
"""
import argparse
import ast
import datetime
import os
import shutil
import sys

WS = os.path.expanduser('~/robo_jazzy_ws')


def rodar(nome, marca, arquivos, doc=''):
    """arquivos: lista de (caminho relativo ao workspace, [(velho, novo), ...])."""
    ap = argparse.ArgumentParser(description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--raiz', default=WS, help='workspace (padrao ~/robo_jazzy_ws); use uma copia para testar')
    ap.add_argument('--desfazer', action='store_true')
    a = ap.parse_args()
    data = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    novos = []
    for rel, trocas in arquivos:
        caminho = os.path.join(a.raiz, rel)
        txt = open(caminho).read()
        if a.desfazer:
            if marca not in txt:
                print('%s nao esta aplicado em %s: nada feito' % (marca, rel))
                continue
            novo = txt
            for velho, rep in reversed(trocas):
                if novo.count(rep) != 1:
                    sys.exit('ERRO em %s: trecho do patch alterado depois de aplicado; volte o backup '
                             '.bak_%s_* a mao' % (rel, nome))
                novo = novo.replace(rep, velho)
        else:
            if marca in txt:
                print('%s ja aplicado em %s (marca encontrada): nada feito' % (marca, rel))
                continue
            novo = txt
            for velho, rep in trocas:
                n = novo.count(velho)
                if n != 1:
                    sys.exit('ERRO em %s: trecho esperado aparece %d vezes (versao diferente?):\n%s' % (
                        rel, n, velho[:200]))
                novo = novo.replace(velho, rep)
            if marca not in novo:
                sys.exit('ERRO em %s: o patch nao deixa a marca %s no arquivo' % (rel, marca))
        if caminho.endswith('.py'):
            ast.parse(novo)   # conferencia: compila
        novos.append((caminho, novo))
    for caminho, novo in novos:
        bak = '%s.bak_%s%s_%s' % (caminho, 'desfazer_' if a.desfazer else '', nome, data)
        shutil.copy2(caminho, bak)
        open(caminho, 'w').write(novo)
        print('%s %s (anterior em %s)' % ('desfeito' if a.desfazer else 'gravado', caminho, bak))
