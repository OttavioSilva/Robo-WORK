#!/usr/bin/env python3
"""TAREFA_JUIZES (24/11): executor_prova.py le o arquivo dos juizes e executa com slots.

- Nome da mesa do arquivo (WS_13, ws13, WS013) -> nome cadastrado em service_areas (WS13 ou WS_13).
  Traducao manual: --traduzir WS_13=WS03 (repetivel).
- Com --garra: --capacidade (padrao 4 slots). Pega 1 cubo por chamada, guarda num slot livre;
  a garra pode responder 'ok' ou 'ok <id>' (id lido na tag). Cubo errado -> devolve na mesma mesa.
  Soltar: 1 entrega por cubo, so se o id esta nos slots.

Uso:
    python3 aplicar_tarefa_juizes.py              # aplica no src
    python3 aplicar_tarefa_juizes.py --desfazer
    python3 aplicar_tarefa_juizes.py --arquivo /tmp/copia.py
"""
import argparse
import ast
import datetime
import glob
import os
import shutil
import sys

MARCA = 'TAREFA_JUIZES'
NOME = 'tarefa_juizes'
PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/executor_prova.py')

TROCAS = [
    # 1) slots
    ("CAPACIDADE_GARRA = 1\n",
     "CAPACIDADE_GARRA = 1\n"
     "CAPACIDADE_SLOTS = 4  # TAREFA_JUIZES (24/11): 4 slots no robo com --garra\n"),
    # 2) nomes das mesas
    ("# ---------------------------------------------------------------------------\n"
     "# Plano (sem ROS)\n",
     "def chave_area(nome):\n"
     "    \"\"\"TAREFA_JUIZES: WS_13 / ws13 / WS013 -> ('WS', 13); START -> ('START', None).\"\"\"\n"
     "    import re\n"
     "    s = str(nome).strip().upper()\n"
     "    m = re.match(r'^([A-Z]+)_?0*(\\d+)$', s)\n"
     "    return (m.group(1), int(m.group(2))) if m else (s, None)\n"
     "\n\n"
     "def resolver_areas(tarefa, conhecidas, traducao=None):\n"
     "    \"\"\"TAREFA_JUIZES: troca os nomes do arquivo pelos cadastrados. Devolve {arquivo: cadastro}.\"\"\"\n"
     "    trad = {chave_area(k): str(v).strip().upper() for k, v in (traducao or {}).items()}\n"
     "    cad = {}\n"
     "    for n in sorted(conhecidas):\n"
     "        cad.setdefault(chave_area(n), n)\n"
     "    usados = set(tarefa['origem'].values()) | set(tarefa['destino'].values()) | set(tarefa['areas'])\n"
     "    alvo = {a: trad.get(chave_area(a), a) for a in usados}\n"
     "    mapa = {a: cad.get(chave_area(alvo[a]), alvo[a]) for a in usados}\n"
     "    tarefa['origem'] = {o: mapa[a] for o, a in tarefa['origem'].items()}\n"
     "    tarefa['destino'] = {o: mapa[a] for o, a in tarefa['destino'].items()}\n"
     "    tarefa['areas'] = [mapa[a] for a in tarefa['areas']]\n"
     "    return mapa\n"
     "\n\n"
     "def id_lido(txt):\n"
     "    \"\"\"TAREFA_JUIZES: 'ok 13' -> 13; 'ok' -> None.\"\"\"\n"
     "    p = str(txt).split()\n"
     "    try:\n"
     "        return int(p[1]) if len(p) > 1 and p[0] == 'ok' else None\n"
     "    except ValueError:\n"
     "        return None\n"
     "\n\n"
     "# ---------------------------------------------------------------------------\n"
     "# Plano (sem ROS)\n"),
    # 3) aceitar 'ok <id>'
    ("            return respostas[0] == 'ok', respostas[0]\n",
     "            return respostas[0].split()[:1] == ['ok'], respostas[0]  # TAREFA_JUIZES: 'ok <id>'\n"),
    # 4) execucao com slots
    ("    na_garra = []  # GARRA_COLETA: objetos que a garra esta segurando\n",
     "    na_garra = []  # GARRA_COLETA: objetos que a garra esta segurando\n"
     "    slots = [None] * (args.capacidade or CAPACIDADE_SLOTS)  # TAREFA_JUIZES: id em cada slot\n"),
    ("            elif args.garra:  # GARRA_COLETA (23/11)\n",
     "            elif args.garra:  # GARRA_COLETA (23/11) + TAREFA_JUIZES (24/11): 1 cubo por chamada, slots\n"
     "                z = altura_mesa(p[1]) + MEIO_CUBO\n"
     "                if p[0] == 'pegar':\n"
     "                    faltam = list(p[2])\n"
     "                    for _ in range(len(p[2]) + 1):  # +1: uma devolucao de cubo errado\n"
     "                        if not faltam:\n"
     "                            break\n"
     "                        if None not in slots:\n"
     "                            log.warning('slots cheios %s: nao pego %s' % (slots, faltam))\n"
     "                            break\n"
     "                        s = slots.index(None)\n"
     "                        log.info('[%d/%d] PEGAR um de %s em %s -> slot %d' % (i, len(passos), faltam, p[1], s + 1))\n"
     "                        ok, txt = no.coletar(p[1], faltam, args.tempo_coleta)\n"
     "                        oid = id_lido(txt) if ok else None\n"
     "                        if ok and oid is not None and oid not in faltam:\n"
     "                            log.warning('pegou o %d, que nao e para levar: devolvendo em %s' % (oid, p[1]))\n"
     "                            ok2, txt2 = no.entregar(args.entrega_x, z, args.tempo_entrega)\n"
     "                            feitos.append((('soltar', p[1], [oid]), ok2, 'devolucao ' + str(txt2)))\n"
     "                            continue\n"
     "                        if ok or no.cubo_preso:\n"
     "                            oid = oid if oid is not None else faltam[0]\n"
     "                            slots[s] = oid\n"
     "                            faltam.remove(oid)\n"
     "                        else:\n"
     "                            log.warning('PEGAR em %s: %s %s (a prova segue)' % (\n"
     "                                p[1], 'FALHOU' if ok is False else 'SEM RESPOSTA', txt))\n"
     "                        feitos.append((('pegar', p[1], [oid] if oid is not None else faltam[:1]), ok, txt))\n"
     "                        if not ok:\n"
     "                            break\n"
     "                    log.info('slots: %s' % slots)\n"
     "                else:\n"
     "                    for o in p[2]:\n"
     "                        if o not in slots:\n"
     "                            feitos.append((('soltar', p[1], [o]), False, 'nao esta nos slots %s' % slots))\n"
     "                            continue\n"
     "                        s = slots.index(o)\n"
     "                        log.info('[%d/%d] SOLTAR %d (slot %d) em %s (x=%.3f z=%.3f)' % (\n"
     "                            i, len(passos), o, s + 1, p[1], args.entrega_x, z))\n"
     "                        ok, txt = no.entregar(args.entrega_x, z, args.tempo_entrega)\n"
     "                        if ok or no.cubo_preso is False:\n"
     "                            slots[s] = None\n"
     "                        else:\n"
     "                            log.warning('SOLTAR %d em %s: %s (a prova segue)' % (o, p[1], txt))\n"
     "                        feitos.append((('soltar', p[1], [o]), ok, txt))\n"
     "                    log.info('slots: %s' % slots)\n"
     "            elif False:  # TAREFA_JUIZES: fluxo GARRA_COLETA de 1 cubo (mantido para o --desfazer)\n"),
    # 5) argumentos
    ("    ap.add_argument('--tentativas', type=int, default=2)\n",
     "    ap.add_argument('--tentativas', type=int, default=2)\n"
     "    # TAREFA_JUIZES (24/11)\n"
     "    ap.add_argument('--capacidade', type=int, default=None,\n"
     "                    help='cubos no robo (padrao: 4 slots com --garra, 3 sem)')\n"
     "    ap.add_argument('--traduzir', action='append', default=[], metavar='ARQ=MAPA',\n"
     "                    help='nome da mesa no arquivo = nome cadastrado (ex.: WS_13=WS03)')\n"),
    ("    pos = posicoes_conhecidas()\n"
     "    passos = montar_plano(tarefa, pos, capacidade=CAPACIDADE_GARRA if args.garra else CAPACIDADE)  # GARRA_COLETA\n",
     "    pos = posicoes_conhecidas()\n"
     "    trad = dict(t.split('=', 1) for t in args.traduzir)  # TAREFA_JUIZES\n"
     "    for a, b in sorted(resolver_areas(tarefa, pos, trad).items()):\n"
     "        print('mesa %-8s -> %s%s' % (a, b, '' if b in pos else '  (SEM POSICAO)'))\n"
     "    cap = args.capacidade or (CAPACIDADE_SLOTS if args.garra else CAPACIDADE)\n"
     "    args.capacidade = cap\n"
     "    passos = montar_plano(tarefa, pos, capacidade=cap)  # GARRA_COLETA + TAREFA_JUIZES\n"),
]


def desfazer(arq):
    baks = sorted(glob.glob('%s.bak_%s_*' % (arq, NOME)))
    if not baks:
        print('Sem backup %s.bak_%s_*' % (arq, NOME))
        return 1
    shutil.copy2(baks[-1], arq)
    print('Restaurado de %s' % baks[-1])
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arquivo', default=PADRAO)
    ap.add_argument('--desfazer', action='store_true')
    a = ap.parse_args()
    arq = os.path.abspath(a.arquivo)
    if a.desfazer:
        return desfazer(arq)
    txt = open(arq).read()
    if MARCA in txt:
        print('%s ja aplicado em %s: nada feito.' % (MARCA, arq))
        return 0
    novo = txt
    for velho, subst in TROCAS:
        if novo.count(velho) != 1:
            print('ERRO: trecho esperado nao encontrado (ou repetido):\n%s' % velho[:120])
            return 1
        novo = novo.replace(velho, subst)
    ast.parse(novo)
    bak = '%s.bak_%s_%s' % (arq, NOME, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(arq, bak)
    open(arq, 'w').write(novo)
    print('%s aplicado em %s (backup %s)' % (MARCA, arq, bak))
    return 0


if __name__ == '__main__':
    sys.exit(main())
