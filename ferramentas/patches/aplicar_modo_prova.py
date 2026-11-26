#!/usr/bin/env python3
"""MODO_PROVA (24/11/2026): executor_prova.py com --modo-prova nunca encerra a prova por falha.

Sem --modo-prova o executor fica IGUAL a antes. Com --modo-prova:
  - cada 'ir' passa pelo piloto (scripts/piloto_prova.py): escada reenvia -> bloqueio + replaneja
    -> ponto seguro; pausa com a localizacao perdida (vigia_localizacao); orcamento por area;
  - area que esgota a escada e ADIADA com os pegar/soltar dela (volta antes do FINISH);
  - relogio: --tempo-prova (min; padrao duration_min da tarefa ou 20); antes de cada visita confere
    se cabe ir + encostar + voltar ao FINISH (x1,5 + 20 s); senao vai direto ao FINISH;
  - FINISH: insiste ate o tempo acabar.
Uso: aplicar_modo_prova.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/executor_prova.py')

TROCAS = [
    ("""    python3 executor_prova.py tarefa.yaml --garra
\"\"\"
""",
     """    python3 executor_prova.py tarefa.yaml --garra
    # MODO_PROVA (24/11): competicao: nunca para por falha, adia areas e guarda tempo para o FINISH
    python3 executor_prova.py tarefa.yaml --modo-prova --tempo-prova 20
\"\"\"
"""),
    ("""    slots = [None] * (args.capacidade or CAPACIDADE_SLOTS)  # TAREFA_JUIZES: id em cada slot
    try:
        for i, p in enumerate(passos, 1):
            if p[0] == 'ir':
""",
     """    slots = [None] * (args.capacidade or CAPACIDADE_SLOTS)  # TAREFA_JUIZES: id em cada slot
    piloto = None
    if args.modo_prova:  # MODO_PROVA (24/11)
        import piloto_prova
        limite = 60.0 * (args.tempo_prova or tarefa['minutos'] or 20.0)
        piloto = piloto_prova.Piloto(no, limite, posicoes_conhecidas(), log)
    fila, i = list(passos), 0  # MODO_PROVA: fila (areas adiadas voltam para o fim)
    try:
        while fila:
            p = fila.pop(0)
            i += 1
            if p[0] == 'ir' and piloto is not None:  # MODO_PROVA
                fim = p[1] == 'FINISH' or not any(q[0] == 'ir' for q in fila)
                if not fim and not piloto.cabe_visita(p[1]):
                    log.warning('MODO_PROVA: sem tempo para %s e voltar: vou ao FINISH' % p[1])
                    fila = [q for q in fila if q[0] == 'ir' and q[1] == 'FINISH'] or [('ir', 'FINISH')]
                    continue
                log.info('[%d] ir ate %s (modo prova%s)' % (i, p[1], ', FINISH' if fim else ''))
                if not getattr(no, 'ligado', False) and not no.sondar():
                    log.warning('MODO_PROVA: alinhador nao respondeu a sonda; mando assim mesmo')
                ok, txt = piloto.ir(p[1], no.pub, no.res, finish=fim)
                feitos.append((p, ok, txt))
                if ok:
                    log.info('%s: SUCESSO %s' % (p[1], txt))
                elif ok is None:
                    log.error('MODO_PROVA: %s' % txt)
                    break
                elif not fim:
                    fila = piloto_prova.adiar(fila, p)
                    log.warning('MODO_PROVA: %s ADIADA (%s); fila: %s' % (
                        p[1], txt, ' '.join(q[1] for q in fila if q[0] == 'ir')))
                continue
            if p[0] == 'ir':
"""),
    ("""    ap.add_argument('--timeout', type=float, default=240.0, help='segundos maximos por deslocamento')
""",
     """    ap.add_argument('--timeout', type=float, default=240.0, help='segundos maximos por deslocamento')
    # MODO_PROVA (24/11)
    ap.add_argument('--modo-prova', action='store_true',
                    help='competicao: escada de recuperacao, adia areas, relogio e FINISH garantido')
    ap.add_argument('--tempo-prova', type=float, default=None,
                    help='minutos da prova com --modo-prova (padrao: duration_min da tarefa ou 20)')
"""),
]


def conferir(velho, novo):
    assert novo.count('MODO_PROVA') >= 5


if __name__ == '__main__':
    rodar('modo_prova', 'MODO_PROVA', PADRAO, TROCAS, conferir, doc=__doc__)
