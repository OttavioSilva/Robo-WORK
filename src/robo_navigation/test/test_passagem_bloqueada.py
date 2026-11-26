"""PASSAGEM_BLOQUEADA (11/11/2026): corredor bloqueado por objeto do momento -> rota pelo outro corredor do mapa.

Mapa sintetico (0,05 m): sala oeste e sala leste ligadas pelo corredor A (direto, 0,80 m) e pelo corredor B (norte,
mais longo). Sem bloqueio a rota vai por A; com a caixa em A vai por B; caixa parcial num corredor largo continua nele.
A logica do no (_desviar_bloqueio, _bloqueios_ativos) e extraida do fonte e roda com um 'self' falso.
"""
import ast
import math
import sys
import time
import types
from pathlib import Path

import pytest

NAV = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAV / 'scripts'))
import alinhador_passagem as ap  # noqa: E402

RES, W, H = 0.05, 120, 80


def arena(larg_a=0.80, corredor_b=True):
    d = [100] * (W * H)

    def livre(x0, x1, y0, y1):
        for j in range(H):
            for i in range(W):
                x, y = (i + 0.5) * RES, (j + 0.5) * RES
                if x0 <= x <= x1 and y0 <= y <= y1:
                    d[j * W + i] = 0
    livre(0.1, 1.3, 0.1, 3.9)                    # sala oeste
    livre(4.7, 5.9, 0.1, 3.9)                    # sala leste
    livre(1.3, 4.7, 1.0 - larg_a / 2, 1.0 + larg_a / 2)   # corredor A (y = 1,0)
    if corredor_b:
        livre(1.3, 4.7, 2.8, 3.6)                # corredor B (y = 3,2)
    return ap.Mapa(RES, W, H, 0.0, 0.0, 0.0, d)


INI, FIM = (0.7, 1.0), (5.3, 1.0)


def caixa(x0, x1, y0, y1, r=0.35):
    pts = []
    y = y0
    while y <= y1 + 1e-9:
        x = x0
        while x <= x1 + 1e-9:
            pts.append((x, y, r))
            x += 0.05
        y += 0.05
    return pts


def max_y(rota):
    return max(p[1] for p in rota)


def test_sem_bloqueio_vai_pelo_corredor_direto():
    rota = ap.rota_no_mapa(arena(), INI, FIM)
    assert rota is not None and max_y(rota) < 2.0


def test_caixa_no_corredor_direto_vai_pelo_outro():
    rota = ap.rota_no_mapa(arena(), INI, FIM, bloqueios=caixa(3.0, 3.1, 0.75, 1.25))
    assert rota is not None, 'devia achar o corredor B'
    assert max_y(rota) > 2.8, 'a rota devia passar pelo corredor norte (B)'
    assert not ap.rota_cruza_bloqueio(rota, caixa(3.0, 3.1, 0.75, 1.25))


def test_caixa_e_sem_outro_corredor_nao_tem_rota():
    assert ap.rota_no_mapa(arena(corredor_b=False), INI, FIM, bloqueios=caixa(3.0, 3.1, 0.75, 1.25)) is None


def test_caixa_parcial_em_corredor_largo_continua_nele():
    """BTT II, obstaculo parcial: sobra espaco para o corpo -> nao desvia para o corredor longo."""
    m = arena(larg_a=1.40)                               # A: y 0,30 .. 1,70
    blq = caixa(3.0, 3.1, 0.45, 0.55)                    # encostado na parede sul
    rota = ap.rota_no_mapa(m, INI, FIM, bloqueios=blq)
    assert rota is not None and max_y(rota) < 2.0
    assert not ap.rota_cruza_bloqueio(rota, blq)


def test_pontos_obstaculo_novo_so_objeto_livre_no_mapa_e_a_frente():
    m = arena()
    pose = (2.0, 1.0, 0.0)                               # dentro de A, de frente para leste
    pts = [(0.50, 0.0),     # 2,50; 1,00: livre -> objeto
           (0.50, 0.40),    # y 1,40: na parede do corredor -> fora
           (0.20, 0.0),     # atras do para-choque -> fora
           (1.20, 0.0)]     # alem do alcance -> fora
    achou = ap.pontos_obstaculo_novo(m, pts, pose)
    assert len(achou) == 1 and math.hypot(achou[0][0] - 2.5, achou[0][1] - 1.0) < 1e-6


def test_pontos_obstaculo_novo_gira_com_o_robo():
    m = arena()
    achou = ap.pontos_obstaculo_novo(m, [(0.5, 0.0)], (0.7, 2.0, math.pi / 2))   # sala oeste, de frente para norte
    assert len(achou) == 1 and abs(achou[0][0] - 0.7) < 1e-6 and abs(achou[0][1] - 2.5) < 1e-6


def test_parede_a_frente_nao_vira_bloqueio():
    m = arena()
    # robo na sala leste olhando para a parede leste (x 5,9): tudo e parede no mapa
    assert ap.pontos_obstaculo_novo(m, [(0.45, 0.0), (0.50, 0.1)], (5.4, 2.0, 0.0)) == []


# ---------------------------------------------------------------- logica do no (fonte real)
SRC = NAV / 'scripts/alinhador_passagem.py'


def _metodos(*nomes):
    arv = ast.parse(SRC.read_text())
    fns = [n for n in ast.walk(arv) if isinstance(n, ast.FunctionDef) and n.name in nomes]
    assert {f.name for f in fns} == set(nomes)
    ns = {'time': time, 'math': math, 'rota_no_mapa': ap.rota_no_mapa, '_comp_rota': ap._comp_rota,
          'com_barreiras': ap.com_barreiras}   # MEMORIA_BARREIRA (25/11)
    exec(compile(ast.Module(body=fns, type_ignores=[]), str(SRC), 'exec'), ns)
    return [ns[n] for n in nomes]


class Log:
    def __init__(self): self.msgs = []
    def info(self, m): self.msgs.append(('info', m))
    def warn(self, m): self.msgs.append(('warn', m))


def no_falso(mapa, motivo, obst, pose=(1.0, 1.0), ativo=120.0):
    desviar, ativos = _metodos('_desviar_bloqueio', '_bloqueios_ativos')
    s = types.SimpleNamespace(log=Log(), bloqueios=[], trav_motivo=motivo, trav_obstaculo=obst, desvios_bloq=0)
    s.get_logger = lambda: s.log
    s.p = lambda n: {'passagem_bloqueada_s': ativo, 'passagem_bloqueada_raio': 0.35}[n]
    s._mapa = lambda: mapa
    s._pose_mapa = lambda: pose
    s._bloqueios_ativos = ativos.__get__(s)
    s._desviar_bloqueio = desviar.__get__(s)
    return s


CAIXA_A = [(3.0, y) for y in (0.75, 0.85, 0.95, 1.05, 1.15, 1.25)]


def test_desvia_quando_ha_outro_corredor():
    s = no_falso(arena(), 'algo a 10 cm do para-choque', CAIXA_A)
    assert s._desviar_bloqueio(*FIM, 'TRAVESSIA 1') is True
    assert len(s._bloqueios_ativos()) == len(CAIXA_A)
    assert any('nao insisto' in m for _, m in s.log.msgs)


def test_nao_desvia_sem_outro_corredor():
    s = no_falso(arena(corredor_b=False), 'algo a 10 cm do para-choque', CAIXA_A)
    assert s._desviar_bloqueio(*FIM, 'TRAVESSIA 1') is False
    assert any('tento de novo por aqui' in m for _, m in s.log.msgs)


@pytest.mark.parametrize('motivo', ['tempo maximo', 'sem TF map->base_link', 'nao consegui alinhar em 8 correcoes'])
def test_outros_motivos_nao_desviam(motivo):
    s = no_falso(arena(), motivo, CAIXA_A)
    assert s._desviar_bloqueio(*FIM, 'TRAVESSIA 1') is False and s.bloqueios == []


def test_objeto_so_em_parede_nao_desvia():
    s = no_falso(arena(), 'algo a 10 cm do para-choque', [])
    assert s._desviar_bloqueio(*FIM, 'TRAVESSIA 1') is False


def test_desligado_e_como_antes():
    s = no_falso(arena(), 'algo a 10 cm do para-choque', CAIXA_A, ativo=0.0)
    assert s._desviar_bloqueio(*FIM, 'TRAVESSIA 1') is False and s._bloqueios_ativos() == []


def test_no_maximo_dois_desvios_por_goal():
    s = no_falso(arena(), 'algo a 10 cm do para-choque', CAIXA_A)
    assert s._desviar_bloqueio(*FIM, 'T') and s._desviar_bloqueio(*FIM, 'T')
    assert s._desviar_bloqueio(*FIM, 'T') is False


def test_bloqueio_expira():
    s = no_falso(arena(), 'algo a 10 cm do para-choque', CAIXA_A, ativo=120.0)
    s.bloqueios = [(3.0, 1.0, 0.35, time.monotonic() - 121.0)]
    assert s._bloqueios_ativos() == []


def test_fluxo_legado_desvia_antes_da_2a_tentativa():
    fonte = SRC.read_text()
    i_desvio = fonte.index("if r != 'ok' and self._desviar_bloqueio(gx, gy, 'TRAVESSIA %d' % n):")
    i_2a = fonte.index("'PREPARO %d (2a tentativa)' % n")
    assert i_desvio < i_2a
    trecho = fonte[i_desvio:i_desvio + 300]
    assert 'tratadas.pop()' in trecho and 'continue' in trecho
