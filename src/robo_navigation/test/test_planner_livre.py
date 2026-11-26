"""PLANNER_LIVRE + PASSAGEM_IMPOSSIVEL (25/11): _analisar do alinhador com o mapa manual da arena (sem ROS)."""
import math
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import alinhador_passagem as al  # noqa: E402

PGM = Path.home() / 'mapas/arena_manual/arena.pgm'


def _metodo(nome):
    src = (SCRIPTS / 'alinhador_passagem.py').read_text()
    a = src.index('        def %s(' % nome)
    b = src.index('\n        def ', a + 10)
    ns = dict(vars(al))
    exec(textwrap.dedent(src[a:b]), ns)
    return ns[nome], ns


def mapa_manual():
    d = PGM.read_bytes()
    i = d.index(b'255\n') + 4
    a = np.frombuffer(d[i:], np.uint8).reshape(217, 170)[::-1]
    occ = np.where(a == 0, 100, np.where(a == 254, 0, -1)).astype(int)
    return al.Mapa(0.05, 170, 217, -0.66, -0.66, 0.0, occ.ravel().tolist())


class Log:
    def __init__(self):
        self.linhas = []

    def info(self, m):
        self.linhas.append(m)

    warn = error = info


class No:
    def __init__(self, mapa, pose, caminho, params=None):
        self.mapa, self.pose, self.caminho = mapa, pose, caminho
        self.par = dict(rota_mapa=True, planner_livre=True, rota_mapa_obstaculo='recusar',
                        passagem_bloqueada_s=0.0, passagem_bloqueada_raio=0.35, largura_estreita=0.90,
                        distancia_preparo=0.60, distancia_saida=0.45, preparo_lateral_max=0.15,
                        passagem_largura_minima=0.40, passagem_replanejar_max=5,
                        memoria_barreira=True)
        self.par.update(params or {})
        self.rota_tent = {}
        self.log = Log()
        self.limpezas = 0
        self.so_local = None
        self.planejados = 0

    def p(self, k):
        return self.par[k]

    def _mapa(self):
        return self.mapa

    def _caminho(self, goal):
        self.planejados += 1
        return self.caminho() if callable(self.caminho) else self.caminho

    def _pose_mapa(self):
        return self.pose

    def _bloqueios_ativos(self):
        return []

    def _obs_global(self):
        return None

    def get_logger(self):
        return self.log

    def _limpar_costmaps(self, so_local=False):
        self.limpezas += 1
        self.so_local = so_local

    barreiras_msg = None   # MEMORIA_BARREIRA: sem barreiras lembradas

    def _novo_goal_chegou(self):
        return False


def reta(pts, passo=0.05):
    out = []
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / passo))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n)]
    return out + [pts[-1]]


@pytest.fixture(scope='module')
def mapa():
    if not PGM.exists():
        pytest.skip('sem mapa manual')
    return mapa_manual()


def test_planner_livre_fica_com_o_desvio_longo_do_planner(mapa, monkeypatch):
    analisar, ns = _metodo('_analisar')
    usados = []
    ns['planejar_etapas'] = lambda m, pontos, *a, **k: (usados.append(pontos), ([], 'nenhuma'))[1]
    ns['time'].sleep = lambda s: None
    # do meio-oeste ate perto da WS08 dando a volta pelo leste (caminho livre no mapa, bem mais longo)
    desvio = reta([(1.3, 6.0), (2.5, 5.6), (5.5, 5.6), (5.5, 8.3), (1.4, 8.3), (1.0, 8.75)])
    no = No(mapa, (1.3, 6.0), desvio)
    analisar(no, 1.0, 8.75, [], goal=object())
    assert usados and usados[-1] is desvio
    assert any('PLANNER_LIVRE' in l for l in no.log.linhas)


def test_sem_planner_livre_troca_pela_rota_do_mapa(mapa):
    analisar, ns = _metodo('_analisar')
    usados = []
    ns['planejar_etapas'] = lambda m, pontos, *a, **k: (usados.append(pontos), ([], 'nenhuma'))[1]
    desvio = reta([(1.3, 6.0), (2.5, 5.6), (5.5, 5.6), (5.5, 8.3), (1.4, 8.3), (1.0, 8.75)])
    no = No(mapa, (1.3, 6.0), desvio, dict(planner_livre=False))
    analisar(no, 1.0, 8.75, [], goal=object())
    assert usados and usados[-1] is not desvio          # como antes: rota mais curta do mapa


def _info(larg):
    return [{'tipo': 'preparo'}], dict(entrada=(4.9, 9.26), eixo_graus=157, largura_mapa=larg, distancia_preparo=0.7)


def test_passagem_impossivel_replaneja_e_resolve(mapa):
    analisar, ns = _metodo('_analisar')
    larguras = iter([0.0, 0.0, 0.85])
    ns['planejar_etapas'] = lambda *a, **k: _info(next(larguras))
    no = No(mapa, None, reta([(1.0, 6.0), (1.0, 7.0)]), dict(rota_mapa=False))
    etapas, info = analisar(no, 1.0, 7.0, [], goal=object())
    assert etapas and info['largura_mapa'] == 0.85
    assert no.limpezas == 0   # MEMORIA_BARREIRA: largura ~0 = global recem-limpo, so espera e replaneja


def test_passagem_impossivel_persistente_segue_com_nav2_sem_falhar(mapa):
    analisar, ns = _metodo('_analisar')
    ns['planejar_etapas'] = lambda *a, **k: _info(0.0)
    no = No(mapa, None, reta([(1.0, 6.0), (1.0, 7.0)]), dict(rota_mapa=False))
    etapas, info = analisar(no, 1.0, 7.0, [], goal=object())
    assert etapas == [] and 'sigo com o Nav2' in info
    assert no.limpezas == 0 and no.planejados == 6   # 1o plano + 5 replanejamentos


def test_memoria_barreira_estreita_limpa_so_o_local(mapa):
    analisar, ns = _metodo('_analisar')
    ns['planejar_etapas'] = lambda *a, **k: _info(0.30)
    no = No(mapa, None, reta([(1.0, 6.0), (1.0, 7.0)]), dict(rota_mapa=False, passagem_replanejar_max=2))
    etapas, info = analisar(no, 1.0, 7.0, [], goal=object())
    assert etapas == [] and no.limpezas == 2 and no.so_local is True


def test_sem_memoria_barreira_como_antes(mapa):
    analisar, ns = _metodo('_analisar')
    ns['planejar_etapas'] = lambda *a, **k: _info(0.0)
    no = No(mapa, None, reta([(1.0, 6.0), (1.0, 7.0)]), dict(rota_mapa=False, memoria_barreira=False))
    etapas, info = analisar(no, 1.0, 7.0, [], goal=object())
    assert etapas == [] and no.limpezas == 5 and no.so_local is False


def test_passagem_impossivel_vigia_nao_interrompe(mapa):
    analisar, ns = _metodo('_analisar')
    ns['planejar_etapas'] = lambda *a, **k: _info(0.35)
    no = No(mapa, None, reta([(1.0, 6.0), (1.0, 7.0)]), dict(rota_mapa=False))
    etapas, info = analisar(no, 1.0, 7.0, [], pontos=reta([(1.0, 6.0), (1.0, 7.0)]))
    assert etapas == [] and no.limpezas == 0


def test_passagem_real_nao_e_afetada(mapa):
    analisar, ns = _metodo('_analisar')
    ns['planejar_etapas'] = lambda *a, **k: _info(0.80)
    no = No(mapa, None, reta([(1.0, 6.0), (1.0, 7.0)]), dict(rota_mapa=False))
    etapas, info = analisar(no, 1.0, 7.0, [], goal=object())
    assert etapas and no.limpezas == 0


def test_com_barreiras_fecha_a_rota_do_mapa(mapa):
    """MEMORIA_BARREIRA: a rota do mapa nao atravessa celula lembrada no /bloqueios_mapa."""
    from types import SimpleNamespace as NS
    com = al.com_barreiras
    info = NS(width=mapa.w, height=mapa.h, resolution=0.05,
              origin=NS(position=NS(x=-0.66, y=-0.66), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))
    d = PGM.read_bytes()
    i = d.index(b'255\n') + 4
    a = np.frombuffer(d[i:], np.uint8).reshape(217, 170)[::-1]
    occ = np.where(a == 0, 100, np.where(a == 254, 0, -1)).astype(int).ravel().tolist()
    barr = np.zeros((217, 170), int)
    # vao OESTE da linha do meio (x 1,36-2,77, y ~4,4) fechado
    c0, c1 = int((1.30 + 0.66) / 0.05), int((2.85 + 0.66) / 0.05)
    l0, l1 = int((4.10 + 0.66) / 0.05), int((4.70 + 0.66) / 0.05)
    barr[l0:l1, c0:c1] = 100
    no = NS(barreiras_msg=NS(info=info, data=barr.ravel().tolist()), mapa_msg=NS(info=info, data=occ),
            p=lambda k: True)
    fechado = com(no, mapa)
    ini, fim = (2.0, 5.4), (2.0, 3.9)
    livre = al.rota_no_mapa(mapa, ini, fim)
    desvio = al.rota_no_mapa(fechado, ini, fim)
    assert livre and desvio
    comp = lambda r: sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(r, r[1:]))
    assert comp(desvio) > comp(livre) + 1.0                  # deu a volta pelo leste
    assert not any(1.30 < x < 2.85 and 4.10 < y < 4.70 for x, y in desvio)
    no.p = lambda k: False
    assert com(no, mapa) is mapa                              # desligado: como antes
