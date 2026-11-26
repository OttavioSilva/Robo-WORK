"""MODO_PROVA / VIGIA_LOCALIZACAO / BLOQUEIO_ROTA (24/11): logica pura, sem ROS."""
import os
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import piloto_prova as pp  # noqa: E402
import vigia_localizacao as vl  # noqa: E402
import bloqueios_rota as br  # noqa: E402


def info(w=40, h=40, r=0.05):
    return SimpleNamespace(width=w, height=h, resolution=r,
                           origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)))


# ---------------- progresso
def test_progresso_parado_dispara_depois_da_janela():
    p = pp.Progresso(janela=20, minimo=0.2)
    assert not p.passo(0, (0, 0))
    assert not p.passo(19.9, (0.1, 0))
    assert p.passo(20.0, (0.1, 0))


def test_progresso_rota_longa_que_afasta_do_alvo_conta_como_progresso():
    p = pp.Progresso(janela=20, minimo=0.2)
    for t in range(0, 60):
        assert not p.passo(t, (0.05 * t, 0))   # 5 cm/s, qualquer direcao


def test_progresso_ciscado_no_lugar():
    p = pp.Progresso(janela=20, minimo=0.2)
    res = [p.passo(t * 0.5, (0.05 * (-1) ** t, 0)) for t in range(60)]
    assert res[-1]


# ---------------- relogio
def test_relogio_reserva_finish_e_visita():
    r = pp.Relogio(600, 0)
    assert r.restante(100) == 500
    reserva = r.reserva_finish(3.0)          # 3 m: 3*1,4/0,15=28 s x1,5 + 60 = 102 s
    assert abs(reserva - 102.0) < 0.1
    assert r.cabe_visita(0, 3.0, 3.0)
    assert not r.cabe_visita(560, 3.0, 3.0)
    assert r.hora_do_finish(500, 3.0)
    assert not r.hora_do_finish(480, 3.0)


def test_orcamento_area_cresce_com_a_distancia():
    assert pp.orcamento_area(1.0) < pp.orcamento_area(5.0)
    assert pp.orcamento_area(0.0) >= 90.0


# ---------------- pontos seguros
def test_pontos_seguros_espacados_e_escolha():
    s = pp.PontosSeguros(passo=0.5)
    for x in np.arange(0, 3.01, 0.1):
        s.oferecer((float(x), 0.0, 0.0))
    assert len(s.pontos) == 7      # 0, 0,5 ... 3,0
    p = s.escolher((3.0, 0.0), evitar=(3.0, 0.0), min_dist=1.0)
    assert abs(p[0] - 2.0) < 1e-6
    assert s.escolher((0.0, 0.0), evitar=(0.0, 0.0), min_dist=10) is None


# ---------------- adiar
def test_adiar_leva_visita_e_manipulacao_para_antes_do_finish():
    fila = [('pegar', 'WS01', [1]), ('ir', 'WS02'), ('soltar', 'WS02', [1]), ('ir', 'FINISH')]
    nova = pp.adiar(fila, ('ir', 'WS01'))
    assert nova == [('ir', 'WS02'), ('soltar', 'WS02', [1]), ('ir', 'WS01'), ('pegar', 'WS01', [1]),
                    ('ir', 'FINISH')]


def test_adiar_ultima_area_fica_antes_do_finish():
    fila = [('soltar', 'WS02', [1]), ('ir', 'FINISH')]
    assert pp.adiar(fila, ('ir', 'WS02')) == [('ir', 'WS02'), ('soltar', 'WS02', [1]), ('ir', 'FINISH')]


# ---------------- vigia
def test_encaixe_pontos_na_parede_e_fora():
    occ = np.zeros((40, 40), bool)
    occ[:, 20] = True
    perto = vl.dilatar(occ, 2)
    i = info()
    xs = np.array([1.02, 1.03, 0.5, 0.2])
    ys = np.array([0.5, 1.0, 0.5, 1.0])
    assert vl.encaixe(xs, ys, perto, i) == 0.5
    assert vl.encaixe(np.array([]), np.array([]), perto, i) is None


def test_classificador_histerese():
    c = vl.Classificador()
    assert c.passo(0.0, 0.9) == vl.DUVIDA
    assert c.passo(2.0, 0.9) == vl.OK
    assert c.passo(2.5, 0.6) == vl.DUVIDA
    assert c.passo(3.0, 0.3) == vl.DUVIDA
    assert c.passo(4.0, 0.3) == vl.PERDIDO
    assert c.passo(4.5, 0.9) == vl.PERDIDO     # precisa de 2 s bons
    assert c.passo(6.5, 0.9) == vl.OK


def test_classificador_salto_do_amcl_tira_do_ok():
    c = vl.Classificador()
    c.passo(0.0, 0.9)
    c.passo(2.0, 0.9)
    assert c.passo(2.1, 0.9, salto=True) == vl.DUVIDA


# ---------------- bloqueios
def test_pontos_novos_ignora_parede_e_longe():
    occ = np.zeros((40, 40), bool)
    occ[:, 20] = True
    parede = vl.dilatar(occ, 3)
    xs = np.array([1.0, 0.6, 1.9])
    ys = np.array([1.0, 1.0, 1.0])
    pts = br.pontos_novos(xs, ys, parede, info(), robo=(0.3, 1.0), alcance=1.5)
    assert pts.tolist() == [[0.6, 1.0]]


def test_memoria_marca_expira_e_grade():
    m = br.Memoria()
    assert m.marcar([(1.0, 1.0)], agora=0, validade=10) == 1
    assert m.marcar([(1.01, 1.0)], agora=5, validade=10) == 0      # mesmo ponto: renova
    g = m.grade(info())
    assert g[20, 20] == 100 and g[0, 0] == 0
    assert not m.expirar(12)
    assert m.expirar(16) and m.discos == []
