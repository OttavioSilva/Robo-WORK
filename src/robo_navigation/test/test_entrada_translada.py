"""ENTRADA_TRANSLADA (24/11): ponto de giro no mapa manual da arena (sem ROS)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from alinhador_passagem import Mapa, giro_livre, ponto_de_giro, pose_livre  # noqa: E402

PGM = Path.home() / 'mapas/arena_manual/arena.pgm'


def mapa_manual():
    d = PGM.read_bytes()
    i = d.index(b'255\n') + 4
    a = np.frombuffer(d[i:], np.uint8).reshape(217, 170)[::-1]      # linha 0 = y minimo
    occ = np.where(a == 0, 100, np.where(a == 254, 0, -1)).astype(int)
    return Mapa(0.05, 170, 217, -0.66, -0.66, 0.0, occ.ravel().tolist())


@pytest.fixture(scope='module')
def mapa():
    if not PGM.exists():
        pytest.skip('sem mapa manual')
    return mapa_manual()


def reta(a, b, passo=0.05):
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / passo))
    return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)]


@pytest.mark.parametrize('start', [(0.30, 5.391), (0.30, 4.137)])
def test_start_nao_deixa_girar_e_acha_ponto_fora_da_caixa(mapa, start):
    assert not giro_livre(mapa, *start)
    caminho = reta((2.0, start[1]), start)            # chegando do leste
    p = ponto_de_giro(caminho, start[0], start[1], 0.0, mapa)
    assert p is not None and p[0] >= 0.85            # fora da caixa (toco ate x 0,51)
    assert giro_livre(mapa, *p)
    assert pose_livre(mapa, start[0], start[1], 0.0, margem=0.02)


def test_ponto_aberto_mantem_fluxo_antigo(mapa):
    caminho = reta((2.0, 7.9), (3.0, 7.9))
    assert giro_livre(mapa, 3.0, 7.9)
    assert ponto_de_giro(caminho, 3.0, 7.9, 0.0, mapa) is None
