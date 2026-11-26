"""PREPARO_LATERAL (19/11/2026): volta do L (teste 19/11 11:51) sem pose de preparo no eixo.

Mapa real da sessao (dados/mapa_volta_L_20261119.json.gz). Sem ROS."""
import gzip
import json
import math
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, '..', 'scripts'))
import alinhador_passagem as A  # noqa: E402


def _mapa():
    d = json.load(gzip.open(os.path.join(AQUI, 'dados', 'mapa_volta_L_20261119.json.gz'), 'rt'))
    return A.Mapa(d['res'], d['w'], d['h'], d['ox'], d['oy'], d['yaw'], d['data'])


VOLTA = ((2.29, 5.01), (0.03, 0.06))
IDA = ((0.09, 0.06), (1.90, 2.64))


def _planejar(m, ini, fim, mx):
    return A.planejar_etapas(m, A.rota_no_mapa(m, ini, fim), fim, 0.9, 0.6, 0.45, preparo_lateral_max=mx)


def test_desligado_reproduz_a_falha_de_1151():
    et, motivo = _planejar(_mapa(), *VOLTA, 0.0)
    assert et == [] and 'sem espaco livre para a pose de preparo' in motivo


def test_ligado_acha_preparo_deslocado_livre_e_mantem_a_curva():
    m = _mapa()
    et, info = _planejar(m, *VOLTA, 0.15)
    assert et and et[0]['tipo'] == 'preparo'
    off = et[0]['preparo_lateral']
    assert 0.01 <= abs(off) <= 0.15
    assert A.pose_livre(m, et[0]['x'], et[0]['y'], et[0]['yaw'], margem=0.0)
    assert info['curva'] is not None                     # continua: travessia + saida de lado
    assert abs(info['eixo_graus'] - (-87)) < 5


def test_nao_muda_quando_o_eixo_cabe():
    m = _mapa()
    a, ia = _planejar(m, *IDA, 0.0)
    b, ib = _planejar(m, *IDA, 0.15)
    assert a == b and 'preparo_lateral' not in b[0]
    assert ia['distancia_preparo'] == ib['distancia_preparo']
