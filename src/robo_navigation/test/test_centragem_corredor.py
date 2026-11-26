import math
import sys
from pathlib import Path
from unittest.mock import Mock, call

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from alinhador_passagem import Travessia
from suavizador_nav import comando_corredor
from test_nav2_principal import metodo


POSE = (0.0, 0.0, 0.0)


def medida(desvio, graus=0.0):
    return dict(lados=2, n=36, largura=0.76, desvio=desvio, angulo=math.radians(graus))


@pytest.mark.parametrize('desvio,graus,estado', [
    (-0.047, 0.8, 'assentar'),
    (-0.002, 2.4, 'assentar'),
    (0.010, 0.9, 'atravessar'),
])
def test_entrada_confere_as_duas_paredes(desvio, graus, estado):
    tr = Travessia()
    tr.entrada_rapida = True
    tr.passo(0.0, POSE, 1.0)
    for t in (0.15, 0.32):
        tr.nova_medicao(t, medida(desvio, graus), POSE)
    vx, _, _ = tr.passo(0.33, POSE, 1.0)
    assert tr.estado == estado
    assert (vx > 0) == (estado == 'atravessar')


@pytest.mark.parametrize('primeiro,instante', [(0.04, 0.2), (0.00, 0.0)])
def test_uma_leitura_centrada_recente_nao_basta(primeiro, instante):
    tr = Travessia()
    tr.entrada_rapida = True
    tr.passo(0.0, POSE, 1.0)
    tr.nova_medicao(instante, medida(primeiro), POSE)
    tr.nova_medicao(0.59, medida(0.0), POSE)
    assert tr.passo(0.60, POSE, 1.0) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('fe,fd,desvio,sinal', [
    (0.115, 0.006, 0.052, 1),
    (0.006, 0.115, -0.052, -1),
    (0.055, 0.012, 0.03, 0),
])
def test_nao_avanca_ao_corrigir_quina(fe, fd, desvio, sinal):
    tr = Travessia()
    tr.t0, tr.estado, tr.yaw_ref = 0.0, 'atravessar', 0.0
    tr.nova_medicao(1.0, medida(desvio), POSE)
    tr.nova_folgas(1.0, fe, fd)
    vx, vy, wz = tr.passo(1.01, POSE, 1.0)
    assert vx == wz == 0.0
    assert vy == pytest.approx(sinal * 0.016)
    tr.nova_medicao(1.20, medida(0.005), POSE)
    tr.nova_folgas(1.20, 0.07, 0.07)
    assert tr.passo(1.21, POSE, 1.0)[0] > 0


def test_nao_libera_entrada_apos_esgotar_ajustes_fora_do_centro():
    tr = Travessia()
    tr.t0, tr.estado, tr.t_estado = 0.0, 'medir', 1.0
    tr.n_lat, tr.ciclos = 5, tr.ciclos_max
    for t in (1.1, 1.2, 1.3, 1.4):
        tr.nova_medicao(t, medida(0.04), POSE)
    assert tr.passo(1.41, POSE, 1.0) == (0.0, 0.0, 0.0)
    assert tr.estado == 'abortar'


@pytest.mark.parametrize('poses,resultado', [
    ([(0.19, 0.0, 0.0)] * 12, 'falha'),
    ([POSE, POSE, None, None], 'falha'),
    ([(0.02, 0.0, 0.0)] * 6, 'ok'),
])
def test_confere_pose_final_do_preparo(poses, resultado):
    no = Mock()
    no._pose_mapa_yaw.side_effect = poses
    no.p.return_value = True
    alvo = dict(x=0.0, y=0.0, yaw=0.0)
    assert metodo('_ajustar_pose_preparo')(no, alvo, 'preparo') == resultado


@pytest.mark.parametrize('cmd', [
    (0.060, 0.0, 0.0), (0.0, -0.080, 0.0), (0.06, 0.08, -0.06),
])
def test_velocidades_continuas_preservam_proporcao(cmd):
    assert comando_corredor(*cmd, 0.5) == pytest.approx(tuple(v / 2 for v in cmd))


@pytest.mark.parametrize('cmd', [
    (0.03, 0.0, 0.0), (0.0, -0.016, 0.0), (-0.035, 0.016, -0.022),
    (0.06, -0.08, 0.032), (0.0, 0.0, 0.025),
])
def test_ajuste_fino_continuo_sem_acelerar_a_diagonal(cmd):
    vx, vy, wz = comando_corredor(*cmd, 0.5)
    assert (vx, vy) == pytest.approx((cmd[0] / 2, cmd[1] / 2))
    if cmd[2]:
        piso = 0.020 if (vx or vy) else 0.015
        assert piso < abs(wz) <= abs(cmd[2]) and wz * cmd[2] > 0
    else:
        assert wz == 0
    assert comando_corredor(*cmd, 1.0) == cmd
    assert comando_corredor(*cmd, 0.0) == POSE
    assert comando_corredor(*POSE, 0.5) == POSE


def test_cancelamento_restaura_velocidade_de_rota():
    no = Mock(fator_corredor=0.5, em_preparo=False, zona_morta_original=None)
    faixa = metodo('_movimento_continuo')
    faixa.__globals__['GetParameters'] = Mock()
    original = [0.015, 0.018, 0.02]
    no.consulta_suavizador.call.return_value.values = [Mock(double_array_value=original)]
    no._movimento_continuo.side_effect = lambda *a, **kw: faixa(no, *a, **kw)
    no._restaurar_zona_morta.side_effect = lambda: metodo('_restaurar_zona_morta')(no)
    def navegar(*args, **kwargs):
        assert no.em_preparo
        raise InterruptedError()
    no._navegar.side_effect = navegar
    no._ajustar_pose_preparo.side_effect = lambda *args: metodo('_navegar_corredor')(no, *args)
    with pytest.raises(InterruptedError):
        metodo('_chegar_preparo')(no, POSE, 'preparo')
    assert no._definir_ganho_nav2.call_args_list == [call(0.5), call(1.0)]
    assert no._definir_zona_morta.call_args_list == [call([0.0075, 0.009, 0.02]), call(original)]
    assert no.zona_morta_original is None
    assert not no.em_preparo
