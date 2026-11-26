import math
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from alinhador_passagem import FOOTPRINT, TravessiaLateral, folgas_lateral, medir_corredor_lateral, planejar_etapas
from saida_curva import SaidaCurva, abertura_observada
from test_nav2_principal import metodo


LASER = (0.33, 0.005)


def raios(sentido):
    return [(LASER[0] + sentido * 1.6, LASER[1] + i * 0.035) for i in range(-36, 37)]


def mapa(estado=lambda x, y: 0):
    return SimpleNamespace(res=0.05, estado=estado)


def saida(lado=1, outra=False):
    curva = dict(eixo_b=(0, lado), saida_b=(0, 0), lado=lado, outra_juncao=outra)
    d = SaidaCurva(curva, FOOTPRINT, 0.70)
    dentro = (0., -lado * 0.25, 0.)
    d.observar(0., [], LASER, mapa(), dentro, dentro, True)
    return d


@pytest.mark.parametrize('lado', [-1, 1])
@pytest.mark.parametrize('sentido', [-1, 1])
def test_saida_frontal_ou_traseira_nos_dois_sentidos(lado, sentido):
    d = saida(lado)
    pose = (0., lado * 0.41, 0.)
    assert [d.observar(t, raios(sentido), LASER, mapa(), pose, pose, False)
            for t in (1., 1.17, 1.34)] == [0, 0, sentido]


@pytest.mark.parametrize('tipo', ['T', '+', 'desconhecido', 'nova curva', 'meio corpo', 'sem odometria'])
def test_intersecoes_e_evidencia_incompleta(tipo):
    # O lidar vê uma abertura local. Na transversal ou no corredor que retoma
    # adiante, o mapa contém a parede do outro lado do cruzamento.
    def celula(x, y):
        if tipo == 'T' and 1.50 < y < 1.60:
            return 1
        if tipo == '+' and 1.20 < y < 1.70 and 0.41 < abs(x) < 0.47:
            return 1
        return -1 if tipo == 'desconhecido' and y > 1.2 else 0
    d = saida(outra=tipo == 'nova curva')
    pose = (0., 0.20 if tipo == 'meio corpo' else 0.42, 0.)
    od = (0., -0.24, 0.) if tipo == 'sem odometria' else pose
    assert not any(d.observar(t, raios(1), LASER, mapa(celula), pose, od, False)
                   for t in (1., 1.17, 1.34, 1.51))


@pytest.mark.parametrize('pontos', [
    [], [(math.inf, 0), (math.nan, 0)], [(2., 0.)]*10, raios(1) + [(0.65, 0.12)],
    [p for p in raios(1) if abs(p[1]) > 0.45],
])
def test_um_feixe_ou_ausencia_de_retorno_nao_abre(pontos):
    assert not abertura_observada(pontos, LASER, 1, .70, FOOTPRINT)


def test_scans_repetidos_interrompidos_e_lados_alternados():
    d = saida()
    pose = (0, .42, 0)
    for t, lado in ((1., 1), (1., 1), (1.17, 1), (1.8, 1), (1.97, -1), (2.14, 1)):
        assert not d.observar(t, raios(lado), LASER, mapa(), pose, pose, False)


def test_planejador_atravessa_o_cruzamento_no_segundo_braco():
    def parede(x, y):
        return int((.70 < x < 1.63 and .36 < abs(y) < .42)
                   or ((.43 < y < 1.13 or 1.94 < y < 2.70) and .36 < abs(x - 2) < .43))
    arena = SimpleNamespace(res=.04, estado=parede, ocupado_firme=parede)
    caminho = [(i*.05, 0.) for i in range(41)] + [(2., i*.05) for i in range(1, 72)]
    _, passagem = planejar_etapas(arena, caminho, caminho[-1])
    assert passagem['curva']['entrada_b'][1] < .55
    assert passagem['curva']['saida_b'][1] > 2.60


def test_reconhece_abertura_mas_so_conclui_depois_da_folga_e_parada():
    d = saida()
    pose = (0., .42, 0.)
    assert d.finalizar(1., 1, pose, .02, False, True, False) == 'parar'
    assert d.finalizar(1.3, 1, pose, .02, False, True, True) == 'parar'
    assert d.finalizar(1.4, 1, pose, .02, False, True, True) == 'afastar'
    assert d.finalizar(1.6, 1, pose, .02, False, False, False) == 'parar'
    assert d.finalizar(1.8, 0, pose, .08, False, True, True) == 'parar'
    assert d.finalizar(2.0, 1, pose, .08, False, True, True) == 'parar'
    assert d.finalizar(2.5, 1, pose, .08, False, True, True) == 'fim'


def test_afastamento_limitado_e_folga_desconhecida():
    d = saida()
    pose = (0., .42, 0.)
    for t in (1., 2., 3.):
        assert d.finalizar(t, -1, pose, None, False, True, True) == 'parar'
    assert d.finalizar(4.0, -1, pose, .03, False, True, True) == 'parar'
    assert d.finalizar(4.1, -1, pose, .03, False, True, True) == 'afastar'
    assert d.finalizar(5.7, -1, pose, .03, False, True, False) == 'bloqueada'


@pytest.mark.parametrize('cm,resultado', [(False, 'ok'), (True, 'falha')])
def test_rotina_ros_da_curva_para_e_confere_o_monitor(cm, resultado):
    # O modelo para 15 cm ANTES do alvo antigo. Portanto só a nova
    # confirmação pode concluir esta travessia.
    parametros = dict(margem_saida_lateral=.60, abertura_saida_curva=.70, angulo_fundido=False,
                      fundido_k_ff=.2, vel_lateral=.08, largura_passagem=.82, centro_corpo=.08)
    relogio = SimpleNamespace(t=0., passo=0)
    no = SimpleNamespace(lock=threading.Lock(), fator_corredor=.5, gyro_z=0., laser_tf=LASER,
                         p=parametros.__getitem__, _conferir_evidencia_passagem=lambda: None,
                         _esperar_sensores=lambda _: 0., _novo_goal_chegou=lambda: False,
                         _mapa=mapa, scan_seq=0, mensagens=[], comandos=[])
    no.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(relogio.t*1e9)))
    no._pose_mapa_yaw = lambda **_: no.odom[1:]
    no._pose_mapa = lambda: no.odom[1:3]
    no.get_logger = lambda: SimpleNamespace(info=no.mensagens.append)
    no._publicar_vel = lambda *cmd, **kw: no.comandos.append(cmd)
    no._parar = lambda: no.comandos.append((0., 0., 0.))
    paredes = [(x, i*.025) for x in (-.30, .52) for i in range(-15, 25)]
    def avancar(dt):
        relogio.t += dt
        relogio.passo += 1
        y = min(.45, -.55 + relogio.t*.12)
        od = (0., y, 0.)
        no.odom = (relogio.t, *od)
        no.cm_stop = cm and y > .37
        no.cmd_final = (relogio.t, not no.comandos or not any(no.comandos[-1]))
        if relogio.passo % 3 == 1:
            pontos = paredes if y < .30 else raios(1) + [(x, v) for x, v in paredes if x < 0]
            no.scan_seq += 1
            no.scan = (relogio.t, pontos, od, .15, relogio.t - .01)
    avancar(0.)
    rotina = metodo('_percorrer_de_lado')
    rotina.__globals__.update(TravessiaLateral=TravessiaLateral, SaidaCurva=SaidaCurva,
        abertura_observada=abertura_observada, FOOTPRINT=FOOTPRINT, medir_corredor_lateral=medir_corredor_lateral,
        folgas_lateral=folgas_lateral, rclpy=SimpleNamespace(ok=lambda: relogio.t < 20.),
        time=SimpleNamespace(monotonic=lambda: relogio.t, sleep=avancar))
    assert rotina(no, dict(eixo_b=(0, 1), saida_b=(0, 0), lado=1), 'curva') == resultado
    assert any('corpo fora da curva' in m for m in no.mensagens)
    assert any('saída da curva confirmada' in m for m in no.mensagens) == (not cm)
    assert no.comandos[-1] == (0., 0., 0.)
