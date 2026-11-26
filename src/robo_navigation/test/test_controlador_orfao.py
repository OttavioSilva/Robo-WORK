"""CONTROLADOR_ORFAO (11/11/2026): FollowPath orfao comandando durante a recuperacao (bag ensaio_goal_superior).

O destravar cancela o FollowPath (so quando quem comanda e o controlador) e espera parado a exclusividade.
A decisao continua exigindo exclusividade: comando de outra origem, ou controlador que nao para, ainda falha
sem mover o robo. Simulacao sem ROS (mesmo Recuperador do destravar_node).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_recuperacao import SimIO  # noqa: E402
from test_recuperacao_avancada import rodar, SALA, CAIXA_25, PELA_ESQ  # noqa: E402

JANELA = {'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2}


class SimOrfao(SimIO):
    """Controlador orfao: comanda (e o suavizador repassa em /cmd_vel_nav) ate ser cancelado + atraso."""
    def __init__(self, *a, para_apos_cancelar=0.2, obedece=True, **k):
        super().__init__(*a, **k)
        self.cancelamentos = []
        self.para_apos_cancelar, self.obedece = para_apos_cancelar, obedece
        self.logs = []

    def _ativo(self):
        if not self.cancelamentos or not self.obedece:
            return True
        return self.t < self.cancelamentos[0] + self.para_apos_cancelar

    def ultimo_ctrl(self):
        return self.t if self._ativo() else (self.cancelamentos[0] + self.para_apos_cancelar
                                             if self.cancelamentos else None)

    def ultimo_estrangeiro(self):   # saida do suavizador em /cmd_vel_nav acompanha o controlador
        return self.ultimo_ctrl()

    def cancelar_controlador(self):
        self.cancelamentos.append(self.t)
        return True

    def log(self, nivel, txt):
        self.logs.append((nivel, txt))
        try:
            super().log(nivel, txt)
        except AttributeError:
            pass


def test_orfao_cancelado_e_recuperacao_segue():
    io = SimOrfao([CAIXA_25] + SALA, waypoint=PELA_ESQ, janela=JANELA)
    ok, msg, rel, _ = rodar(io)
    assert len(io.cancelamentos) == 1 and io.cancelamentos[0] < 0.2
    assert any('CONTROLADOR_ORFAO: comando exclusivo' in t for _, t in io.logs)
    assert 'sem controle exclusivo' not in msg
    assert rel['decisoes'] and rel['decisoes'][0]['acao'] != 'falhar'
    assert io.colisoes == 0


def test_orfao_que_nao_para_continua_falhando_sem_mover():
    io = SimOrfao([CAIXA_25] + SALA, waypoint=PELA_ESQ, janela=JANELA, obedece=False)
    ok, msg, rel, _ = rodar(io)
    assert not ok and 'exclusivo' in msg
    assert len(io.cancelamentos) == 1
    assert math.hypot(io.x, io.y) < 1e-6 and io.cmd == (0.0, 0.0, 0.0)
    assert any('continua comandando' in t for _, t in io.logs)


def test_sem_suporte_a_cancelar_mantem_recusa_antiga():
    io = SimOrfao([CAIXA_25] + SALA, waypoint=PELA_ESQ, janela=JANELA)
    io.__class__ = type('SemMetodo', (SimIO,), {'ultimo_ctrl': lambda s: s.t, 'ultimo_estrangeiro': lambda s: s.t})
    ok, msg, rel, _ = rodar(io)
    assert not ok and 'exclusivo' in msg and math.hypot(io.x, io.y) < 1e-6


def test_comando_estrangeiro_que_nao_e_do_controlador_nao_e_cancelado():
    io = SimOrfao([CAIXA_25] + SALA, waypoint=PELA_ESQ, janela=JANELA)
    io.ultimo_ctrl = lambda: None            # so /cmd_vel_nav de outro no (ex.: alinhador)
    io.ultimo_estrangeiro = lambda: io.t
    ok, msg, rel, _ = rodar(io)
    assert io.cancelamentos == []
    assert not ok and 'exclusivo' in msg and math.hypot(io.x, io.y) < 1e-6


def test_cancelamento_do_goal_durante_a_espera_encerra():
    io = SimOrfao([CAIXA_25] + SALA, waypoint=PELA_ESQ, janela=JANELA, obedece=False, cancelar_em=0.6)
    ok, msg, rel, _ = rodar(io)
    assert not ok and io.t < 3.0 and io.cmd == (0.0, 0.0, 0.0)
