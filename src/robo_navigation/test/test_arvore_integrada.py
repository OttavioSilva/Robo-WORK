"""NAV_INTEGRADA: logica da arvore principal comparada com a anterior e a reduzida (offline, sem ROS).

Os servidores Nav2 sao simulados (bt_mini.py); os codigos de erro sao os do nav2_msgs Jazzy.
Estes testes provam a logica da arvore, nao o comportamento fisico.
"""
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bt_mini import Arvore, S, F, R  # noqa: E402

BT = Path(__file__).resolve().parents[1] / 'behavior_trees'
INTEGRADA = BT / 'navigate_to_pose_smooth.xml'
ANTERIOR = BT / 'navigate_to_pose_smooth_anterior_20261110.xml'
REDUZIDA = BT / 'navigate_to_pose_early_test.xml'

NO_VALID_CONTROL, FAILED_TO_MAKE_PROGRESS = 106, 105
NO_VALID_PATH = 208
SMOOTHED_PATH_IN_COLLISION = 503
CTRL_AJUDA = {100, 104, 105, 106}
PLAN_AJUDA = {200, 207, 208}
SUAV_AJUDA = {500, 502, 503, 504}


class Mundo:
    """Cenario configuravel. Caminhos: ('plan', n), ('smooth', n), ('rest', caminho)."""

    def __init__(self, duracao=6.0, invalido=None, planner=None, suaviza=None, seguir=None,
                 destravar=None, spin=None, goal_novo_em=None, original=None):
        self.n_plan = 0
        self.log = []
        self.duracao = duracao            # s de FollowPath ate chegar com o caminho atual
        self.invalido = invalido or (lambda mundo, cam, t: False)
        self.planner = planner or (lambda mundo, t: True)
        self.suaviza = suaviza or (lambda mundo, cam, check, t: True)
        self.seguir = seguir or (lambda mundo, t: None)        # None: normal; int: falha com o codigo
        self.destravar = destravar or (lambda mundo, t: True)
        self.spin = spin or (lambda mundo, t: True)
        self.goal_novo_em = goal_novo_em
        # CAMINHO_ORIGINAL (11/11): veredito da condicao CaminhoOriginalSeguro no modo "aplicar"
        self.original = original or (lambda mundo, cam, t: True)
        self.goal_do_caminho = {}
        self.t_follow = None
        self.progresso = 0.0
        self.estado_folha = {}
        self.arv = None
        self.goal = 'G0'

    def conta(self, nome):
        return sum(1 for e in self.log if e[1] == nome)

    def novo_goal(self, goal):
        self.goal = goal
        self.progresso = 0.0
        self.t_follow = None

    def passo(self, t):
        if self.goal_novo_em is not None and t >= self.goal_novo_em and self.goal == 'G0':
            self.goal = 'G1'
            self.arv.bb['goal'] = 'G1'

    def halt(self, no):
        self.log.append((self.arv.t, 'HALT:' + no.tag))
        if no.tag == 'FollowPath':
            self.t_follow = None

    def folha(self, no):
        t, tag, bb = self.arv.t, no.tag, self.arv.bb
        k = id(no)
        mem = self.estado_folha.setdefault(k, {})
        if tag in ('GlobalUpdatedGoal', 'GoalUpdated'):
            g = bb.get('goal')
            if 'g' not in mem:
                mem['g'] = g
                return F
            if g != mem['g']:
                mem['g'] = g
                return S
            return F
        if tag == 'PathExpiringTimer':
            cam = no.porta('path')
            if 'ini' not in mem or mem.get('cam') != cam:
                mem['ini'], mem['cam'] = t, cam
                return F
            if t - mem['ini'] > float(no.a.get('seconds', 1.0)):
                mem['ini'] = t
                return S
            return F
        if tag == 'TruncatePathLocal':
            cam = no.porta('input_path')
            if cam is None:
                return F
            no.escrever('output_path', ('rest', cam))
            return S
        if tag == 'IsPathValid':
            cam = no.porta('path')
            self.log.append((t, 'IsPathValid', cam))
            if cam is None:
                return F
            return F if self.invalido(self, cam, t) else S
        if tag == 'ComputePathToPose':
            ok = self.planner(self, t)
            self.log.append((t, 'ComputePathToPose', ok))
            if ok:
                self.n_plan += 1
                self.goal_do_caminho[self.n_plan] = self.arv.bb.get('goal')
                no.escrever('path', ('plan', self.n_plan))
                no.escrever('error_code_id', 0)
                return S
            no.escrever('error_code_id', NO_VALID_PATH)
            return F
        if tag == 'SmoothPath':
            cam = no.porta('unsmoothed_path')
            check = no.a.get('check_for_collisions', 'false') == 'true'
            ok = self.suaviza(self, cam, check, t)
            self.log.append((t, 'SmoothPath', ok))
            if ok:
                no.escrever('smoothed_path', ('smooth', cam[1]))
                no.escrever('error_code_id', 0)
                return S
            no.escrever('error_code_id', SMOOTHED_PATH_IN_COLLISION)
            return F
        if tag == 'FollowPath':
            cam = no.porta('path')
            if cam is None:
                return F
            if self.t_follow is None:
                self.t_follow = t
                self.log.append((t, 'FollowPath:start', cam))
            falha = self.seguir(self, t)
            if falha:
                self.t_follow = None
                no.escrever('error_code_id', falha)
                self.log.append((t, 'FollowPath:fail', falha))
                return F
            self.progresso += self.arv.dt
            if self.goal_do_caminho.get(cam[1]) != self.arv.bb.get('goal'):
                # caminho de OUTRO goal: o robo ja esta no fim dele; o controller declara chegada
                self.log.append((t, 'SUCESSO_FALSO', cam))
                no.escrever('error_code_id', 0)
                return S
            if self.progresso >= self.duracao:
                no.escrever('error_code_id', 0)
                return S
            return R
        if tag == 'ClearEntireCostmap':
            self.log.append((t, 'Clear', no.a.get('service_name')))
            return S
        if tag in ('DriveOnHeading', 'Spin', 'BackUp'):
            if 'ini' not in mem:
                mem['ini'] = t
                self.log.append((t, tag, no.a.get('server_name'), no.a.get('dist_to_travel')))
            if t - mem['ini'] < 1.0:
                return R
            del mem['ini']
            ok = (self.destravar if tag == 'DriveOnHeading' else self.spin)(self, t)
            self.log.append((t, tag + (':ok' if ok else ':fail')))
            return S if ok else F
        if tag.startswith('WouldA'):
            code = no.porta('error_code')
            sets = {'WouldAControllerRecoveryHelp': CTRL_AJUDA, 'WouldAPlannerRecoveryHelp': PLAN_AJUDA,
                    'WouldASmootherRecoveryHelp': SUAV_AJUDA}[tag]
            return S if code in sets else F
        if tag == 'CaminhoSemCarimbo':  # GOAL_TF_ATUAL (11/11): copia o caminho (o carimbo nao existe aqui)
            cam = no.porta('input_path')
            if cam is None:
                return F
            no.escrever('output_path', cam)
            return S
        if tag == 'CaminhoOriginalSeguro':  # CAMINHO_ORIGINAL (11/11)
            cam = no.porta('path')
            ok = cam is not None and self.original(self, cam, t)
            self.log.append((t, 'CaminhoOriginalSeguro', no.a.get('modo'), ok))
            if no.a.get('modo', 'registrar') == 'registrar':
                return S
            return S if ok else F
        raise AssertionError('folha nao simulada: ' + tag)


def rodar(xml, mundo, max_ticks=3000):
    arv = Arvore(xml, mundo)
    return arv.rodar(max_ticks), mundo


ARVORES = {'integrada': INTEGRADA, 'anterior': ANTERIOR, 'reduzida': REDUZIDA}


def test_arquivos_existem_e_a_anterior_foi_preservada():
    for p in ARVORES.values():
        assert p.exists(), p
    txt = ANTERIOR.read_text()
    assert 'RoundRobin' in txt and 'smart_spin' in txt and 'destravar' in txt


# 1. caminho livre
@pytest.mark.parametrize('nome', ARVORES)
def test_caminho_livre_chega_sem_replanejar(nome):
    st, m = rodar(ARVORES[nome], Mundo())
    assert st == S
    assert m.conta('ComputePathToPose') == 1
    assert m.conta('DriveOnHeading') == 0 and m.conta('Spin') == 0


def test_integrada_confere_o_caminho_a_2hz():
    st, m = rodar(INTEGRADA, Mundo(duracao=6.0))
    checks = [e for e in m.log if e[1] == 'IsPathValid']
    assert st == S and len(checks) >= 10                      # ~2 Hz em 6 s (anterior: 1 Hz)
    assert all(e[2][0] == 'rest' for e in checks)             # sempre o trecho restante


# 2/4. desvio antecipado: obstaculo novo no caminho restante
def _obstaculo_em(t0):
    return lambda m, cam, t: t >= t0 and m.n_plan == 1        # o 1o caminho fica invalido em t0


@pytest.mark.parametrize('nome', ARVORES)
def test_replaneja_com_obstaculo_novo_sem_parar_o_seguimento(nome):
    st, m = rodar(ARVORES[nome], Mundo(duracao=8.0, invalido=_obstaculo_em(2.3)))   # fora da fase dos tiques
    assert st == S
    planos = [e[0] for e in m.log if e[1] == 'ComputePathToPose']
    assert len(planos) == 2
    atraso = planos[1] - 2.3
    assert atraso <= (0.55 if nome != 'anterior' else 1.05)   # 2 Hz contra 1 Hz
    assert m.conta('FollowPath:fail') == 0 and m.conta('DriveOnHeading') == 0


def test_obstaculo_no_trecho_ja_percorrido_nao_replaneja():
    # invalido so quando conferido o caminho INTEIRO (o trecho percorrido tem o obstaculo)
    inv = lambda m, cam, t: t >= 2.0 and cam[0] == 'smooth'    # noqa: E731
    st_i, mi = rodar(INTEGRADA, Mundo(duracao=6.0, invalido=inv))
    st_a, ma = rodar(ANTERIOR, Mundo(duracao=6.0, invalido=inv))
    assert st_i == S and mi.conta('ComputePathToPose') == 1
    assert st_a == S and ma.conta('ComputePathToPose') > 1      # a anterior replanejava a toa


def test_caminho_valido_preservado_ate_15_s():
    st, m = rodar(INTEGRADA, Mundo(duracao=40.0))
    planos = [e[0] for e in m.log if e[1] == 'ComputePathToPose']
    assert st == S and 2 <= len(planos) <= 4
    assert all(b - a >= 14.9 for a, b in zip(planos, planos[1:]))


# 3. suavizacao em colisao: usa o original se ele for valido
def test_suavizacao_em_colisao_usa_o_caminho_original_seguro():
    suav = lambda m, cam, check, t: not check                  # noqa: E731  com checagem: colide
    st, m = rodar(INTEGRADA, Mundo(suaviza=suav))
    seguidos = [e[2] for e in m.log if e[1] == 'FollowPath:start']
    assert st == S and seguidos[0] == ('plan', 1)
    st_r, mr = rodar(REDUZIDA, Mundo(suaviza=suav))
    assert st_r == F                                            # a reduzida desistia


def test_suavizacao_colide_e_original_reprovado_pelo_isvalid_segue_o_original():
    # bag 10/11: o IsPathValid reprovava o caminho do planner pelo trecho do goal (zona inflada) e a
    # arvore parava o robo no meio do trajeto. ORIGINAL_SEM_GATE: segue o caminho do planner.
    suav = lambda m, cam, check, t: not check                  # noqa: E731
    inv = lambda m, cam, t: cam[0] == 'plan'                   # noqa: E731
    st, m = rodar(INTEGRADA, Mundo(suaviza=suav, invalido=inv))
    assert st == S and m.conta('DriveOnHeading') == 0


# CAMINHO_DO_GOAL: varios goals na MESMA arvore (o bt_navigator mantem o blackboard)
def _dois_goals(xml, mundo2):
    m = Mundo()
    arv = Arvore(xml, m)
    st1 = arv.navegar('G1')
    m2 = mundo2
    for k in ('n_plan', 'goal_do_caminho', 'estado_folha'):
        setattr(m2, k, getattr(m, k))
    m2.arv = arv
    arv.mundo = m2
    st2 = arv.navegar('G2')
    return st1, st2, m2


@pytest.mark.parametrize('caso', ['planner_falha', 'recuperacao_replan_falha'])
def test_sem_sucesso_falso_com_caminho_do_goal_anterior(caso):
    if caso == 'planner_falha':
        m2 = Mundo(planner=lambda m, t: False, destravar=lambda m, t: True)
    else:  # como no bag: o planner falha depois que o destravar devolve
        m2 = Mundo(planner=lambda m, t: m.conta('DriveOnHeading:ok') == 0 and False,
                   destravar=lambda m, t: True)
    st1, st2, m = _dois_goals(INTEGRADA, m2)
    assert st1 == S
    assert m.conta('SUCESSO_FALSO') == 0, m.log
    assert st2 == F                                         # falha explicita, nunca sucesso falso


def test_arvore_anterior_tinha_o_mesmo_furo():
    m2 = Mundo(planner=lambda m, t: False, destravar=lambda m, t: True)
    st1, st2, m = _dois_goals(ANTERIOR, m2)
    assert m.conta('SUCESSO_FALSO') >= 1                    # documenta: nao e regressao so da integrada


def test_goal_seguinte_normal_continua_funcionando():
    st1, st2, m = _dois_goals(INTEGRADA, Mundo())
    assert st1 == S and st2 == S and m.conta('SUCESSO_FALSO') == 0


# 5/9. DWB sem trajetoria / CM bloqueando: recupera e replaneja
@pytest.mark.parametrize('codigo', [NO_VALID_CONTROL, FAILED_TO_MAKE_PROGRESS])
def test_falha_do_controle_chama_destravar_e_replaneja(codigo):
    seg = lambda m, t: codigo if 1.0 <= t < 1.15 else None     # noqa: E731
    st, m = rodar(INTEGRADA, Mundo(seguir=seg))
    assert st == S
    i_rec = next(i for i, e in enumerate(m.log) if e[1] == 'DriveOnHeading')
    assert m.log[i_rec][2] == 'destravar'
    depois = [e for e in m.log[i_rec:] if e[1] == 'ComputePathToPose']
    assert depois                                               # caminho recalculado depois
    assert m.conta('Spin') == 0 and m.conta('Clear') == 0       # sem limpeza/giro cegos


def test_reduzida_desiste_onde_a_integrada_recupera():
    seg = lambda m, t: NO_VALID_CONTROL if 1.0 <= t < 1.15 else None  # noqa: E731
    assert rodar(REDUZIDA, Mundo(seguir=seg))[0] == F
    assert rodar(INTEGRADA, Mundo(seguir=seg))[0] == S


# 8/15. sem alternativa: falha explicita, sem repetir manobras as cegas
def test_sem_alternativa_falha_explicita_na_primeira_recusa():
    seg = lambda m, t: NO_VALID_CONTROL if t >= 1.0 else None  # noqa: E731
    st, m = rodar(INTEGRADA, Mundo(seguir=seg, destravar=lambda m, t: False))
    assert st == F
    assert m.conta('DriveOnHeading') == 1 and m.conta('Spin') == 0
    st_a, ma = rodar(ANTERIOR, Mundo(seguir=seg, destravar=lambda m, t: False))
    assert st_a == F and ma.conta('Spin') >= 1 and ma.conta('DriveOnHeading') >= 2  # anterior: em ciclo


def test_tentativas_limitadas_mesmo_se_o_destravar_insistir():
    seg = lambda m, t: NO_VALID_CONTROL if t >= 1.0 else None  # noqa: E731
    st, m = rodar(INTEGRADA, Mundo(seguir=seg))
    assert st == F and m.conta('DriveOnHeading') == 6


def test_planner_sem_caminho_limpa_global_uma_vez_e_depois_recupera():
    pl = lambda m, t: m.n_plan >= 1 or t >= 3.0                # noqa: E731
    calls = {'n': 0}

    def planner(m, t):
        calls['n'] += 1
        return calls['n'] >= 3
    st, m = rodar(INTEGRADA, Mundo(planner=planner))
    assert st == S
    limpezas = [e for e in m.log if e[1] == 'Clear']
    assert limpezas and all('global' in e[2] for e in limpezas)
    assert pl is not None


def test_goal_novo_interrompe_a_recuperacao():
    seg = lambda m, t: NO_VALID_CONTROL if 1.0 <= t < 1.15 else None  # noqa: E731
    st, m = rodar(INTEGRADA, Mundo(seguir=seg, goal_novo_em=1.5))
    assert st == S
    assert any(e[1] == 'HALT:DriveOnHeading' for e in m.log)


def test_falha_do_planner_depois_da_recuperacao_nao_derruba_a_navegacao():
    seg = lambda m, t: NO_VALID_CONTROL if 1.0 <= t < 1.15 else None  # noqa: E731
    estado = {'pos': False}

    def planner(m, t):
        if m.conta('DriveOnHeading:ok') and not estado['pos']:
            estado['pos'] = True
            return False                                       # o 1o depois da recuperacao falha
        return True
    st, m = rodar(INTEGRADA, Mundo(seguir=seg, planner=planner))
    assert st == S


# CAMINHO_ORIGINAL (11/11): Patch 01. O original do planner so vai ao FollowPath se a condicao aprovar
# (modo "aplicar"); no modo "registrar" (o instalado) o comportamento e o de antes.
def _integrada_aplicar(tmp_path):
    txt = INTEGRADA.read_text()
    assert txt.count('modo="registrar" folga_inicio') == 2, 'a arvore instalada deve vir em modo registrar'
    p = tmp_path / 'aplicar.xml'
    p.write_text(txt.replace('modo="registrar" folga_inicio', 'modo="aplicar" folga_inicio'))
    return p


def test_original_registrar_mantem_o_comportamento_anterior():
    suav = lambda m, cam, check, t: not check                  # noqa: E731
    bloq = lambda m, cam, t: False                              # noqa: E731  reprovaria sempre
    st, m = rodar(INTEGRADA, Mundo(suaviza=suav, original=bloq))
    seguidos = [e[2] for e in m.log if e[1] == 'FollowPath:start']
    assert st == S and seguidos[0] == ('plan', 1) and m.conta('DriveOnHeading') == 0
    assert any(e[1] == 'CaminhoOriginalSeguro' and e[3] is False for e in m.log)  # registrou


def test_original_aprovado_vai_ao_followpath(tmp_path):
    suav = lambda m, cam, check, t: not check                  # noqa: E731
    st, m = rodar(_integrada_aplicar(tmp_path), Mundo(suaviza=suav))
    seguidos = [e[2] for e in m.log if e[1] == 'FollowPath:start']
    assert st == S and seguidos[0] == ('plan', 1) and m.conta('DriveOnHeading') == 0


def test_original_bloqueado_nao_vai_ao_followpath_e_recupera(tmp_path):
    # bloqueio nos 2 primeiros planos; o destravar resolve e o 3o plano passa
    suav = lambda m, cam, check, t: not check                  # noqa: E731
    bloq = lambda m, cam, t: cam[1] >= 3                       # noqa: E731
    st, m = rodar(_integrada_aplicar(tmp_path), Mundo(suaviza=suav, original=bloq))
    seguidos = [e[2] for e in m.log if e[1] == 'FollowPath:start']
    assert st == S
    assert ('plan', 1) not in seguidos and ('plan', 2) not in seguidos
    assert m.conta('DriveOnHeading') >= 1                      # recuperacao avaliada (destravar v4)
    assert m.conta('SUCESSO_FALSO') == 0


def test_original_bloqueado_sempre_falha_explicita_limitada(tmp_path):
    suav = lambda m, cam, check, t: not check                  # noqa: E731
    bloq = lambda m, cam, t: False                              # noqa: E731
    st, m = rodar(_integrada_aplicar(tmp_path), Mundo(suaviza=suav, original=bloq), max_ticks=6000)
    assert st == F                                              # nao segue caminho bloqueado
    assert m.conta('FollowPath:start') == 0
    assert 1 <= m.conta('DriveOnHeading') <= 6                  # RecoveryNode de 6 tentativas


def test_sem_suavizador_original_bloqueado_falha_mesmo_com_suavizacao_ok(tmp_path):
    bloq = lambda m, cam, t: False                              # noqa: E731
    st, m = rodar(_integrada_aplicar(tmp_path), Mundo(original=bloq))
    assert st == F and m.conta('CaminhoOriginalSeguro') > 0
    assert m.conta('FollowPath:start') == 0
    assert m.conta('SmoothPath') == 0


def test_original_goal_novo_durante_bloqueio(tmp_path):
    suav = lambda m, cam, check, t: not check                  # noqa: E731
    bloq = lambda m, cam, t: m.arv.bb.get('goal') == 'G1'      # noqa: E731  so o goal novo passa
    st, m = rodar(_integrada_aplicar(tmp_path), Mundo(suaviza=suav, original=bloq, goal_novo_em=3.0))
    assert st == S and m.conta('SUCESSO_FALSO') == 0
    seguidos = [e[2] for e in m.log if e[1] == 'FollowPath:start']
    assert seguidos and all(m.goal_do_caminho[c[1]] == 'G1' for c in seguidos)


# GOAL_TF_ATUAL (11/11): o FollowPath recebe a copia do caminho feita pelo CaminhoSemCarimbo
def test_followpath_recebe_a_copia_do_caminho_atual():
    suav = lambda m, cam, check, t: not check                  # noqa: E731  fallback da suavizacao
    for mundo in (Mundo(), Mundo(suaviza=suav)):
        st, m = rodar(INTEGRADA, mundo)
        seguidos = [e[2] for e in m.log if e[1] == 'FollowPath:start']
        assert st == S and seguidos and seguidos[0][1] == 1
        assert m.conta('SUCESSO_FALSO') == 0


def test_sem_suavizador_nunca_chama_smoothpath():
    def proibido(*args):
        raise AssertionError('SmoothPath chamado com SEM_SUAVIZADOR')
    st, m = rodar(INTEGRADA, Mundo(suaviza=proibido))
    assert st == S and m.conta('CaminhoOriginalSeguro') > 0
    assert m.conta('SmoothPath') == 0
