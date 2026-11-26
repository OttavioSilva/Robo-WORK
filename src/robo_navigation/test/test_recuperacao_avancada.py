"""RECUPERACAO_AVANCADA: decisao + execucao do destravar v4 em simulacao 2D (sem ROS, sem robo).

Cada teste roda o MESMO Recuperador que o destravar_node.py usa, contra sim_recuperacao.SimIO.
Nao prova seguranca fisica: prova a logica de decisao, os bloqueios e a verificacao.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_recuperacao import SimIO, recuperador  # noqa: E402
import recuperacao_avancada as ra  # noqa: E402

# arena fechada (raios sem retorno a 8 m nao existem na prova)
SALA = [(-2.1, 4.1, 1.5, 1.6), (-2.1, 4.1, -1.6, -1.5), (4.0, 4.1, -1.6, 1.6), (-2.1, -2.0, -1.6, 1.6)]
CAIXA_25 = (0.625, 0.825, -0.10, 0.10)     # caixa 20x20 a 25 cm do para-choque
CAIXA_8 = (0.455, 0.655, -0.10, 0.10)      # a 8 cm
PELA_ESQ = (lambda io: (1.0, 0.75))        # o planner contorna pela esquerda


def rodar(io, dist=0.40, tmax=30.0, **cfg):
    rec = recuperador(io, **cfg)
    ok, msg, rel = rec.executar(dist, 0.03, tmax)
    return ok, msg, rel, rec


def acoes(rel):
    return [d['acao'] for d in rel['decisoes']]


def recusa(rel, acao):
    return ' '.join(r for d in rel['decisoes'] for r in d['recusas'] if r.startswith(acao + ':'))


# ------------------------------------------------------------------ ETAPA A
def test_diagnostico_distingue_as_causas():
    base = dict(planner='ok', ctrl_parado_s=0.1, ctrl_ativo_s=0.0)
    assert ra.diagnosticar(ra.Evidencias(caminho_valido=False, **base)) == ['caminho_invalido']
    assert 'dwb_sem_trajetoria' in ra.diagnosticar(ra.Evidencias(planner='ok', ctrl_parado_s=3.0))
    assert 'cm_bloqueou' in ra.diagnosticar(ra.Evidencias(cm_stop_s=0.5, **base))
    sem = ra.Evidencias(ctrl_ativo_s=3.0, desloc_janela=0.005, giro_janela=0.01, planner='ok', ctrl_parado_s=0.1)
    assert 'sem_progresso_fisico' in ra.diagnosticar(sem)
    andou = ra.Evidencias(ctrl_ativo_s=3.0, desloc_janela=0.20, planner='ok', ctrl_parado_s=0.1)
    assert 'sem_progresso_fisico' not in ra.diagnosticar(andou)
    assert 'oscilacao' in ra.diagnosticar(ra.Evidencias(inversoes=5, **base))
    assert 'sem_caminho_global' in ra.diagnosticar(ra.Evidencias(planner='sem_caminho'))
    # comando sem medida fisica nao vira "sem progresso" (a odom e o comando: nao serve de prova)
    assert 'sem_progresso_fisico' not in ra.diagnosticar(
        ra.Evidencias(ctrl_ativo_s=3.0, desloc_janela=None, planner='ok', ctrl_parado_s=0.1))


def test_quina_detectada_pelo_canto():
    io = SimIO([(0.40, 0.8, 0.22, 1.0)] + SALA, waypoint=lambda io: (1.2, 0.0))
    assert ra.detectar_quina(io.scan().pontos()) == 'esq'
    io = SimIO([CAIXA_8] + SALA)
    assert ra.detectar_quina(io.scan().pontos()) is None


# ------------------------------------------------------------------ ETAPA B
def test_caixa_parcial_com_contorno_normal_devolve_ao_nav2_sem_mexer():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ)
    ok, msg, rel, _ = rodar(io)
    assert ok and acoes(rel) == ['retomar']
    assert math.hypot(io.x, io.y) < 1e-6 and io.colisoes == 0     # recuperacao fisica desnecessaria evitada


def test_retomar_nao_se_repete_no_mesmo_lugar():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ)
    ok, _, rel, rec = rodar(io)
    assert ok and acoes(rel) == ['retomar']
    ok, _, rel = rec.executar(0.40, 0.03, 30.0)          # o DWB voltou a travar no mesmo lugar
    assert acoes(rel)[0] != 'retomar'
    assert 'ja devolvido ao Nav2' in recusa(rel, 'retomar')


# ------------------------------------------------------------------ ETAPA C
def test_oscilacao_vira_escape_lateral_verificado_pelo_icp():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ,
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, rec = rodar(io)
    assert ok and acoes(rel) == ['lateral'], msg
    assert io.y > 0.10 and io.colisoes == 0              # foi para o lado do caminho, sem encostar
    assert 'ICP' in msg
    assert rec.hist.itens[-1].resultado == 'ok'


def test_lateral_escolhe_lado_pela_continuacao_nao_pelo_raio_mais_longo():
    # esquerda: muito espaco ate a parede, mas uma caixa bloqueia a frente depois do deslize;
    # direita: menos espaco lateral, porem com o caminho livre depois.
    mundo = [CAIXA_25, (0.55, 1.5, 0.30, 0.90), (-2.1, 4.1, -1.0, -0.95)] + SALA
    io = SimIO(mundo, waypoint=lambda io: (1.2, -0.6),
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, _ = rodar(io)
    assert ok and acoes(rel) == ['lateral'], msg
    assert io.y < -0.05 and io.colisoes == 0


def test_cruzamento_em_t_desliza_para_o_ramo_sem_girar():
    mundo = [(0.475, 0.6, -1.5, 1.5), (-2.0, -0.35, 0.45, 0.55), (-2.0, -0.35, -0.55, -0.45),
             (-2.1, 0.6, 1.5, 1.6), (-2.1, 0.6, -1.6, -1.5), (-2.1, -2.0, -1.6, 1.6)]
    io = SimIO(mundo, goal=(0.05, 1.2))
    ok, msg, rel, _ = rodar(io)
    assert ok and acoes(rel) == ['retomar'], msg          # DWB direto anda de lado no ramo: sem recuperacao
    io = SimIO(mundo, goal=(0.05, 1.2), janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, _ = rodar(io)
    assert ok and acoes(rel) in (['lateral'], ['mover']), msg
    assert io.spins == [] and io.y > 0.03 and io.colisoes == 0


def test_quina_sem_re_falha_explicitamente_com_causa():
    io = SimIO([(0.40, 0.8, 0.22, 1.0)] + SALA, waypoint=lambda io: (1.2, 0.0))
    ok, msg, rel, _ = rodar(io)
    assert not ok and acoes(rel) == ['falhar']
    assert 'BLOQUEADA' in recusa(rel, 're')
    assert io.colisoes == 0 and math.hypot(io.x, io.y) < 1e-6


# ------------------------------------------------------------------ ETAPA D/E
def test_re_recusada_por_padrao_e_registrada():
    io = SimIO([CAIXA_8] + SALA, waypoint=PELA_ESQ)
    ok, msg, rel, _ = rodar(io)
    assert not ok
    assert 'permitir_re=false' in recusa(rel, 're')
    assert math.hypot(io.x, io.y) < 1e-6


def test_re_recusada_sem_cobertura_traseira():
    io = SimIO([CAIXA_8] + SALA, waypoint=PELA_ESQ, sombra_traseira=(2.6, 3.15))
    io._gerar_scan()
    ok, msg, rel, _ = rodar(io, permitir_re=True)
    assert not ok and 'observada' in recusa(rel, 're')
    assert math.hypot(io.x, io.y) < 1e-6


def test_re_recusada_com_obstaculo_atras():
    io = SimIO([CAIXA_8, (-0.40, -0.30, -0.3, 0.3)] + SALA, waypoint=PELA_ESQ)
    ok, msg, rel, _ = rodar(io, permitir_re=True)
    assert not ok and ('ponto' in recusa(rel, 're') or 'Collision Monitor' in recusa(rel, 're'))
    assert io.colisoes == 0


def test_re_recusada_quando_o_icp_nao_conseguiria_confirmar():
    # corredor longo e liso para tras: nada para o ICP medir ao longo dele
    mundo = [CAIXA_8, (-6.0, 0.45, 0.42, 0.45), (-6.0, 0.45, -0.45, -0.42)]
    io = SimIO(mundo, waypoint=PELA_ESQ, inf=0.0)
    ok, msg, rel, _ = rodar(io, permitir_re=True)
    assert not ok
    r = recusa(rel, 're')
    assert 'verificavel' in r or 'observada' in r or 'Collision' in r, r


def test_re_seguida_de_reavaliacao_em_simulacao():
    io = SimIO([CAIXA_8] + SALA, waypoint=PELA_ESQ)
    ok, msg, rel, rec = rodar(io, permitir_re=True)
    assert ok, msg
    a = acoes(rel)
    assert a[0] == 're' and len(a) == 2 and a[1] != 're'          # nunca duas res; segunda decisao NOVA
    assert any('REAVALIACAO' in l for _, l in io.logs)
    i_re = next(i for i, (_, l) in enumerate(io.logs) if l.startswith('RESULTADO re'))
    i_rea = next(i for i, (_, l) in enumerate(io.logs) if 'REAVALIACAO' in l)
    i_dec2 = max(i for i, (_, l) in enumerate(io.logs) if l.startswith('DECISAO'))
    assert i_re < i_rea < i_dec2                                   # para, confirma, reavalia, decide
    assert 'ICP' in rec.hist.itens[0].detalhe and rec.hist.itens[0].resultado == 'ok'
    assert io.x < -0.05 and io.colisoes == 0


def test_re_nao_confirmada_nao_vira_sucesso():
    io = SimIO([CAIXA_8] + SALA, waypoint=PELA_ESQ)
    rec = recuperador(io, permitir_re=True)
    # robo "patinando": o comando sai, a odom anda, o corpo nao (sem evidencia no LiDAR)
    andar = io.dormir

    def patina(s):
        x, y = io.x, io.y
        andar(s)
        io.x, io.y = x, y
        io._gerar_scan()
    io.dormir = patina
    ok, msg, rel = rec.executar(0.40, 0.03, 30.0)
    assert not ok
    assert rec.hist.itens and rec.hist.itens[0].acao == 're' and rec.hist.itens[0].resultado != 'ok'


# ------------------------------------------------------------------ ETAPA F
def test_giro_quando_o_caminho_esta_atras_e_ha_espaco():
    io = SimIO(SALA, goal=(-1.5, 0.0))
    ok, msg, rel, _ = rodar(io)
    assert ok and acoes(rel) == ['giro'], msg
    assert io.spins and abs(io.spins[0]) <= 0.70 + 1e-9 and 'IMU' in msg
    assert 'continuacao' not in msg


def test_giro_recusado_por_falta_de_espaco():
    io = SimIO([(-2.0, 2.0, 0.40, 0.50), (-2.0, 2.0, -0.50, -0.40), (2.0, 2.1, -0.5, 0.5),
                (-2.1, -2.0, -0.5, 0.5)], goal=(-1.5, 0.0))
    ok, msg, rel, _ = rodar(io)
    assert not ok and io.spins == []
    assert 'ponto' in recusa(rel, 'giro') or 'Collision' in recusa(rel, 'giro')


def test_giro_que_nao_resolveu_nao_se_repete():
    io = SimIO(SALA, goal=(-1.5, 0.0), spin_ok=False)
    ok, msg, rel, rec = rodar(io)
    assert io.spins and rec.hist.itens[0].acao == 'giro' and rec.hist.itens[0].resultado != 'ok'
    n = len(io.spins)
    ok, msg, rel = rec.executar(0.40, 0.03, 30.0)
    assert len(io.spins) == n and 'giro' not in acoes(rel)


def test_lateral_preferido_ao_giro_quando_resolve():
    # caminho a 60 graus a esquerda, lado livre: desliza, nao gira
    io = SimIO([CAIXA_25] + SALA, waypoint=lambda io: (0.4, 0.75),
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, _ = rodar(io)
    assert ok and io.spins == [] and acoes(rel)[0] in ('lateral', 'mover'), msg


# ------------------------------------------------------------------ ETAPA G
def test_recuperacao_sem_progresso_detecta_retorno_ao_mesmo_bloqueio():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ,
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    rec = recuperador(io)
    feitas = []
    for _ in range(5):
        io.x, io.y, io.th = 0.0, 0.0, 0.0        # o Nav2 trouxe o robo de volta ao mesmo lugar
        io.ox, io.oy, io.oth = 0.0, 0.0, 0.0
        io._gerar_scan()
        ok, msg, rel = rec.executar(0.40, 0.03, 30.0)
        feitas.append((ok, acoes(rel), msg))
        if not ok:
            break
    executadas = [a for ok_, ac, _ in feitas for a in ac if a not in ('falhar', 'retomar')]
    assert len(executadas) == len(set(executadas)), feitas        # nenhuma manobra repetida no mesmo lugar
    assert not feitas[-1][0]                                      # termina em falha explicita
    assert 'mesmo bloqueio' in feitas[-1][2] or 'sem alternativa' in feitas[-1][2] or \
        'nenhuma manobra' in feitas[-1][2]
    assert any(i.resultado == 'sem_efeito' for i in rec.hist.itens)


def test_goal_novo_zera_o_historico():
    h = ra.Historico()
    h.novo_goal((1.0, 1.0))
    h.registrar(0.0, 0.0, 0.0, 'lateral', 'falhou')
    assert 'lateral' in h.bloqueadas(1.0, 0.0, 0.0)
    h.novo_goal((1.05, 1.0))
    assert 'lateral' in h.bloqueadas(1.0, 0.0, 0.0)
    h.novo_goal((3.0, 1.0))
    assert not h.bloqueadas(1.0, 0.0, 0.0)


# ------------------------------------------------------------------ cadeia de comando
def test_collision_monitor_inativo_bloqueia_todas_as_manobras():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ, cm_ativo=False,
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, _ = rodar(io)
    assert not ok and 'Collision Monitor inativo' in msg and math.hypot(io.x, io.y) < 1e-6


def test_collision_monitor_bloqueando_movimento():
    # caixa a 3 cm do lado esquerdo: o CM pararia o deslize para a esquerda
    io = SimIO([(-0.1, 0.3, 0.355, 0.6), CAIXA_25] + SALA, waypoint=PELA_ESQ,
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2, 'cm_stop_s': 0.4})
    ok, msg, rel, _ = rodar(io)
    assert 'cm_bloqueou' in rel['causas']
    assert 'Collision Monitor barraria' in recusa(rel, 'lateral') or io.y < 0
    assert io.y <= 0.0 + 1e-6 and io.colisoes == 0


def test_comando_estrangeiro_impede_manobra():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ, estrangeiro_em=0.0,
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, _ = rodar(io)
    assert not ok and 'exclusivo' in msg


def test_comando_estrangeiro_no_meio_da_manobra_para():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ, estrangeiro_em=2.0,
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, rec = rodar(io)
    assert 'estrangeiro' in rec.hist.itens[0].detalhe
    assert io.cmd == (0.0, 0.0, 0.0)


def test_cancelamento_durante_recuperacao_para_e_nao_e_sucesso():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ, cancelar_em=2.0,
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, _ = rodar(io)
    assert not ok and 'cancelado' in msg
    assert io.cmd == (0.0, 0.0, 0.0) and io.t < 4.0


def test_cancelamento_durante_giro():
    io = SimIO(SALA, goal=(-1.5, 0.0), cancelar_em=2.5)
    ok, msg, rel, _ = rodar(io)
    assert not ok and 'cancelado' in msg and io.cmd == (0.0, 0.0, 0.0)


def test_sensor_parado_nao_autoriza():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ)
    io.t = 5.0                                   # ultimo scan ficou 5 s para tras
    io._t_scan = 1e9                             # e nao chega outro
    ok, msg, rel, _ = rodar(io)
    assert not ok and 'scan' in msg


def test_timeout_para_o_robo():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ,
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, _ = rodar(io, tmax=4.0)
    assert io.cmd == (0.0, 0.0, 0.0) and io.t < 9.0


# ------------------------------------------------------------------ falha segura
def test_sem_alternativa_falha_explicita_sem_mover():
    caixa_fechada = [(0.40, 0.6, -0.6, 0.6), (-0.6, 0.4, 0.36, 0.5), (-0.6, 0.4, -0.5, -0.36),
                     (-0.4, -0.25, -0.5, 0.5)]
    io = SimIO(caixa_fechada, goal=(2.0, 0.0), planner=False)
    ok, msg, rel, _ = rodar(io, permitir_re=True)
    assert not ok and acoes(rel) == ['falhar'] and 'sem alternativa segura' in msg
    assert math.hypot(io.x, io.y) < 1e-6 and io.spins == [] and io.colisoes == 0
    assert 'sem_caminho_global' in rel['causas']


@pytest.mark.parametrize('semente', [1, 2, 3, 4, 5])
def test_nenhuma_colisao_em_cenarios_com_ruido(semente):
    for mundo, wp, jan in (([CAIXA_25] + SALA, PELA_ESQ, {'inversoes': 6, 'ctrl_ativo_s': 3.0}),
                           ([CAIXA_8] + SALA, PELA_ESQ, {}),
                           ([(0.40, 0.8, 0.22, 1.0)] + SALA, lambda io: (1.2, 0.0), {})):
        for re_ in (False, True):
            io = SimIO(mundo, waypoint=wp, janela=jan, semente=semente, inf=0.35)
            rodar(io, permitir_re=re_)
            assert io.colisoes == 0


# ------------------------------------------------------------------ bag 10/11 (nav_integrada_corredor_aberta)
def test_sem_caminho_e_encostado_afasta_do_lado_mais_livre():
    # parede a 3 cm do lado esquerdo, sem caminho global: o v4 desistia; agora se afasta (v2/v3)
    io = SimIO([(-0.4, 0.5, 0.355, 0.45)] + SALA, planner=False)
    ok, msg, rel, rec = rodar(io)
    assert ok and acoes(rel)[-1] == 'afastar', msg
    assert io.y < -0.03 and io.colisoes == 0
    assert rec.hist.itens[-1].resultado == 'ok'


def test_sem_caminho_e_nao_encostado_nao_se_mexe_a_toa():
    io = SimIO(SALA, planner=False)
    ok, msg, rel, _ = rodar(io)
    assert not ok and math.hypot(io.x, io.y) < 1e-6
    assert 'nao esta encostado' in recusa(rel, 'afastar')


def test_passagem_fechada_por_fantasma_espera_limpa_e_replaneja():
    chamadas = {'n': 0}

    def planner(io):
        chamadas['n'] += 1
        return chamadas['n'] >= 2          # o costmap reabre depois de limpar e esperar
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ, planner=planner)
    ok, msg, rel, _ = rodar(io)
    assert ok and acoes(rel) == ['retomar'], msg
    assert io.limpezas == 1 and any('SEM CAMINHO' in l for _, l in io.logs)
    assert math.hypot(io.x, io.y) < 1e-6           # resolveu sem manobra fisica


def test_passagem_fechada_desiste_depois_de_duas_esperas():
    io = SimIO([CAIXA_25] + SALA, waypoint=PELA_ESQ, planner=False)
    ok, msg, rel, _ = rodar(io)
    assert not ok and io.limpezas == 2 and 'sem_caminho_global' in rel['causas']
