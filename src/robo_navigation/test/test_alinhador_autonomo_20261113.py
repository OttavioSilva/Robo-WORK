"""Melhorias de 13/11/2026 (bag arena_start_20261113_112926), SEM ROS e sem robo:

ORIENTAR_CAMINHO (destravar), SEM_CAMINHO_UNICO (destravar), PULSO_GIRO_ADAPTA (alinhador, saida lateral) e
CURTO_SEM_PROGRESSO (alinhador, trajeto curto). O destravar roda o MESMO Recuperador do no contra o simulador 2D
(sim_recuperacao.SimIO); a saida lateral roda a MESMA TravessiaLateral contra um modelo simples de motor.
Prova a logica (decisao, parada, verificacao, historico, exclusividade); NAO prova desempenho fisico.
"""
import ast
import importlib.util
import math
import subprocess
import sys
import time
import types
from pathlib import Path

import pytest

TEST = Path(__file__).resolve().parent
NAV = TEST.parent
WS = NAV.parents[1]
sys.path.insert(0, str(TEST))
sys.path.insert(0, str(NAV / 'scripts'))
from sim_recuperacao import SimIO, recuperador  # noqa: E402
import recuperacao_avancada as ra  # noqa: E402
import alinhador_passagem as ap  # noqa: E402

SALA = [(-2.1, 4.1, 1.5, 1.6), (-2.1, 4.1, -1.6, -1.5), (4.0, 4.1, -1.6, 1.6), (-2.1, -2.0, -1.6, 1.6)]
CAIXA_25 = (0.625, 0.825, -0.10, 0.10)
CORREDOR = [(-2.0, 2.0, 0.40, 0.50), (-2.0, 2.0, -0.50, -0.40), (2.0, 2.1, -0.5, 0.5), (-2.1, -2.0, -0.5, 0.5)]
BACKUP = WS / 'entregas/alinhador_autonomo_20261113/backup_antes'
G = math.degrees


def rodar(io, tmax=30.0, **cfg):
    cfg.setdefault('giro_orientar', True)
    rec = recuperador(io, **cfg)
    ok, msg, rel = rec.executar(0.40, 0.03, tmax)
    return ok, msg, rel, rec


def acoes(rel):
    return [d['acao'] for d in rel['decisoes']]


def recusa(rel, acao):
    return ' '.join(r for d in rel['decisoes'] for r in d['recusas'] if r.startswith(acao + ':'))


def motor_travado(io):
    """O robo nao gira (motor nao arranca / CM fora do sim): o comando nao muda o yaw."""
    original = io.dormir

    def dormir(s):
        vx, vy, wz = io.cmd
        if abs(wz) > 1e-6 and abs(vx) < 1e-6 and abs(vy) < 1e-6:
            io.cmd = (0.0, 0.0, 0.0)
            original(s)
            io.cmd = (vx, vy, wz)
        else:
            original(s)
    io.dormir = dormir


# ================================================================== ORIENTAR_CAMINHO
def test_1_rota_longa_caminho_atras_orienta_de_verdade():
    # cenario do bag 11:41: caminho a ~+165 graus; antes: varredura de 40 graus que parava em 7
    io = SimIO(SALA, goal=(-1.5, 0.3))
    ok, msg, rel, rec = rodar(io)
    assert ok and acoes(rel) == ['orientar'], msg
    assert io.spins == [] and io.colisoes == 0          # nao usou o smart_spin (varredura)
    alvo = math.atan2(0.3, -1.5)
    assert abs(ra.ang_norm(io.th - alvo)) < math.radians(8)   # ficou de frente para o caminho
    assert 'IMU: girou' in msg


def test_desligado_igual_a_antes_varredura_de_40_graus():
    io = SimIO(SALA, goal=(-1.5, 0.0))
    ok, msg, rel, _ = rodar(io, giro_orientar=False)
    assert ok and acoes(rel) == ['giro'] and io.spins and abs(io.spins[0]) <= 0.70 + 1e-9


@pytest.mark.parametrize('graus', [90, 180, 360, -90, -180])
def test_2_giros_90_180_360_medidos_pela_imu_acumulada(graus):
    io = SimIO(SALA)
    rec = recuperador(io, giro_orientar=True)
    motivo, girou = rec.girar_orientar(math.radians(graus), io.t + 90.0)
    assert motivo == 'chegou', motivo
    assert abs(G(girou) - graus) <= 5.0          # 360 nao vira 0: o yaw e acumulado
    assert io.colisoes == 0 and io.cmd == (0.0, 0.0, 0.0)
    assert io.max_vel == 0.0                      # giro parado: nenhuma translacao


def test_velocidade_de_giro_nunca_passa_do_configurado():
    io = SimIO(SALA)
    rec = recuperador(io, giro_orientar=True, giro_orientar_wz=0.20)
    wz = []
    enviar = io.enviar
    io.enviar = lambda vx, vy, w: (wz.append(abs(w)), enviar(vx, vy, w))
    rec.girar_orientar(math.radians(120), io.t + 60.0)
    assert wz and max(wz) <= 0.20 + 1e-9 and min(w for w in wz if w > 0) >= 0.15 - 1e-9


def test_3_giro_interrompido_por_obstaculo_que_aparece():
    io = SimIO(SALA)
    rec = recuperador(io, giro_orientar=True)
    dormir = io.dormir
    caixa = (-0.05, 0.15, 0.52, 0.70)            # aparece a ~0,5 m, na frente do canto que esta girando

    def dormir_com_caixa(s):
        if io.t >= 2.0 and caixa not in io.caixas:
            io.caixas.append(caixa)
        dormir(s)
    io.dormir = dormir_com_caixa
    motivo, girou = rec.girar_orientar(math.radians(180), io.t + 60.0)
    assert motivo.startswith('LiDAR') or 'Collision' in motivo, motivo
    assert 0 < G(girou) < 170 and io.colisoes == 0 and io.cmd == (0.0, 0.0, 0.0)


def test_4_caixa_com_alternativa_pela_esquerda_prefere_contorno():
    io = SimIO([CAIXA_25] + SALA, waypoint=lambda io: (0.4, 0.75),
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, _ = rodar(io)
    assert ok and acoes(rel)[0] == 'lateral' and io.y > 0.03 and io.colisoes == 0, msg


def test_5_caixa_com_alternativa_pela_direita_prefere_contorno():
    io = SimIO([CAIXA_25] + SALA, waypoint=lambda io: (0.4, -0.75),
               janela={'inversoes': 6, 'ctrl_ativo_s': 3.0, 'ctrl_parado_s': 0.2})
    ok, msg, rel, _ = rodar(io)
    assert ok and acoes(rel)[0] == 'lateral' and io.y < -0.03 and io.colisoes == 0, msg


def test_6_sem_passagem_segura_para_sem_girar():
    io = SimIO(CORREDOR, goal=(-1.5, 0.0))
    th0 = io.th
    ok, msg, rel, _ = rodar(io)
    assert not ok and io.spins == [] and io.colisoes == 0
    assert 'orientar' not in acoes(rel) and abs(io.th - th0) < 1e-6
    assert recusa(rel, 'orientar')                # o motivo fica registrado


def test_orientar_pelo_outro_lado_quando_o_curto_esta_barrado(monkeypatch):
    original = ra.avaliar

    def avaliar(raios, movs, **kw):
        if len(movs) == 1 and movs[0][:2] == (0.0, 0.0) and 0 < movs[0][2] < math.pi + 1e-6:
            return ra.Avaliacao(False, 3, 1.0, 10, '3 ponto(s) do LiDAR na area varrida')
        return original(raios, movs, **kw)
    monkeypatch.setattr(ra, 'avaliar', avaliar)
    io = SimIO(SALA, goal=(-1.5, 0.5))
    es = ra.Estado(raios=io.scan(), ang_caminho=io.direcao_caminho(), causas=['sem_caminho_global'],
                   bloqueadas=set(), planner_ok=True)
    dec = ra.decidir(es, ra.Config(giro_orientar=True))
    ang = io.direcao_caminho()
    assert dec.acao == 'orientar' and dec.mov[2] < 0
    assert abs(abs(dec.mov[2]) - (2 * math.pi - ang)) < 1e-6 and 'outro lado' in dec.motivo
    dec = ra.decidir(es, ra.Config(giro_orientar=True, giro_orientar_max=math.pi))
    assert dec.acao != 'orientar'
    assert 'acima do limite' in ' '.join(r[1] for r in dec.recusas if r[0] == 'orientar')


def test_nao_orienta_por_direcao_antiga_sem_caminho_atual():
    io = SimIO(SALA, goal=(-1.5, 0.0))
    es = ra.Estado(raios=io.scan(), ang_caminho=math.radians(170), causas=['sem_caminho_global'],
                   bloqueadas=set(), planner_ok=False)
    dec = ra.decidir(es, ra.Config(giro_orientar=True))
    assert dec.acao != 'orientar'
    assert 'direcao antiga' in ' '.join(r[1] for r in dec.recusas if r[0] == 'orientar')


def test_8_giro_seguido_de_retomada_do_goal():
    io = SimIO(SALA, goal=(-1.5, 0.0))
    ok, msg, rel, rec = rodar(io)
    assert ok and acoes(rel) == ['orientar']
    ok, msg, rel = rec.executar(0.40, 0.03, 30.0)    # o Nav2 travaria de novo: agora o caminho esta a frente
    assert ok and acoes(rel) == ['retomar'], msg


def test_9_giro_sem_progresso_na_imu_para_e_nao_se_repete():
    io = SimIO(SALA, goal=(-1.5, 0.0))
    motor_travado(io)
    ok, msg, rel, rec = rodar(io)
    it = [i for i in rec.hist.itens if i.acao == 'orientar']
    assert it and it[0].resultado != 'ok' and 'IMU sem progresso' in it[0].detalhe
    assert io.colisoes == 0
    ok, msg, rel = rec.executar(0.40, 0.03, 30.0)
    assert 'orientar' not in acoes(rel)
    assert 'ja tentei orientar aqui' in recusa(rel, 'orientar')


def test_12_goal_novo_libera_o_orientar_de_novo():
    io = SimIO(SALA, goal=(-1.5, 0.0))
    motor_travado(io)
    _, _, _, rec = rodar(io)
    io.goal_xy = (-1.0, -1.2)                     # goal novo (> 0,30 m): o historico do goal anterior nao vale
    ok, msg, rel = rec.executar(0.40, 0.03, 30.0)
    assert 'orientar' in acoes(rel)


def test_11_cancelamento_durante_o_giro():
    io = SimIO(SALA, goal=(-1.5, 0.0), cancelar_em=5.0)
    ok, msg, rel, rec = rodar(io)
    assert not ok and 'cancelado' in msg and io.cmd == (0.0, 0.0, 0.0)
    assert 0 < abs(G(io.th)) < 170 and io.colisoes == 0


def test_16_comando_estrangeiro_durante_o_giro_para_na_hora():
    io = SimIO(SALA, goal=(-1.5, 0.0), estrangeiro_em=5.0)
    ok, msg, rel, rec = rodar(io)
    it = [i for i in rec.hist.itens if i.acao == 'orientar']
    assert it and 'comando estrangeiro' in it[0].detalhe and io.cmd == (0.0, 0.0, 0.0)
    assert abs(G(io.th)) < 170


def test_16_sem_exclusividade_nenhuma_manobra():
    io = SimIO(SALA, goal=(-1.5, 0.0), estrangeiro_em=0.0)
    ok, msg, rel, _ = rodar(io)
    assert not ok and io.th == 0.0 and io.max_vel == 0.0


# ================================================================== SEM_CAMINHO_UNICO
def _limpezas_por_chamada(unico):
    io = SimIO(CORREDOR, goal=(-1.5, 0.0), planner=False)
    rec = recuperador(io, giro_orientar=True, sem_caminho_unico=unico)
    n = []
    for _ in range(2):
        antes = getattr(io, 'limpezas', 0)
        rec.executar(0.40, 0.03, 30.0)
        n.append(getattr(io, 'limpezas', 0) - antes)
    return n, io


def test_10_bloqueio_repetido_no_mesmo_lugar_nao_repete_a_espera():
    n, io = _limpezas_por_chamada(True)
    assert n == [2, 0]
    assert any('nao repito a espera' in t for _, t in io.logs)
    n, _ = _limpezas_por_chamada(False)           # desligado: igual a antes
    assert n == [2, 2]


# ================================================================== PULSO_GIRO_ADAPTA (saida lateral)
def _carregar(caminho, nome):
    spec = importlib.util.spec_from_file_location(nome, caminho)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def saida_lateral(mod, adapta=None, deriva=-2.0, lag=0.35, w=math.radians(9.0), tmax=150.0, dist=1.0):
    """Modelo (hipotese, nao medida completa): o deslize entorta o rumo 'deriva' graus/s; o giro parado so comeca
    depois de 'lag' s de comando continuo (bag 13/11 11:45:35: pulso de ~0,3 s nao mudou o yaw) e gira 'w'."""
    logs = []
    x = mod.TravessiaLateral(1, fundido=True, log=logs.append)
    if adapta is not None:
        x.pulso_adapta = adapta
    x.t0, x.yaw_ref = 0.0, 0.0
    x._mudar('deslizar', 0.0, (0.0, 0.0, 0.0))
    py = th = t = 0.0
    giro_desde, cmds = None, []
    while t < tmax and x.estado not in ('fim', 'abortar'):
        x.nova_folga(t, 0.20, 0.20, False, False)
        vx, vy, wz = x.passo(t, (0.0, py, th), dist - py)
        cmds.append((round(t, 2), round(vx, 4), round(vy, 4), round(wz, 4)))
        if abs(wz) > 1e-6 and abs(vx) < 1e-6 and abs(vy) < 1e-6:
            giro_desde = t if giro_desde is None else giro_desde
            if t - giro_desde >= lag:
                th += math.copysign(w, wz) * 0.05
        else:
            giro_desde = None
        if abs(vy) > 1e-6:
            py += 0.6 * vy * 0.05
            th += math.radians(deriva) * 0.05
        t += 0.05
    return types.SimpleNamespace(estado=x.estado, t=t, th=G(th), logs=logs, cmds=cmds, x=x)


@pytest.fixture(scope='module')
def modulos():
    return _carregar(BACKUP / 'alinhador_passagem.py', 'alinhador_antes_20261113'), ap


def test_15_regressao_lateral_desligado_comandos_identicos_ao_antes(modulos):
    velho, novo = modulos
    for deriva, lag in ((-2.0, 0.35), (2.0, 0.35), (-1.0, 0.0), (0.0, 0.0)):
        a = saida_lateral(velho, deriva=deriva, lag=lag)
        b = saida_lateral(novo, adapta=False, deriva=deriva, lag=lag)
        assert a.cmds == b.cmds and a.estado == b.estado


def test_15_ligado_nao_muda_o_deslize_quando_o_giro_funciona(modulos):
    velho, novo = modulos
    a = saida_lateral(velho, lag=0.0, w=0.26)
    b = saida_lateral(novo, adapta=True, lag=0.0, w=0.26)
    assert a.cmds == b.cmds and not any('PULSO_GIRO_ADAPTA' in m for m in b.logs)


def test_pulso_que_nao_girou_fica_mais_longo_sem_mudar_a_velocidade(modulos):
    velho, novo = modulos
    a = saida_lateral(velho)
    b = saida_lateral(novo, adapta=True)
    assert a.estado == 'abortar'                 # modelo do bag: o pulso de 0,3 s nao gira e a saida aborta
    assert b.estado == 'fim' and abs(b.th) < 7.0
    assert any('PULSO_GIRO_ADAPTA' in m for m in b.logs)
    wz = {abs(c[3]) for c in b.cmds if c[3]}
    assert wz <= {abs(c[3]) for c in a.cmds if c[3]} | {0.02}   # o mesmo wz de antes: so o tempo muda
    vel = {(c[1], c[2]) for c in b.cmds if c[2]}
    assert vel <= {(c[1], c[2]) for c in a.cmds if c[2]} | vel and max(abs(v[1]) for v in vel) <= 0.08 + 1e-9


def test_pulso_unidade_ganho_limite_e_parada():
    x = ap.TravessiaLateral(1, log=lambda _: None)
    x.pulso_adapta = True
    for k in range(6):
        x.alvo, x.pulso = math.radians(5.0), 0.29
        x._mudar('girar', 10.0 * k, (0.0, 0.0, 0.0))
        assert x.pulso <= 1.2
        x._mudar('assentar', 10.0 * k + 1.0, (0.0, 0.0, 0.0))
        x._mudar('medir', 10.0 * k + 2.0, (0.0, 0.0, 0.0))      # yaw nao mudou: pulso fraco
    assert x.pulso_ganho == 3.0 and x.pulsos_fracos == 6 and x.correcoes == x.correcoes_max
    y = ap.TravessiaLateral(1, log=lambda _: None)
    y.pulso_adapta = True
    y.alvo, y.pulso = math.radians(5.0), 0.29
    y._mudar('girar', 0.0, (0.0, 0.0, 0.0))
    y._mudar('assentar', 1.0, (0.0, 0.0, math.radians(4.0)))
    y._mudar('medir', 2.0, (0.0, 0.0, math.radians(4.0)))       # girou 4 de 5: bom
    assert y.pulso_ganho == 1.0 and y.pulsos_fracos == 0


# ================================================================== CURTO_SEM_PROGRESSO
def _trajeto_curto():
    src = (NAV / 'scripts/alinhador_passagem.py').read_text()
    fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == '_trajeto_curto')
    ns = dict(math=math, time=time, _ang=ap._ang, yaw_de=lambda o: 0.0, pose_livre=lambda *a, **k: True)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), 'alinhador_passagem.py', 'exec'), ns)
    return ns['_trajeto_curto']


class _Log:
    def __init__(self):
        self.msgs = []

    def info(self, m):
        self.msgs.append(m)
    warn = warning = error = info


def _robo_preso(param):
    s = types.SimpleNamespace(log=_Log(), tentativas=0)
    s.p = lambda n: {'curto_max': 1.0, 'curto_sem_progresso': param}[n]
    s._pose_mapa_yaw = lambda: (0.0, 0.0, 0.0)
    s._caminho = lambda g: [(0.1 * k, 0.0) for k in range(4)]
    s._mapa = lambda: None
    s.get_logger = lambda: s.log

    def deslocar(gx, gy, nome, tol, passo_max):
        s.tentativas += 1          # frente barrada: "andei 0 cm"
    s._deslocar_ate = deslocar
    return s


def test_curto_sem_progresso_para_de_repetir():
    f = _trajeto_curto()
    goal = types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.31, y=0.0),
                                                            orientation=None))
    s = _robo_preso(True)
    assert f(s, goal, True) is None and s.tentativas == 2
    assert any('CURTO_SEM_PROGRESSO' in m for m in s.log.msgs)
    s = _robo_preso(False)                       # desligado: igual a antes (4 tentativas)
    assert f(s, goal, True) is None and s.tentativas == 4


# ================================================================== patches: aplicar / recusar / desfazer
PATCHES = ['orientar_caminho', 'sem_caminho_unico', 'pulso_giro_adapta', 'curto_sem_progresso',
           'virar_replaneja', 'chegada_perto']
ARQS = ['alinhador_passagem.py', 'destravar_node.py', 'recuperacao_avancada.py']


def test_patches_aplicados_com_marca_e_parametros():
    txt = {a: (NAV / 'scripts' / a).read_text() for a in ARQS}
    assert 'ORIENTAR_CAMINHO (13/11)' in txt['recuperacao_avancada.py'] and \
        'ORIENTAR_CAMINHO (13/11)' in txt['destravar_node.py']
    assert "('giro_orientar', True)" in txt['destravar_node.py']               # a mudanca do proximo teste
    assert "self.declare_parameter('sem_caminho_unico', False)" in txt['destravar_node.py']   # opt-in
    assert "self.declare_parameter('pulso_giro_adapta', False)" in txt['alinhador_passagem.py']
    assert "self.declare_parameter('curto_sem_progresso', False)" in txt['alinhador_passagem.py']
    assert "('giro_orientar_reavaliar', 3)" in txt['destravar_node.py']                # v2 ORIENTAR_ROBUSTO
    assert "self.declare_parameter('virar_replaneja', True)" in txt['alinhador_passagem.py']
    assert "self.declare_parameter('chegada_perto', True)" in txt['alinhador_passagem.py']


def _patches_antigos():
    """Patches de texto anteriores (_patch_texto: --arquivo) que miram os 3 arquivos mexidos hoje."""
    out = []
    for p in sorted((WS / 'ferramentas/patches').glob('aplicar_*.py')):
        t = p.read_text()
        if 'from _patch_texto import rodar' not in t:
            continue
        for a in ARQS:
            if "scripts/%s'" % a in t:
                out.append((p, a))
    return out


@pytest.mark.parametrize('patch,arq', _patches_antigos(), ids=lambda v: getattr(v, 'name', v))
def test_desfazer_dos_patches_anteriores_continua_igual(tmp_path, patch, arq):
    """Os patches de hoje nao podem quebrar o --desfazer de nenhum patch anterior (bug pego pela suite em
    test_recuo_curva_re_longa: a 1a versao do ORIENTAR_CAMINHO reescrevia a linha do RE_LONGA)."""
    def desfazer(origem):
        f = tmp_path / ('%s_%s' % (origem.parent.name, arq))
        f.write_text(origem.read_text())
        r = subprocess.run([sys.executable, str(patch), '--arquivo', str(f), '--desfazer'],
                           capture_output=True, text=True)
        return r.returncode, 'nao esta aplicado' in r.stdout
    assert desfazer(NAV / 'scripts' / arq) == desfazer(BACKUP / arq)


def test_patches_desfazer_em_qualquer_ordem_volta_ao_backup(tmp_path):
    scripts = tmp_path / 'src/robo_navigation/scripts'
    scripts.mkdir(parents=True)
    for a in ARQS:
        (scripts / a).write_text((BACKUP / a).read_text())
    run = lambda n, *a: subprocess.run([sys.executable, str(WS / 'ferramentas/patches' / ('aplicar_%s.py' % n)),
                                        '--raiz', str(tmp_path), *a], capture_output=True, text=True, check=True)
    for n in PATCHES:
        run(n)
    for n in PATCHES:
        assert 'ja aplicado' in run(n).stdout
    for a in ARQS:   # o resultado dos patches e o src atual
        assert (scripts / a).read_text() == (NAV / 'scripts' / a).read_text(), a
    for n in reversed(PATCHES[1:] + PATCHES[:1]):
        run(n, '--desfazer')
    for a in ARQS:
        assert (scripts / a).read_text() == (BACKUP / a).read_text(), a


# ================================================================== v2 ORIENTAR_ROBUSTO (teste 13/11 13:17:33)
FANTASMA = (0.30, 0.40)   # ponto a ~0,5 m do centro, na frente-esquerda: cai no arco de um giro anti-horario


def _scan_com_fantasma(sc):
    """Copia do scan com 1 retorno falso em FANTASMA (base_link), como o ponto que derrubou o giro as 13:17:34."""
    lx, ly, _ = ra.LASER
    a = math.atan2(FANTASMA[1] - ly, FANTASMA[0] - lx)
    r = math.hypot(FANTASMA[0] - lx, FANTASMA[1] - ly)
    rs = list(sc.ranges)
    rs[int(round((a - sc.angle_min) / sc.inc))] = r
    return ra.Raios(sc.angle_min, sc.inc, tuple(rs), sc.rmin, sc.rmax, sc.laser, sc.t)


def _io_com_fantasma():
    io = SimIO(SALA)
    real = io.scan
    estado = {'usado': None}

    def scan():
        sc = real()
        girando = abs(io.cmd[2]) > 1e-6
        if estado['usado'] is None and girando and io.t > 1.0:
            estado['usado'] = sc.t
        return _scan_com_fantasma(sc) if estado['usado'] == sc.t else sc
    io.scan = scan
    return io


def test_fantasma_mesmo_e_acusado_pela_checagem_em_movimento():
    sc = _scan_com_fantasma(SimIO(SALA).scan())
    assert not ra.avaliar(sc, [(0.0, 0.0, math.radians(30))], margem=0.03).livre


def test_orientar_v1_aborta_no_ponto_fantasma():
    io = _io_com_fantasma()
    rec = recuperador(io, giro_orientar=True, giro_orientar_reavaliar=0)
    motivo, girou = rec.girar_orientar(math.radians(164), io.t + 90.0)
    assert motivo.startswith('LiDAR') and G(girou) < 60      # o que aconteceu as 13:17:34 (6 de 164)


def test_orientar_v2_para_reavalia_parado_e_termina():
    io = _io_com_fantasma()
    rec = recuperador(io, giro_orientar=True, giro_orientar_reavaliar=3)
    motivo, girou = rec.girar_orientar(math.radians(164), io.t + 90.0)
    assert motivo == 'chegou' and abs(G(girou) - 164) <= 5 and io.colisoes == 0
    assert any('ORIENTAR_ROBUSTO 1' in t and 'continuo' in t for _, t in io.logs)


def test_orientar_v2_obstaculo_real_continua_parando():
    io = SimIO(SALA)
    rec = recuperador(io, giro_orientar=True, giro_orientar_reavaliar=3)
    dormir = io.dormir
    caixa = (-0.05, 0.15, 0.52, 0.70)

    def dormir_com_caixa(s):
        if io.t >= 2.0 and caixa not in io.caixas:
            io.caixas.append(caixa)
        dormir(s)
    io.dormir = dormir_com_caixa
    motivo, girou = rec.girar_orientar(math.radians(180), io.t + 90.0)
    assert motivo != 'chegou' and G(girou) < 170 and io.colisoes == 0 and io.cmd == (0.0, 0.0, 0.0)


# ================================================================== VIRAR_REPLANEJA (teste 13/11 13:17:04)
def _metodo(nome, extra=None):
    src = (NAV / 'scripts/alinhador_passagem.py').read_text()
    fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == nome)
    ns = dict(math=math, time=types.SimpleNamespace(sleep=lambda s: None, monotonic=time.monotonic),
              _ang=ap._ang, rumo_do_caminho=ap.rumo_do_caminho, yaw_de=lambda o: o,
              com_barreiras=ap.com_barreiras)   # MEMORIA_BARREIRA (25/11)
    ns.update(extra or {})
    exec(compile(ast.Module(body=[fn], type_ignores=[]), 'alinhador_passagem.py', 'exec'), ns)
    return ns[nome]


ATRAS = [(0.0 - 0.1 * k, 0.0) for k in range(20)]     # caminho saindo para tras (robo virado para +x)


def _robo_sem_caminho(replaneja=True, caminho_depois=None, rota=ATRAS, obst=(), obs=True):
    s = types.SimpleNamespace(pre_giro_memo=None, giros=[], log=_Log(), limpezas=0, pedidos=0)
    s.p = lambda n: {'virar_caminho_graus': 45.0, 'pre_giro_repetir_s': 20.0, 'virar_replaneja': replaneja}[n]
    s._pose_mapa_yaw = lambda: (0.0, 0.0, 0.0)
    s._pose_mapa = lambda: (0.0, 0.0)
    s._mapa = lambda: 'mapa'
    s._obs_global = lambda: ('obs' if obs else None)
    s.get_logger = lambda: s.log

    def caminho(a):
        s.pedidos += 1
        return caminho_depois if s.limpezas else None
    s._caminho = caminho

    def limpar():
        s.limpezas += 1
    s._limpar_costmaps = limpar
    s._girar_seguro = lambda rumo, n, tempo_max, movs_max: s.giros.append(rumo) or True
    val = _metodo('_caminho_validado', dict(rota_no_mapa=lambda m, p, g: rota,
                                             obstaculo_novo_na_rota=lambda m, o, r: list(obst)))
    s._caminho_validado = lambda a, n: val(s, a, n)
    return s


def _alvo_atras():
    return types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=-1.9, y=0.0)))


def test_virar_replaneja_planner_volta_depois_de_limpar():
    f = _metodo('_virar_para_caminho')
    s = _robo_sem_caminho(caminho_depois=ATRAS)
    f(s, _alvo_atras(), 'GOAL FINAL')
    assert s.limpezas == 1 and len(s.giros) == 1 and abs(abs(s.giros[0]) - math.pi) < 0.1
    assert any('limpei os costmaps e ele achou' in m for m in s.log.msgs)


def test_virar_replaneja_rota_do_mapa_validada_pelo_costmap():
    f = _metodo('_virar_para_caminho')
    s = _robo_sem_caminho(caminho_depois=None)
    f(s, _alvo_atras(), 'GOAL FINAL')
    assert len(s.giros) == 1 and any('1o metro livre' in m for m in s.log.msgs)


def test_virar_replaneja_rota_do_mapa_com_objeto_nao_vira():
    f = _metodo('_virar_para_caminho')
    s = _robo_sem_caminho(caminho_depois=None, obst=[(-0.5, 0.0)])
    f(s, _alvo_atras(), 'GOAL FINAL')
    assert s.giros == [] and any('objeto do momento' in m for m in s.log.msgs)


def test_virar_replaneja_sem_costmap_nao_vira_pela_rota_estatica():
    f = _metodo('_virar_para_caminho')
    s = _robo_sem_caminho(caminho_depois=None, obs=False)
    f(s, _alvo_atras(), 'GOAL FINAL')
    assert s.giros == []


def test_virar_replaneja_desligado_igual_a_antes():
    f = _metodo('_virar_para_caminho')
    s = _robo_sem_caminho(replaneja=False, caminho_depois=ATRAS)
    f(s, _alvo_atras(), 'GOAL FINAL')
    assert s.giros == [] and s.limpezas == 0 and s.pedidos == 1


# ================================================================== CHEGADA_PERTO (teste 13/11 13:18:57)
def _robo_no_goal(pose, anda=True, gira=True, raio=0.15):
    s = types.SimpleNamespace(pose=list(pose), log=_Log(), desl=0, giros=0)
    s.p = lambda n: {'chegada_perto_raio': raio}[n]
    s._pose_mapa_yaw = lambda: tuple(s.pose)
    s.get_logger = lambda: s.log
    s._esperar_sensores = lambda n: 0.0

    def deslocar(x, y, n, tol, passo_max):
        s.desl += 1
        if anda:
            s.pose[0], s.pose[1] = x + 0.02, y
    s._deslocar_ate = deslocar

    def girar(yaw, n, tempo_max, movs_max):
        s.giros += 1
        if gira:
            s.pose[2] = yaw
        return gira
    s._girar_seguro = girar
    return s


def _goal(x, y, yaw):
    return types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y), orientation=yaw))


def test_chegada_perto_termina_os_9_cm_do_teste():
    f = _metodo('_chegada_perto')
    s = _robo_no_goal((2.10, 3.76, math.radians(88)))
    assert f(s, _goal(2.19, 3.78, math.radians(90)), 'GOAL FINAL') is True
    assert s.desl == 1 and s.giros == 0 and any('SUCESSO' in m for m in s.log.msgs)


def test_chegada_perto_preserva_o_angulo_do_goal():
    f = _metodo('_chegada_perto')
    s = _robo_no_goal((2.10, 3.76, math.radians(60)))
    assert f(s, _goal(2.19, 3.78, math.radians(90)), 'GOAL FINAL') is True and s.giros == 1
    s = _robo_no_goal((2.10, 3.76, math.radians(60)), gira=False)          # nao conseguiu girar: falha
    assert f(s, _goal(2.19, 3.78, math.radians(90)), 'GOAL FINAL') is False


def test_chegada_perto_longe_nao_mexe():
    f = _metodo('_chegada_perto')
    s = _robo_no_goal((1.90, 3.76, math.radians(90)))
    assert f(s, _goal(2.19, 3.78, math.radians(90)), 'GOAL FINAL') is False and s.desl == 0 and s.giros == 0


def test_chegada_perto_bloqueado_continua_falhando():
    f = _metodo('_chegada_perto')
    s = _robo_no_goal((2.06, 3.76, math.radians(90)), anda=False)
    assert f(s, _goal(2.19, 3.78, math.radians(90)), 'GOAL FINAL') is False
    assert any('nao cheguei' in m for m in s.log.msgs)
