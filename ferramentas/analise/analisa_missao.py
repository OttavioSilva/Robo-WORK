#!/usr/bin/env python3
"""Analisa uma missao gravada pelo gravar_teste.sh. So LE o bag: nao publica nada no ROS.

Entrada: o missaoNN_<data>.tar.gz, a pasta extraida ou o proprio .mcap / .mcap.zstd.
Saida (na pasta --saida):
  relatorio.txt      tudo abaixo em texto: trechos, planner, bloqueios, replanejamento, folgas,
                     Collision Monitor, giros, deslizes laterais, encostos, SLAM, avisos do pacote
  linha_do_tempo.tsv /rosout completo, com o horario do no (nao o de chegada no bag)
  folgas.tsv         por scan: folga do corpo (frente, tras, esq, dir, minima) e comando na hora
  deslizes.tsv       cada deslize lateral do alinhador (giro, deriva, folgas antes/depois)
  planner.tsv        cada "failed to plan" e o teste de conectividade no costmap gravado
  fig_*.png          (com --figuras) costmap global nas falhas do planner e nos bloqueios

Uso (rode FORA da pasta extraida; -P evita importar .py que venha junto do bag):
  source /opt/ros/jazzy/setup.bash
  python3 -P ferramentas/analise/analisa_missao.py ~/bags/missao21_20261105_171908.tar.gz \
      --saida /tmp/missao21 --figuras --ros-log-dir ~/.ros/log
Olhar um intervalo, scan a scan:
  ... --janela 17:25:34-17:25:38 --janela 17:26:42-17:26:48

Limites do que o bag permite afirmar:
  - /scan_nav, /bloqueio_virtual e /cmd_vel_ctrl nao sao gravados. A memoria do ponto cego e o
    bloqueio nao aparecem como dado: o bloqueio e reconstruido da pose na hora do log.
  - a odom vem do comando (sem encoder). "Andou" pela odom significa "foi mandado andar".
  - o teste de conectividade e aproximado: usa o costmap publicado (1 Hz) e vizinhanca 4.
"""
import argparse
import bisect
import collections
import datetime as dt
import glob
import math
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile

import numpy as np

# Medidas do robo (CLAUDE.md). O footprint do Nav2 e do filtro e 1 cm menor de cada lado.
CORPO_X0, CORPO_X1, CORPO_Y = -0.215, 0.375, 0.325
RAIO_INSCRITO = 0.215
BLOQ_FRENTE_X, BLOQ_MEIA_Y, BLOQ_PROF, BLOQ_VALIDADE = 0.365, 0.315, 0.30, 30.0
GRADE_BLOQUEADA = 99    # OccupancyGrid do costmap: 99 = inscrito (253), 100 = letal (254)
FUSO = dt.timezone(dt.timedelta(hours=-3))
TOPICOS_ESPERADOS = ['/scan_nav', '/bloqueio_virtual', '/cmd_vel_ctrl']


def hora(t, casas=3):
    s = dt.datetime.fromtimestamp(t, FUSO).strftime('%H:%M:%S.%f')
    return s[:-7] if casas == 0 else s[:casas - 6]


def mmss(s):
    return '%d:%04.1f' % (int(s // 60), s % 60)


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def dang(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ------------------------------------------------------------------ entrada
def achar_mcap(alvo, tmp):
    """Devolve (caminho do .mcap, pasta da missao ou None). Descompacta em tmp."""
    alvo = os.path.abspath(os.path.expanduser(alvo))
    pasta = None
    if os.path.isfile(alvo) and tarfile.is_tarfile(alvo):
        destino = os.path.join(tmp, 'extraido')
        os.makedirs(destino)
        with tarfile.open(alvo) as tf:
            def seguro(membro, caminho):
                # os pacotes antigos trazem link absoluto em ros_logs/latest: pula so esse item
                try:
                    return tarfile.data_filter(membro, caminho)
                except tarfile.FilterError:
                    return None
            tf.extractall(destino, filter=seguro)
        alvo = destino
    if os.path.isdir(alvo):
        achados = sorted(glob.glob(os.path.join(alvo, '**', '*.mcap*'), recursive=True))
        if not achados:
            sys.exit('Nenhum .mcap em %s' % alvo)
        notas = glob.glob(os.path.join(alvo, '**', 'notas.txt'), recursive=True)
        pasta = os.path.dirname(notas[0]) if notas else None
        alvo = achados[0]
    if alvo.endswith('.zstd'):
        saida = os.path.join(tmp, 'bag.mcap')
        subprocess.run(['zstd', '-d', '-q', '-f', alvo, '-o', saida], check=True)
        alvo = saida
    return alvo, pasta


class Dados:
    pass


def carregar(mcap):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=mcap, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    tipos = {t.name: t.type for t in r.get_all_topics_and_types()}
    d = Dados()
    d.topicos = tipos
    querer = ['/rosout', '/tf', '/tf_static', '/scan_front', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel',
              '/collision_monitor_state', '/plan', '/plan_smoothed', '/behavior_tree_log',
              '/global_costmap/costmap', '/map', '/imu/data']
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in querer if t in tipos]))
    cls = {t: get_message(tipos[t]) for t in querer if t in tipos}
    d.rosout, d.cm, d.planos, d.suav, d.bt, d.costmaps, d.mapas = [], [], [], [], [], [], []
    cmd = {'/cmd_vel_nav': [], '/cmd_vel_smoothed': [], '/cmd_vel': []}
    mo, ob, imu, scans = [], [], [], []
    d.estatico = {}
    d.scan_info = None
    d.t_ini = d.t_fim = None
    while r.has_next():
        topico, raw, ts = r.read_next()
        t = ts * 1e-9
        d.t_ini = t if d.t_ini is None else min(d.t_ini, t)
        d.t_fim = t if d.t_fim is None else max(d.t_fim, t)
        m = deserialize_message(raw, cls[topico])
        if topico == '/rosout':
            d.rosout.append((m.stamp.sec + m.stamp.nanosec * 1e-9, t, m.level, m.name, m.msg))
        elif topico == '/tf':
            for x in m.transforms:
                if x.child_frame_id == 'base_link' and x.header.frame_id == 'odom':
                    alvo = ob
                elif x.child_frame_id == 'odom' and x.header.frame_id == 'map':
                    alvo = mo
                else:
                    continue
                tr = x.transform
                alvo.append((t, tr.translation.x, tr.translation.y, yaw_q(tr.rotation)))
        elif topico == '/tf_static':
            for x in m.transforms:
                d.estatico[x.child_frame_id] = (x.header.frame_id, x.transform.translation.x,
                                                x.transform.translation.y, yaw_q(x.transform.rotation))
        elif topico == '/scan_front':
            if d.scan_info is None:
                d.scan_info = (m.header.frame_id, m.angle_min, m.angle_increment, len(m.ranges))
            if len(m.ranges) == d.scan_info[3]:
                scans.append((t, np.asarray(m.ranges, dtype=np.float32)))
        elif topico in cmd:
            cmd[topico].append((t, m.twist.linear.x, m.twist.linear.y, m.twist.angular.z))
        elif topico == '/collision_monitor_state':
            d.cm.append((t, m.action_type, m.polygon_name))
        elif topico == '/plan':
            d.planos.append((t, np.array([(p.pose.position.x, p.pose.position.y, yaw_q(p.pose.orientation)) for p in m.poses]).reshape(-1, 3)))
        elif topico == '/plan_smoothed':
            d.suav.append(t)
        elif topico == '/behavior_tree_log':
            for e in m.event_log:
                d.bt.append((e.timestamp.sec + e.timestamp.nanosec * 1e-9, e.node_name, e.previous_status, e.current_status))
        elif topico == '/global_costmap/costmap':
            d.costmaps.append((t, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                               np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)))
        elif topico == '/map':
            d.mapas.append((t, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y,
                            np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)))
        elif topico == '/imu/data':
            imu.append((t, yaw_q(m.orientation), m.angular_velocity.z))
    d.rosout.sort(key=lambda x: x[0])
    d.cmd = {k: np.array(v).reshape(-1, 4) for k, v in cmd.items()}
    d.map_odom = np.array(mo).reshape(-1, 4)
    d.odom_base = np.array(ob).reshape(-1, 4)
    d.imu = np.array(imu).reshape(-1, 3)
    d.imu_yaw = np.unwrap(d.imu[:, 1]) if len(d.imu) else np.array([])
    d.scan_t = np.array([s[0] for s in scans])
    d.scan_r = np.array([s[1] for s in scans]) if scans else np.empty((0, 0))
    d.costmap_t = [c[0] for c in d.costmaps]
    d.plano_t = [p[0] for p in d.planos]
    return d


# ------------------------------------------------------------------ geometria
def amostra(arr, t):
    return arr[min(max(np.searchsorted(arr[:, 0], t, side='right') - 1, 0), len(arr) - 1)]


def pose_mapa(d, t):
    """Pose de base_link em map no instante t (ultimo valor de cada TF)."""
    if not len(d.odom_base):
        return (0.0, 0.0, 0.0)
    _, ox, oy, oa = amostra(d.odom_base, t)
    if not len(d.map_odom):
        return (ox, oy, oa)
    _, mx, my, ma = amostra(d.map_odom, t)
    c, s = math.cos(ma), math.sin(ma)
    return (mx + c * ox - s * oy, my + s * ox + c * oy, float(dang(ma + oa)))


def pontos_corpo(d):
    """Pontos do /scan_front em base_link, por scan: (x, y) de shape (n_scans, n_raios)."""
    quadro, a0, da, n = d.scan_info
    lx = ly = la = 0.0
    while quadro != 'base_link' and quadro in d.estatico:
        pai, x, y, a = d.estatico[quadro]
        c, s = math.cos(a), math.sin(a)
        lx, ly, la = x + c * lx - s * ly, y + s * lx + c * ly, a + la
        quadro = pai
    ang = a0 + np.arange(n) * da + la
    r = np.where(np.isfinite(d.scan_r), d.scan_r, np.nan)
    return lx + r * np.cos(ang), ly + r * np.sin(ang)


def dist_corpo(x, y):
    dx = np.maximum(np.maximum(CORPO_X0 - x, x - CORPO_X1), 0.0)
    dy = np.maximum(np.abs(y) - CORPO_Y, 0.0)
    return np.hypot(dx, dy)


def raios_do_proprio_robo(x, y):
    """Raios que batem no proprio robo: ficam a < 3 cm do corpo em mais de 25% dos scans."""
    perto = dist_corpo(x, y) < 0.03
    return np.where(np.nanmean(perto, axis=0) > 0.25)[0], np.nanmean(perto, axis=0)


def folgas_scan(x, y, excluir):
    """Folga direcional de cada scan: anda reto naquela direcao, como o destravar mede."""
    x, y = x.copy(), y.copy()
    x[:, excluir] = np.nan
    y[:, excluir] = np.nan
    dentro = (x >= CORPO_X0) & (x <= CORPO_X1) & (np.abs(y) <= CORPO_Y)
    x = np.where(dentro, np.nan, x)
    y = np.where(dentro, np.nan, y)
    m = 0.0

    def minimo(v):
        v = np.where(np.isfinite(v), v, np.inf)
        return v.min(axis=1)
    with np.errstate(invalid='ignore'):
        f = minimo(np.where((x > CORPO_X1) & (np.abs(y) <= CORPO_Y + m), x - CORPO_X1, np.nan))
        t = minimo(np.where((x < CORPO_X0) & (np.abs(y) <= CORPO_Y + m), CORPO_X0 - x, np.nan))
        e = minimo(np.where((y > CORPO_Y) & (x >= CORPO_X0 - m) & (x <= CORPO_X1 + m), y - CORPO_Y, np.nan))
        di = minimo(np.where((y < -CORPO_Y) & (x >= CORPO_X0 - m) & (x <= CORPO_X1 + m), -CORPO_Y - y, np.nan))
        dmin = dist_corpo(x, y)
        n2 = np.nansum(dmin < 0.02, axis=1)
        dm = minimo(dmin)
    return f, t, e, di, dm, n2, dentro.sum(axis=1)


# ------------------------------------------------------------------ fases
RE_FASE = re.compile(r'^(PREPARO \d+ refinado|PREPARO \d+[^:]*|TRAVESSIA \d+[^:]*|CURVA \d+|TRAJETO CURTO|'
                     r'ENCOSTAR \w+|GOAL FINAL[^:]*|SAIR DA MESA|ANTI_CISCADO \d+)')


def fases(d):
    """Sequencia (inicio, nome) a partir dos logs do alinhador e do destravar."""
    ev = []
    for st, _, _, nome, msg in d.rosout:
        if nome == 'alinhador_passagem':
            m = RE_FASE.match(msg)
            if m and (not ev or ev[-1][1] != m.group(1)):
                ev.append((st, m.group(1)))
            elif msg.startswith('RESULTADO'):
                ev.append((st, '(fim do goal)'))
        elif nome == 'destravar':
            if msg.startswith('BLOQUEIO') or msg.startswith('DESTRAVAR passo 1'):
                if not ev or ev[-1][1] != 'DESTRAVAR':
                    ev.append((st, 'DESTRAVAR'))
            elif msg.startswith('DESTRAVAR:') and 'caminho novo' not in msg:
                ev.append((st, 'Nav2 (arvore)'))
    return ev


def fase_em(ev, t):
    i = bisect.bisect_right([e[0] for e in ev], t) - 1
    return ev[i][1] if i >= 0 else '(antes)'


def trechos(d):
    """Passos do executor: 'ir ate' com inicio/fim e passos simulados."""
    saida, atual = [], None
    for st, _, _, nome, msg in d.rosout:
        if nome != 'executor_prova':
            continue
        m = re.match(r'\[(\d+)/(\d+)\] (.*)', msg)
        if m:
            if atual:
                atual['fim'] = st
                saida.append(atual)
            atual = {'ini': st, 'fim': None, 'txt': m.group(3), 'res': ''}
        elif atual and re.match(r'\w+: (SUCESSO|FALHOU)', msg):
            atual['res'] += ('; ' if atual['res'] else '') + msg
            atual['fim'] = st
    if atual:
        saida.append(atual)
    return saida


# ------------------------------------------------------------------ planner
RE_FALHA = re.compile(r'failed to plan from \(([-\d.]+), ([-\d.]+)\) to \(([-\d.]+), ([-\d.]+)\)')


def falhas_planner(d):
    out = []
    for st, _, _, nome, msg in d.rosout:
        if nome == 'planner_server':
            m = RE_FALHA.search(msg)
            if m:
                out.append((st,) + tuple(float(v) for v in m.groups()))
    return out


def bloqueios(d):
    out = []
    for st, _, _, nome, msg in d.rosout:
        if nome == 'destravar' and msg.startswith('BLOQUEIO'):
            out.append((st,) + pose_mapa(d, st))
    return out


def pontos_bloqueio(x0, y0, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    xf, pts = BLOQ_FRENTE_X + 0.03, []
    while xf <= BLOQ_FRENTE_X + BLOQ_PROF + 1e-6:
        y = -BLOQ_MEIA_Y
        while y <= BLOQ_MEIA_Y + 1e-6:
            pts.append((x0 + c * xf - s * y, y0 + s * xf + c * y))
            y += 0.05
        xf += 0.05
    return np.array(pts)


def costmap_em(d, t):
    i = bisect.bisect_right(d.costmap_t, t) - 1
    return d.costmaps[i] if i >= 0 else None


def celula(res, ox, oy, x, y):
    return int((x - ox) / res), int((y - oy) / res)


def disco(res, raio):
    n = int(math.ceil(raio / res))
    yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
    return (xx * xx + yy * yy) * res * res <= raio * raio


def conecta(bloq, res, ox, oy, ini, fim, tol=0.10):
    """Ha caminho de ini ate algum ponto a <= tol de fim? (a celula do robo e liberada, como no NavFn)"""
    from scipy import ndimage
    h, w = bloq.shape
    ci, cj = celula(res, ox, oy, *ini)
    gi, gj = celula(res, ox, oy, *fim)
    if not (0 <= ci < w and 0 <= cj < h and 0 <= gi < w and 0 <= gj < h):
        return None
    b = bloq.copy()
    b[cj, ci] = False
    rot, _ = ndimage.label(~b)
    n = int(math.ceil(tol / res))
    ja = rot[max(cj, 0), max(ci, 0)]
    janela = rot[max(gj - n, 0):gj + n + 1, max(gi - n, 0):gi + n + 1]
    return bool((janela == ja).any())


def variantes(d, t, ini, fim, blq):
    """Testa tres mapas: (A) costmap publicado; (B) A sem as marcas dos bloqueios;
    (C) so o /map + o scan do instante, inflados pelo raio inscrito."""
    from scipy import ndimage
    cm = costmap_em(d, t)
    if cm is None:
        return None
    _, res, ox, oy, g = cm
    h, w = g.shape
    bloq = g >= GRADE_BLOQUEADA
    r = {'A': conecta(bloq, res, ox, oy, ini, fim)}
    ci, cj = celula(res, ox, oy, *ini)
    gi, gj = celula(res, ox, oy, *fim)
    r['custo_ini'] = int(g[cj, ci]) if 0 <= ci < w and 0 <= cj < h else None
    r['custo_fim'] = int(g[gj, gi]) if 0 <= gi < w and 0 <= gj < h else None
    fixos = np.zeros_like(bloq)
    i_mapa = bisect.bisect_right([m[0] for m in d.mapas], t) - 1
    if i_mapa >= 0:
        _, mres, mox, moy, mg = d.mapas[i_mapa]
        yy, xx = np.nonzero(mg >= 65)
        wx, wy = mox + (xx + 0.5) * mres, moy + (yy + 0.5) * mres
        ii, jj = ((wx - ox) / res).astype(int), ((wy - oy) / res).astype(int)
        ok = (ii >= 0) & (ii < w) & (jj >= 0) & (jj < h)
        fixos[jj[ok], ii[ok]] = True
    k = np.searchsorted(d.scan_t, t)
    if len(d.scan_t):
        k = min(k, len(d.scan_t) - 1)
        px, py, pa = pose_mapa(d, d.scan_t[k])
        xs, ys = d._xb[k], d._yb[k]
        ok = np.isfinite(xs) & ~np.isin(np.arange(len(xs)), d._self)
        ok &= ~((xs >= CORPO_X0) & (xs <= CORPO_X1) & (np.abs(ys) <= CORPO_Y))
        c, s = math.cos(pa), math.sin(pa)
        wx, wy = px + c * xs[ok] - s * ys[ok], py + s * xs[ok] + c * ys[ok]
        ii, jj = ((wx - ox) / res).astype(int), ((wy - oy) / res).astype(int)
        v = (ii >= 0) & (ii < w) & (jj >= 0) & (jj < h)
        fixos[jj[v], ii[v]] = True
    inf = disco(res, RAIO_INSCRITO)
    fixos_inf = ndimage.binary_dilation(fixos, structure=inf)
    marca = np.zeros_like(bloq)
    for tb, bx, by, ba in blq:
        if tb > t:
            continue
        for x, y in pontos_bloqueio(bx, by, ba):
            i, j = celula(res, ox, oy, x, y)
            if 0 <= i < w and 0 <= j < h:
                marca[j, i] = True
    marca_inf = ndimage.binary_dilation(marca, structure=inf)
    r['B'] = conecta(bloq & ~(marca_inf & ~fixos_inf), res, ox, oy, ini, fim)
    r['C'] = conecta(fixos_inf, res, ox, oy, ini, fim)
    # o costmap esta mesmo com a marca do bloqueio? (celulas da marca que estao bloqueadas e nao sao fixas)
    r['marca_viva'] = int((marca & bloq & ~fixos_inf).sum())
    return r


# ------------------------------------------------------------------ relatorio
class Saida:
    def __init__(self, caminho):
        self.f = open(caminho, 'w')

    def p(self, txt=''):
        print(txt)
        self.f.write(txt + '\n')

    def titulo(self, txt):
        self.p('\n' + '=' * 92)
        self.p(txt)
        self.p('=' * 92)


def secao_pacote(o, d, pasta, ros_log_dir):
    o.titulo('PACOTE: o que veio e o que falta')
    o.p('Bag: %s -> %s (%s). Mensagens de /rosout: %d' % (hora(d.t_ini, 0), hora(d.t_fim, 0), mmss(d.t_fim - d.t_ini), len(d.rosout)))
    if pasta:
        arq = os.path.join(pasta, 'notas.txt')
        if os.path.exists(arq):
            o.p('notas.txt: ' + ' | '.join(open(arq, errors='replace').read().strip().split('\n')))
        rl = os.path.join(pasta, 'ros_logs')
        n = sum(len(f) for _, _, f in os.walk(rl)) if os.path.isdir(rl) else 0
        o.p('ros_logs/: %d arquivo(s)%s' % (n, '  <-- VAZIA' if n == 0 else ''))
        for f in sorted(glob.glob(os.path.join(pasta, 'params_*.yaml'))):
            if os.path.getsize(f) < 80:
                o.p('params vazio: %s (%d bytes)' % (os.path.basename(f), os.path.getsize(f)))
    falta = [t for t in TOPICOS_ESPERADOS if t not in d.topicos]
    if falta:
        o.p('Topicos que nao foram gravados: ' + ', '.join(falta))
    atraso = np.array([r[1] - r[0] for r in d.rosout]) if d.rosout else np.zeros(1)
    o.p('Atraso do /rosout no bag: mediana %.3f s, max %.1f s, %d mensagens > 0,5 s (use o horario do no, nao o de chegada)'
        % (np.median(atraso), atraso.max(), (atraso > 0.5).sum()))
    extras = sorted({r[3].split('.')[0] for r in d.rosout if re.match(r'(move_group|moveit|rviz|interactive_marker|controller_manager|joint_|spawner)', r[3])})
    if extras:
        o.p('Outros nos falando no /rosout durante a gravacao: ' + ', '.join(extras[:10]) + (' ...' if len(extras) > 10 else ''))
    if ros_log_dir:
        comparar_logs(o, d, os.path.expanduser(ros_log_dir))


def comparar_logs(o, d, pasta):
    """Confere os arquivos do ~/.ros/log do periodo contra o /rosout do bag."""
    re_l = re.compile(r'^\[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)')
    no_bag = {(round(r[0], 3), r[3]) for r in d.rosout}
    por_no, faltam = collections.Counter(), []
    ignorar = ('rviz2', 'rosbag2_cpp', 'rosbag2_recorder')
    arquivos = 0
    for f in glob.glob(os.path.join(pasta, '*.log')):
        m = re.search(r'_(\d{13})\.log$', f)
        if not m or int(m.group(1)) / 1000 > d.t_fim or os.path.getmtime(f) < d.t_ini:
            continue
        arquivos += 1
        for linha in open(f, errors='replace'):
            ml = re_l.match(linha)
            if not ml:
                continue
            t = float(ml.group(2))
            if d.t_ini <= t <= d.t_fim:
                por_no[ml.group(3)] += 1
                if (round(t, 3), ml.group(3)) not in no_bag and ml.group(3) not in ignorar:
                    faltam.append((t, ml.group(3), ml.group(1), ml.group(4)))
    o.p('~/.ros/log: %d arquivos do periodo, %d linhas dentro do bag, %d linhas (fora RViz/rosbag) que NAO estao no /rosout'
        % (arquivos, sum(por_no.values()), len(faltam)))
    resumo = collections.Counter((no, re.sub(r'[\d.]+', '#', msg)[:70]) for _, no, _, msg in faltam)
    for (no, msg), n in resumo.most_common(12):
        o.p('   %3dx %s: %s' % (n, no, msg))


def secao_trechos(o, d, ev):
    o.titulo('TRECHOS (executor_prova; horarios do no)')
    ts = trechos(d)
    if not ts:
        o.p('executor_prova nao aparece no /rosout.')
        return
    for x in ts:
        fim = x['fim'] or d.t_fim
        o.p('%s -> %s  %7s  %s' % (hora(x['ini'], 0), hora(fim, 0), mmss(fim - x['ini']), x['txt'][:60]))
        if x['res']:
            o.p('    ' + x['res'][:150])
    ini, fim = ts[0]['ini'], max((x['fim'] or d.t_fim) for x in ts)
    sim = sum((x['fim'] or x['ini']) - x['ini'] for x in ts if 'simulado' in x['txt'])
    o.p('\nTotal: %s  (%s -> %s); %s disso em manipulacao simulada' % (mmss(fim - ini), hora(ini, 0), hora(fim, 0), mmss(sim)))
    o.p('\nFases dentro dos trechos (ate a proxima fase):')
    acum = collections.defaultdict(float)
    for i, (t, nome) in enumerate(ev):
        prox = ev[i + 1][0] if i + 1 < len(ev) else fim
        if nome == '(fim do goal)':
            continue
        o.p('  %s %7.1f s  %s' % (hora(t, 0), prox - t, nome))
        acum[re.sub(r' \d+.*| \w+_\d+$', '', nome)] += prox - t
    o.p('\nTempo somado por tipo de fase:')
    for k, v in sorted(acum.items(), key=lambda kv: -kv[1]):
        o.p('  %7.1f s  %s' % (v, k))


def secao_planner(o, d, saida, figuras, pasta=None):
    o.titulo('PLANNER: falhas, bloqueios e conectividade')
    fal, blq = falhas_planner(d), bloqueios(d)
    contornos = [r for r in d.rosout if r[3] == 'destravar' and r[4].startswith('CONTORNAR')]
    if contornos:
        o.p('CONTORNAR: %d venceu, %d nao venceu.' % (sum('venci:' in r[4] for r in contornos), sum('nao venci' in r[4] for r in contornos)))
        for r in contornos:
            x_, y_, _ = pose_mapa(d, r[0])
            o.p('  %s (%.2f, %.2f) %s' % (hora(r[0], 0), x_, y_, r[4]))
    # BLOQUEIO_LOCAL (05/11): sem a fonte no global, as marcas nao entram na conta da conectividade
    blq_global = blq
    arq = os.path.join(pasta, 'params_global_costmap_global_costmap.yaml') if pasta else ''
    if arq and os.path.exists(arq) and 'observation_sources: scan bloqueio' not in open(arq, errors='replace').read():
        blq_global = []
        o.p('Costmap global sem a fonte bloqueio nesta missao: variante B = A.')
    n_solta = sum(1 for r in d.rosout if r[3] == 'destravar' and r[4].startswith('BLOQUEIO: sem caminho'))
    if n_solta:
        o.p('Bloqueio apagado por falta de caminho: %d vez(es).' % n_solta)
    if not fal:
        o.p('Nenhum "failed to plan".')
    surtos = []
    for f in fal:
        if surtos and f[0] - surtos[-1][-1][0] <= 3.0:
            surtos[-1].append(f)
        else:
            surtos.append([f])
    total = 0.0
    o.p('Surtos (falhas separadas por no maximo 3 s):')
    for s in surtos:
        dur = s[-1][0] - s[0][0]
        total += dur
        ant = [b for b in blq if b[0] <= s[0][0]]
        idade = 'sem bloqueio antes' if not ant else 'bloqueio ha %.1f s' % (s[0][0] - ant[-1][0])
        alvos = sorted({'(%.2f, %.2f)' % (f[3], f[4]) for f in s})
        o.p('  %s -> %s  %5.1f s  %2d falhas  destino %s  [inicio: %s]' % (hora(s[0][0], 0), hora(s[-1][0], 0), dur, len(s), ' '.join(alvos), idade))
    o.p('Total em surtos: %.1f s, %d falhas' % (total, len(fal)))
    o.p('Janelas sem caminho (da 1a falha ate o proximo caminho entregue ao smoother ou ao destravar):')
    bons = [st for st, _, _, no, m in d.rosout if (no == 'smoother_server' and 'path to smooth' in m) or (no == 'destravar' and 'caminho novo' in m)]
    fim_j, soma_j = 0.0, 0.0
    for f in fal:
        if f[0] < fim_j:
            continue
        i = bisect.bisect_right(bons, f[0])
        fim_j = bons[i] if i < len(bons) else d.t_fim
        ult = [g[0] for g in fal if g[0] < fim_j][-1]
        ate = fim_j if fim_j - ult < 10 else ult
        soma_j += ate - f[0]
        alvos = sorted({'(%.2f, %.2f)' % (g[3], g[4]) for g in fal if f[0] <= g[0] < fim_j})
        o.p('  %s -> %s  %5.1f s  destino %s' % (hora(f[0], 0), hora(ate, 0), ate - f[0], ' '.join(alvos)))
    o.p('Soma das janelas: %.1f s' % soma_j)
    o.p('\nBloqueios marcados:')
    for b in blq:
        res = '?'
        for st, _, _, nome, msg in d.rosout:
            if st < b[0]:
                continue
            if st > b[0] + 3.0:
                res = 'sem resposta em 3 s'
                break
            if nome == 'destravar' and 'caminho novo' in msg:
                res = 'planner ACHOU caminho com o bloqueio'
                break
            if nome == 'planner_server' and 'failed to plan' in msg and st > b[0] + 1.0:
                res = 'planner FALHOU com o bloqueio'
                break
        o.p('  %s robo em (%.2f, %.2f) yaw %+4.0f: %s' % (hora(b[0], 0), b[1], b[2], math.degrees(b[3]), res))
    with open(os.path.join(saida, 'planner.tsv'), 'w') as f:
        f.write('hora\tde_x\tde_y\tpara_x\tpara_y\tidade_bloqueio_s\tA_costmap\tB_sem_bloqueio\tC_mapa+scan\tcusto_ini\tcusto_fim\tmarca_viva\n')
        cont = collections.Counter()
        culpas = collections.Counter()
        vistos = set()
        for x in fal:
            ant = [b for b in blq if b[0] <= x[0]]
            idade = x[0] - ant[-1][0] if ant else float('nan')
            r = variantes(d, x[0], x[1:3], x[3:5], blq_global)
            if r is None:
                continue
            f.write('%s\t%.2f\t%.2f\t%.2f\t%.2f\t%.1f\t%s\t%s\t%s\t%s\t%s\t%d\n' % (hora(x[0]), x[1], x[2], x[3], x[4], idade, r['A'], r['B'], r['C'], r['custo_ini'], r['custo_fim'], r['marca_viva']))
            chave = 'A=%s B=%s C=%s' % (r['A'], r['B'], r['C'])
            if r['A'] and r['B']:
                culpa = 'costmap de 1 Hz ja mostrava caminho (nao da para atribuir)'
            elif r['B']:
                culpa = 'marca do bloqueio fecha o caminho'
            elif r['custo_fim'] is not None and r['custo_fim'] >= GRADE_BLOQUEADA and not r['C']:
                culpa = 'destino (%.2f, %.2f) em celula bloqueada, sem caminho mesmo sem as marcas' % (x[3], x[4])
            elif not r['C']:
                culpa = 'sem caminho mesmo so com /map + scan (corredor/obstaculo real)'
            else:
                culpa = 'marcas antigas de scan no costmap (so /map + scan conectaria)'
            culpas[culpa] += 1
            cont[chave] += 1
            rotulo = (round(x[0] / 3), x[3], x[4])
            if rotulo not in vistos:
                vistos.add(rotulo)
                o.p('  %s (%.2f,%.2f)->(%.2f,%.2f) bloqueio %5.1f s  A=%-5s B=%-5s C=%-5s custo ini %s fim %s marca %d' % (
                    hora(x[0], 0), x[1], x[2], x[3], x[4], idade, r['A'], r['B'], r['C'], r['custo_ini'], r['custo_fim'], r['marca_viva']))
    o.p('Causa provavel por falha (conectividade + custo do destino):')
    for k, v in culpas.most_common():
        o.p('  %3d  %s' % (v, k))
    o.p('Conectividade nas falhas (True = existe caminho naquele mapa):')
    o.p('  A = costmap publicado | B = A sem as marcas de bloqueio | C = so /map + scan da hora')
    for k, v in cont.most_common():
        o.p('  %3d  %s' % (v, k))
    if figuras:
        figuras_planner(d, saida, surtos, blq)


def figuras_planner(d, saida, surtos, blq):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    alvos = [('falha', s[0][0], s[0][1:3], s[0][3:5]) for s in surtos] + [('falha_fim', s[-1][0], s[-1][1:3], s[-1][3:5]) for s in surtos]
    for rot, t, ini, fim in alvos:
        cm = costmap_em(d, t)
        if cm is None:
            continue
        _, res, ox, oy, g = cm
        fig, ax = plt.subplots(figsize=(7, 7))
        im = np.where(g < 0, 50, g)
        ax.imshow(im, origin='lower', cmap='Greys', vmin=0, vmax=100,
                  extent=(ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res))
        ax.contour(np.arange(g.shape[1]) * res + ox + res / 2, np.arange(g.shape[0]) * res + oy + res / 2,
                   (g >= GRADE_BLOQUEADA).astype(float), levels=[0.5], colors='r', linewidths=0.8)
        x, y, a = pose_mapa(d, t)
        c, s = math.cos(a), math.sin(a)
        cant = [(CORPO_X0, -CORPO_Y), (CORPO_X1, -CORPO_Y), (CORPO_X1, CORPO_Y), (CORPO_X0, CORPO_Y), (CORPO_X0, -CORPO_Y)]
        ax.plot([x + c * u - s * v for u, v in cant], [y + s * u + c * v for u, v in cant], 'b-')
        ax.plot(ini[0], ini[1], 'bo', ms=4)
        ax.plot(fim[0], fim[1], 'g*', ms=14)
        for tb, bx, by, ba in blq:
            if tb <= t:
                p = pontos_bloqueio(bx, by, ba)
                ax.plot(p[:, 0], p[:, 1], '.', color='orange' if t - tb < BLOQ_VALIDADE else 'gold', ms=3)
        i = bisect.bisect_right(d.plano_t, t) - 1
        if i >= 0 and len(d.planos[i][1]):
            ax.plot(d.planos[i][1][:, 0], d.planos[i][1][:, 1], 'c-', lw=1)
        cx, cy = (ini[0] + fim[0]) / 2, (ini[1] + fim[1]) / 2
        m = max(abs(ini[0] - fim[0]), abs(ini[1] - fim[1])) / 2 + 1.2
        ax.set_xlim(cx - m, cx + m)
        ax.set_ylim(cy - m, cy + m)
        ax.set_title('%s %s  vermelho: bloqueado p/ NavFn; laranja: bloqueio; estrela: destino' % (rot, hora(t, 0)), fontsize=9)
        fig.savefig(os.path.join(saida, 'fig_%s_%s.png' % (rot, hora(t, 0).replace(':', ''))), dpi=90)
        plt.close(fig)


def secao_replan(o, d):
    o.titulo('REPLANEJAMENTO: o MANTER_CAMINHO segurou?')
    goals, atual = [], None
    for st, _, _, nome, msg in d.rosout:
        if nome != 'bt_navigator':
            continue
        if msg.startswith('Begin navigating'):
            if atual:
                atual['fim'] = st
                goals.append(atual)
            atual = {'ini': st, 'txt': msg[17:], 'fim': None, 'res': ''}
        elif atual and msg in ('Goal succeeded', 'Goal failed', 'Goal canceled'):
            atual['fim'] = st
            atual['res'] = msg[5:]
            goals.append(atual)
            atual = None
    if atual:
        atual['fim'] = d.t_fim
        goals.append(atual)
    o.p('%-12s %6s %6s %6s %7s  %s' % ('goal', 'dur_s', 'plan/s', 'planos', 'mudou_m', 'resultado'))
    for g in goals:
        pl = [p for p in d.planos if g['ini'] <= p[0] <= g['fim']]
        mud = []
        for a, b in zip(pl, pl[1:]):
            if len(a[1]) and len(b[1]):
                dist = np.min(np.hypot(b[1][:, None, 0] - a[1][None, :, 0], b[1][:, None, 1] - a[1][None, :, 1]), axis=1)
                mud.append(float(dist.max()))
        dur = g['fim'] - g['ini']
        o.p('%-12s %6.1f %6.2f %6d %7.2f  %s  %s' % (hora(g['ini'], 0), dur, len(pl) / max(dur, 0.1), len(pl), max(mud) if mud else 0, g['res'], g['txt']))
    cont = collections.Counter((n, c) for _, n, _, c in d.bt)
    for no in ('CaminhoAtualServe', 'ComputeAndSmoothPath', 'GlobalUpdatedGoal', 'PathExpiringTimer', 'IsPathValid'):
        o.p('  BT %-22s SUCCESS %3d  FAILURE %3d' % (no, cont[(no, 'SUCCESS')], cont[(no, 'FAILURE')]))
    falhas = [e[0] for e in d.bt if e[1] == 'IsPathValid' and e[3] == 'FAILURE']
    com_inscrito = com_letal = sem = 0
    exemplos = []
    for t in falhas:
        i = bisect.bisect_right(d.plano_t, t) - 1
        cm = costmap_em(d, t)
        if i < 0 or cm is None or not len(d.planos[i][1]):
            continue
        _, res, ox, oy, g = cm
        cam = d.planos[i][1]
        x, y, _ = pose_mapa(d, t)
        k = int(np.argmin(np.hypot(cam[:, 0] - x, cam[:, 1] - y)))
        rest = cam[k:]
        ii = np.clip(((rest[:, 0] - ox) / res).astype(int), 0, g.shape[1] - 1)
        jj = np.clip(((rest[:, 1] - oy) / res).astype(int), 0, g.shape[0] - 1)
        c = g[jj, ii]
        if (c == 100).any():
            com_letal += 1
        elif (c >= GRADE_BLOQUEADA).any():
            com_inscrito += 1
        else:
            sem += 1
            exemplos.append((t, int(c.max()), len(rest)))
    testar_footprint(o, d)
    o.p('IsPathValid FAILURE (%d): resto do caminho, so no ponto central: %d com letal, %d com inscrito, %d sem nenhum dos dois'
        % (len(falhas), com_letal, com_inscrito, sem))
    if exemplos:
        o.p('  custo maximo nos casos "sem" (0-98): ' + ' '.join(str(e[1]) for e in exemplos[:30]))


def custo_footprint(g, res, ox, oy, x, y, a):
    """Maior valor do costmap sobre o contorno do footprint do Nav2 na pose (x, y, a)."""
    c, s = math.cos(a), math.sin(a)
    u = np.concatenate([np.arange(-0.205, 0.365, res / 2), np.full(int(0.63 / (res / 2)), 0.365),
                        np.arange(-0.205, 0.365, res / 2), np.full(int(0.63 / (res / 2)), -0.205)])
    v = np.concatenate([np.full(int(math.ceil(0.57 / (res / 2))), 0.315), np.linspace(-0.315, 0.315, int(0.63 / (res / 2))),
                        np.full(int(math.ceil(0.57 / (res / 2))), -0.315), np.linspace(-0.315, 0.315, int(0.63 / (res / 2)))])
    n = min(len(u), len(v))
    u, v = u[:n], v[:n]
    ii = ((x + c * u - s * v - ox) / res).astype(int)
    jj = ((y + s * u + c * v - oy) / res).astype(int)
    fora = (ii < 0) | (jj < 0) | (ii >= g.shape[1]) | (jj >= g.shape[0])
    if fora.any():
        return 100
    return int(g[jj, ii].max())


def testar_footprint(o, d):
    """O is_path_valid do Nav2 1.3 usa o footprint em cada pose do caminho (nao so o centro):
    o caminho vem do NavFn, que so respeita o raio inscrito. Compara com o que a arvore recebeu."""
    res_ev = {'FAILURE': [0, 0], 'SUCCESS': [0, 0]}
    yaw_zero = yaw_total = 0
    for t, no, _, st in d.bt:
        if no != 'IsPathValid' or st not in res_ev:
            continue
        i = bisect.bisect_right(d.plano_t, t) - 1
        cm = costmap_em(d, t)
        if i < 0 or cm is None or not len(d.planos[i][1]):
            continue
        _, res, ox, oy, g = cm
        cam = d.planos[i][1]
        x, y, _ = pose_mapa(d, t)
        k = int(np.argmin(np.hypot(cam[:, 0] - x, cam[:, 1] - y)))
        ruim = any(custo_footprint(g, res, ox, oy, px, py, pa) >= GRADE_BLOQUEADA for px, py, pa in cam[k:])
        res_ev[st][0] += 1
        res_ev[st][1] += ruim
        yaw_total += len(cam)
        yaw_zero += int((np.abs(cam[:, 2]) < 1e-6).sum())
    o.p('Teste do footprint sobre o caminho (yaw do proprio caminho), do ponto mais perto em diante:')
    for st, v in res_ev.items():
        o.p('  BT disse %-8s (%3d vezes): footprint pega celula >= inscrito em %3d' % (st, v[0], v[1]))
    if yaw_total:
        o.p('  poses do caminho com yaw = 0: %.0f%% (NavFn nao orienta o caminho)' % (100.0 * yaw_zero / yaw_total))


def secao_contato(o, d, ev):
    o.titulo('CONTATO: pontos do LiDAR DENTRO do retangulo do corpo (o filtro os apaga do /scan_nav)')
    x, y = d._xb, d._yb
    dentro = (x >= CORPO_X0) & (x <= CORPO_X1) & (np.abs(y) <= CORPO_Y)
    n = dentro.sum(axis=1)
    grupos = []
    for k in np.where(n >= 2)[0]:
        if grupos and d.scan_t[k] - d.scan_t[grupos[-1][-1]] <= 1.0:
            grupos[-1].append(k)
        else:
            grupos.append([k])
    if not grupos:
        o.p('Nenhum scan com 2 ou mais pontos dentro do corpo.')
    for g in grupos:
        k = g[int(np.argmax(n[g]))]
        xs, ys = x[k][dentro[k]], y[k][dentro[k]]
        px, py, pa = pose_mapa(d, d.scan_t[k])
        c, s = math.cos(pa), math.sin(pa)
        o.p('%s %5.1f s  ate %2d pontos; no corpo x %+.2f..%+.2f y %+.2f..%+.2f; no mapa (%.2f, %.2f); %s' % (
            hora(d.scan_t[g[0]], 0), d.scan_t[g[-1]] - d.scan_t[g[0]], n[k], xs.min(), xs.max(), ys.min(), ys.max(),
            px + c * xs.mean() - s * ys.mean(), py + s * xs.mean() + c * ys.mean(), fase_em(ev, d.scan_t[k])))


def secao_folgas(o, d, saida, ev, janelas):
    o.titulo('FOLGAS DO CORPO (/scan_front, corpo real x %.3f..%.3f y +-%.3f)' % (CORPO_X0, CORPO_X1, CORPO_Y))
    x, y = pontos_corpo(d)
    d._xb, d._yb = x, y
    proprio, frac = raios_do_proprio_robo(x, y)
    d._self = proprio
    o.p('Raios que batem no proprio robo (> 25%% dos scans a < 3 cm do corpo): %s' % (', '.join(
        '%d (%.0f%%)' % (i, 100 * frac[i]) for i in proprio) or 'nenhum'))
    f, t, e, di, dm, n2, dentro = folgas_scan(x, y, proprio)
    cmdv = d.cmd['/cmd_vel_nav']

    def cmd_em(tt):
        if not len(cmdv):
            return (0, 0, 0)
        s = amostra(cmdv, tt)
        return (s[1], s[2], s[3]) if tt - s[0] < 0.5 else (0.0, 0.0, 0.0)
    with open(os.path.join(saida, 'folgas.tsv'), 'w') as fo:
        fo.write('hora\tfrente_cm\ttras_cm\tesq_cm\tdir_cm\tmin_cm\tn_pts_lt2cm\tn_dentro\tvx\tvy\twz\tfase\n')
        for k, tt in enumerate(d.scan_t):
            vx, vy, wz = cmd_em(tt)
            fo.write('%s\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%d\t%d\t%.3f\t%.3f\t%.3f\t%s\n' % (
                hora(tt), min(f[k] * 100, 999), min(t[k] * 100, 999), min(e[k] * 100, 999), min(di[k] * 100, 999),
                min(dm[k] * 100, 999), n2[k], dentro[k], vx, vy, wz, fase_em(ev, tt)))
    o.p('\nJanelas com folga < 2 cm (ao menos 2 pontos no scan; junta se a distancia for <= 1,5 s):')
    idx = np.where((dm < 0.02) & (n2 >= 2))[0]
    grupos = []
    for k in idx:
        if grupos and d.scan_t[k] - d.scan_t[grupos[-1][-1]] <= 1.5:
            grupos[-1].append(k)
        else:
            grupos.append([k])
    o.p('%-10s %5s %6s %-8s %-22s %-20s %s' % ('inicio', 'dur_s', 'min_cm', 'lado', 'comando medio vx,vy,wz', 'ponto no corpo/mapa', 'fase'))
    xv = x.copy()
    yv = y.copy()
    xv[:, proprio] = np.nan
    yv[:, proprio] = np.nan
    for g in grupos:
        k0 = g[np.argmin(dm[g])]
        valores = {'frente': f[k0], 'tras': t[k0], 'esq': e[k0], 'dir': di[k0]}
        lado = min(valores, key=valores.get)
        c = np.array([cmd_em(d.scan_t[k]) for k in g])
        dist = dist_corpo(xv[k0], yv[k0])
        dist = np.where(((xv[k0] >= CORPO_X0) & (xv[k0] <= CORPO_X1) & (np.abs(yv[k0]) <= CORPO_Y)) | ~np.isfinite(dist), np.inf, dist)
        j = int(np.argmin(dist))
        px, py, pa = pose_mapa(d, d.scan_t[k0])
        cc, ss = math.cos(pa), math.sin(pa)
        o.p('%-10s %5.1f %6.1f %-8s %+.2f %+.2f %+.2f         (%+.2f,%+.2f) (%.2f,%.2f) %s' % (
            hora(d.scan_t[g[0]], 0), d.scan_t[g[-1]] - d.scan_t[g[0]], dm[k0] * 100, lado, *c.mean(axis=0),
            xv[k0, j], yv[k0, j], px + cc * xv[k0, j] - ss * yv[k0, j], py + ss * xv[k0, j] + cc * yv[k0, j], fase_em(ev, d.scan_t[g[0]])))
    for jan in janelas:
        a, b = jan
        o.p('\nJanela %s-%s, scan a scan (cm):' % (hora(a, 0), hora(b, 0)))
        o.p('%-13s %6s %6s %6s %6s %6s %3s %3s  %s' % ('hora', 'frente', 'tras', 'esq', 'dir', 'min', 'n2', 'in', 'vx vy wz'))
        for k in np.where((d.scan_t >= a) & (d.scan_t <= b))[0]:
            c = cmd_em(d.scan_t[k])
            o.p('%-13s %6.1f %6.1f %6.1f %6.1f %6.1f %3d %3d  %+.2f %+.3f %+.2f' % (
                hora(d.scan_t[k]), min(f[k] * 100, 999), min(t[k] * 100, 999), min(e[k] * 100, 999), min(di[k] * 100, 999),
                min(dm[k] * 100, 999), n2[k], dentro[k], *c))
    d._f, d._t, d._e, d._di, d._dm = f, t, e, di, dm


def secao_cm(o, d, ev):
    o.titulo('COLLISION MONITOR E COMANDOS')
    jan, ini = [], None
    for t, a, _ in d.cm:
        if a == 1 and ini is None:
            ini = t
        elif a != 1 and ini is not None:
            jan.append((ini, t))
            ini = None
    if ini is not None:
        jan.append((ini, d.t_fim))
    por = collections.defaultdict(lambda: [0, 0.0])
    for a, b in jan:
        n = fase_em(ev, a)
        por[n][0] += 1
        por[n][1] += b - a
    o.p('STOP: %d vezes, %.1f s no total. Por fase:' % (len(jan), sum(b - a for a, b in jan)))
    for k, v in sorted(por.items(), key=lambda kv: -kv[1][1]):
        o.p('  %3d vezes %6.1f s  %s' % (v[0], v[1], k))
    nav = d.cmd['/cmd_vel_nav']
    if not len(nav):
        return
    o.p('\nInversoes de sentido do /cmd_vel_nav por fase (|cmd| > 0,01; wz > 0,03):')
    fases_t = [fase_em(ev, t) for t in nav[:, 0]]
    cont = collections.defaultdict(lambda: [0, 0, 0, 0.0])
    ult = [0, 0, 0]
    limiares = (0.01, 0.01, 0.03)
    for i in range(len(nav)):
        for j in range(3):
            v = nav[i, j + 1]
            if abs(v) < limiares[j]:
                continue
            s = 1 if v > 0 else -1
            if ult[j] and s != ult[j]:
                cont[fases_t[i]][j] += 1
            ult[j] = s
        if i:
            if np.any(np.abs(nav[i, 1:]) > (0.005, 0.005, 0.01)):
                cont[fases_t[i]][3] += min(nav[i, 0] - nav[i - 1, 0], 0.2)
    o.p('%6s %6s %6s %8s  %s' % ('vx', 'vy', 'wz', 'cmd_s', 'fase'))
    for k, v in sorted(cont.items(), key=lambda kv: -sum(kv[1][:3])):
        o.p('%6d %6d %6d %8.1f  %s' % (v[0], v[1], v[2], v[3], k))


def secao_giros(o, d, ev):
    o.titulo('GIROS (comando wz com vx,vy ~ 0; angulo medido na IMU)')
    nav = d.cmd['/cmd_vel_nav']
    if not len(nav) or not len(d.imu):
        return
    grupos = []
    for i in np.where(np.abs(nav[:, 3]) > 0.05)[0]:
        t, n = nav[i, 0], fase_em(ev, nav[i, 0])
        if grupos and t - grupos[-1][1] <= 0.3 and grupos[-1][2] == n:
            grupos[-1][1] = t
        else:
            grupos.append([t, t, n])
    grupos = [(a, b) for a, b, _ in grupos]
    por = collections.defaultdict(lambda: [0, 0.0, 0.0])
    o.p('Giros >= 15 graus:')
    for a, b in grupos:
        ya = np.interp(a, d.imu[:, 0], d.imu_yaw)
        yb = np.interp(b, d.imu[:, 0], d.imu_yaw)
        graus = math.degrees(yb - ya)
        n = fase_em(ev, a)
        por[n][0] += 1
        por[n][1] += abs(graus)
        por[n][2] += b - a
        if abs(graus) >= 15:
            o.p('  %s %+6.0f graus %5.1f s  %s' % (hora(a, 0), graus, b - a, n))
    o.p('\nGiro somado por fase:')
    for k, v in sorted(por.items(), key=lambda kv: -kv[1][1]):
        o.p('  %3d giros %6.0f graus %6.1f s  %s' % (v[0], v[1], v[2], k))
    o.p('Giro total comandado na missao: %.0f graus' % sum(v[1] for v in por.values()))
    nav2 = np.array([fase_em(ev, t) in ('Nav2 (arvore)', 'GOAL FINAL') for t in nav[:, 0]])
    shim = nav2 & (np.abs(nav[:, 3]) > 0.20)
    dtn = np.minimum(np.diff(nav[:, 0], prepend=nav[0, 0]), 0.2)
    gi = np.interp(nav[:, 0], d.imu[:, 0], d.imu[:, 2])
    o.p('Durante o Nav2, |wz| > 0,20 (acima do max_vel_theta do DWB: so o RotationShim manda isso): %.1f s, %.0f graus'
        % (dtn[shim].sum(), math.degrees(np.abs(gi[shim] * dtn[shim]).sum())))


def secao_deslizes(o, d, saida, ev):
    o.titulo('DESLIZES LATERAIS NA CURVA (/cmd_vel_nav so com vy)')
    nav = d.cmd['/cmd_vel_nav']
    if not len(nav) or not hasattr(d, '_f'):
        return
    fases_t = np.array([fase_em(ev, t).startswith('CURVA') for t in nav[:, 0]])
    so_vy = fases_t & (np.abs(nav[:, 2]) > 0.03) & (np.abs(nav[:, 1]) < 0.01) & (np.abs(nav[:, 3]) < 0.01)
    grupos, ini = [], None
    for i, a in enumerate(so_vy):
        if a and ini is None:
            ini = i
        elif not a and ini is not None:
            if nav[i - 1, 0] - nav[ini, 0] >= 0.5:
                grupos.append((ini, i - 1))
            ini = None

    def fol(arr, t0):
        k = min(np.searchsorted(d.scan_t, t0), len(d.scan_t) - 1)
        return float(arr[k])
    def ang_parede(t0, frente=True):
        """Angulo da parede a frente/atras do corpo (reta nos pontos do scan), None se nao der."""
        k = min(np.searchsorted(d.scan_t, t0), len(d.scan_t) - 1)
        xs, ys = d._xb[k], d._yb[k]
        if frente:
            m = (xs > CORPO_X1) & (xs < CORPO_X1 + 0.45) & (np.abs(ys) < 0.7)
        else:
            m = (xs < CORPO_X0) & (xs > CORPO_X0 - 0.45) & (np.abs(ys) < 0.7)
        if m.sum() < 8 or np.ptp(ys[m]) < 0.35:
            return None
        cf = np.polyfit(ys[m], xs[m], 1)
        res = xs[m] - np.polyval(cf, ys[m])
        bom = np.abs(res) < 0.02
        if bom.sum() < 8 or np.ptp(ys[m][bom]) < 0.35:
            return None
        return math.degrees(math.atan(np.polyfit(ys[m][bom], xs[m][bom], 1)[0]))

    parede = []
    giro_mapa = []
    linhas = []
    for a, b in grupos:
        t0, t1 = nav[a, 0], nav[b, 0]
        for lado in (True, False):
            pa, pb = ang_parede(t0, lado), ang_parede(t1 + 0.3, lado)
            if pa is not None and pb is not None:
                parede.append(pb - pa)
                break
        dy = math.degrees(np.interp(t1 + 0.3, d.imu[:, 0], d.imu_yaw) - np.interp(t0, d.imu[:, 0], d.imu_yaw))
        p0, p1 = pose_mapa(d, t0), pose_mapa(d, t1 + 0.3)
        giro_mapa.append(math.degrees(dang(p1[2] - p0[2])))
        c, s = math.cos(p0[2]), math.sin(p0[2])
        fx = c * (p1[0] - p0[0]) + s * (p1[1] - p0[1])
        fy = -s * (p1[0] - p0[0]) + c * (p1[1] - p0[1])
        f0, f1, t0f, t1f = fol(d._f, t0), fol(d._f, t1 + 0.3), fol(d._t, t0), fol(d._t, t1 + 0.3)
        linhas.append((t0, t1 - t0, float(np.median(nav[a:b + 1, 2])), dy, fx * 100, fy * 100, f0 * 100, f1 * 100, t0f * 100, t1f * 100))
    with open(os.path.join(saida, 'deslizes.tsv'), 'w') as fo:
        fo.write('hora\tdur_s\tvy\tgiro_graus\tfrente_mapa_cm\tlado_mapa_cm\tfolga_frente_ini\tfolga_frente_fim\tfolga_tras_ini\tfolga_tras_fim\n')
        for l in linhas:
            fo.write('%s\t%.1f\t%.3f\t' % (hora(l[0]), l[1], l[2]) + '\t'.join('%.1f' % v for v in l[3:]) + '\n')
    if not linhas:
        o.p('Nenhum deslize.')
        return
    arr = np.array([l[1:] for l in linhas])
    o.p('%d deslizes; duracao mediana %.1f s; vy mediano %.3f m/s' % (len(arr), np.median(arr[:, 0]), np.median(np.abs(arr[:, 1]))))
    o.p('Giro durante o deslize (IMU): mediana |%.1f| graus, faixa %+.1f a %+.1f' % (np.median(np.abs(arr[:, 2])), arr[:, 2].min(), arr[:, 2].max()))
    o.p('Giro pela pose no mapa (inclui as correcoes do SLAM): mediana |%.1f| graus, faixa %+.1f a %+.1f' % (
        np.median(np.abs(giro_mapa)), min(giro_mapa), max(giro_mapa)))
    if parede:
        o.p('Giro medido pela parede no scan (independente da IMU), %d deslizes: mediana |%.1f| graus, faixa %+.1f a %+.1f' % (
            len(parede), np.median(np.abs(parede)), min(parede), max(parede)))
    o.p('Deriva para frente/tras (pose no mapa, depende do SLAM): faixa %+.1f a %+.1f cm' % (arr[:, 3].min(), arr[:, 3].max()))
    v = np.isfinite(arr[:, 5]) & np.isfinite(arr[:, 7]) & (arr[:, 5] < 100) & (arr[:, 6] < 100) & (arr[:, 7] < 100) & (arr[:, 8] < 100)
    if v.sum() >= 3:
        df, dt_ = arr[v, 6] - arr[v, 5], arr[v, 8] - arr[v, 7]
        trans = (dt_ - df) / 2
        rot = (df + dt_) / 2
        o.p('Com as 2 folgas vistas (%d deslizes), decompondo a variacao das folgas:' % v.sum())
        o.p('  deslocamento frente/tras = (d_tras - d_frente)/2: mediana |%.1f| cm, faixa %+.1f a %+.1f' % (np.median(np.abs(trans)), trans.min(), trans.max()))
        o.p('  folga perdida nas duas pontas (giro/parede) = (d_frente + d_tras)/2: mediana |%.1f| cm, faixa %+.1f a %+.1f' % (np.median(np.abs(rot)), rot.min(), rot.max()))
        g = np.abs(arr[v, 2])
        if g.std() > 0:
            o.p('  correlacao |giro IMU| x |deslocamento|: %.2f; |giro IMU| x |perda nas pontas|: %.2f' % (np.corrcoef(g, np.abs(trans))[0, 1], np.corrcoef(g, np.abs(rot))[0, 1]))
    pos = arr[:, 1] > 0
    for nome, m in (('esquerda', pos), ('direita', ~pos)):
        if m.any():
            o.p('  %-8s: %2d deslizes, giro mediano %+.1f graus, duracao somada %.1f s' % (nome, m.sum(), np.median(arr[m, 2]), arr[m, 0].sum()))
    for i, (t, nome) in enumerate(ev):
        if nome.startswith('CURVA'):
            fim = ev[i + 1][0] if i + 1 < len(ev) else d.t_fim
            q = (nav[:, 0] >= t) & (nav[:, 0] < fim)
            lateral = sum(l[1] for l in linhas if t <= l[0] < fim)
            parado = (np.abs(nav[q, 1]) + np.abs(nav[q, 2]) + np.abs(nav[q, 3]) < 0.005).mean() if q.any() else 0
            n_ar = sum(1 for st, _, _, no, m in d.rosout if t <= st < fim and no == 'alinhador_passagem' and ('afastando' in m or 'PARADA' in m))
            n_cor = sum(1 for st, _, _, no, m in d.rosout if t <= st < fim and no == 'alinhador_passagem' and 'Corrigindo parado' in m)
            n_med = sum(1 for st, _, _, no, m in d.rosout if t <= st < fim and no == 'alinhador_passagem' and 'Medicao' in m)
            o.p('%s %s: %.1f s; %.1f s em deslize puro; comando zero em %.0f%% das amostras; %d medicoes, %d corrigindo parado, %d paradas/afastamentos' % (
                hora(t, 0), nome, fim - t, lateral, 100 * parado, n_med, n_cor, n_ar))


def secao_encostos(o, d, ev):
    o.titulo('ENCOSTOS (/scan_front na faixa a frente do para-choque; memoria do filtro nao e gravada)')
    if not hasattr(d, '_xb'):
        return
    x, y = d._xb, d._yb
    for i, (t, nome) in enumerate(ev):
        if not nome.startswith('ENCOSTAR'):
            continue
        fim = ev[i + 1][0] if i + 1 < len(ev) else d.t_fim
        q = np.where((d.scan_t >= t) & (d.scan_t < fim))[0]
        if not len(q):
            continue
        nf = np.nansum((x[q] > CORPO_X1 + 0.0) & (x[q] <= CORPO_X1 + 0.05) & (np.abs(y[q]) <= CORPO_Y), axis=1)
        cm = sum(1 for tc, a, _ in d.cm if t <= tc < fim and a == 1)
        pulsos = sum(1 for st, _, _, no, m in d.rosout if t <= st < fim and no == 'alinhador_passagem' and 'pulso' in m)
        fr = d._f[q]
        o.p('%s %s: %.1f s, %d pulsos, CM em STOP %d vezes' % (hora(t, 0), nome, fim - t, pulsos, cm))
        o.p('   pontos reais a <= 5 cm da frente, por scan: mediana %.0f, max %d; folga da frente (cm): min %.1f mediana %.1f' % (np.median(nf), nf.max(), fr.min() * 100, np.median(fr) * 100))
        # raio mais perto de cada scan: onde esta o que barra (y em cm)
        mx = (x[q] > CORPO_X1) & (np.abs(y[q]) <= CORPO_Y)
        ys, ds = [], []
        for k in range(len(q)):
            if mx[k].any():
                j = np.where(mx[k])[0]
                m = j[np.nanargmin(x[q][k, j])]
                ys.append(y[q][k, m] * 100)
                ds.append((x[q][k, m] - CORPO_X1) * 100)
        if ys:
            k = q[len(q) // 2]
            px, py, pa = pose_mapa(d, d.scan_t[k])
            c, s = math.cos(pa), math.sin(pa)
            xm = CORPO_X1 + np.median(ds) / 100
            ym = np.median(ys) / 100
            o.p('   ponto mais perto da frente: y mediano %+.0f cm (esq > 0), distancia mediana %.1f cm; no mapa ~(%.2f, %.2f); robo em (%.2f, %.2f)' % (
                np.median(ys), np.median(ds), px + c * xm - s * ym, py + s * xm + c * ym, px, py))


def secao_trajeto(o, d):
    o.titulo('TRAJETO CURTO: o que o alinhador contou x o que a pose no mapa mostrou')
    ini = None
    soma = 0.0
    for st, _, _, nome, msg in d.rosout:
        if nome != 'alinhador_passagem' or not msg.startswith('TRAJETO CURTO'):
            continue
        m = re.search(r'andei (\d+) cm', msg)
        if 'em linha reta' in msg:
            ini, soma, txt = st, 0.0, msg
        elif m:
            soma += float(m.group(1)) / 100
        elif ('SUCESSO' in msg or 'FALHOU' in msg) and ini:
            p0, p1 = pose_mapa(d, ini), pose_mapa(d, st)
            giro = math.degrees(np.interp(st, d.imu[:, 0], d.imu_yaw) - np.interp(ini, d.imu[:, 0], d.imu_yaw)) if len(d.imu) else 0
            o.p('%s %5.1f s  %-44s somou %.2f m, mapa %.2f m, giro %+.0f graus; %s' % (
                hora(ini, 0), st - ini, txt[15:], soma, math.hypot(p1[0] - p0[0], p1[1] - p0[1]), giro, msg[15:]))
            ini = None


def secao_slam(o, d):
    o.titulo('SLAM: saltos do map->odom')
    a = d.map_odom
    if len(a) < 2:
        return
    # Mede o salto na pose do ROBO: mesma odom com o map->odom antes e depois. A diferenca crua de
    # map->odom engana (um giro pequeno em map->odom vira metros longe da origem do odom).
    saltos = []
    for i in range(1, len(a)):
        if a[i, 1] == a[i - 1, 1] and a[i, 2] == a[i - 1, 2] and a[i, 3] == a[i - 1, 3]:
            continue
        _, ox, oy, oa = amostra(d.odom_base, a[i, 0]) if len(d.odom_base) else (0, 0, 0, 0)
        px = []
        for k in (i - 1, i):
            c, s = math.cos(a[k, 3]), math.sin(a[k, 3])
            px.append((a[k, 1] + c * ox - s * oy, a[k, 2] + s * ox + c * oy))
        saltos.append((a[i, 0], math.hypot(px[1][0] - px[0][0], px[1][1] - px[0][1]), float(dang(a[i, 3] - a[i - 1, 3]))))
    s = np.array(saltos).reshape(-1, 3)
    if not len(s):
        o.p('map->odom nao mudou.')
        return
    o.p('%d correcoes do SLAM. Na pose do robo: mediana %.1f cm; %d acima de 5 cm; %d acima de 10 cm; maior %.1f cm. Giro: %d acima de 3 graus; maior %.1f graus'
        % (len(s), np.median(s[:, 1]) * 100, (s[:, 1] > 0.05).sum(), (s[:, 1] > 0.10).sum(), s[:, 1].max() * 100,
           (np.abs(s[:, 2]) > math.radians(3)).sum(), math.degrees(np.abs(s[:, 2]).max())))
    for k in np.argsort(-s[:, 1])[:10]:
        o.p('  %s  %5.1f cm  %+5.1f graus' % (hora(s[k, 0], 0), s[k, 1] * 100, math.degrees(s[k, 2])))


def ler_janela(txt, base):
    a, b = txt.split('-')
    dia = dt.datetime.fromtimestamp(base, FUSO).date()

    def conv(s):
        p = [int(v) for v in s.split(':')]
        return dt.datetime(dia.year, dia.month, dia.day, p[0], p[1], p[2], tzinfo=FUSO).timestamp()
    return conv(a), conv(b)


def main():
    ap = argparse.ArgumentParser(description='Analisa uma missao (so leitura do bag).')
    ap.add_argument('alvo', help='missaoNN_*.tar.gz, pasta extraida, .mcap ou .mcap.zstd')
    ap.add_argument('--saida', default=None, help='pasta de saida (padrao: ./analise_<nome>)')
    ap.add_argument('--janela', action='append', default=[], help='HH:MM:SS-HH:MM:SS para ver scan a scan')
    ap.add_argument('--figuras', action='store_true', help='gera PNG do costmap nas falhas do planner')
    ap.add_argument('--ros-log-dir', default=None, help='confere ~/.ros/log do periodo contra o /rosout')
    a = ap.parse_args()
    nome = re.sub(r'(_tar)?\.tar\.gz$|\.mcap.*$', '', os.path.basename(a.alvo.rstrip('/')))
    saida = os.path.abspath(os.path.expanduser(a.saida or 'analise_' + nome))
    os.makedirs(saida, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix='analisa_missao_')
    try:
        mcap, pasta = achar_mcap(a.alvo, tmp)
        d = carregar(mcap)
        o = Saida(os.path.join(saida, 'relatorio.txt'))
        o.p('Analise de %s' % nome)
        ev = fases(d)
        with open(os.path.join(saida, 'linha_do_tempo.tsv'), 'w') as f:
            niv = {10: 'D', 20: 'I', 30: 'W', 40: 'E', 50: 'F'}
            for st, _, nv, no, msg in d.rosout:
                f.write('%s\t%s\t%s\t%s\n' % (hora(st), niv.get(nv, nv), no, msg.replace('\n', ' | ')))
        secao_pacote(o, d, pasta, a.ros_log_dir)
        secao_trechos(o, d, ev)
        janelas = [ler_janela(j, d.t_ini) for j in a.janela]
        secao_folgas(o, d, saida, ev, janelas)
        secao_planner(o, d, saida, a.figuras, pasta)
        secao_contato(o, d, ev)
        secao_replan(o, d)
        secao_cm(o, d, ev)
        secao_giros(o, d, ev)
        secao_deslizes(o, d, saida, ev)
        secao_encostos(o, d, ev)
        secao_trajeto(o, d)
        secao_slam(o, d)
        o.p('\nArquivos em %s' % saida)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
