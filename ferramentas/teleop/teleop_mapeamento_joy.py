#!/usr/bin/env python3
"""Teleop manual por controle (Xbox Series X ou PS5) para mapear a arena e marcar pontos.

Le o controle direto em /dev/input/jsN (API joystick do Linux, sem joy_node), detecta o modelo
pelo nome do dispositivo, publica TwistStamped em /cmd_vel_nav (o Collision Monitor continua no
pipeline) e grava poses map->base_link nos arquivos que o executor_prova.py ja le:

  START / FINISH         -> --pontos         (padrao ~/mapas/pontos.yaml, chave 'pontos')
  WS_n / SH_n / PP_n     -> --service-areas  (padrao ~/mapas/service_areas.yaml)
     type: workstation | shelf | precision
     aproximacao = pose do robo no momento da marcacao (robo DE FRENTE para a mesa)
     face        = estimada a --dist-face m a frente do robo (origem: teleop)

Comandos (Xbox / PS5) — BOTOES_PROVA (23/11):
  LB / L1 (segurar)     homem-morto: so anda segurando (soltar = para na hora)
  analogico esquerdo    frente/re e lateral (mecanum)
  analogico direito X   girar
  RB / R1 (segurar)     velocidade rapida
  X / Quadrado          marca a WS que esta NA FRENTE do robo (altura 10 cm)
  A / Cruz              marca a SH que esta na frente do robo (altura 5 cm)
  B / Circulo           marca a PP que esta na frente do robo (altura 15 cm)
  Y / Triangulo (2x)    REINICIA posicao e arena (ver abaixo)
  D-pad cima (2x)       marca o START na pose atual
  D-pad baixo (2x)      marca o FINISH na pose atual
  D-pad esquerda (2x)   marca o START_2 (caixa 2) na pose atual   # TELEOP_CAIXA2 (24/11)
  D-pad direita (2x)    marca o FINISH_2 (caixa 2) na pose atual
  Menu / Options (2x)   desfaz a ultima marcacao
  View / Create         lista o que ja foi marcado
  "2x" = apertar de novo em ate 2 s (confirmacao).
Marcar mesa: robo DE FRENTE para a mesa, a ~50-90 cm, no meio dela. A face e medida pelo LiDAR
  (como o marcar_mesa.py); se o LiDAR nao achar a face (mesa baixa, so uma ponta), marca pela pose
  do robo com a face estimada a --dist-face m (origem: teleop) e avisa. Se ja existe uma mesa do
  mesmo tipo a ate 0,40 m dessa face, ela e ATUALIZADA (mesmo numero); senao vira o proximo numero
  livre (WS_1, WS_2, ...). Toda marcacao e gravada na hora (backup .bak_teleop_<hora> na 1a gravacao).
Reiniciar (Y 2x):
  - SLAM em mapping (subir_tudo, mapa do zero): zera a odometria, reseta o slam_toolbox (mapa novo
    com origem onde o robo esta), START = (0,0,0) e APAGA as mesas e o FINISH (eram do mapa velho).
  - SLAM em localization (arena, mapa fixo): recarrega a arena com o robo no START gravado
    (deserialize_map, match_type 3). Ponha o robo no START antes. Mesas mantidas.
Teclado (terminal): Ctrl+C sai.

Nao usar junto de goal Nav2 ativo.

TELEOP_LOCAL (23/11, copia local de Arthur-Demetrio/robo_work_cajumakers 9b9e434):
  - SO_COM_LB: so publica em /cmd_vel_nav com o LB apertado (e mais ZEROS_AO_SOLTAR zeros ao soltar,
    na PARADA ou ao perder o controle). Parado, nao publica: nao briga com alinhador/Nav2/ajuste_base.
  - FACE_MEDIDA: remarcar uma mesa que ja tem face medida pelo LiDAR (marcar_mesa.py) so troca a
    aproximacao e MANTEM a face (e largura_medida). --substituir-face volta ao comportamento antigo.
  - Controle por cabo USB: desconectar some o /dev/input/jsN (ou da erro de leitura) -> zera e para.
    Nao ha tempo limite por falta de evento: com o analogico parado o driver nao manda evento.
"""
import argparse
import array
import copy
import fcntl
import glob
import math
import os
import select
import shutil
import statistics
import struct
import sys
import time

import yaml

# ---------------------------------------------------------------------------
# Mapeamentos de botoes/eixos (API js do Linux)
# ---------------------------------------------------------------------------
# Xbox Series X/S pelo driver xpad (USB) ou xpadneo (Bluetooth): mesma numeracao.
# PS5 DualSense pelo hid-playstation (USB e Bluetooth).
# Se o seu controle numerar diferente, rode com --mostrar-eventos e ajuste/use --mapa.
# BOTOES_PROVA (23/11): X=WS, A=SH, B=PP, Y=reiniciar, Menu=desfazer (era A marca/B parada/Y apaga/Menu grava).
PERFIS = {
    'xbox': {
        'botoes': {'WS': 2, 'SH': 0, 'PP': 1, 'reiniciar': 3, 'deadman': 4, 'rapido': 5,
                   'listar': 6, 'desfazer': 7},
        'eixos': {'lx': 0, 'ly': 1, 'rx': 3, 'hat_x': 6, 'hat_y': 7},
        'nomes': {'WS': 'X', 'SH': 'A', 'PP': 'B', 'reiniciar': 'Y', 'deadman': 'LB', 'rapido': 'RB',
                  'listar': 'View', 'desfazer': 'Menu'},
    },
    # Xbox por Bluetooth sem xpadneo (hid-generic): X/Y deslocados e LB/RB em 6/7.
    'xbox_bt': {
        'botoes': {'WS': 3, 'SH': 0, 'PP': 1, 'reiniciar': 4, 'deadman': 6, 'rapido': 7,
                   'listar': 10, 'desfazer': 11},
        'eixos': {'lx': 0, 'ly': 1, 'rx': 2, 'hat_x': 6, 'hat_y': 7},
        'nomes': {'WS': 'X', 'SH': 'A', 'PP': 'B', 'reiniciar': 'Y', 'deadman': 'LB', 'rapido': 'RB',
                  'listar': 'View', 'desfazer': 'Menu'},
    },
    'ps5': {
        'botoes': {'WS': 3, 'SH': 0, 'PP': 1, 'reiniciar': 2, 'deadman': 4, 'rapido': 5,
                   'listar': 8, 'desfazer': 9},
        'eixos': {'lx': 0, 'ly': 1, 'rx': 3, 'hat_x': 6, 'hat_y': 7},
        'nomes': {'WS': 'Quadrado', 'SH': 'Cruz', 'PP': 'Circulo', 'reiniciar': 'Triangulo',
                  'deadman': 'L1', 'rapido': 'R1', 'listar': 'Create', 'desfazer': 'Options'},
    },
}

TIPO_AREA = {'WS': 'workstation', 'SH': 'shelf', 'PP': 'precision'}
ALTURA_PADRAO = {'WS': 0.10, 'SH': 0.05, 'PP': 0.15}   # BOTOES_PROVA: medidas do usuario (23/11)
RAIO_MESMA_MESA = 0.40      # face nova a ate isto de uma marcada do mesmo tipo = mesma mesa
MARCAR_MESA = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/marcar_mesa.py')


def escolher_chave(areas, tipo, face_xy, raio=RAIO_MESMA_MESA, max_indice=9):
    """Mesa do mesmo tipo com face a ate `raio` -> a mesma chave; senao o menor numero livre."""
    perto, dmin = None, raio
    for k, a in areas.items():
        if not k.startswith(tipo + '_'):
            continue
        f = a.get('face') or {}
        if 'x' in f:
            d = math.hypot(f['x'] - face_xy[0], f['y'] - face_xy[1])
            if d <= dmin:
                perto, dmin = k, d
    if perto:
        return perto
    for n in range(1, max_indice + 1):
        if '%s_%d' % (tipo, n) not in areas:
            return '%s_%d' % (tipo, n)
    return None

JS_EVENT_FMT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80
JSIOCGNAME = lambda n: 0x80006A13 + (0x10000 * n)  # noqa: E731
JSIOCGAXES, JSIOCGBUTTONS = 0x80016A11, 0x80016A12


# ---------------------------------------------------------------------------
# Controle
# ---------------------------------------------------------------------------

class Joystick:
    def __init__(self, dev):
        self.fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        buf = array.array('B', [0] * 128)
        fcntl.ioctl(self.fd, JSIOCGNAME(len(buf)), buf)
        self.nome = buf.tobytes().split(b'\0')[0].decode(errors='replace')
        n = array.array('B', [0])
        fcntl.ioctl(self.fd, JSIOCGAXES, n)
        self.eixos = [0.0] * n[0]
        fcntl.ioctl(self.fd, JSIOCGBUTTONS, n)
        self.botoes = [0] * n[0]
        self.ultimo_evento = time.monotonic()

    def ler(self):
        """Le todos os eventos pendentes; devolve lista de (tipo, numero, valor) de bordas."""
        bordas = []
        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break
            if len(data) < JS_EVENT_SIZE:
                break
            _, valor, tipo, num = struct.unpack(JS_EVENT_FMT, data)
            init = bool(tipo & JS_EVENT_INIT)
            tipo &= ~JS_EVENT_INIT
            self.ultimo_evento = time.monotonic()
            if tipo == JS_EVENT_BUTTON and num < len(self.botoes):
                self.botoes[num] = valor
                if not init:
                    bordas.append(('b', num, valor))
            elif tipo == JS_EVENT_AXIS and num < len(self.eixos):
                antigo = self.eixos[num]
                self.eixos[num] = valor / 32767.0
                if not init:
                    bordas.append(('a', num, self.eixos[num], antigo))
        return bordas

    def fechar(self):
        os.close(self.fd)


def achar_controle(dev_arg):
    devs = [dev_arg] if dev_arg else sorted(glob.glob('/dev/input/js*'))
    for d in devs:
        try:
            return Joystick(d), d
        except OSError as e:
            print('Nao abriu %s: %s' % (d, e))
    raise SystemExit('Nenhum controle em /dev/input/js*. Conecte o controle (USB ou Bluetooth) '
                     'e confira permissao (grupo input).')


def detectar_perfil(nome):
    n = nome.lower()
    if any(k in n for k in ('dualsense', 'playstation', 'sony', 'wireless controller')):
        return 'ps5'
    if 'xbox' in n or 'x-box' in n or 'microsoft' in n:
        return 'xbox'
    return None


# ---------------------------------------------------------------------------
# Arquivos
# ---------------------------------------------------------------------------

def ler_yaml(caminho):
    if os.path.exists(caminho):
        with open(caminho) as f:
            return yaml.safe_load(f) or {}
    return {}


def backup(caminho):
    if os.path.exists(caminho):
        bak = caminho + '.bak_teleop_' + time.strftime('%Y%m%d_%H%M%S')
        shutil.copy2(caminho, bak)
        return bak
    return None


def gravar_yaml(caminho, doc, cabecalho):
    os.makedirs(os.path.dirname(os.path.abspath(caminho)), exist_ok=True)
    bak = backup(caminho)
    tmp = caminho + '.tmp'
    with open(tmp, 'w') as f:
        f.write(cabecalho)
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
    os.replace(tmp, caminho)
    return bak


def pose_dict(x, y, yaw):
    return {'x': round(float(x), 3), 'y': round(float(y), 3), 'yaw': round(float(yaw), 4)}


# ---------------------------------------------------------------------------
# Programa
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description='Teleop por controle Xbox/PS5 para mapeamento e marcacao '
                                            'de START/FINISH/WS/SH/PP')
    p.add_argument('--dev', help='dispositivo do controle (padrao: primeiro /dev/input/js*)')
    p.add_argument('--controle', choices=sorted(PERFIS), help='forca o perfil (padrao: autodetecta)')
    # TELEOP_RAPIDO (24/11): era 0.10 / 0.08 / 0.30 (teto do velocity_smoother: 0.20 / 0.16 / 0.40)
    p.add_argument('--linear', type=float, default=0.15, help='m/s maximo frente/re (normal)')
    p.add_argument('--lateral', type=float, default=0.12, help='m/s maximo lateral (normal)')
    p.add_argument('--angular', type=float, default=0.35, help='rad/s maximo giro (normal)')
    p.add_argument('--fator-rapido', type=float, default=2.0, help='multiplicador com RB/R1')
    p.add_argument('--zona-morta', type=float, default=0.12)
    p.add_argument('--rampa', type=float, default=0.30, help='aceleracao max em m/s^2 (giro: x3)')
    p.add_argument('--topic', default='/cmd_vel_nav')
    # SEM_CM (23/11): publica direto no /cmd_vel da ponte, SEM suavizador e SEM Collision Monitor,
    # so enquanto este teleop roda (o Nav2 continua com o CM). Para mapear com ruido do LiDAR no corpo.
    p.add_argument('--sem-cm', action='store_true',
                   help='publica em /cmd_vel (pula o Collision Monitor). CUIDADO: nada para o robo antes de bater')
    p.add_argument('--taxa', type=float, default=20.0, help='Hz de publicacao')
    p.add_argument('--pontos', default=os.path.expanduser('~/mapas/pontos.yaml'))
    p.add_argument('--service-areas', default=os.path.expanduser('~/mapas/service_areas.yaml'))
    p.add_argument('--dist-face', type=float, default=0.60,
                   help='distancia (m) da pose do robo ate a face estimada da mesa')
    p.add_argument('--max-indice', type=int, default=9)
    p.add_argument('--substituir-face', action='store_true',
                   help='FACE_MEDIDA: ao remarcar mesa, troca tambem a face medida pelo LiDAR pela estimada')
    p.add_argument('--mostrar-eventos', action='store_true',
                   help='so imprime botoes/eixos do controle (para conferir o mapeamento)')
    # BOTOES_PROVA (23/11)
    p.add_argument('--altura-ws', type=float, default=10.0, help='cm (X)')
    p.add_argument('--altura-sh', type=float, default=5.0, help='cm (A)')
    p.add_argument('--altura-pp', type=float, default=15.0, help='cm (B)')
    p.add_argument('--arena-mapa', default=os.environ.get('ARENA_MAPA', 'arena_21-11_1727'),
                   help='mapa recarregado pelo Y quando o SLAM esta em localization (~/mapas/<nome>)')
    a = p.parse_args()
    a.alturas = {'WS': a.altura_ws / 100.0, 'SH': a.altura_sh / 100.0, 'PP': a.altura_pp / 100.0}
    if not (0 < a.linear <= 0.25 and 0 < a.lateral <= 0.25 and 0 < a.angular <= 0.8):
        p.error('limites: 0 < linear,lateral <= 0.25 m/s; 0 < angular <= 0.8 rad/s')
    if not 1.0 <= a.fator_rapido <= 3.0:
        p.error('fator-rapido entre 1 e 3')
    if a.sem_cm:
        a.topic = '/cmd_vel'  # SEM_CM
    return a


def mostrar_eventos(js):
    print('Controle: "%s" (%d eixos, %d botoes). Ctrl+C sai.' % (js.nome, len(js.eixos), len(js.botoes)))
    try:
        while True:
            select.select([js.fd], [], [], 0.2)
            for ev in js.ler():
                if ev[0] == 'b':
                    print('botao %2d -> %d' % (ev[1], ev[2]))
                elif abs(ev[2] - ev[3]) > 0.25 or ev[2] in (-1.0, 1.0):
                    print('eixo  %2d -> %+.2f' % (ev[1], ev[2]))
    except KeyboardInterrupt:
        pass


def main():
    a = parse_args()
    js, dev = achar_controle(a.dev)
    if a.mostrar_eventos:
        mostrar_eventos(js)
        return

    perfil_nome = a.controle or detectar_perfil(js.nome)
    if perfil_nome is None:
        raise SystemExit('Controle "%s" nao reconhecido; use --controle xbox|xbox_bt|ps5.' % js.nome)
    perfil = PERFIS[perfil_nome]
    B, E, N = perfil['botoes'], perfil['eixos'], perfil['nomes']
    precisa = max(B.values()) + 1
    if len(js.botoes) < precisa or len(js.eixos) <= max(E.values()):
        print('AVISO: controle tem %d botoes/%d eixos; o perfil %s espera mais. '
              'Confira com --mostrar-eventos.' % (len(js.botoes), len(js.eixos), perfil_nome))

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from slam_toolbox.srv import DeserializePoseGraph, Reset
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    # BOTOES_PROVA: sem o tratador de SIGINT do rclpy (ele invalidava o contexto antes do zero final do Ctrl+C)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('teleop_mapeamento_joy')
    pub = node.create_publisher(TwistStamped, a.topic, 10)
    tfb = Buffer()
    # TF_SPIN (23/11): o no nunca e girado (sem spin); sem a thread propria o listener nunca recebia TF.
    # BOTOES_PROVA: essa mesma thread gira o no inteiro (scan do medidor e respostas dos servicos).
    tf_listener = TransformListener(tfb, node, spin_thread=True)  # noqa: F841
    cli_odom = node.create_client(Trigger, '/reset_odometry')
    cli_reset = node.create_client(Reset, '/slam_toolbox/reset')
    cli_deser = node.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
    cli_param = node.create_client(GetParameters, '/slam_toolbox/get_parameters')
    # LIDAR_M2M2 (24/11): com a ponte do M2M2 o Y usa os servicos dela (sem slam_toolbox)
    cli_m2_limpar = node.create_client(Trigger, '/m2m2_bridge/limpar_mapa')
    cli_m2_carregar = node.create_client(Trigger, '/m2m2_bridge/carregar_mapa')
    cli_m2_param = node.create_client(GetParameters, '/m2m2_bridge/get_parameters')
    from geometry_msgs.msg import PoseWithCovarianceStamped
    pub_initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    # BOTOES_PROVA: face pelo LiDAR com o mesmo medidor do marcar_mesa.py (medir_face do alinhador)
    mm, medidor = None, None
    try:
        import importlib.util
        sys.path.insert(0, os.path.dirname(MARCAR_MESA))   # alinhador_passagem importa face_ws dali
        sp = importlib.util.spec_from_file_location('marcar_mesa', MARCAR_MESA)
        mm = importlib.util.module_from_spec(sp)
        sp.loader.exec_module(mm)
        medidor = mm.Medidor(node, mm.carregar_alinhador())
    except Exception as e:  # noqa: BLE001
        print('AVISO: sem medida da face pelo LiDAR (%s: %s); mesas serao marcadas pela pose.'
              % (type(e).__name__, e))

    # Estado das marcacoes (comeca com o que ja existe nos arquivos)
    doc_pts = ler_yaml(a.pontos)
    pontos = dict(doc_pts.get('pontos') or {})
    doc_sa = ler_yaml(a.service_areas)
    areas = dict(doc_sa.get('service_areas') or {})
    historico = []          # (descricao, pontos_antes, areas_antes) para o desfazer
    ja_fez_backup = set()   # backup .bak_teleop_* so na 1a gravacao de cada arquivo

    v = [0.0, 0.0, 0.0]
    zeros_restantes = 0   # SO_COM_LB: zeros a mandar depois de soltar
    ZEROS_AO_SOLTAR = 6   # 0,3 s a 20 Hz
    confirmar_pedido = {'acao': None, 't': 0.0}
    dt = 1.0 / a.taxa
    ultima = {'msg': ''}

    def status(msg=None):
        if msg is not None:
            ultima['msg'] = msg
        sys.stdout.write('\r\033[K[%s] mesas: %d | %s' % (
            'ANDANDO' if any(v) else 'parado ', len(areas), ultima['msg']))
        sys.stdout.flush()

    def log(msg):
        sys.stdout.write('\r\033[K' + msg + '\n')
        status()

    def confirmado(acao, texto):
        """Acao destrutiva: so executa no 2o aperto em ate 2 s."""
        agora = time.monotonic()
        if confirmar_pedido['acao'] == acao and agora - confirmar_pedido['t'] <= 2.0:
            confirmar_pedido['acao'] = None
            return True
        confirmar_pedido.update(acao=acao, t=agora)
        log('Aperte de novo em 2 s para %s.' % texto)
        return False

    def parar_robo():
        nonlocal v, zeros_restantes
        v = [0.0, 0.0, 0.0]
        for _ in range(3):
            publicar(0.0, 0.0, 0.0)
        zeros_restantes = ZEROS_AO_SOLTAR

    def pose_atual():
        try:
            tr = tfb.lookup_transform('map', 'base_link', Time())
        except Exception as e:  # noqa: BLE001
            log('ERRO: sem TF map->base_link (%s). SLAM/localizacao no ar?' % type(e).__name__)
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def gravar(motivo):
        hora = time.strftime('%Y-%m-%d %H:%M:%S')
        for caminho, doc, cab in (
                (a.pontos, doc_pts, '# START e FINISH (frame map). Gravado por teleop_mapeamento_joy.py em %s\n' % hora),
                (a.service_areas, doc_sa, '# Areas marcadas (WS/SH/PP). Frame map. Gravado por teleop_mapeamento_joy.py '
                                          'em %s; face do LiDAR, ou estimada a %.2f m quando origem: teleop\n'
                                          % (hora, a.dist_face))):
            if caminho == a.pontos:
                doc['pontos'] = pontos
            else:
                doc['frame'] = doc.get('frame', 'map')
                doc['service_areas'] = areas
                doc['gerado_em'] = hora
            os.makedirs(os.path.dirname(os.path.abspath(caminho)), exist_ok=True)
            if caminho not in ja_fez_backup:
                b = backup(caminho)
                ja_fez_backup.add(caminho)
                if b:
                    log('  backup: %s' % b)
            tmp = caminho + '.tmp'
            with open(tmp, 'w') as f:
                f.write(cab)
                yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
            os.replace(tmp, caminho)
        status('gravado (%s)' % motivo)

    def lembrar(descricao):
        historico.append((descricao, copy.deepcopy(pontos), copy.deepcopy(areas)))
        del historico[:-20]

    def medir_face(n=6, timeout=4.0):
        """Mediana de n faces boas (duas pontas, largura 0,60-0,98 m) -> dict ou motivo (str)."""
        if medidor is None:
            return 'medidor indisponivel'
        medidor.scans.clear()
        medidas, vistas = [], 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and len(medidas) < n:
            time.sleep(0.05)
            med, face = medidor.medir()
            if face is None:
                continue
            vistas += 1
            if mm.marcavel(med):
                medidas.append(face + (med['span'],))
        if len(medidas) < 4:
            return 'so %d faces boas (%d vistas) em %.0f s' % (len(medidas), vistas, timeout)
        fx = statistics.median(m[0] for m in medidas)
        fy = statistics.median(m[1] for m in medidas)
        fyaw = math.atan2(statistics.median(math.sin(m[2]) for m in medidas),
                          statistics.median(math.cos(m[2]) for m in medidas))
        return {'x': fx, 'y': fy, 'yaw': fyaw, 'larg': statistics.median(m[3] for m in medidas),
                'disp': max(math.hypot(m[0] - fx, m[1] - fy) for m in medidas), 'n': len(medidas)}

    def marcar_mesa(tipo):
        parar_robo()
        altura = a.alturas[tipo]
        log('Marcando %s (%.0f cm): medindo a face pelo LiDAR...' % (tipo, altura * 100))
        p = pose_atual()
        if p is None:
            return
        x, y, yaw = p
        med = medir_face()
        if isinstance(med, dict):
            face = dict(pose_dict(med['x'], med['y'], med['yaw']), largura_medida=round(med['larg'], 2))
            fyaw = med['yaw']
            aprox = pose_dict(med['x'] - mm.DIST_APROX * math.cos(fyaw),
                              med['y'] - mm.DIST_APROX * math.sin(fyaw), fyaw)
            origem = 'LiDAR: largura %.2f m, espalhamento %.1f cm, %d medidas' % (
                med['larg'], 100 * med['disp'], med['n'])
        else:
            face = dict(pose_dict(x + a.dist_face * math.cos(yaw), y + a.dist_face * math.sin(yaw), yaw),
                        origem='teleop')
            aprox = pose_dict(x, y, yaw)
            origem = '!!! LiDAR nao achou a face (%s): face ESTIMADA a %.2f m a frente do robo' % (
                med, a.dist_face)
        k = escolher_chave(areas, tipo, (face['x'], face['y']), max_indice=a.max_indice)
        if k is None:
            log('ERRO: ja existem %d %s; nada marcado.' % (a.max_indice, tipo))
            return
        antigo = areas.get(k) or {}
        face_antiga = antigo.get('face') or {}
        # FACE_MEDIDA: estimativa do teleop nao substitui face medida pelo LiDAR (salvo --substituir-face)
        if face.get('origem') == 'teleop' and face_antiga and face_antiga.get('origem') != 'teleop' \
                and not a.substituir_face:
            face, aprox = dict(face_antiga), dict(antigo.get('aproximacao') or aprox)
            origem += '; FACE_MEDIDA: mantida a face do LiDAR ja gravada'
        lembrar('%s %s' % ('atualizar' if antigo else 'marcar', k))
        novo = dict(antigo)
        tabela = dict(antigo.get('table') or {'width': 0.8, 'depth': 0.5})
        tabela['height'] = round(altura, 2)
        novo.update({'type': TIPO_AREA[tipo], 'table': tabela, 'face': face, 'aproximacao': aprox,
                     'marcado_em': time.strftime('%Y-%m-%d %H:%M:%S')})
        areas[k] = novo
        log('%s %s (%.0f cm): face (%.3f, %.3f) %.1f graus | %s' % (
            'ATUALIZADA' if antigo else 'MARCADA', k, altura * 100, face['x'], face['y'],
            math.degrees(face['yaw']), origem))
        gravar(k)

    def marcar_ponto(nome):
        p = pose_atual()
        if p is None:
            return
        lembrar('marcar %s' % nome)
        pontos[nome] = pose_dict(*p)
        log('MARCADO %s x=%.3f y=%.3f yaw=%.1f graus' % (nome, p[0], p[1], math.degrees(p[2])))
        gravar(nome)

    def desfazer():
        nonlocal pontos, areas
        if not historico:
            log('Nada para desfazer.')
            return
        desc, pontos_antes, areas_antes = historico.pop()
        pontos.clear()
        pontos.update(pontos_antes)
        areas.clear()
        areas.update(areas_antes)
        log('DESFEITO: %s' % desc)
        gravar('desfazer')

    def chamar(cli, req, timeout):
        if not cli.wait_for_service(timeout_sec=2.0):
            return None, 'servico %s ausente' % cli.srv_name
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        if not fut.done():
            fut.cancel()
            return None, '%s sem resposta em %.0f s' % (cli.srv_name, timeout)
        return fut.result(), ''

    def modo_slam():
        req = GetParameters.Request()
        req.names = ['mode']
        r, err = chamar(cli_param, req, 3.0)
        if r is None or not r.values:
            return None, err or 'parametro mode vazio'
        return r.values[0].string_value, ''

    def m2m2_pose(x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        pub_initial.publish(msg)

    def reiniciar_m2m2():
        req = GetParameters.Request()
        req.names = ['mapeando', 'arquivo_mapa']
        r, err = chamar(cli_m2_param, req, 3.0)
        mapeando = bool(r.values[0].bool_value) if r is not None and r.values else True
        if not mapeando:
            st = pontos.get('START')
            if not st:
                log('REINICIAR (M2M2 localizacao) falhou: sem START marcado. Nada mudou.')
                return
            log('REINICIAR (M2M2 localizacao): recarregando %s com o robo no START (%.3f, %.3f, %.1f graus)...'
                % (r.values[1].string_value, st['x'], st['y'], math.degrees(st['yaw'])))
            rr, err = chamar(cli_m2_carregar, Trigger.Request(), 30.0)
            if rr is None or not rr.success:
                log('REINICIAR falhou: %s' % (err or rr.message))
                return
            time.sleep(0.5)
            m2m2_pose(st['x'], st['y'], st['yaw'])
            log('REINICIAR: mapa do M2M2 recarregado com o robo no START.')
            return
        # M2M2_SEM_SETPOSE (24/11): reposicionar o M2M2 enquanto mapeia o deixa perdido
        # (qualidade 0, para de gravar a arena). So limpa o mapa; START = pose atual.
        log('REINICIAR (M2M2 mapeando): limpando o mapa do M2M2 (START = pose atual)...')
        rr, err = chamar(cli_m2_limpar, Trigger.Request(), 10.0)
        if rr is None or not rr.success:
            log('REINICIAR falhou ao limpar o mapa do M2M2 (%s). Nada mudou.' % (err or rr.message))
            return
        time.sleep(1.0)
        pose = pose_atual()
        if pose is None:
            log('REINICIAR: mapa limpo, mas sem TF map->base_link para o START. Marque o START no D-pad.')
            return
        lembrar('reiniciar')
        areas.clear()
        pontos.clear()
        pontos['START'] = pose_dict(*pose)
        log('REINICIADO (M2M2): mapa novo; START = pose atual (%.3f, %.3f, %.1f graus); mesas e FINISH apagados.'
            % (pose[0], pose[1], math.degrees(pose[2])))
        gravar('reiniciar')

    def m2m2_dono_do_mapa():
        # REINICIAR_SLAM_M2M2 (25/11): era so 'servico limpar_mapa existe'. Com publish_map=false
        # (base_m2m2_slam.launch.py) o mapa e do slam_toolbox.
        if not cli_m2_limpar.wait_for_service(timeout_sec=0.5):
            return False
        req = GetParameters.Request()
        req.names = ['publish_map']
        r, _ = chamar(cli_m2_param, req, 3.0)
        return bool(r.values[0].bool_value) if r is not None and r.values else True

    def reiniciar():
        parar_robo()
        if m2m2_dono_do_mapa():
            reiniciar_m2m2()
            return
        modo, err = modo_slam()
        if modo is None:
            log('REINICIAR falhou: nao li o modo do slam_toolbox (%s). Nada mudou.' % err)
            return
        if modo == 'localization':
            base = os.path.expanduser('~/mapas/%s' % a.arena_mapa)
            try:
                st = yaml.safe_load(open(base + '_pontos.yaml'))['pontos']['START']
            except Exception as e:  # noqa: BLE001
                log('REINICIAR falhou: START de %s_pontos.yaml (%s). Nada mudou.' % (base, e))
                return
            req = DeserializePoseGraph.Request()
            req.filename, req.match_type = base, DeserializePoseGraph.Request.LOCALIZE_AT_POSE
            req.initial_pose.x, req.initial_pose.y, req.initial_pose.theta = (
                float(st['x']), float(st['y']), float(st['yaw']))
            log('REINICIAR (localization): recarregando %s com o robo no START (%.3f, %.3f, %.1f graus)...'
                % (a.arena_mapa, st['x'], st['y'], math.degrees(st['yaw'])))
            r, err = chamar(cli_deser, req, 30.0)
            log('REINICIAR: arena recarregada no START.' if r is not None else 'REINICIAR falhou: %s' % err)
            return
        log('REINICIAR (mapping): zerando odometria e resetando o SLAM (mapa novo a partir daqui)...')
        r, err = chamar(cli_odom, Trigger.Request(), 3.0)
        if r is None or not r.success:
            log('REINICIAR falhou ao zerar a odometria (%s). SLAM NAO resetado.' % (err or r.message))
            return
        time.sleep(0.3)
        r, err = chamar(cli_reset, Reset.Request(), 10.0)
        if r is None:
            log('REINICIAR: odometria zerada, mas o reset do SLAM falhou (%s)!' % err)
            return
        lembrar('reiniciar')
        areas.clear()
        pontos.clear()
        pontos['START'] = pose_dict(0.0, 0.0, 0.0)
        log('REINICIADO: mapa novo com origem aqui; START = (0, 0, 0); mesas e FINISH apagados.')
        # REINICIAR_SLAM_M2M2 v2 (25/11): limpa os costmaps (obstaculos do mapa velho)
        from nav2_msgs.srv import ClearEntireCostmap
        for c in ('global', 'local'):
            cli_c = node.create_client(ClearEntireCostmap, '/%s_costmap/clear_entirely_%s_costmap' % (c, c))
            rc, errc = chamar(cli_c, ClearEntireCostmap.Request(), 5.0)
            log('REINICIAR: costmap %s %s' % (c, 'limpo' if rc is not None else 'NAO limpo (%s)' % errc))
        gravar('reiniciar')

    def listar():
        linhas = ['--- marcados ---']
        for k in ('START', 'FINISH', 'START_2', 'FINISH_2'):   # TELEOP_CAIXA2 (24/11)
            if k in pontos:
                p = pontos[k]
                linhas.append('  %-7s x=%7.3f y=%7.3f yaw=%6.1f°' % (k, p['x'], p['y'], math.degrees(p['yaw'])))
        for k in sorted(areas):
            p = areas[k].get('face') or areas[k].get('aproximacao') or {}
            linhas.append('  %-7s %-11s %4.0f cm  face x=%7.3f y=%7.3f yaw=%6.1f°%s' % (
                k, areas[k].get('type', '?'), 100 * (areas[k].get('table') or {}).get('height', 0),
                p.get('x', 0), p.get('y', 0), math.degrees(p.get('yaw', 0)),
                ' (estimada)' if p.get('origem') == 'teleop' else ''))
        log('\n'.join(linhas))

    def publicar(vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = float(vx), float(vy), float(wz)
        pub.publish(msg)

    def eixo(i):
        val = js.eixos[i] if i < len(js.eixos) else 0.0
        if abs(val) < a.zona_morta:
            return 0.0
        return math.copysign((abs(val) - a.zona_morta) / (1 - a.zona_morta), val)

    def rampa(atual, alvo, passo):
        return atual + max(-passo, min(passo, alvo - atual))

    print('TELEOP MAPEAMENTO — controle "%s" em %s, perfil %s' % (js.nome, dev, perfil_nome))
    print('  segure %s e use os analogicos (soltar = para) | %s rapido' % (N['deadman'], N['rapido']))
    print('  %s marca WS (%.0f cm) | %s marca SH (%.0f cm) | %s marca PP (%.0f cm)  <- robo DE FRENTE para a mesa' % (
        N['WS'], a.alturas['WS'] * 100, N['SH'], a.alturas['SH'] * 100, N['PP'], a.alturas['PP'] * 100))
    print('  %s 2x REINICIA posicao e arena | D-pad cima 2x START, baixo 2x FINISH | %s 2x desfaz | %s lista' % (
        N['reiniciar'], N['desfazer'], N['listar']))
    print('  Face pelo LiDAR: %s. Cada marcacao ja grava os arquivos.' % ('SIM' if medidor else 'NAO (so pose)'))
    if a.sem_cm:  # SEM_CM
        print('  !!! SEM COLLISION MONITOR: publicando direto em %s. Nada para o robo antes de bater. !!!' % a.topic)
        print('  !!! Nao use com goal Nav2 ativo. Soltar o %s para na hora.                       !!!' % N['deadman'])
    else:
        print('  Publicando %s (TwistStamped). Collision Monitor continua ativo. Sem goal Nav2 ativo!' % a.topic)
    print('  Arquivos: %s | %s' % (a.pontos, a.service_areas))
    status('pronto')

    try:
        proximo = time.monotonic()
        while rclpy.ok():
            select.select([js.fd], [], [], max(0.0, proximo - time.monotonic()))
            for ev in js.ler():
                if ev[0] == 'b' and ev[2] == 1:
                    n = ev[1]
                    for tipo in ('WS', 'SH', 'PP'):
                        if n == B[tipo]:
                            marcar_mesa(tipo)
                    if n == B['reiniciar']:
                        if confirmado('reiniciar', 'REINICIAR posicao e arena'):
                            reiniciar()
                    elif n == B['desfazer']:
                        if confirmado('desfazer', 'desfazer a ultima marcacao'):
                            desfazer()
                    elif n == B['listar']:
                        listar()
                elif ev[0] == 'a' and ev[1] == E['hat_y']:
                    novo, antigo = round(ev[2]), round(ev[3])
                    if novo == 0 or novo == antigo:
                        continue
                    nome = 'START' if novo < 0 else 'FINISH'   # cima = -1
                    if confirmado(nome, 'marcar o %s aqui' % nome):
                        marcar_ponto(nome)
                elif ev[0] == 'a' and ev[1] == E['hat_x']:   # TELEOP_CAIXA2 (24/11): caixa 2
                    novo, antigo = round(ev[2]), round(ev[3])
                    if novo == 0 or novo == antigo:
                        continue
                    nome = 'START_2' if novo < 0 else 'FINISH_2'   # esquerda = -1
                    if confirmado(nome, 'marcar o %s (caixa 2) aqui' % nome):
                        marcar_ponto(nome)
            agora = time.monotonic()
            if agora < proximo:
                continue
            proximo = agora + dt

            ligado = os.path.exists(dev)
            deadman = ligado and B['deadman'] < len(js.botoes) and bool(js.botoes[B['deadman']])
            if not ligado and any(v):
                log('CONTROLE DESCONECTADO: parando.')
            if deadman:
                f = a.fator_rapido if js.botoes[B['rapido']] else 1.0
                alvo = [-eixo(E['ly']) * a.linear * f, -eixo(E['lx']) * a.lateral * f,
                        -eixo(E['rx']) * a.angular * f]
                passo = a.rampa * dt
                v = [rampa(v[0], alvo[0], passo), rampa(v[1], alvo[1], passo), rampa(v[2], alvo[2], 3 * passo)]
            else:
                v = [0.0, 0.0, 0.0]  # soltar o deadman para na hora, sem rampa
            # SO_COM_LB: publica so enquanto comanda; ao soltar/parar, alguns zeros e depois silencio
            if deadman:
                publicar(*v)
                zeros_restantes = ZEROS_AO_SOLTAR
            elif zeros_restantes > 0:
                publicar(0.0, 0.0, 0.0)
                zeros_restantes -= 1
            status()
    except KeyboardInterrupt:
        pass
    except OSError as e:
        print('\nErro lendo o controle: %s' % e)
    finally:
        for _ in range(5):
            publicar(0.0, 0.0, 0.0)
            time.sleep(0.04)
        print('\nComando zero publicado. Marcacoes ja estavam gravadas.')
        js.fechar()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
