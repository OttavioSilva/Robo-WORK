#!/usr/bin/env python3
"""Marca uma mesa (Service Area) pela pose do robo + LiDAR, sem depender da deteccao no mapa.

Por que: em 29/10 o service_areas.py nao achou as mesas no mapa (contornos irregulares).
Aqui o robo fica de frente para a mesa; o LiDAR mede a face (a mesma medida do encosto do
alinhador) e a pose da face vai para ~/mapas/service_areas.yaml, no formato que o
alinhador e o executor ja leem (face + aproximacao a 60 cm).

Como usar:
  1. Pare o service_areas.py (Ctrl+C), para ele nao regravar o arquivo.
  2. Num terminal, deixe rodando o modo de visualizacao (RViz: MarkerArray no topico
     /marcar_mesa/marcadores):
        python3 marcar_mesa.py --mostrar
       - AMARELO: a face que o LiDAR esta vendo agora na frente do robo (candidata), com a
         largura. Fica VERDE-CLARO quando da para marcar (as duas pontas aparecem e a
         largura esta entre 0,60 e 0,98 m).
       - VERDE com nome: mesas ja marcadas no arquivo; a seta e a pose de aproximacao.
         Atualiza sozinho quando o arquivo muda.
  3. Com o teleop, ponha o robo DE FRENTE para a mesa, a uns 50-90 cm da face, mais ou
     menos no meio dela, ate a candidata ficar verde-claro.
  4. Noutro terminal:  python3 marcar_mesa.py WS_1 10    (nome e altura em cm: 5, 10 ou 15)
  5. Repita para as outras mesas. Listar: python3 marcar_mesa.py --listar
     Apagar uma: python3 marcar_mesa.py --apagar WS_3
As outras mesas do arquivo sao mantidas; antes de gravar faz backup (.bak_marcar_<hora>).
"""
import math
import os
import shutil
import statistics
import sys
import time

ARQ = os.path.expanduser('~/mapas/service_areas.yaml')
DIST_APROX = 0.60
N_SCANS = 8
TOPICO_MARCADORES = '/marcar_mesa/marcadores'


# ---------------------------------------------------------------------------
# Arquivo e geometria (sem ROS)
# ---------------------------------------------------------------------------

def ler_doc():
    import yaml
    if not os.path.exists(ARQ):
        return {'frame': 'map', 'service_areas': {}}
    doc = yaml.safe_load(open(ARQ)) or {}
    doc.setdefault('frame', 'map')
    if not isinstance(doc.get('service_areas'), dict):
        doc['service_areas'] = {}
    return doc


def gravar_doc(doc):
    import yaml
    os.makedirs(os.path.dirname(ARQ), exist_ok=True)
    if os.path.exists(ARQ):
        bak = ARQ + '.bak_marcar_' + time.strftime('%Y%m%d_%H%M%S')
        shutil.copy2(ARQ, bak)
        print('Backup: %s' % bak)
    doc['gerado_em'] = time.strftime('%Y-%m-%d %H:%M:%S')
    tmp = ARQ + '.tmp'
    with open(tmp, 'w') as f:
        f.write('# Mesas marcadas com marcar_mesa.py (pose do robo + LiDAR). Frame map.\n')
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
    os.replace(tmp, ARQ)


def tipo_do_nome(nome):
    for pref, tipo in (('WS', 'workstation'), ('SH', 'shelf'), ('PP', 'precision_placement'),
                       ('ST', 'stacking')):
        if nome.startswith(pref):
            return tipo
    return 'workstation'


def face_no_robo(med):
    """medir_face -> (x, y, yaw) do centro da face em base_link (yaw = para onde o robo olha).
    Sem as duas pontas (centro None) devolve o ponto da face na frente do robo."""
    b = math.tan(-med['ang'])
    k = math.sqrt(1.0 + b * b)
    d0 = med['dist'] / k                       # distancia da origem ate a reta da face
    pe = (d0 / k, -d0 * b / k)                 # pe da perpendicular (normal (1,-b)/k)
    t = (b / k, 1.0 / k)                       # direcao ao longo da face
    c = med['centro'] if med['centro'] is not None else 0.0
    return pe[0] + c * t[0], pe[1] + c * t[1], med['ang']


def marcavel(med):
    return med is not None and med['centro'] is not None and 0.60 <= med['span'] <= 0.98


def listar():
    areas = ler_doc()['service_areas']
    if not areas:
        print('Nenhuma mesa em %s' % ARQ)
    for n, a in areas.items():
        f, ap = a.get('face', {}), a.get('aproximacao', {})
        print('%-6s face (%.2f, %.2f) %4.0f graus | aproximacao (%.2f, %.2f) | altura %.0f cm' % (
            n, f.get('x', 0), f.get('y', 0), math.degrees(f.get('yaw', 0)), ap.get('x', 0),
            ap.get('y', 0), 100 * (a.get('table') or {}).get('height', 0.10)))


# ---------------------------------------------------------------------------
# ROS
# ---------------------------------------------------------------------------

def carregar_alinhador():
    """medir_face do alinhador instalado (a mesma medida usada no encosto)."""
    import importlib.util
    caminho = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
    sp = importlib.util.spec_from_file_location('al', caminho)
    al = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(al)
    return al


def yaw_de(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class Medidor:
    """Le /scan_front e o TF; devolve a face vista na frente do robo, no frame map."""

    def __init__(self, no, al):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener
        self.al = al
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, no)
        self.scans = []
        self.laser_tf = None
        no.create_subscription(LaserScan, '/scan_front', self._ao_scan, qos_profile_sensor_data)

    def _ao_scan(self, msg):
        self.scans.append(msg)
        del self.scans[:-3]

    def medir(self):
        """(med, face_mapa) do scan mais novo; face_mapa = (x, y, yaw) ou None."""
        from rclpy.time import Time
        if not self.scans:
            return None, None
        msg = self.scans.pop()
        self.scans.clear()
        try:
            if self.laser_tf is None:
                tr = self.tfb.lookup_transform('base_link', msg.header.frame_id, Time())
                self.laser_tf = (tr.transform.translation.x, tr.transform.translation.y,
                                 yaw_de(tr.transform.rotation))
            tm = self.tfb.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None, None
        lx, ly, lth = self.laser_tf
        pts = []
        a = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                pts.append((lx + r * math.cos(a + lth), ly + r * math.sin(a + lth)))
            a += msg.angle_increment
        med = self.al.medir_face(pts)
        if med is None:
            return None, None
        fx, fy, fyaw = face_no_robo(med)
        rx, ry = tm.transform.translation.x, tm.transform.translation.y
        ryaw = yaw_de(tm.transform.rotation)
        c, s = math.cos(ryaw), math.sin(ryaw)
        return med, (rx + c * fx - s * fy, ry + s * fx + c * fy, ryaw + fyaw)


def marcar(nome, altura):
    import rclpy
    from rclpy.node import Node
    al = carregar_alinhador()
    rclpy.init()
    no = Node('marcar_mesa')
    medidor = Medidor(no, al)
    t0 = time.monotonic()
    medidas = []
    print('Medindo a face de %s (%d scans)...' % (nome, N_SCANS))
    while time.monotonic() - t0 < 15.0 and len(medidas) < N_SCANS:
        rclpy.spin_once(no, timeout_sec=0.1)
        med, face = medidor.medir()
        if marcavel(med):
            medidas.append(face + (med['span'],))
    no.destroy_node()
    rclpy.shutdown()

    if len(medidas) < 4:
        print('FALHOU: so %d medidas boas da face. Ponha o robo de frente para a mesa, a 50-90 cm,'
              ' no meio dela (a candidata do --mostrar fica verde-claro), e confira se o SLAM esta'
              ' no ar (TF map->base_link).' % len(medidas))
        return 1
    fx = statistics.median(m[0] for m in medidas)
    fy = statistics.median(m[1] for m in medidas)
    fyaw = math.atan2(statistics.median(math.sin(m[2]) for m in medidas),
                      statistics.median(math.cos(m[2]) for m in medidas))
    larg = statistics.median(m[3] for m in medidas)
    disp = max(math.hypot(m[0] - fx, m[1] - fy) for m in medidas)
    ax, ay = fx - DIST_APROX * math.cos(fyaw), fy - DIST_APROX * math.sin(fyaw)
    doc = ler_doc()
    if nome == 'WS':   # WS_AUTO (26/11): mesma mesa (face a ate 0,40 m) atualiza; senao o menor numero livre
        perto = [(math.hypot(a['face']['x'] - fx, a['face']['y'] - fy), k) for k, a in doc['service_areas'].items()
                 if k.startswith('WS_') and 'x' in (a.get('face') or {})]
        perto = [p for p in perto if p[0] <= 0.40]
        nome = min(perto)[1] if perto else next('WS_%d' % n for n in range(1, 100)
                                                 if 'WS_%d' % n not in doc['service_areas'])
    velho = doc['service_areas'].get(nome)
    doc['service_areas'][nome] = {
        'type': tipo_do_nome(nome),
        'table': {'width': 0.8, 'depth': 0.5, 'height': round(altura, 2)},
        'face': {'x': round(fx, 3), 'y': round(fy, 3), 'yaw': round(fyaw, 4), 'largura_medida': round(larg, 2)},
        'aproximacao': {'x': round(ax, 3), 'y': round(ay, 3), 'yaw': round(fyaw, 4)},
    }
    gravar_doc(doc)
    print('%s %s: face (%.2f, %.2f) %.0f graus, largura vista %.2f m, espalhamento %.1f cm (%d medidas)' % (
        nome, 'ATUALIZADA' if velho else 'MARCADA', fx, fy, math.degrees(fyaw), larg, 100 * disp, len(medidas)))
    print('Mesas no arquivo:')
    listar()
    return 0


def mostrar():
    """Publica no RViz as mesas do arquivo (verde) e a face vista agora (candidata)."""
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
    al = carregar_alinhador()
    rclpy.init()
    no = Node('marcar_mesa_mostrar')
    medidor = Medidor(no, al)
    pub = no.create_publisher(MarkerArray, TOPICO_MARCADORES, 10)

    def mk(ns, i, tipo, cor, pose=None, escala=(0.1, 0.1, 0.1), texto=''):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = no.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, i, tipo, Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = escala
        m.color.r, m.color.g, m.color.b, m.color.a = cor
        if pose is not None:
            x, y, z, yaw = pose
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        else:
            m.pose.orientation.w = 1.0
        m.text = texto
        return m

    def mesa(ns, i, fx, fy, fyaw, cor, texto, altura=0.10, largura=0.8):
        """Caixa da mesa (atras da face), texto e ponto da face."""
        cx, cy = fx + 0.25 * math.cos(fyaw), fy + 0.25 * math.sin(fyaw)
        h = max(altura, 0.03)
        return [mk(ns, 3 * i, Marker.CUBE, cor, (cx, cy, h / 2, fyaw), (0.5, largura, h)),
                mk(ns, 3 * i + 1, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), (cx, cy, h + 0.25, 0.0),
                   (0.0, 0.0, 0.16), texto),
                mk(ns, 3 * i + 2, Marker.SPHERE, (0.1, 0.1, 0.1, 1.0), (fx, fy, h, 0.0), (0.06, 0.06, 0.06))]

    estado = {'mtime': None, 'areas': {}}
    ultimo = 0.0
    print('Publicando em %s (frame map). Ctrl+C para sair.' % TOPICO_MARCADORES)
    try:
        while rclpy.ok():
            rclpy.spin_once(no, timeout_sec=0.05)
            agora = time.monotonic()
            if agora - ultimo < 0.3:
                continue
            ultimo = agora
            try:
                mt = os.path.getmtime(ARQ)
            except OSError:
                mt = None
            if mt != estado['mtime']:
                estado['mtime'] = mt
                try:
                    estado['areas'] = ler_doc()['service_areas'] if mt else {}
                    print('Arquivo: %d mesa(s): %s' % (len(estado['areas']), ', '.join(estado['areas']) or '-'))
                except Exception as exc:
                    print('Nao consegui ler %s: %s' % (ARQ, exc))
            arr = MarkerArray()
            limpar = Marker()
            limpar.action = Marker.DELETEALL
            arr.markers.append(limpar)
            for i, (n, a) in enumerate(sorted(estado['areas'].items())):
                f, ap = a.get('face') or {}, a.get('aproximacao') or {}
                if 'x' not in f:
                    continue
                alt = float((a.get('table') or {}).get('height', 0.10))
                arr.markers += mesa('mesas', i, f['x'], f['y'], f.get('yaw', 0.0), (0.1, 0.8, 0.2, 0.6),
                                    '%s (%.0f cm)' % (n, alt * 100), alt)
                if 'x' in ap:
                    arr.markers.append(mk('aproximacao', i, Marker.ARROW, (0.1, 0.8, 0.2, 1.0),
                                          (ap['x'], ap['y'], 0.05, ap.get('yaw', 0.0)), (0.35, 0.05, 0.05)))
            med, face = medidor.medir()
            if face is not None:
                ok = marcavel(med)
                cor = (0.6, 1.0, 0.4, 0.7) if ok else (1.0, 0.85, 0.0, 0.6)
                if med['centro'] is None:
                    txt = 'candidata: uma ponta cortada'
                else:
                    txt = 'candidata: %.2f m%s' % (med['span'], ' - pode marcar' if ok else '')
                arr.markers += mesa('candidata', 0, face[0], face[1], face[2], cor, txt, 0.10,
                                    max(0.2, min(med['span'], 1.2)))
            pub.publish(arr)
    except KeyboardInterrupt:
        pass
    no.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    if '--listar' in args:
        listar()
        return 0
    if '--mostrar' in args:
        return mostrar()
    if '--apagar' in args:
        nome = args[args.index('--apagar') + 1].upper()
        doc = ler_doc()
        if doc['service_areas'].pop(nome, None) is None:
            print('%s nao esta no arquivo.' % nome)
            return 1
        gravar_doc(doc)
        print('Apagada: %s' % nome)
        listar()
        return 0
    if not args:
        print(__doc__)
        return 1
    nome = args[0].upper()
    altura = float(args[1]) / 100.0 if len(args) > 1 else 0.10
    if altura not in (0.0, 0.05, 0.10, 0.15):
        print('Aviso: altura %.0f cm fora de 0/5/10/15' % (altura * 100))
    return marcar(nome, altura)


if __name__ == '__main__':
    sys.exit(main())
