#!/usr/bin/env python3
"""Executor de prova do RoboCup@Work Entry-Level (Brasil) - sem braco por enquanto.

Le o arquivo de tarefa da equipe (formato do atwork_commander: active_service_areas,
objects, start_state, finish_state), monta o plano de transporte (no maximo 3 objetos
no robo, regra 5.1) e executa pelo alinhador (/ir_para_area -> /ir_para_area/resultado).
Pegar e soltar ainda sao "ganchos": o robo para, espera um tempo (ou o seu Enter com
--confirmar) e segue. Termina no FINISH.

Uso:
    # 1) com o robo parado no START (=FINISH), grave a pose uma vez por sessao de mapa:
    python3 executor_prova.py --marcar-start
    # 2) confira o plano sem mexer no robo:
    python3 executor_prova.py tarefa.yaml --so-plano
    # 3) execute:
    python3 executor_prova.py tarefa.yaml
    python3 executor_prova.py tarefa.yaml --manipulacao 5 --confirmar
    # GARRA_COLETA (23/11): pegar/soltar pela garra (subir antes ~/robo_jazzy_ws/ferramentas/subida_unica/subir_garra.sh)
    python3 executor_prova.py tarefa.yaml --garra
    # MODO_PROVA (24/11): competicao: nunca para por falha, adia areas e guarda tempo para o FINISH
    python3 executor_prova.py tarefa.yaml --modo-prova --tempo-prova 20
"""
import argparse
import json  # GARRA_COLETA
import math
import os
import sys
import time

import yaml

CAPACIDADE = 3
PONTOS = os.path.expanduser('~/mapas/pontos.yaml')
AREAS = os.path.expanduser('~/mapas/service_areas.yaml')
# GARRA_COLETA (23/11): a garra segura 1 cubo; entrega com o centro do cubo a ENTREGA_X de robo_base
# (0,445 = 17 cm a frente do para-choque, ponto conferido pelo time da garra) e meia aresta acima do tampo.
CAPACIDADE_GARRA = 1
CAPACIDADE_SLOTS = 3  # TAREFA_JUIZES (24/11): 3 slots no robo com --garra
MEIO_CUBO = 0.021
ALTURA_MESA_PADRAO = 0.10


def altura_mesa(area):
    """GARRA_COLETA: table.height da mesa em service_areas.yaml (padrao 0,10 m)."""
    try:
        a = ((yaml.safe_load(open(AREAS)) or {}).get('service_areas') or {}).get(area) or {}
        return float((a.get('table') or {}).get('height', ALTURA_MESA_PADRAO))
    except (OSError, TypeError, ValueError):
        return ALTURA_MESA_PADRAO


def chave_area(nome):
    """TAREFA_JUIZES: WS_13 / ws13 / WS013 -> ('WS', 13); START -> ('START', None)."""
    import re
    s = str(nome).strip().upper()
    m = re.match(r'^([A-Z]+)_?0*(\d+)$', s)
    return (m.group(1), int(m.group(2))) if m else (s, None)


def resolver_areas(tarefa, conhecidas, traducao=None):
    """TAREFA_JUIZES: troca os nomes do arquivo pelos cadastrados. Devolve {arquivo: cadastro}."""
    trad = {chave_area(k): str(v).strip().upper() for k, v in (traducao or {}).items()}
    cad = {}
    for n in sorted(conhecidas):
        cad.setdefault(chave_area(n), n)
    usados = set(tarefa['origem'].values()) | set(tarefa['destino'].values()) | set(tarefa['areas'])
    alvo = {a: trad.get(chave_area(a), a) for a in usados}
    mapa = {a: cad.get(chave_area(alvo[a]), alvo[a]) for a in usados}
    tarefa['origem'] = {o: mapa[a] for o, a in tarefa['origem'].items()}
    tarefa['destino'] = {o: mapa[a] for o, a in tarefa['destino'].items()}
    tarefa['areas'] = [mapa[a] for a in tarefa['areas']]
    return mapa


def id_lido(txt):
    """TAREFA_JUIZES: 'ok 13' -> 13; 'ok' -> None."""
    p = str(txt).split()
    try:
        return int(p[1]) if len(p) > 1 and p[0] == 'ok' else None
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# Plano (sem ROS)
# ---------------------------------------------------------------------------

def ler_tarefa(caminho):
    doc = yaml.safe_load(open(caminho)) or {}
    ini = doc.get('start_state') or {}
    fim = doc.get('finish_state') or {}
    origem, destino = {}, {}
    for area, st in ini.items():
        for o in (st or {}).get('obj_ids') or []:
            origem[int(o)] = str(area).upper()
    for area, st in fim.items():
        for o in (st or {}).get('obj_ids') or []:
            destino[int(o)] = str(area).upper()
    objetos = [int(o['id']) if isinstance(o, dict) else int(o) for o in doc.get('objects') or []]
    return dict(id=doc.get('task_id', '?'), nome=doc.get('name', ''),
                minutos=float(doc.get('duration_min') or 0), areas=[str(a).upper() for a in
                                                                  doc.get('active_service_areas') or []],
                objetos=objetos, origem=origem, destino=destino)


def montar_plano(tarefa, posicoes, inicio='START', fim='FINISH', capacidade=CAPACIDADE):
    """Lista de passos: ('ir', area) / ('pegar', area, [ids]) / ('soltar', area, [ids]).

    PLANO_V2 (29/10): em cada mesa o robo primeiro solta o que e dali e depois pega o que
    couber (antes ele decidia o que pegar sem contar o que ia soltar ali, e a missao de
    WS_1/WS_2 fazia uma ida e volta a mais). Proxima parada: a mesa mais perto entre os
    destinos do que esta no robo e, se ainda cabe objeto, as origens pendentes.
    """
    pend = {o: (tarefa['origem'][o], tarefa['destino'][o]) for o in tarefa['objetos']
            if o in tarefa['origem'] and o in tarefa['destino']
            and tarefa['origem'][o] != tarefa['destino'][o]}

    def dist(a, b):
        pa, pb = posicoes.get(a), posicoes.get(b)
        if pa is None or pb is None:
            return 99.0
        return math.hypot(pa[0] - pb[0], pa[1] - pb[1])

    passos, aqui, carga = [], inicio, []
    while pend:
        # 1) soltar o que e desta mesa
        ids = sorted(o for o in carga if pend[o][1] == aqui)
        if ids:
            passos.append(('soltar', aqui, ids))
            for o in ids:
                carga.remove(o)
                del pend[o]
        # 2) pegar o que esta nesta mesa, ate encher
        livres = capacidade - len(carga)
        ids = sorted(o for o in pend if o not in carga and pend[o][0] == aqui)[:livres]
        if ids:
            passos.append(('pegar', aqui, ids))
            carga += ids
        if not pend:
            break
        # 3) proxima mesa
        alvos = {pend[o][1] for o in carga}
        if len(carga) < capacidade:
            alvos |= {pend[o][0] for o in pend if o not in carga}
        alvos.discard(aqui)
        if not alvos:
            break  # nada mais a fazer (tarefa inconsistente)
        aqui = sorted(alvos, key=lambda a: (dist(aqui, a), a))[0]
        passos.append(('ir', aqui))
    passos.append(('ir', fim))
    return passos


def descrever(passos):
    linhas = []
    for i, p in enumerate(passos, 1):
        if p[0] == 'ir':
            linhas.append('%2d. ir ate %s' % (i, p[1]))
        else:
            linhas.append('%2d. %s %s em %s' % (i, p[0], ', '.join(str(o) for o in p[2]), p[1]))
    return '\n'.join(linhas)


def posicoes_conhecidas():
    pos = {}
    try:
        for n, a in ((yaml.safe_load(open(AREAS)) or {}).get('service_areas') or {}).items():
            ap = a.get('aproximacao') or {}
            if 'x' in ap:
                pos[str(n).upper()] = (ap['x'], ap['y'])
    except OSError:
        pass
    try:
        for n, p in ((yaml.safe_load(open(PONTOS)) or {}).get('pontos') or {}).items():
            pos[str(n).upper()] = (p['x'], p['y'])
    except OSError:
        pass
    return pos


# ---------------------------------------------------------------------------
# ROS
# ---------------------------------------------------------------------------

def folga_no_mapa(info, dados, x, y, alcance=1.0):
    """Distancia (m) do ponto (x, y) ate a celula ocupada mais proxima no OccupancyGrid."""
    res, w, h = info.resolution, info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y
    ci, cj = int((x - ox) / res), int((y - oy) / res)
    r = int(alcance / res)
    melhor = alcance
    for j in range(max(0, cj - r), min(h, cj + r + 1)):
        for i in range(max(0, ci - r), min(w, ci + r + 1)):
            if dados[j * w + i] >= 65:
                d = math.hypot((i + 0.5) * res + ox - x, (j + 0.5) * res + oy - y)
                melhor = min(melhor, d)
    return melhor


GIRO_LIVRE = 0.50  # canto mais longe do robo esta a 0,48 m do centro


def marcar_start():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    no = Node('marcar_start')
    buf = Buffer()
    TransformListener(buf, no)
    mapa = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    no.create_subscription(OccupancyGrid, '/map', lambda m: mapa.append(m), qos)
    tr = None
    fim = time.monotonic() + 5.0
    while time.monotonic() < fim and tr is None:
        rclpy.spin_once(no, timeout_sec=0.1)
        try:
            tr = buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            pass
    if tr is None:
        print('Nao achei a TF map -> base_link (SLAM rodando?).')
        return 1
    q = tr.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    p = {'x': round(tr.transform.translation.x, 3), 'y': round(tr.transform.translation.y, 3),
         'yaw': round(yaw, 4)}
    fim = time.monotonic() + 3.0
    while time.monotonic() < fim and not mapa:
        rclpy.spin_once(no, timeout_sec=0.1)
    if mapa:
        folga = folga_no_mapa(mapa[-1].info, mapa[-1].data, p['x'], p['y'])
        print('Folga ate o obstaculo mais proximo no mapa: %.2f m' % folga)
        if folga < GIRO_LIVRE:
            print('ATENCAO: aqui o robo NAO tem espaco para girar (precisa de %.2f m). '
                  'Mova o robo para o meio da area START e rode de novo.' % GIRO_LIVRE)
            if '--forcar' not in sys.argv:
                print('Nada gravado. (Para gravar mesmo assim: --marcar-start --forcar)')
                no.destroy_node()
                rclpy.try_shutdown()
                return 1
    else:
        print('Sem /map: nao conferi a folga em volta do robo.')
    doc = {}
    try:
        doc = yaml.safe_load(open(PONTOS)) or {}
    except OSError:
        pass
    doc.setdefault('pontos', {})
    doc['pontos']['START'] = dict(p)
    doc['pontos']['FINISH'] = dict(p)
    os.makedirs(os.path.dirname(PONTOS), exist_ok=True)
    with open(PONTOS, 'w') as f:
        f.write('# START e FINISH (frame map desta sessao). Gerado por executor_prova.py\n')
        yaml.safe_dump(doc, f, sort_keys=False)
    print('START e FINISH gravados em %s: x=%.2f y=%.2f yaw=%.0f graus' % (
        PONTOS, p['x'], p['y'], math.degrees(p['yaw'])))
    no.destroy_node()
    rclpy.try_shutdown()
    return 0


def executar(tarefa, passos, args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import Point  # GARRA_COLETA
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # GARRA_COLETA
    from std_msgs.msg import Bool  # GARRA_COLETA

    class Exec(Node):
        def __init__(self):
            super().__init__('executor_prova')
            self.pub = self.create_publisher(String, '/ir_para_area', 10)
            self.create_subscription(String, '/ir_para_area/resultado', self.ao_res, 10)
            self.res = []
            # GARRA_COLETA (23/11): coleta (alcance_cubo.py) e entrega (pegar_cubo.py)
            self.garra = args.garra
            if self.garra:
                self.pub_coleta = self.create_publisher(String, '/coleta/pedido', 10)
                self.create_subscription(String, '/coleta/resultado',
                                         lambda m: self.res_coleta.append(m.data), 10)
                self.pub_entrega = self.create_publisher(Point, '/entregar_cubo', 10)
                self.create_subscription(String, '/entregar_cubo/resultado',
                                         lambda m: self.res_entrega.append(m.data), 10)
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(Bool, '/garra/cubo_preso',
                                         lambda m: setattr(self, 'cubo_preso', m.data), qos)
            self.res_coleta, self.res_entrega, self.cubo_preso = [], [], None

        def ao_res(self, m):
            self.res.append(m.data)

        def _pedir(self, pub, msg, respostas, timeout, quem):
            """GARRA_COLETA: publica e espera a 1a resposta. (True/False/None, texto)."""
            if not self.esperar(5.0, lambda: pub.get_subscription_count() > 0):
                return None, 'ninguem escutando %s (%s rodando?)' % (pub.topic_name, quem)
            del respostas[:]
            pub.publish(msg)
            if not self.esperar(timeout, lambda: bool(respostas)):
                return None, 'sem resposta em %.0f s' % timeout
            return respostas[0].split()[:1] == ['ok'], respostas[0]  # TAREFA_JUIZES: 'ok <id>'

        def coletar(self, area, objetos, timeout):
            return self._pedir(self.pub_coleta, String(data=json.dumps(
                {'area': area, 'objetos': [str(o) for o in objetos]})),
                self.res_coleta, timeout, 'alcance_cubo.py')

        def entregar(self, x, z, timeout):
            return self._pedir(self.pub_entrega, Point(x=float(x), y=0.0, z=float(z)),
                               self.res_entrega, timeout, 'pegar_cubo.py')

        def esperar(self, s, cond=lambda: False):
            fim = time.monotonic() + s
            while rclpy.ok() and time.monotonic() < fim:
                rclpy.spin_once(self, timeout_sec=0.1)
                if cond():
                    return True
            return cond()

        # SONDA_ENTREGA (09/11): get_subscription_count() > 0 nao garante que o
        # alinhador ja descobriu este publicador; a 1a mensagem se perdia (medido:
        # 5/10 perdidas sem espera, 3/10 com 1 s). Manda um nome invalido ate o
        # alinhador responder "FALHOU area desconhecida" (nao mexe no robo):
        # so entao o caminho esta ligado nos dois sentidos.
        def sondar(self, limite=20.0):
            sonda = '_SONDA_EXECUTOR'
            fim = time.monotonic() + limite
            while rclpy.ok() and time.monotonic() < fim:
                self.pub.publish(String(data=sonda))
                if self.esperar(0.5, lambda: any(r.split()[:1] == [sonda] for r in self.res)):
                    self.ligado = True
                    return True
            return False

        def ir(self, area, timeout):
            if not self.esperar(10.0, lambda: self.pub.get_subscription_count() > 0):
                return None, 'ninguem escutando /ir_para_area (alinhador v3.9 rodando?)'
            if not getattr(self, 'ligado', False) and not self.sondar():
                return None, 'alinhador nao respondeu a sonda em 20 s (SONDA_ENTREGA)'
            self.res.clear()
            m = String()
            m.data = area
            self.pub.publish(m)

            def chegou():
                return any(r.split() and r.split()[0].upper() == area for r in self.res)
            if not self.esperar(timeout, chegou):
                return None, 'sem resposta em %.0f s' % timeout
            r = [r for r in self.res if r.split() and r.split()[0].upper() == area][-1].split(None, 2)
            return (len(r) > 1 and r[1] == 'SUCESSO'), (r[2] if len(r) > 2 else '')

    rclpy.init()
    no = Exec()
    log = no.get_logger()
    t0 = time.monotonic()
    feitos, codigo = [], 0
    na_garra = []  # GARRA_COLETA: objetos que a garra esta segurando
    slots = [None] * (args.capacidade or CAPACIDADE_SLOTS)  # TAREFA_JUIZES: id em cada slot
    piloto = None
    if args.modo_prova:  # MODO_PROVA (24/11)
        import piloto_prova
        limite = 60.0 * (args.tempo_prova or tarefa['minutos'] or 20.0)
        piloto = piloto_prova.Piloto(no, limite, posicoes_conhecidas(), log)
    fila, i = list(passos), 0  # MODO_PROVA: fila (areas adiadas voltam para o fim)
    try:
        while fila:
            p = fila.pop(0)
            i += 1
            if p[0] == 'ir' and piloto is not None:  # MODO_PROVA
                fim = p[1] == 'FINISH' or not any(q[0] == 'ir' for q in fila)
                if not fim and not piloto.cabe_visita(p[1]):
                    log.warning('MODO_PROVA: sem tempo para %s e voltar: vou ao FINISH' % p[1])
                    fila = [q for q in fila if q[0] == 'ir' and q[1] == 'FINISH'] or [('ir', 'FINISH')]
                    continue
                log.info('[%d] ir ate %s (modo prova%s)' % (i, p[1], ', FINISH' if fim else ''))
                if not getattr(no, 'ligado', False) and not no.sondar():
                    log.warning('MODO_PROVA: alinhador nao respondeu a sonda; mando assim mesmo')
                ok, txt = piloto.ir(p[1], no.pub, no.res, finish=fim)
                feitos.append((p, ok, txt))
                if ok:
                    log.info('%s: SUCESSO %s' % (p[1], txt))
                elif ok is None:
                    log.error('MODO_PROVA: %s' % txt)
                    break
                elif not fim:
                    fila = piloto_prova.adiar(fila, p)
                    log.warning('MODO_PROVA: %s ADIADA (%s); fila: %s' % (
                        p[1], txt, ' '.join(q[1] for q in fila if q[0] == 'ir')))
                continue
            if p[0] == 'ir':
                ok = None
                for tent in range(1, args.tentativas + 1):
                    log.info('[%d/%d] ir ate %s (tentativa %d)' % (i, len(passos), p[1], tent))
                    ok, txt = no.ir(p[1], args.timeout)
                    if ok:
                        break
                    log.warning('%s: %s %s' % (p[1], 'FALHOU' if ok is False else 'SEM RESPOSTA', txt))
                    if ok is None:
                        break
                feitos.append((p, ok, txt))
                if not ok:
                    log.error('Prova interrompida: nao cheguei em %s.' % p[1])
                    codigo = 1
                    break
                log.info('%s: SUCESSO %s' % (p[1], txt))
            elif args.garra:  # GARRA_COLETA (23/11) + TAREFA_JUIZES (24/11): 1 cubo por chamada, slots
                z = altura_mesa(p[1]) + MEIO_CUBO
                if p[0] == 'pegar':
                    faltam = list(p[2])
                    for _ in range(len(p[2]) + 1):  # +1: uma devolucao de cubo errado
                        if not faltam:
                            break
                        if None not in slots:
                            log.warning('slots cheios %s: nao pego %s' % (slots, faltam))
                            break
                        s = slots.index(None)
                        log.info('[%d/%d] PEGAR um de %s em %s -> slot %d' % (i, len(passos), faltam, p[1], s + 1))
                        ok, txt = no.coletar(p[1], faltam, args.tempo_coleta)
                        oid = id_lido(txt) if ok else None
                        if ok and oid is not None and oid not in faltam:
                            log.warning('pegou o %d, que nao e para levar: devolvendo em %s' % (oid, p[1]))
                            ok2, txt2 = no.entregar(args.entrega_x, z, args.tempo_entrega)
                            feitos.append((('soltar', p[1], [oid]), ok2, 'devolucao ' + str(txt2)))
                            continue
                        if ok or no.cubo_preso:
                            oid = oid if oid is not None else faltam[0]
                            slots[s] = oid
                            faltam.remove(oid)
                        else:
                            log.warning('PEGAR em %s: %s %s (a prova segue)' % (
                                p[1], 'FALHOU' if ok is False else 'SEM RESPOSTA', txt))
                        feitos.append((('pegar', p[1], [oid] if oid is not None else faltam[:1]), ok, txt))
                        if not ok:
                            break
                    log.info('slots: %s' % slots)
                else:
                    for o in p[2]:
                        if o not in slots:
                            feitos.append((('soltar', p[1], [o]), False, 'nao esta nos slots %s' % slots))
                            continue
                        s = slots.index(o)
                        log.info('[%d/%d] SOLTAR %d (slot %d) em %s (x=%.3f z=%.3f)' % (
                            i, len(passos), o, s + 1, p[1], args.entrega_x, z))
                        ok, txt = no.entregar(args.entrega_x, z, args.tempo_entrega)
                        if ok or no.cubo_preso is False:
                            slots[s] = None
                        else:
                            log.warning('SOLTAR %d em %s: %s (a prova segue)' % (o, p[1], txt))
                        feitos.append((('soltar', p[1], [o]), ok, txt))
                    log.info('slots: %s' % slots)
            elif False:  # TAREFA_JUIZES: fluxo GARRA_COLETA de 1 cubo (mantido para o --desfazer)
                objs = ', '.join(str(o) for o in p[2])
                if p[0] == 'pegar' and na_garra:
                    ok, txt = False, 'garra ocupada com %s: nao pega (derrubaria o cubo aqui)' % na_garra
                elif p[0] == 'soltar' and not na_garra:
                    ok, txt = False, 'nada na garra para soltar'
                elif p[0] == 'pegar':
                    log.info('[%d/%d] PEGAR %s em %s pela garra' % (i, len(passos), objs, p[1]))
                    ok, txt = no.coletar(p[1], p[2], args.tempo_coleta)
                    if ok or no.cubo_preso:
                        na_garra = list(p[2])
                else:
                    z = altura_mesa(p[1]) + MEIO_CUBO
                    log.info('[%d/%d] SOLTAR %s em %s pela garra (x=%.3f z=%.3f)' % (
                        i, len(passos), objs, p[1], args.entrega_x, z))
                    ok, txt = no.entregar(args.entrega_x, z, args.tempo_entrega)
                    if ok or no.cubo_preso is False:
                        na_garra = []
                if not ok:
                    log.warning('%s %s em %s: %s %s (a prova segue)' % (
                        p[0].upper(), objs, p[1], 'FALHOU' if ok is False else 'SEM RESPOSTA', txt))
                feitos.append((p, ok, txt))
            else:
                acao = 'PEGAR' if p[0] == 'pegar' else 'SOLTAR'
                log.info('[%d/%d] %s objeto(s) %s em %s (simulado: sem braco)' % (
                    i, len(passos), acao, ', '.join(str(o) for o in p[2]), p[1]))
                if args.confirmar:
                    input('   ... faca a %s na mao e aperte Enter ' % acao.lower())
                else:
                    no.esperar(args.manipulacao)
                feitos.append((p, True, 'simulado'))
            if tarefa['minutos'] and time.monotonic() - t0 > tarefa['minutos'] * 60:
                log.warning('Tempo da prova (%.0f min) estourado.' % tarefa['minutos'])
    except KeyboardInterrupt:
        log.warning('Interrompido pelo usuario (o alinhador termina o que estava fazendo).')
        codigo = 2
    finally:
        dur = time.monotonic() - t0
        print('\n===== %s %s =====' % (tarefa['id'], tarefa['nome']))
        for p, ok, txt in feitos:
            nome = ('ir ate %s' % p[1]) if p[0] == 'ir' else '%s %s em %s' % (
                p[0], ','.join(str(o) for o in p[2]), p[1])
            print('%-28s %-12s %s' % (nome, {True: 'OK', False: 'FALHOU', None: 'SEM RESPOSTA'}[ok], txt))
        print('tempo total: %d min %02d s%s' % (dur // 60, dur % 60,
                                               ('  (limite %.0f min)' % tarefa['minutos']) if tarefa['minutos'] else ''))
        no.destroy_node()
        rclpy.try_shutdown()
    return codigo


def main():
    from_ros = sys.argv
    try:
        from rclpy.utilities import remove_ros_args
        from_ros = remove_ros_args(sys.argv)
    except ImportError:
        pass
    ap = argparse.ArgumentParser(description='Executor de prova RoboCup@Work (sem braco).')
    ap.add_argument('tarefa', nargs='?', help='arquivo de tarefa da equipe (YAML do commander)')
    ap.add_argument('--marcar-start', action='store_true', help='grava a pose atual como START e FINISH')
    ap.add_argument('--forcar', action='store_true', help='com --marcar-start: grava mesmo sem espaco para girar')
    ap.add_argument('--so-plano', action='store_true', help='so mostra o plano, nao mexe no robo')
    ap.add_argument('--manipulacao', type=float, default=5.0, help='segundos parado ao pegar/soltar')
    ap.add_argument('--confirmar', action='store_true', help='espera Enter em cada pegar/soltar')
    # GARRA_COLETA (23/11)
    ap.add_argument('--garra', action='store_true',
                    help='pegar/soltar pela garra (alcance_cubo.py + pegar_cubo.py); capacidade 1')
    ap.add_argument('--tempo-coleta', type=float, default=120.0, help='s maximos por PEGAR com --garra')
    ap.add_argument('--tempo-entrega', type=float, default=90.0, help='s maximos por SOLTAR com --garra')
    ap.add_argument('--entrega-x', type=float, default=0.445,
                    help='x (m, robo_base) do centro do cubo ao soltar na mesa (padrao 0.445)')
    ap.add_argument('--tentativas', type=int, default=2)
    # TAREFA_JUIZES (24/11)
    ap.add_argument('--capacidade', type=int, default=None,
                    help='cubos no robo (padrao: 4 slots com --garra, 3 sem)')
    ap.add_argument('--traduzir', action='append', default=[], metavar='ARQ=MAPA',
                    help='nome da mesa no arquivo = nome cadastrado (ex.: WS_13=WS03)')
    ap.add_argument('--timeout', type=float, default=240.0, help='segundos maximos por deslocamento')
    # MODO_PROVA (24/11)
    ap.add_argument('--modo-prova', action='store_true',
                    help='competicao: escada de recuperacao, adia areas, relogio e FINISH garantido')
    ap.add_argument('--tempo-prova', type=float, default=None,
                    help='minutos da prova com --modo-prova (padrao: duration_min da tarefa ou 20)')
    args = ap.parse_args(from_ros[1:])
    if args.marcar_start:
        return marcar_start()
    if not args.tarefa:
        ap.error('informe o arquivo de tarefa (ou use --marcar-start)')
    tarefa = ler_tarefa(args.tarefa)
    pos = posicoes_conhecidas()
    trad = dict(t.split('=', 1) for t in args.traduzir)  # TAREFA_JUIZES
    for a, b in sorted(resolver_areas(tarefa, pos, trad).items()):
        print('mesa %-8s -> %s%s' % (a, b, '' if b in pos else '  (SEM POSICAO)'))
    cap = args.capacidade or (CAPACIDADE_SLOTS if args.garra else CAPACIDADE)
    args.capacidade = cap
    passos = montar_plano(tarefa, pos, capacidade=cap)  # GARRA_COLETA + TAREFA_JUIZES
    print('Tarefa %s (%s), %d objeto(s), limite %.0f min' % (
        tarefa['id'], tarefa['nome'], len(tarefa['objetos']), tarefa['minutos']))
    print(descrever(passos))
    faltam = sorted({p[1] for p in passos if p[0] == 'ir'} - set(pos))
    if faltam:
        print('\nATENCAO: sem posicao conhecida para: %s' % ', '.join(faltam))
        print('  - mesas: cadastre no service_areas (C? -> WS_?)')
        print('  - START/FINISH: rode com --marcar-start com o robo no START')
        if not args.so_plano:
            return 1
    if args.so_plano:
        return 0
    return executar(tarefa, passos, args)


if __name__ == '__main__':
    sys.exit(main())
