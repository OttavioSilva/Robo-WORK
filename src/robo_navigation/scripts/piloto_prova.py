#!/usr/bin/env python3
"""MODO_PROVA (24/11): piloto do executor_prova na competicao. Nunca encerra por 'goal failed':
tenta tudo o que tem ate o tempo da prova acabar, sempre guardando tempo para chegar no FINISH.

Ordem das perguntas a cada 0,1 s durante um deslocamento:
  1. sei onde estou? (/localizacao/estado do vigia_localizacao) -> perdido: cancela e espera o vigia
  2. estou progredindo? (deslocamento >= 0,20 m em 20 s, fora dos 0,5 m finais) -> nao: sobe a escada
  3. ainda cabe no tempo? (tempo restante x reserva para voltar ao FINISH) -> nao: vai ao FINISH
Escada por area (cada degrau uma vez; o alinhador/destravar continuam fazendo a parte deles):
  0 normal -> 1 reenvia -> 2 marca o bloqueio (BLOQUEIO_ROTA) + limpa costmaps + reenvia (o planner
  procura outra rota pela arena inteira) -> 3 recua ate o ponto seguro mais proximo + reenvia ->
  area adiada (o executor faz as outras e volta depois). FINISH: a escada recomeca ate o tempo acabar.
Pontos seguros: a cada 0,5 m, pose com localizacao ok e circulo de giro (0,50 m) livre no mapa.
Seguranca (CM, distancias, fita) nunca e relaxada aqui: so a estrategia muda.
"""
import json
import math
import time

VEL_MEDIA = 0.15          # m/s efetivos medidos na simulacao (24/11), com giros e encostos
FATOR_CAMINHO = 1.4       # caminho real / linha reta
TEMPO_ENCOSTO = 25.0
FOLGA_FINISH = 60.0      # volta ao FINISH na simulacao 24/11: 32-198 s (entrar na caixa do START)
JANELA_PROGRESSO = 20.0   # giro de orientacao do destravar leva ~16 s parado no lugar
MIN_PROGRESSO = 0.20
RAIO_FINAL = 0.50
RAIO_GIRO = 0.50
PASSO_SEGURO = 0.50


# ---------------------------------------------------------------- logica pura (testavel)
class Progresso:
    def __init__(self, janela=JANELA_PROGRESSO, minimo=MIN_PROGRESSO):
        self.janela, self.minimo = janela, minimo
        self.ref = None
        self.t_ref = None

    def reiniciar(self, t, pose):
        self.ref, self.t_ref = pose[:2], t

    def passo(self, t, pose):
        """True = sem progresso ha 'janela' s."""
        if self.ref is None:
            self.reiniciar(t, pose)
            return False
        if math.hypot(pose[0] - self.ref[0], pose[1] - self.ref[1]) >= self.minimo:
            self.reiniciar(t, pose)
            return False
        return t - self.t_ref >= self.janela


class Relogio:
    def __init__(self, limite_s, t0):
        self.limite, self.t0 = limite_s, t0

    def restante(self, t):
        return self.limite - (t - self.t0)

    @staticmethod
    def tempo_trecho(dist, encosto=False):
        return dist * FATOR_CAMINHO / VEL_MEDIA + (TEMPO_ENCOSTO if encosto else 0.0)

    def reserva_finish(self, dist_finish):
        return 1.5 * self.tempo_trecho(dist_finish) + FOLGA_FINISH

    def cabe_visita(self, t, d_ate_area, d_area_finish):
        return self.restante(t) >= self.tempo_trecho(d_ate_area, True) + self.reserva_finish(d_area_finish)

    def hora_do_finish(self, t, d_finish):
        return self.restante(t) <= self.reserva_finish(d_finish)


def orcamento_area(dist):
    return 90.0 + 2.0 * Relogio.tempo_trecho(dist, True)


class PontosSeguros:
    def __init__(self, passo=PASSO_SEGURO):
        self.passo = passo
        self.pontos = []

    def oferecer(self, pose):
        if not self.pontos or math.hypot(pose[0] - self.pontos[-1][0], pose[1] - self.pontos[-1][1]) >= self.passo:
            self.pontos.append(tuple(pose))
            return True
        return False

    def escolher(self, aqui, evitar, min_dist=1.0):
        """Ponto seguro mais proximo a >= min_dist daqui e do lugar da falha."""
        cand = [p for p in self.pontos
                if math.hypot(p[0] - aqui[0], p[1] - aqui[1]) >= min_dist
                and math.hypot(p[0] - evitar[0], p[1] - evitar[1]) >= min_dist]
        return min(cand, key=lambda p: math.hypot(p[0] - aqui[0], p[1] - aqui[1])) if cand else None


def adiar(fila, p):
    """Tira de 'fila' os passos da visita (pegar/soltar ate o proximo 'ir') e recoloca a visita
    antes do ultimo 'ir' (FINISH). Devolve a fila nova."""
    bloco = []
    while fila and fila[0][0] != 'ir':
        bloco.append(fila.pop(0))
    fim = len(fila) - 1 if fila and fila[-1][0] == 'ir' else len(fila)
    return fila[:fim] + [p] + bloco + fila[fim:]


# ---------------------------------------------------------------- ROS
class Piloto:
    def __init__(self, no, limite_s, posicoes, log):
        import rclpy
        from nav2_msgs.action import NavigateToPose
        from nav2_msgs.srv import ClearEntireCostmap
        from nav_msgs.msg import OccupancyGrid
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformListener
        self.rclpy, self.no, self.log = rclpy, no, log
        self.String, self.Trigger, self.Clear, self.NTP = String, Trigger, ClearEntireCostmap, NavigateToPose
        self.posicoes = posicoes
        self.relogio = Relogio(limite_s, time.monotonic())
        self.seguros = PontosSeguros()
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, no)
        self.loc = 'ok'
        self.livre_giro = None
        self.info = None
        no.create_subscription(String, '/localizacao/estado', self.ao_loc, 10)
        qm = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        no.create_subscription(OccupancyGrid, '/map', self.ao_mapa, qm)
        self.cli_cancelar = no.create_client(Trigger, '/alinhador/cancelar_t')
        self.cli_bloqueio = no.create_client(Trigger, '/bloqueios/marcar')
        self.cli_limpar = [no.create_client(ClearEntireCostmap, n) for n in (
            '/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap')]
        self.nav = ActionClient(no, NavigateToPose, 'navigate_to_pose')
        log.info('MODO_PROVA: piloto pronto, %.0f s de prova' % limite_s)

    # ---- entradas
    def ao_loc(self, m):
        try:
            self.loc = json.loads(m.data).get('estado', 'ok')
        except ValueError:
            pass

    def ao_mapa(self, m):
        import numpy as np
        g = np.array(m.data, np.int16).reshape(m.info.height, m.info.width) >= 65
        n = int(math.ceil(RAIO_GIRO / m.info.resolution))
        ocupado = np.zeros_like(g)
        for dl in range(-n, n + 1):
            for dc in range(-n, n + 1):
                if dl * dl + dc * dc <= n * n:
                    ocupado |= np.roll(np.roll(g, dl, 0), dc, 1)
        self.livre_giro, self.info = ~ocupado, m.info

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', self.rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def giro_cabe(self, p):
        if self.livre_giro is None:
            return False
        r = self.info.resolution
        c = int((p[0] - self.info.origin.position.x) / r)
        l = int((p[1] - self.info.origin.position.y) / r)
        return 0 <= l < self.info.height and 0 <= c < self.info.width and bool(self.livre_giro[l, c])

    def dist(self, a, b):
        pa = a if isinstance(a, tuple) else self.posicoes.get(a)
        pb = b if isinstance(b, tuple) else self.posicoes.get(b)
        if pa is None or pb is None:
            return 5.0
        return math.hypot(pa[0] - pb[0], pa[1] - pb[1])

    def girar(self, s=0.1):
        self.rclpy.spin_once(self.no, timeout_sec=s)
        p = self.pose()
        if p is not None and self.loc == 'ok' and self.giro_cabe(p) and self.seguros.oferecer(p):
            self.log.debug('MODO_PROVA: ponto seguro (%.2f, %.2f)' % p[:2])
        return p

    def chamar(self, cli, req, espera=3.0):
        if not cli.wait_for_service(timeout_sec=1.0):
            return None
        f = cli.call_async(req)
        fim = time.monotonic() + espera
        while not f.done() and time.monotonic() < fim:
            self.rclpy.spin_once(self.no, timeout_sec=0.05)
        return f.result() if f.done() else None

    # ---- decisoes
    def aqui(self):
        p = self.pose()
        return p[:2] if p else None

    def cabe_visita(self, area):
        aqui = self.aqui() or self.posicoes.get('START')
        return self.relogio.cabe_visita(time.monotonic(), self.dist(aqui, area), self.dist(area, 'FINISH'))

    def hora_do_finish(self):
        aqui = self.aqui()
        return aqui is not None and self.relogio.hora_do_finish(time.monotonic(), self.dist(aqui, 'FINISH'))

    def cancelar(self, area, resultados):
        self.chamar(self.cli_cancelar, self.Trigger.Request())
        fim = time.monotonic() + 4.0
        while time.monotonic() < fim and not any(r.split()[:1] == [area] for r in resultados):
            self.girar(0.1)

    def tentativa(self, area, pub, resultados, fim_area, finish):
        """'ok' | 'falhou' | 'sem_progresso' | 'perdido' | 'orcamento' | 'sem_tempo' | 'fim'; texto."""
        del resultados[:]
        pub.publish(self.String(data=area))
        prog = Progresso()
        alvo = self.posicoes.get(area)
        while self.rclpy.ok():
            p = self.girar(0.1)
            agora = time.monotonic()
            r = [x for x in resultados if x.split()[:1] == [area]]
            if r:
                partes = r[-1].split(None, 2)
                txt = partes[2] if len(partes) > 2 else ''
                return ('ok' if len(partes) > 1 and partes[1] == 'SUCESSO' else 'falhou'), txt
            if self.relogio.restante(agora) <= 0:
                self.cancelar(area, resultados)
                return 'fim', 'tempo da prova acabou'
            if self.loc == 'perdido':
                self.cancelar(area, resultados)
                return 'perdido', 'localizacao perdida'
            if not finish and agora > fim_area:
                self.cancelar(area, resultados)
                return 'orcamento', 'orcamento da area estourado'
            if not finish and self.hora_do_finish():
                self.cancelar(area, resultados)
                return 'sem_tempo', 'hora de voltar ao FINISH'
            if p is not None and (alvo is None or self.dist(p[:2], alvo) > RAIO_FINAL) and prog.passo(agora, p):
                self.cancelar(area, resultados)
                return 'sem_progresso', 'menos de %.0f cm em %.0f s' % (MIN_PROGRESSO * 100, JANELA_PROGRESSO)
        return 'fim', 'ROS encerrado'

    def esperar_localizacao(self, limite=40.0):
        fim = time.monotonic() + limite
        while self.loc == 'perdido' and time.monotonic() < fim and self.relogio.restante(time.monotonic()) > 0:
            self.girar(0.2)
        return self.loc != 'perdido'

    def marcar_bloqueio(self):
        r = self.chamar(self.cli_bloqueio, self.Trigger.Request())
        for c in self.cli_limpar:
            self.chamar(c, self.Clear.Request())
        return r.message if r else 'bloqueios_rota indisponivel'

    def recuar_ponto_seguro(self, lugar_falha, finish=False):
        aqui = self.aqui()
        if aqui is None:
            return 'sem pose'
        p = self.seguros.escolher(aqui, lugar_falha) or (self.posicoes.get('START') + (0.0,)
                                                          if self.posicoes.get('START') else None)
        if p is None:
            return 'nenhum ponto seguro'
        from geometry_msgs.msg import PoseStamped
        g = self.NTP.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = p[0], p[1]
        yaw = p[2] if len(p) > 2 else 0.0
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return 'navigate_to_pose indisponivel'
        f = self.nav.send_goal_async(g)
        fim = time.monotonic() + 60.0
        while not f.done() and time.monotonic() < fim:
            self.girar(0.1)
        h = f.result() if f.done() else None
        if h is None or not h.accepted:
            return 'goal do ponto seguro recusado'
        res = h.get_result_async()
        prog = Progresso()
        while not res.done() and time.monotonic() < fim:
            q = self.girar(0.1)
            if self.loc == 'perdido' or (q is not None and prog.passo(time.monotonic(), q)):
                break
            if self.relogio.restante(time.monotonic()) <= 0 or (not finish and self.hora_do_finish()):
                break
        if not res.done():
            h.cancel_goal_async()
            self.girar(0.5)
            return 'nao cheguei no ponto seguro (%.2f, %.2f)' % p[:2]
        return 'no ponto seguro (%.2f, %.2f)' % p[:2]

    def ir(self, area, pub, resultados, finish=False):
        """(True, txt) chegou; (False, motivo) area adiada/sem tempo; (None, motivo) prova acabou."""
        t_ini = time.monotonic()
        fim_area = t_ini + orcamento_area(self.dist(self.aqui() or self.posicoes.get('START'), area))
        nivel = 0
        while self.rclpy.ok():
            if not self.esperar_localizacao():
                self.log.warning('MODO_PROVA: %s: localizacao nao voltou em 40 s; sigo assim mesmo' % area)
            self.log.info('PILOTO: area=%s nivel=%d tempo_area=%.0fs resta=%.0fs' % (
                area, nivel, time.monotonic() - t_ini, self.relogio.restante(time.monotonic())))
            res, txt = self.tentativa(area, pub, resultados, fim_area, finish)
            if res == 'ok':
                return True, txt
            if res == 'fim':
                return None, txt
            if res in ('orcamento', 'sem_tempo'):
                return False, txt
            self.log.warning('PILOTO: area=%s nivel=%d %s: %s' % (area, nivel, res.upper(), txt))
            if res == 'perdido':
                continue
            lugar = self.aqui() or (0.0, 0.0)
            if not finish and self.hora_do_finish():
                return False, 'hora de voltar ao FINISH'
            nivel += 1
            if nivel == 2:
                self.log.warning('PILOTO: nivel 2: %s; replanejando' % self.marcar_bloqueio())
            elif nivel == 3:
                self.marcar_bloqueio()
                self.log.warning('PILOTO: nivel 3: %s' % self.recuar_ponto_seguro(lugar, finish))
            elif nivel >= 4:
                if not finish:
                    return False, 'escada esgotada (%s)' % txt
                nivel = 1
                self.log.warning('PILOTO: FINISH: recomeco a escada (bloqueios continuam marcados)')
        return None, 'ROS encerrado'
