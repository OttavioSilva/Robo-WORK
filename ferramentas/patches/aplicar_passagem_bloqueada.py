#!/usr/bin/env python3
"""PASSAGEM_BLOQUEADA (11/11/2026) - alinhador, caminho LEGADO.
Teste 18:44 (~/logs/11-11_184040): um corredor bloqueado por uma caixa e outro livre. O planner estava sem caminho, e o
ROTA_MAPA seguiu a rota mais curta do mapa estatico pelo corredor bloqueado. A travessia abortou 2x ("algo a 10 cm do
para-choque") e o robo parou. A rota pelo outro corredor nunca foi calculada: rota_no_mapa nao conhecia o obstaculo e
o ROTA_MAPA_OBSTACULO=recusar so vale quando o planner TEM caminho.
O que muda:
 1. A travessia pelo LiDAR aborta por objeto a frente ("algo a N cm do para-choque"). Os pontos do LiDAR a frente que
    estao LIVRES no mapa (a > 10 cm de parede) viram bloqueios (raio 0,35 m = meia largura do corpo + 2,5 cm) por
    passagem_bloqueada_s. Se o mapa tem outro caminho sem eles, o robo nao repete a travessia: reanalisa e vai pelo
    outro. Sem caminho alternativo: igual a antes (2a tentativa no mesmo corredor).
 2. Planner sem caminho + ROTA_MAPA_OBSTACULO (recusar) com objeto do momento na rota do mapa: tenta a rota do mapa
    sem esses objetos antes de seguir pela rota bloqueada.
 3. Com bloqueio ativo, o caminho do planner que passa pelo bloqueio e trocado pela rota do mapa sem ele. A vigia
    nao interrompe o Nav2 para uma passagem pelo bloqueio.
O objeto parcial (sobra espaco para o corpo) continua no mesmo corredor: o A* passa ao lado, a >= 0,35 m dele.
passagem_bloqueada_s 0 = como antes. Limite: o Nav2 nao recebe o bloqueio; ele so ve o objeto pelo costmap.
Uso: python3 aplicar_passagem_bloqueada.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    # ---- funcoes puras (testaveis sem ROS)
    ("""def rota_no_mapa(mapa, ini, fim, folga=0.30, folga_ponta=0.12, raio_ponta=0.60):
    \"\"\"ROTA_MAPA: caminho mais curto so pelo mapa, sem obstaculos do momento, onde o corpo cabe. Devolve
    [(x, y), ...] a cada celula, ou None. Celula solta (ruido do SLAM) nao conta como parede.\"\"\"
    w, h, res = mapa.w, mapa.h, mapa.res
""", """def rota_no_mapa(mapa, ini, fim, folga=0.30, folga_ponta=0.12, raio_ponta=0.60, bloqueios=()):
    \"\"\"ROTA_MAPA: caminho mais curto so pelo mapa, sem obstaculos do momento, onde o corpo cabe. Devolve
    [(x, y), ...] a cada celula, ou None. Celula solta (ruido do SLAM) nao conta como parede.
    PASSAGEM_BLOQUEADA (11/11): bloqueios = [(x, y, raio), ...]: o centro do robo nao passa a menos de raio deles.\"\"\"
    w, h, res = mapa.w, mapa.h, mapa.res
"""),
    ("""                k = nj * w + ni
                ponta = math.hypot(ni - i0, nj - j0) <= r_p or math.hypot(ni - i1, nj - j1) <= r_p
                if desc[k] or dist[k] < (lim_p if ponta else lim):
                    continue
""", """                k = nj * w + ni
                ponta = math.hypot(ni - i0, nj - j0) <= r_p or math.hypot(ni - i1, nj - j1) <= r_p
                if desc[k] or dist[k] < (lim_p if ponta else lim):
                    continue
                if bloqueios:   # PASSAGEM_BLOQUEADA
                    cx, cy = mundo(ni, nj)
                    if any(math.hypot(cx - bx, cy - by) < br for bx, by, br in bloqueios):
                        continue
"""),
    ("""def _comp_rota(p):
""", """def pontos_obstaculo_novo(mapa, pts_base, pose, alcance=0.60, meia_largura=0.45, dist_parede=0.10,
                          para_choque=0.375, max_pts=40):
    \"\"\"PASSAGEM_BLOQUEADA (11/11): pontos do LiDAR (base_link) a frente do para-choque, ate `alcance`, levados ao
    mapa com `pose` (x, y, yaw). Fica so o que esta LIVRE no mapa e a mais de `dist_parede` de parede (objeto do
    momento, nao a parede do corredor). Devolve [(x, y), ...] (no maximo max_pts, espalhados).\"\"\"
    if mapa is None or pose is None or not pts_base:
        return []
    c, s = math.cos(pose[2]), math.sin(pose[2])
    n = int(math.ceil(dist_parede / mapa.res))
    achados = []
    for x, y in pts_base:
        if not (para_choque < x <= para_choque + alcance and abs(y) <= meia_largura):
            continue
        mx, my = pose[0] + c * x - s * y, pose[1] + s * x + c * y
        if mapa.estado(mx, my) != 0:
            continue
        if any(mapa.estado(mx + a * mapa.res, my + b * mapa.res) == 1
               for a in range(-n, n + 1) for b in range(-n, n + 1) if math.hypot(a, b) <= n):
            continue
        achados.append((mx, my))
    if len(achados) > max_pts:
        passo = len(achados) / float(max_pts)
        achados = [achados[int(i * passo)] for i in range(max_pts)]
    return achados


def rota_cruza_bloqueio(pontos, bloqueios):
    \"\"\"PASSAGEM_BLOQUEADA: algum ponto do caminho passa a menos do raio de um bloqueio?\"\"\"
    return bool(pontos) and any(math.hypot(px - bx, py - by) < br
                                for px, py in pontos for bx, by, br in bloqueios)


def _comp_rota(p):
"""),
    # ---- parametros
    ("""            self.declare_parameter('rota_mapa_obstaculo', 'ignorar')
""", """            self.declare_parameter('rota_mapa_obstaculo', 'ignorar')
            # PASSAGEM_BLOQUEADA (11/11): objeto que barrou a travessia vale por N s (0 = como antes)
            self.declare_parameter('passagem_bloqueada_s', 120.0)
            self.declare_parameter('passagem_bloqueada_raio', 0.35)
            self.bloqueios = []          # [(x, y, raio, t)]
            self.trav_motivo, self.trav_obstaculo = '', []
"""),
    # ---- captura do obstaculo no momento do aborto da travessia (antes da re)
    ("""            ok = trav.estado == 'fim'
            if not ok and not self.p('nav2_principal') and not self.controle_t_ativo and trav.p_aprox_odom is not None:
""", """            ok = trav.estado == 'fim'
            # PASSAGEM_BLOQUEADA (11/11): guarda o que barrou a frente, com o scan do aborto (antes da re)
            self.trav_motivo, self.trav_obstaculo = trav.motivo, []
            if not ok and trav.motivo.startswith('algo a') and float(self.p('passagem_bloqueada_s')) > 0.0:
                with self.lock:
                    sc_ = self.scan
                if sc_ is not None:
                    self.trav_obstaculo = pontos_obstaculo_novo(self._mapa(), sc_[1], self._pose_mapa_yaw())
            if not ok and not self.p('nav2_principal') and not self.controle_t_ativo and trav.p_aprox_odom is not None:
"""),
    # ---- metodos do no
    ("""        def _analisar(self, gx, gy, tratadas, pontos=None, goal=None):
            mapa = self._mapa()
""", """        def _bloqueios_ativos(self):
            \"\"\"PASSAGEM_BLOQUEADA: [(x, y, raio)] ainda validos.\"\"\"
            val = float(self.p('passagem_bloqueada_s'))
            if val <= 0.0:
                return []
            agora = time.monotonic()
            self.bloqueios = [b for b in self.bloqueios if agora - b[3] <= val]
            return [b[:3] for b in self.bloqueios]

        def _desviar_bloqueio(self, gx, gy, nome):
            \"\"\"PASSAGEM_BLOQUEADA: a travessia abortou por objeto a frente. Marca o objeto e devolve True se o mapa tem
            outro caminho ate o goal sem ele (o chamador reanalisa e nao repete a travessia).\"\"\"
            if float(self.p('passagem_bloqueada_s')) <= 0.0 or not self.trav_motivo.startswith('algo a'):
                return False
            if getattr(self, 'desvios_bloq', 0) >= 2:
                self.get_logger().warn('%s: PASSAGEM_BLOQUEADA: ja desviei 2 vezes neste goal: sigo como antes' % nome)
                return False
            pts = self.trav_obstaculo
            if not pts:
                self.get_logger().info('%s: PASSAGEM_BLOQUEADA: %s, mas nada livre no mapa a frente (parede?): '
                                       'sigo como antes' % (nome, self.trav_motivo))
                return False
            agora, raio = time.monotonic(), float(self.p('passagem_bloqueada_raio'))
            self.bloqueios += [(x, y, raio, agora) for x, y in pts]
            mapa, pm = self._mapa(), self._pose_mapa()
            rota = rota_no_mapa(mapa, pm, (gx, gy), bloqueios=self._bloqueios_ativos()) \\
                if mapa is not None and pm is not None else None
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            onde = '(%.2f..%.2f, %.2f..%.2f)' % (min(xs), max(xs), min(ys), max(ys))
            if rota is None:
                self.get_logger().warn('%s: PASSAGEM_BLOQUEADA: objeto em %s (%s) e o mapa nao tem outro caminho: '
                                       'tento de novo por aqui' % (nome, onde, self.trav_motivo))
                return False
            self.get_logger().warn('%s: PASSAGEM_BLOQUEADA: objeto em %s (%s); o mapa tem outro caminho (%.1f m): '
                                   'nao insisto, replanejo por ele' % (nome, onde, self.trav_motivo, _comp_rota(rota)))
            self.desvios_bloq = getattr(self, 'desvios_bloq', 0) + 1
            return True

        def _analisar(self, gx, gy, tratadas, pontos=None, goal=None):
            mapa = self._mapa()
"""),
    # ---- _analisar: bloqueios + alternativa quando o planner nao tem caminho
    ("""            pm = self._pose_mapa()
            if goal is not None and mapa is not None and pm is not None and self.p('rota_mapa') \\
                    and self.rota_tent.get((round(gx, 1), round(gy, 1)), 0) <= 2:
                rota = rota_no_mapa(mapa, pm, (gx, gy))
                if rota is not None and (pontos is None or _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5):
""", """            pm = self._pose_mapa()
            bloq = self._bloqueios_ativos()   # PASSAGEM_BLOQUEADA (11/11)
            if bloq and goal is None and rota_cruza_bloqueio(pontos, bloq):
                return [], 'caminho do planner pelo bloqueio marcado (o goal decide)'   # vigia nao interrompe
            if goal is not None and mapa is not None and pm is not None and self.p('rota_mapa') \\
                    and (self.rota_tent.get((round(gx, 1), round(gy, 1)), 0) <= 2 or bloq):
                rota = rota_no_mapa(mapa, pm, (gx, gy), bloqueios=bloq)
                if bloq and rota is None:
                    self.get_logger().warn('PASSAGEM_BLOQUEADA: sem caminho no mapa sem o objeto marcado: uso o mapa '
                                           'inteiro')
                    rota = rota_no_mapa(mapa, pm, (gx, gy))
                elif bloq and rota_cruza_bloqueio(pontos, bloq):
                    self.get_logger().warn('PASSAGEM_BLOQUEADA: o caminho do planner passa pelo objeto marcado: '
                                           'uso a rota do mapa sem ele (%.1f m)' % _comp_rota(rota))
                    pontos = None
                if rota is not None and (pontos is None or _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5):
"""),
    ("""                    if novos and modo_obs == 'recusar' and pontos is not None:
                        self.get_logger().warn('ROTA_MAPA_OBSTACULO: fico com o planner (%.1f m)' % _comp_rota(pontos))
                    else:
""", """                    alt = None   # PASSAGEM_BLOQUEADA: planner sem caminho e objeto na rota do mapa
                    if novos and modo_obs == 'recusar' and pontos is None and float(self.p('passagem_bloqueada_s')) > 0:
                        r_ = float(self.p('passagem_bloqueada_raio'))
                        passo = max(1, len(novos) // 40)
                        alt = rota_no_mapa(mapa, pm, (gx, gy),
                                           bloqueios=list(bloq) + [(x, y, r_) for x, y in novos[::passo]])
                    if novos and modo_obs == 'recusar' and pontos is not None:
                        self.get_logger().warn('ROTA_MAPA_OBSTACULO: fico com o planner (%.1f m)' % _comp_rota(pontos))
                    elif alt is not None:
                        self.get_logger().warn('PASSAGEM_BLOQUEADA: planner sem caminho e objeto na rota do mapa: vou '
                                               'pela rota do mapa sem ele (%.1f m; a direta tinha %.1f m)' % (
                                                   _comp_rota(alt), _comp_rota(rota)))
                        pontos = alt
                    else:
"""),
    ("""            self.rota_tent = {ch: self.rota_tent.get(ch, 0) + 1}
""", """            self.rota_tent = {ch: self.rota_tent.get(ch, 0) + 1}
            self.desvios_bloq = 0   # PASSAGEM_BLOQUEADA: desvios neste goal (max 2)
"""),
    # ---- _executar_legado: nao insiste no corredor bloqueado quando ha outro
    ("""                        if r != 'ok':
                            self.get_logger().warn('Travessia pelo LiDAR falhou: voltando ao preparo '
                                                   'para tentar de novo.')
""", """                        if r != 'ok' and self._desviar_bloqueio(gx, gy, 'TRAVESSIA %d' % n):
                            tratadas.pop()   # PASSAGEM_BLOQUEADA (11/11): reanalisa pelo outro caminho
                            continue
                        if r != 'ok':
                            self.get_logger().warn('Travessia pelo LiDAR falhou: voltando ao preparo '
                                                   'para tentar de novo.')
"""),
    ("""                        if r == 'ok':
                            continue  # reanalisa o caminho a partir da saida
                        if not (self.p('fallback_nav2') and len(etapas) > 1):
""", """                        if r == 'ok':
                            continue  # reanalisa o caminho a partir da saida
                        if self._desviar_bloqueio(gx, gy, 'TRAVESSIA %d (2a tentativa)' % n):
                            tratadas.pop()   # PASSAGEM_BLOQUEADA (11/11)
                            continue
                        if not (self.p('fallback_nav2') and len(etapas) > 1):
"""),
]
if __name__ == '__main__':
    rodar('passagem_bloqueada', 'PASSAGEM_BLOQUEADA (11/11)', ARQ, TROCAS, doc=__doc__)
