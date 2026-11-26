#!/usr/bin/env python3
"""CONTORNAR (05/11/2026): o destravar ve o fim do obstaculo, desliza ate vence-lo e passa por ele.

Por que (missao22, 18:29:52-18:30:58): obstaculo na quina da frente, sobrepondo so 3-5 cm do corpo.
O destravar respondia "nada livre" porque o ponto ja estava na margem do corpo (ate deslizar para longe
contava como colisao), dava 9 cm de re e o Nav2 voltava ao mesmo lugar: 5 ciclos, 66 s. Nos scans do
bag, 7-10 cm para o lado livre bastavam.

O que faz (so se a frente esta a < 15 cm, o caminho segue para a frente e o destino esta a >= 60 cm):
  1) junta os pontos que barram a frente num objeto. Maior que 25 cm: e parede ou mesa, nao contorna;
  2) escolhe o lado de menor deslize, com o lado livre e a frente livre depois de deslizar;
  3) desliza conferindo o LiDAR ate a faixa a frente do corpo ficar livre (o "venci o obstaculo");
  4) avanca 25 cm para o obstaculo ficar ao lado do corpo e so entao devolve ao Nav2.
Nao repete no mesmo lugar: em ate 30 s e a menos de 10 cm, segue o destravar normal.

Uso:
    python3 aplicar_contornar.py              # aplica (backup antes)
    python3 aplicar_contornar.py --desfazer   # volta o backup mais recente
    python3 aplicar_contornar.py --arquivo <copia>
Depois: reiniciar o ~/subir_robo.sh (o script roda direto do src).
"""
import argparse
import glob
import os
import shutil
import sys
import time

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/destravar_node.py')
MARCA = 'CONTORNAR'
SUF = '.bak_contornar_'

FUNCOES = '''# ---------------- CONTORNAR (05/11): ver o fim do obstaculo e passar por ele ----------------
FAIXA_FRENTE = 0.15      # o que esta ate aqui alem do para-choque barra o caminho
MARGEM_LADO = 0.05       # folga lateral para dar o obstaculo por vencido
OBJETO_MAX = 0.25        # maior que isto e parede ou mesa, nao obstaculo
DESLIZE_MAX = 0.30
AVANCO = 0.25
DESTINO_MIN = 0.60       # perto do destino quem barra costuma ser a mesa


def barrando(pts, meia=None):
    """Pontos na faixa a frente do corpo."""
    _, x1, y1 = FOOTPRINT
    lim = y1 + 0.03 if meia is None else meia
    return [(x, y) for x, y in pts if x1 - 0.02 <= x <= x1 + FAIXA_FRENTE and abs(y) <= lim]


def aglomerado(pts, sementes, gap=0.10):
    """Pontos ligados as sementes por saltos de ate gap: o objeto inteiro que o LiDAR ve."""
    grupo, novos, resto = set(sementes), list(sementes), [p for p in pts if p not in set(sementes)]
    while novos:
        ax, ay = novos.pop()
        perto = [p for p in resto if math.hypot(p[0] - ax, p[1] - ay) <= gap]
        for p in perto:
            grupo.add(p)
            resto.remove(p)
        novos.extend(perto)
    return grupo


def plano_contornar(pts, ang_plano):
    """-> {'uy', 's', 'ymin', 'ymax'} se ha um obstaculo pequeno a vencer de lado, ou None."""
    if ang_plano is None or abs(ang_plano) > math.radians(60):
        return None
    _, _, y1 = FOOTPRINT
    barra = barrando(pts)
    if not barra:
        return None
    grupo = aglomerado(pts, barra)
    xs, ys = [p[0] for p in grupo], [p[1] for p in grupo]
    if max(xs) - min(xs) > OBJETO_MAX or max(ys) - min(ys) > OBJETO_MAX:
        return None
    outros = [p for p in pts if p not in grupo]
    opcoes = []
    for uy, s in ((1.0, max(ys) + y1 + MARGEM_LADO), (-1.0, -min(ys) + y1 + MARGEM_LADO)):
        if s > DESLIZE_MAX or not translacao_livre(outros, 0.0, uy, s + 0.03):
            continue
        if not translacao_livre([(x, y - uy * s) for x, y in pts], 1.0, 0.0, AVANCO, margem=0.02):
            continue
        # empate (2 cm): o lado do caminho
        opcoes.append((round(s / 0.02), 0 if uy * ang_plano >= 0 else 1, s, uy))
    if not opcoes:
        return None
    _, _, s, uy = min(opcoes)
    return {'uy': uy, 's': s, 'ymin': min(ys), 'ymax': max(ys)}


'''
METODOS = '''        # ---------------- CONTORNAR ----------------
        def plano_contorno(self, pts):
            """Decide se contorna: fora do destino e sem repetir no mesmo lugar."""
            with self.lock:
                pl, od = self.plano, self.odom
            p = self.pose_map()
            if pl is None or p is None or not pl[1] or od is None:
                return None
            if math.hypot(pl[1][-1][0] - p[0], pl[1][-1][1] - p[1]) < DESTINO_MIN:
                return None
            if self.ult_contorno is not None and time.monotonic() - self.ult_contorno[0] < 30.0 \\
                    and math.hypot(od[1] - self.ult_contorno[1], od[2] - self.ult_contorno[2]) < 0.10:
                return None
            return plano_contornar(pts, self.direcao_caminho())

        def contornar(self, gh, pl, t_fim):
            """Desliza ate vencer o obstaculo e avanca. Devolve a mensagem, ou None se nao venceu."""
            uy = pl['uy']
            lado = 'esq' if uy > 0 else 'dir'
            with self.lock:
                pts, od = self.pts, self.odom
            self.ult_contorno = (time.monotonic(), od[1], od[2])
            self.get_logger().info('CONTORNAR: obstaculo de y %+.0f a %+.0f cm: deslizo %.0f cm para %s' % (
                100 * pl['ymin'], 100 * pl['ymax'], 100 * pl['s'], lado))
            if folga(pts[1], 'frente') < 0.04 and translacao_livre(pts[1], -1.0, 0.0, 0.05):
                self.andar(gh, -1.0, 0.0, 0.04, 0.05, t_fim)
            with self.lock:
                od0 = self.odom
            c0, s0 = math.cos(od0[3]), math.sin(od0[3])
            _, _, y1 = FOOTPRINT
            lat, livres, ult_scan, cm_desde, venci = 0.0, 0, None, None, False
            while rclpy.ok() and not gh.is_cancel_requested and time.monotonic() < t_fim:
                with self.lock:
                    pts, od = self.pts, self.odom
                dx, dy = od[1] - od0[1], od[2] - od0[2]
                lat = (-s0 * dx + c0 * dy) * uy
                if pts[0] != ult_scan:  # scan novo: a faixa esta livre? 2 seguidos, um raio pode falhar
                    ult_scan = pts[0]
                    livres = 0 if barrando(pts[1], y1 + MARGEM_LADO) else livres + 1
                if livres >= 2 and lat >= 0.5 * pl['s']:  # a odom de lado superestima
                    venci = True
                    break
                # so olha o lado para onde vai: o obstaculo que fica para tras nao barra o deslize
                if lat >= pl['s'] + 0.10 or not translacao_livre([p for p in pts[1] if p[1] * uy > 0], 0.0, uy, 0.04):
                    break
                if self.cm_stop:
                    cm_desde = cm_desde or time.monotonic()
                    if time.monotonic() - cm_desde > 1.0:
                        break
                else:
                    cm_desde = None
                self.vel(0.0, 0.08 * uy)
                time.sleep(0.05)
            self.parar()
            time.sleep(0.3)
            if not venci:
                self.get_logger().warn('CONTORNAR: nao venci depois de %.0f cm para %s' % (100 * lat, lado))
                return None
            f = self.andar(gh, 1.0, 0.0, AVANCO, 0.10, t_fim)
            self.get_logger().info('CONTORNAR: venci: %.0f cm para %s e %.0f cm para frente' % (100 * lat, lado, 100 * f))
            return 'contornei: %.0f cm para %s + %.0f cm para frente' % (100 * lat, lado, 100 * f)

'''
TROCAS = [
    ("""v2 (missao17): (1) encostado""", """CONTORNAR (05/11): com a frente barrada por um objeto pequeno, ve o fim dele, desliza ate vence-lo pelo
LiDAR, avanca e so entao devolve ao Nav2 (antes dava re e o Nav2 voltava ao mesmo lugar).

v2 (missao17): (1) encostado"""),
    ("""def main():
    import rclpy
""", FUNCOES + """def main():
    import rclpy
"""),
    ("""            self.limpar_local_pendente = False
            self.cli_limpar_global""", """            self.limpar_local_pendente = False
            self.ult_contorno = None  # CONTORNAR: (t, x, y) na odom
            self.cli_limpar_global"""),
    ("""        # ---------------- a acao ----------------
""", METODOS + """        # ---------------- a acao ----------------
"""),
    ("""            if folga(pts[1], 'frente') < 0.15:
                self.marcar_frente()
                time.sleep(0.4)          # BLOQUEIO_LOCAL: o costmap local atualiza a 5 Hz
            if self.replanejar() is False:  # BLOQUEIO_LOCAL: sem caminho: apaga, limpa e planeja sem ele
                self.soltar_bloqueio()
                self.replanejar()
""", """            sem_caminho = self.replanejar() is False
            if sem_caminho:              # BLOQUEIO_LOCAL: sem caminho: apaga, limpa e planeja sem ele
                self.soltar_bloqueio()
                self.replanejar()
            with self.lock:
                pts = self.pts
            if folga(pts[1], 'frente') < 0.15:
                pl_cont = self.plano_contorno(pts[1])   # CONTORNAR: vence o obstaculo antes de tudo
                if pl_cont is not None:
                    msg_cont = self.contornar(gh, pl_cont, t_fim)
                    if msg_cont is not None:
                        return fim(True, msg_cont)
                if not sem_caminho:
                    self.marcar_frente()
                    time.sleep(0.4)      # BLOQUEIO_LOCAL: o costmap local atualiza a 5 Hz
"""),
]


def main():
    ap = argparse.ArgumentParser(description='CONTORNAR')
    ap.add_argument('--desfazer', action='store_true')
    ap.add_argument('--arquivo', default=ARQ)
    a = ap.parse_args()
    if a.desfazer:
        baks = sorted(glob.glob(a.arquivo + SUF + '*'))
        if not baks:
            print('Nenhum backup %s* encontrado.' % SUF)
            return 1
        shutil.copy2(baks[-1], a.arquivo)
        print('Restaurado de %s' % baks[-1])
        return 0
    txt = open(a.arquivo).read()
    if MARCA in txt:
        print('Ja aplicado (achei a marca %s). Nada a fazer.' % MARCA)
        return 0
    try:
        if 'BLOQUEIO_LOCAL' not in txt:
            raise RuntimeError('precisa do BLOQUEIO_LOCAL aplicado antes')
        novo = txt
        for velho, troca in TROCAS:
            if novo.count(velho) != 1:
                raise RuntimeError('trecho aparece %d vezes (esperado 1): %s' % (novo.count(velho), velho[:60]))
            novo = novo.replace(velho, troca)
        compile(novo, a.arquivo, 'exec')
    except Exception as exc:
        print('Nao consegui aplicar (%s). Nada alterado.' % exc)
        return 1
    bak = a.arquivo + SUF + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(a.arquivo, bak)
    open(a.arquivo, 'w').write(novo)
    print('Backup: %s\nAplicado. Reinicie o ~/subir_robo.sh.' % bak)
    return 0


if __name__ == '__main__':
    sys.exit(main())
