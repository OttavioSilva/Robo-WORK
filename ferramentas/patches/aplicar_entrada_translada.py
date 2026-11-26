#!/usr/bin/env python3
"""ENTRADA_TRANSLADA (24/11/2026): ponto sem mesa (START/FINISH...) onde o giro nao cabe: gira antes, entra transladando.

Simulacao da arena manual (24/11): volta ao FINISH 0/5 certa na 1a bateria (3 colisoes depois): o ponto
sem mesa usava o angulo de chegada (-177), o DWB ciscava perto do ponto e o destravar girava ate 215
graus dentro da caixa do START (1,09 m; o giro pede ~1,0 m).
Agora, so quando o giro NAO cabe no ponto (pose_livre em 24 angulos, margem 3 cm, pelo mapa):
  1. ponto de giro = ultimo ponto do caminho do planner (ate 2,5 m antes) onde o giro cabe e de onde o
     corpo vai ao ponto em linha reta com o yaw CADASTRADO sem tocar o mapa (margem 2 cm);
  2. Nav2 ate o ponto de giro; _girar_seguro para o yaw cadastrado;
  3. _deslocar_ate o ponto (lado e frente/re pela odom, parando com folga < 7 cm pelo LiDAR), ate 4x;
  4. sucesso a <= 15 cm.
Giro cabe no ponto ou sem ponto de giro: igual a antes. Parametro 'entrada_translada' (false = antes).
Uso: aplicar_entrada_translada.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    ("""def analisar_juncao(pontos, larg, trechos, k, gA, gap_max=1.30):
""",
     """def giro_livre(mapa, x, y, margem=0.03):   # ENTRADA_TRANSLADA (24/11)
    return all(pose_livre(mapa, x, y, math.radians(a), margem=margem) for a in range(0, 360, 15))


def ponto_de_giro(caminho, gx, gy, gyaw, mapa, d_min=0.15, d_max=2.5, passo=0.05):
    \"\"\"ENTRADA_TRANSLADA: ultimo ponto do caminho (vindo do fim) onde o giro cabe e de onde o corpo
    vai ao goal em linha reta com o yaw final. None se o giro ja cabe no goal ou se nao achou.\"\"\"
    if giro_livre(mapa, gx, gy):
        return None
    pts = list(reversed(caminho))
    acum = 0.0
    for k in range(1, len(pts)):
        acum += math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1])
        if acum > d_max:
            break
        x, y = pts[k]
        d = math.hypot(x - gx, y - gy)
        if d < d_min or k % 2:
            continue
        if not giro_livre(mapa, x, y):
            continue
        n = max(2, int(d / passo))
        if all(pose_livre(mapa, x + (gx - x) * i / n, y + (gy - y) * i / n, gyaw, margem=0.02)
               for i in range(n + 1)):
            return (x, y)
    return None


def analisar_juncao(pontos, larg, trechos, k, gA, gap_max=1.30):
"""),
    ("""        def _deslocar_ate(self, x, y, nome, tol=0.03, passo_max=0.30):
""",
     """        def _entrada_translada(self, area, gx, gy, gyaw, vigia):   # ENTRADA_TRANSLADA (24/11)
            \"\"\"Ponto sem mesa onde o giro nao cabe: gira no ultimo ponto que cabe e entra transladando.
            None = nao se aplica (segue o fluxo de antes); 'ok' / 'falha' / 'interrompido'.\"\"\"
            mapa = self._mapa()
            caminho = self._caminho(self._pose(gx, gy, gyaw)) if mapa is not None else None
            if not caminho or len(caminho) < 2:
                return None
            p = ponto_de_giro(caminho, gx, gy, gyaw, mapa)
            if p is None:
                self.get_logger().info('ENTRADA_TRANSLADA: %s: o giro cabe no ponto ou nao achei ponto de giro: '
                                       'como antes' % area)
                return None
            n = 'ENTRADA %s' % area
            self.get_logger().info('ENTRADA_TRANSLADA: %s: giro nao cabe no ponto; giro em (%.2f, %.2f) para %.0f '
                                   'graus e entro transladando %.0f cm' % (
                                       area, p[0], p[1], math.degrees(gyaw), 100 * math.hypot(gx - p[0], gy - p[1])))
            pm = self._pose_mapa_yaw()
            if pm is None or math.hypot(p[0] - pm[0], p[1] - pm[1]) > 0.10:
                yaw_c = yaw_chegada(self._caminho(self._pose(p[0], p[1], gyaw)), p, pm)
                alvo = self._pose(p[0], p[1], yaw_c if yaw_c is not None else gyaw)
                r = self._navegar(alvo, n + ' (ponto de giro)', vigia, aceitar_perto=True)
                if r != 'ok':
                    return r
            self._girar_seguro(gyaw, n, tempo_max=30.0, movs_max=4)
            d = None
            for _ in range(4):
                self._deslocar_ate(gx, gy, n, tol=0.04, passo_max=1.5)
                pm = self._pose_mapa_yaw()
                d = None if pm is None else math.hypot(gx - pm[0], gy - pm[1])
                if d is not None and d <= 0.10:
                    break
            ok = d is not None and d <= 0.15
            self.get_logger().info('ENTRADA_TRANSLADA: %s: %s (%s do ponto)' % (
                area, 'SUCESSO' if ok else 'FALHOU', '?' if d is None else '%.0f cm' % (d * 100)))
            return 'ok' if ok else 'falha'

        def _deslocar_ate(self, x, y, nome, tol=0.03, passo_max=0.30):
"""),
    ("""                livre = area is not None and area in self.sem_encosto
                if livre:
                    # v3.10: ponto sem mesa: angulo de chegada pelo caminho, nada de giro final
""",
     """                livre = area is not None and area in self.sem_encosto
                if livre and self.p('entrada_translada'):   # ENTRADA_TRANSLADA (24/11)
                    r_ent = self._entrada_translada(area, gx, gy, yaw_de(goal.pose.orientation), vigia)
                    if r_ent == 'ok':
                        self._publicar_resultado(area, True, 'chegou (ponto sem mesa, entrada transladando)')
                        return
                    if r_ent == 'falha':
                        return
                    if r_ent == 'interrompido':
                        continue
                if livre:
                    # v3.10: ponto sem mesa: angulo de chegada pelo caminho, nada de giro final
"""),
    ("""            self.declare_parameter('encosto_recuo_cego', True)  # RECUO_CEGO (24/11)
""",
     """            self.declare_parameter('encosto_recuo_cego', True)  # RECUO_CEGO (24/11)
            self.declare_parameter('entrada_translada', True)  # ENTRADA_TRANSLADA (24/11)
"""),
]


def conferir(velho, novo):
    assert novo.count('ENTRADA_TRANSLADA') >= 6


if __name__ == '__main__':
    rodar('entrada_translada', 'ENTRADA_TRANSLADA', PADRAO, TROCAS, conferir, doc=__doc__)
