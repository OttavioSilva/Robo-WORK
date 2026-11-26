#!/usr/bin/env python3
"""GIRO_NA_CURVA (25/11/2026): curva fechada no caminho -> o alinhador leva ate a curva, gira la e segue.

Teste FINISH 25/11 09:06-09:28: as colisoes que sobraram foram todas em (3,7-3,8; 5,4-5,9), rumo 110-150 graus:
o robo sobe pelo vao leste e o caminho vira ~90 graus para oeste no corredor entre a mesa central e a parede sob
a WS09 (1,38 m). O DWB nao gira andando o suficiente (giro andando fraco, sem RotationShim), passa da curva e
raspa a parede/quina; dai vem ciscado e destravar girando perto da parede. Operador: vai ate a curva, para,
gira e segue. Agora (param 'giro_na_curva', false = como antes): antes de mandar o goal, se o caminho tem uma
curva >= 60 graus (janela de 0,4 m) a mais de 0,6 m do robo, num ponto onde o giro cabe no mapa, o alinhador
navega ate la (aceita a 0,25 m), gira com o _girar_seguro para o rumo de saida e manda o goal de novo (no
maximo 3 curvas por goal). O alinhador e o navegador principal; o Nav2 faz so os trechos retos.
Uso: aplicar_giro_na_curva.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""def rumo_do_caminho(caminho, x, y, dist=0.5):
""",
     """def ponto_de_curva(caminho, mapa, ang_min=math.radians(60), janela=0.4, d_min=0.6, d_fim=0.5):
    \"\"\"GIRO_NA_CURVA (25/11): 1a curva fechada do caminho -> (x, y, rumo_entrada, rumo_saida) ou None.\"\"\"
    if not caminho or len(caminho) < 5 or mapa is None:
        return None
    s = [0.0]
    for a, b in zip(caminho, caminho[1:]):
        s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = s[-1]
    for i in range(1, len(caminho) - 1):
        if s[i] < d_min or s[i] > total - d_fim:
            continue
        j0 = max(k for k in range(i + 1) if s[i] - s[k] >= janela) if s[i] >= janela else 0
        j1 = min((k for k in range(i, len(caminho)) if s[k] - s[i] >= janela), default=len(caminho) - 1)
        x, y = caminho[i]
        h_in = math.atan2(y - caminho[j0][1], x - caminho[j0][0])
        h_out = math.atan2(caminho[j1][1] - y, caminho[j1][0] - x)
        dif = abs(_ang(h_out - h_in))
        if dif >= ang_min:
            melhor = (dif, i, h_in, h_out)
            for m in range(i + 1, len(caminho) - 1):   # no mesmo trecho de curva, o vertice (maior mudanca)
                if s[m] - s[i] > janela:
                    break
                m0 = max(k for k in range(m + 1) if s[m] - s[k] >= janela) if s[m] >= janela else 0
                m1 = min((k for k in range(m, len(caminho)) if s[k] - s[m] >= janela), default=len(caminho) - 1)
                hi = math.atan2(caminho[m][1] - caminho[m0][1], caminho[m][0] - caminho[m0][0])
                ho = math.atan2(caminho[m1][1] - caminho[m][1], caminho[m1][0] - caminho[m][0])
                if abs(_ang(ho - hi)) > melhor[0]:
                    melhor = (abs(_ang(ho - hi)), m, hi, ho)
            x, y = caminho[melhor[1]]
            if giro_livre(mapa, x, y):
                return (x, y, melhor[2], melhor[3])
            return None
    return None


def rumo_do_caminho(caminho, x, y, dist=0.5):
"""),
    ("""            self.declare_parameter('raio_ponto_giro', 0.30)   # ENTRADA_ASSUME (25/11): 0 = como antes
""",
     """            self.declare_parameter('raio_ponto_giro', 0.30)   # ENTRADA_ASSUME (25/11): 0 = como antes
            self.declare_parameter('giro_na_curva', True)     # GIRO_NA_CURVA (25/11)
"""),
    ("""            if virar:
                self._virar_para_caminho(alvo, nome)
            self._esperar_sensores(nome)   # SENSOR_PARADO
""",
     """            if virar:
                self._virar_para_caminho(alvo, nome)
                if self.p('giro_na_curva') and _cisc == 0 and not getattr(self, '_em_curva', False):   # GIRO_NA_CURVA
                    r_c = self._giro_na_curva(alvo, nome, vigia)
                    if r_c == 'interrompido':
                        return r_c
            self._esperar_sensores(nome)   # SENSOR_PARADO
"""),
    ("""        def _navegar_legado(self, alvo, nome, vigia=None, aceitar_perto=False, virar=True, _cisc=0):
""",
     """        def _giro_na_curva(self, alvo, nome, vigia):
            \"\"\"GIRO_NA_CURVA (25/11): ate 3 curvas fechadas: vai ate a curva, gira la. 'ok'/'nada'/'interrompido'.\"\"\"
            feito = 'nada'
            for k in range(3):
                pm, mapa = self._pose_mapa_yaw(), self._mapa()
                caminho = self._caminho(alvo)
                c = ponto_de_curva(caminho, mapa) if pm is not None else None
                if c is None:
                    return feito
                n = '%s (curva %d)' % (nome, k + 1)
                self.get_logger().info('GIRO_NA_CURVA: %s: curva de %+.0f graus em (%.2f, %.2f): vou ate la, giro e sigo' % (
                    n, math.degrees(_ang(c[3] - c[2])), c[0], c[1]))
                self._em_curva, self._raio_tmp = True, 0.25
                try:
                    r = self._navegar(self._pose(c[0], c[1], c[2]), n, vigia, aceitar_perto=True, virar=False)
                finally:
                    self._em_curva, self._raio_tmp = False, None
                if r == 'interrompido':
                    return r
                if r != 'ok':
                    self.get_logger().warn('GIRO_NA_CURVA: %s: nao cheguei na curva; sigo pelo Nav2' % n)
                    return feito
                self._girar_seguro(c[3], n, tempo_max=30.0, movs_max=4)
                feito = 'ok'
            return feito

        def _navegar_legado(self, alvo, nome, vigia=None, aceitar_perto=False, virar=True, _cisc=0):
"""),
]


def conferir(velho, novo):
    assert novo.count('GIRO_NA_CURVA') >= 5
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('giro_na_curva', 'GIRO_NA_CURVA', PADRAO, TROCAS, conferir, doc=__doc__)
