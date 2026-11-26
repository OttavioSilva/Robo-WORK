#!/usr/bin/env python3
"""PRE_GIRO_UNICO (11/11/2026) - alinhador, VIRAR_CAMINHO.
O pre-giro roda a cada envio ao Nav2, inclusive nos reenvios do mesmo alvo (ANTI_CISCADO, REINSISTIR, travessia
-> goal final). Depois de um pre-giro tentado para o mesmo alvo, girar de novo em seguida so repete a mesma correcao.
O que muda: se o mesmo alvo (+-15 cm) ja teve pre-giro tentado ha menos de pre_giro_repetir_s (20 s), nao gira de
novo; o Nav2 (DWB direto, mecanum) segue do rumo atual. Primeiro envio para cada alvo: igual a antes (45 graus).
pre_giro_repetir_s 0 = como antes.
Uso: python3 aplicar_pre_giro_unico.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""            self.declare_parameter('virar_caminho_graus', 45.0)  # v3.30 VIRAR_CAMINHO: vira antes do Nav2 (0 desliga)
""", """            self.declare_parameter('virar_caminho_graus', 45.0)  # v3.30 VIRAR_CAMINHO: vira antes do Nav2 (0 desliga)
            # PRE_GIRO_UNICO (11/11): nao repete o pre-giro para o mesmo alvo por N s (0 = como antes)
            self.declare_parameter('pre_giro_repetir_s', 20.0)
            self.pre_giro_memo = None   # (x, y, t) do ultimo alvo com pre-giro tentado
"""),
    ("""            rumo = rumo_do_caminho(caminho, pm[0], pm[1], 0.5)
            if rumo is None:
                return
            ang = _ang(rumo - pm[2])
            if abs(ang) <= math.radians(lim):
                return
""", """            rumo = rumo_do_caminho(caminho, pm[0], pm[1], 0.5)
            if rumo is None:
                return
            ang = _ang(rumo - pm[2])
            if abs(ang) <= math.radians(lim):
                return
            # PRE_GIRO_UNICO (11/11): mesmo alvo com pre-giro tentado ha pouco -> nao repete a correcao
            ax_, ay_ = alvo.pose.position.x, alvo.pose.position.y
            memo, rep = self.pre_giro_memo, float(self.p('pre_giro_repetir_s'))
            if (rep > 0.0 and memo is not None and math.hypot(memo[0] - ax_, memo[1] - ay_) <= 0.15
                    and time.monotonic() - memo[2] < rep):
                self.get_logger().info('%s: caminho a %+.0f graus, mas ja tentei o pre-giro para este alvo ha %.0f s: '
                                       'sigo pelo Nav2 (PRE_GIRO_UNICO)' % (nome, math.degrees(ang),
                                                                            time.monotonic() - memo[2]))
                return
            self.pre_giro_memo = (ax_, ay_, time.monotonic())
"""),
]
if __name__ == '__main__':
    rodar('pre_giro_unico', 'PRE_GIRO_UNICO (11/11)', ARQ, TROCAS, doc=__doc__)
