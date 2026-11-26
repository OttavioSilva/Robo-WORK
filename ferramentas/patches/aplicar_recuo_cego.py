#!/usr/bin/env python3
"""RECUO_CEGO (24/11/2026): encosto que comeca perto demais recua e mede de novo, em vez de abortar.

Simulacao 24/11 (modo prova, rota longa): o robo parou ~9 cm mais perto da WS07 que a aproximacao
(face a d0,42 contra d0,51 na corrida boa); o meio da face caiu na zona cega do LiDAR (< 0,146 m)
e so apareciam pedacos sem as 2 pontas (w0,28): 'nao vejo a face da mesa a frente' em todas as
tentativas. A bancada de encosto ja tinha esse limite (cenario perto_045).
Agora, na 1a medicao (estado 'medir', antes de qualquer alinhamento), sem face inteira: re reta de
12 cm pela odom (guarda 5 s; o poligono traseiro do Collision Monitor continua valendo) e mede de
novo, ate 2 vezes. Parametro do alinhador 'encosto_recuo_cego' (false = como antes).
Uso: aplicar_recuo_cego.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    ("""        self.lat_ini = None            # (folga, |centro|) no inicio do ultimo deslize
""",
     """        self.lat_ini = None            # (folga, |centro|) no inicio do ultimo deslize
        self.recuo_cego_max = 2        # RECUO_CEGO (24/11): re de 12 cm quando a 1a medicao nao ve a face
        self.n_recuo_cego = 0
"""),
    ("""                if e == 'medir' and any(m[1]['centro'] is None for m in self._novas(self.t_estado)):
""",
     """                if e == 'medir' and self.n_recuo_cego < self.recuo_cego_max and not self.avancos \\
                        and self.centro_longe is None:
                    # RECUO_CEGO (24/11): perto demais o meio da face cai na zona cega: recua e mede
                    self.n_recuo_cego += 1
                    self.log('RECUO_CEGO: sem face inteira na 1a medicao (perto demais?): re de 12 cm '
                             'e meco de novo (%d/%d)' % (self.n_recuo_cego, self.recuo_cego_max))
                    self._mudar('recuar_cego', t, pose)
                    return 0.0, 0.0, 0.0
                if e == 'medir' and any(m[1]['centro'] is None for m in self._novas(self.t_estado)):
"""),
    ("""        if e == 'recuar':
            # RECUO_LATERAL: re reta segurando o rumo da face; para pelo LiDAR (folga prevista)
""",
     """        if e == 'recuar_cego':   # RECUO_CEGO (24/11): re reta pela odom
            dx, _ = self._desloc(self.ini, pose)
            if -dx >= 0.12 or t - self.t_estado > 5.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return -self.v_aprox, 0.0, 0.0

        if e == 'recuar':
            # RECUO_LATERAL: re reta segurando o rumo da face; para pelo LiDAR (folga prevista)
"""),
    ("""                          log=lambda m: self.get_logger().info('ENCOSTAR %s: %s' % (nome, m)))
            self.get_logger().info('ENCOSTAR %s: medindo a face da mesa' % nome)
""",
     """                          log=lambda m: self.get_logger().info('ENCOSTAR %s: %s' % (nome, m)))
            if not self.p('encosto_recuo_cego'):   # RECUO_CEGO
                enc.recuo_cego_max = 0
            self.get_logger().info('ENCOSTAR %s: medindo a face da mesa' % nome)
"""),
    ("""            self.declare_parameter('encosto_linear', True)
""",
     """            self.declare_parameter('encosto_linear', True)
            self.declare_parameter('encosto_recuo_cego', True)  # RECUO_CEGO (24/11)
"""),
]


def conferir(velho, novo):
    assert novo.count('RECUO_CEGO') >= 6


if __name__ == '__main__':
    rodar('recuo_cego', 'RECUO_CEGO', PADRAO, TROCAS, conferir, doc=__doc__)
