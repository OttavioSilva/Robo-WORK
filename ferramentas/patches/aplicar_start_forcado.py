#!/usr/bin/env python3
"""START_FORCADO (24/11/2026): vigia nunca espalha a pose antes da 1a localizacao boa; forca o START.

Arena 24/11 13:09 (mapa manual): AMCL partiu do START, o encaixe ficou 30-48% e o vigia disparou a
relocalizacao GLOBAL: pose saltou 2,77 m / 110 graus para um lugar errado. O robo SEMPRE parte do START,
virado para a arena. Agora, enquanto nao houve nenhuma pose boa (encaixe >= 70%):
  - perdido e robo ainda no START (odom < 10 cm / 10 graus) -> a cada 5 s republica /initialpose no START de ~/mapas/pontos.yaml (cov 5 cm / 5 graus);
  - saiu do START sem nunca localizar bem: so avisa; NUNCA chama /reinitialize_global_localization.
Depois da 1a pose boa: igual a antes (ultima boa, depois global). Parametro 'start_forcado' (false = antes).
Uso: aplicar_start_forcado.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/vigia_localizacao.py')

TROCAS = [
    ("""import json
import math
""",
     """import json
import math
import os  # START_FORCADO (24/11)
"""),
    ("""        self.declare_parameter('recuperar', True)
""",
     """        self.declare_parameter('recuperar', True)
        self.declare_parameter('start_forcado', True)  # START_FORCADO (24/11)
        self.t_forcou = None
        self.odom0 = None
        self.odom_mexeu = False
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, '/odom', self.ao_odom_start, 10)
"""),
    ("""    def recuperar(self, agora):
        if self.t_perdido is None:
""",
     """    def ao_odom_start(self, m):   # START_FORCADO: saiu do START = andou 10 cm ou girou 10 graus
        p = m.pose.pose
        a = (p.position.x, p.position.y, yaw_q(p.orientation))
        if self.odom0 is None:
            self.odom0 = a
        elif not self.odom_mexeu:
            dy = abs(math.atan2(math.sin(a[2] - self.odom0[2]), math.cos(a[2] - self.odom0[2])))
            if math.hypot(a[0] - self.odom0[0], a[1] - self.odom0[1]) > 0.10 or dy > math.radians(10):
                self.odom_mexeu = True
                self.get_logger().info('START_FORCADO: robo saiu do START (odom); nao forco mais o START')

    def start_salvo(self):   # START_FORCADO (24/11)
        try:
            import yaml
            p = yaml.safe_load(open(os.path.expanduser('~/mapas/pontos.yaml')))['pontos']['START']
            return float(p['x']), float(p['y']), float(p['yaw'])
        except Exception as e:  # noqa: BLE001
            self.get_logger().error('START_FORCADO: nao li o START de ~/mapas/pontos.yaml (%s)' % e)
            return None

    def forcar_start(self, agora):   # START_FORCADO (24/11)
        if self.t_forcou is not None and agora - self.t_forcou < 5.0:
            return
        st = self.start_salvo()
        if st is None:
            return
        self.t_forcou = agora
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = st[0], st[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(st[2] / 2), math.cos(st[2] / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(5) ** 2
        self.pub_init.publish(m)
        self.get_logger().warn('START_FORCADO: sem localizacao boa ainda; AMCL forcado no START (%.2f, %.2f, %.0f graus)'
                               % (st[0], st[1], math.degrees(st[2])))

    def recuperar(self, agora):
        if self.ultima_boa is None and self.get_parameter('start_forcado').value:   # START_FORCADO
            if not self.odom_mexeu:
                self.forcar_start(agora)
            elif self.t_forcou is None or agora - self.t_forcou > 10.0:
                self.t_forcou = agora
                self.get_logger().warn('START_FORCADO: perdido sem nunca ter localizado bem; NAO faco busca global')
            return
        if self.t_perdido is None:
"""),
]


def conferir(velho, novo):
    assert novo.count('START_FORCADO') >= 5
    assert 'import os' in novo
    compile(novo, 'vigia', 'exec')


if __name__ == '__main__':
    rodar('start_forcado', 'START_FORCADO', PADRAO, TROCAS, conferir, doc=__doc__)
