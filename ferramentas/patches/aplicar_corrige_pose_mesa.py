#!/usr/bin/env python3
"""CORRIGE_POSE_MESA (24/11/2026): depois de cada encosto com SUCESSO, corrige a pose do AMCL pela mesa.

Arena 24/11 (mapa manual + AMCL): de frente para a WS08 a candidata do LiDAR ficou 27 cm mais funda que
a face cadastrada, com a mesa fisicamente no lugar certo (usuario): o AMCL pos o robo ~27 cm a oeste,
com encaixe 92% (a mesa e um bloco preto: ponto dentro dele conta como perfeito; a mesa esconde a parede).
Encostado, o robo sabe onde esta em relacao a uma face CADASTRADA: distancia (folga + para-choque),
angulo e centro medidos pelo LiDAR. Com a face cadastrada (x, y, yaw = direcao robo->mesa):
  yaw_robo = yaw_face - ang;   P = F - d*(cos yaw_face, sin yaw_face) - t*(-sin yaw_face, cos yaw_face)
  (d = dist*cos(ang), t = centro; centro nao medido: t vem do AMCL, so corrige profundidade e angulo)
Publica /initialpose (cov 3 cm / 2 graus) SO se: AMCL rodando (no 'amcl'), correcao >= 2 cm ou 1 grau
e <= 40 cm e 15 graus (maior = face/mesa errada: so avisa). Parametro 'corrige_pose_mesa' (false = antes).
Log 'CORRIGE_POSE_MESA:'. Uso: aplicar_corrige_pose_mesa.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    ("""def giro_livre(mapa, x, y, margem=0.03):   # ENTRADA_TRANSLADA (24/11)
""",
     """def pose_pela_face(face, dist, ang_face, centro, pose_amcl=None):   # CORRIGE_POSE_MESA (24/11)
    \"\"\"Pose (x, y, yaw) do base_link no mapa a partir da face cadastrada e da medicao do LiDAR.
    face: dict x, y, yaw (direcao robo->mesa). dist/ang_face/centro: como RastreioFaceWS (base_link).
    centro None: componente ao longo da face vem de pose_amcl (None se nao houver).\"\"\"
    fy = face['yaw']
    ux, uy = math.cos(fy), math.sin(fy)
    d = dist * math.cos(ang_face)
    if centro is None:
        if pose_amcl is None:
            return None
        t = (face['x'] - pose_amcl[0]) * -uy + (face['y'] - pose_amcl[1]) * ux
    else:
        t = centro
    x = face['x'] - d * ux + t * uy
    y = face['y'] - d * uy - t * ux
    yaw = math.atan2(math.sin(fy - ang_face), math.cos(fy - ang_face))
    return x, y, yaw


def giro_livre(mapa, x, y, margem=0.03):   # ENTRADA_TRANSLADA (24/11)
"""),
    ("""        def _publicar_resultado(self, nome, ok, texto):
""",
     """        def _corrigir_pose_mesa(self, nome, face, r, pc):   # CORRIGE_POSE_MESA (24/11)
            if not self.p('corrige_pose_mesa') or r.get('folga') is None or r.get('ang') is None:
                return
            if 'amcl' not in self.get_node_names():
                self.get_logger().info('CORRIGE_POSE_MESA: %s: sem AMCL (SLAM): nao corrijo' % nome)
                return
            pm = self._pose_mapa_yaw()
            nova = pose_pela_face(face, r['folga'] + pc, r['ang'], r.get('centro'), pm)
            if nova is None or pm is None:
                self.get_logger().warn('CORRIGE_POSE_MESA: %s: sem pose atual no mapa' % nome)
                return
            dd = math.hypot(nova[0] - pm[0], nova[1] - pm[1])
            da = abs(math.atan2(math.sin(nova[2] - pm[2]), math.cos(nova[2] - pm[2])))
            txt = '%s: AMCL (%.3f, %.3f, %.1f) -> mesa (%.3f, %.3f, %.1f): %.1f cm, %.1f graus%s' % (
                nome, pm[0], pm[1], math.degrees(pm[2]), nova[0], nova[1], math.degrees(nova[2]),
                dd * 100, math.degrees(da), '' if r.get('centro') is not None else ' (centro nao medido: so profundidade/angulo)')
            if dd > 0.40 or da > math.radians(15):
                self.get_logger().warn('CORRIGE_POSE_MESA: NAO corrijo (grande demais, face/mesa duvidosa) ' + txt)
                return
            if dd < 0.02 and da < math.radians(1.0):
                self.get_logger().info('CORRIGE_POSE_MESA: ja certo ' + txt)
                return
            from geometry_msgs.msg import PoseWithCovarianceStamped
            if getattr(self, 'pub_init_mesa', None) is None:
                self.pub_init_mesa = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            m = PoseWithCovarianceStamped()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = nova[0], nova[1]
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(nova[2] / 2), math.cos(nova[2] / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.03 ** 2
            m.pose.covariance[35] = math.radians(2.0) ** 2
            self.pub_init_mesa.publish(m)
            self.get_logger().warn('CORRIGE_POSE_MESA: corrigido ' + txt)
            time.sleep(0.5)

        def _publicar_resultado(self, nome, ok, texto):
"""),
    ("""                    ('%+.1fcm' % (r['centro'] * 100)) if r.get('centro') is not None else 'nd')
                self._publicar_resultado(nome, True, det)
""",
     """                    ('%+.1fcm' % (r['centro'] * 100)) if r.get('centro') is not None else 'nd')
                try:   # CORRIGE_POSE_MESA (24/11): nunca derruba o encosto
                    self._corrigir_pose_mesa(nome, face, r, enc.pc)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error('CORRIGE_POSE_MESA: erro %s' % exc)
                self._publicar_resultado(nome, True, det)
"""),
    ("""            self.declare_parameter('entrada_translada', True)  # ENTRADA_TRANSLADA (24/11)
""",
     """            self.declare_parameter('entrada_translada', True)  # ENTRADA_TRANSLADA (24/11)
            self.declare_parameter('corrige_pose_mesa', True)  # CORRIGE_POSE_MESA (24/11)
"""),
]


def conferir(velho, novo):
    assert novo.count('CORRIGE_POSE_MESA') >= 5
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('corrige_pose_mesa', 'CORRIGE_POSE_MESA', PADRAO, TROCAS, conferir, doc=__doc__)
