#!/usr/bin/env python3
"""Tira do LiDAR os pontos que caem DENTRO do corpo do robo (/scan_front -> /scan_nav). FILTRO_NAN v4 (MEMORIA_PERTO v2)

v4 (05/11, missao19): a memoria do ponto cego criava obstaculos FANTASMAS. Ela guarda pontos no
frame do odom; quando o robo se mexe sem a odom ver (deriva de lado, patinada, pulsos curtos), a
copia velha de uma parede/mesa fica no lugar errado, mais perto que a de verdade, e nunca era
apagada enquanto estivesse no ponto cego. Na missao19: na saida lateral (11:34) 3 pontos
lembrados a 4-5 cm da frente, com a parede de verdade a 18 cm, travaram o "afastando +6,7 cm"
7 vezes (CURVA FALHOU); no encosto da WS_1 (11:25) copias de 16-18 s da face da mesa a 5 cm do
para-choque (a face estava a 11 cm) seguraram o encosto 41 s ate o tempo maximo. Agora:
  - se o LiDAR enxerga ALEM do ponto lembrado nos 3 raios dele (o feixe passa por ali e bate
    mais longe), o ponto nao esta la: e apagado. Um obstaculo de verdade no ponto cego bloqueia
    o feixe (o raio volta sem retorno), entao continua lembrado;
  - por raio vale a copia MAIS NOVA (empate de 0,5 s: a mais perto), nao a mais perto de todas.

v3 (05/11, missao17): MEMORIA_PERTO. O Delta-2D nao devolve nada a menos de ~0,15 m (nas missoes
14-16, 1,8 milhao de raios: o menor foi 0,146 m). Com o LiDAR em x = 0,33, um obstaculo no meio
da frente some quando chega a ~11 cm do para-choque: o poligono do Collision Monitor (6 cm a
frente) fica todo dentro do ponto cego, o raio volta "sem retorno" e, com o INF_VALIDO, ainda
apaga o obstaculo do costmap. Resultado: o robo encostava no obstaculo como se nao tivesse visto.
Agora o filtro guarda (no frame do odom) os pontos vistos a menos de 0,5 m do LiDAR e, quando um
deles entra no ponto cego (< 0,17 m do LiDAR, fora do corpo), devolve ele no /scan_nav. Fora do
ponto cego, um ponto guardado que o LiDAR deixou de ver (o raio passa alem dele) e esquecido.
O /scan_nav sai com range_min 0,02 para o CM e o costmap aceitarem esses pontos perto.

v2 (04/11): os pontos tirados viram NaN (antes inf). Com o costmap em inf_is_valid: true
(INF_VALIDO), um inf limparia o costmap na direcao do ponto ate 4-5,5 m; NaN e ignorado.

Por que (prova3, 27/10 08:30-08:32 e 08:36): o LiDAR, a 4 cm do chao, enxergou 3-4 pontos
embaixo do proprio robo, parados no mapa enquanto o robo girava (um objeto no chao que o
robo passou por cima, ex. um cabo). Com >= 3 pontos dentro do poligono o Collision Monitor
da STOP e o costmap marca obstaculo em cima do robo: nada mais se mexe (giro, re, escape),
mesmo em area aberta. Um obstaculo de verdade nao pode estar dentro do robo; o que aparece
la e autorretorno ou algo embaixo do chassi, que o robo ja passou por cima.

O Collision Monitor e o costmap local passam a usar /scan_nav. O SLAM e o alinhador
continuam com /scan_front.

Seguranca: se este no parar, o Collision Monitor fica sem scan e para o robo
(source_timeout 1.0 s). Rode sempre junto com o Nav2.

Uso:
    python3 filtro_corpo_scan.py
    python3 filtro_corpo_scan.py --ros-args -p margem:=0.01
"""
import math

FRENTE, TRAS, MEIA = 0.375, -0.215, 0.325  # CORPO_REAL_M35: envelope fisico, sem ampliar para fora


def dentro_do_robo(x, y, margem=0.0):
    """Ponto (base_link) dentro do retangulo do robo encolhido de 'margem'."""
    return (TRAS + margem) < x < (FRENTE - margem) and abs(y) < (MEIA - margem)


def filtrar(ranges, angle_min, angle_inc, laser, margem=0.0):
    """Devolve (ranges novos, quantos removidos). laser = (x, y, yaw) do LiDAR em base_link."""
    lx, ly, lth = laser
    novo = list(ranges)
    removidos = 0
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
        a = angle_min + i * angle_inc + lth
        if dentro_do_robo(lx + r * math.cos(a), ly + r * math.sin(a), margem):
            novo[i] = float('nan')   # v2 (FILTRO_NAN): NaN nao limpa nada; inf limparia atras do robo
            removidos += 1
    return novo, removidos


class MemoriaPerto:
    """v3: lembra pontos perto do LiDAR e devolve os que cairam no ponto cego. Sem ROS (testavel)."""

    def __init__(self, cego=0.17, perto=0.50, celula=0.02, idade_max=120.0, margem=0.0, folga_alem=0.03):
        self.cego, self.perto, self.cel, self.idade_max, self.margem = cego, perto, celula, idade_max, margem
        self.folga_alem = folga_alem   # v4
        self.pts = {}   # (i, j) -> (x_odom, y_odom, t)

    def passo(self, t, pose, ranges, angle_min, angle_inc, laser):
        """pose = (x, y, yaw) do base_link no odom. Devolve [(indice, alcance)] para injetar."""
        lx, ly, lth = laser
        px, py, pth = pose
        c, s = math.cos(pth), math.sin(pth)
        n = len(ranges)
        # 1) guarda o que o LiDAR ve perto agora
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r > self.perto:
                continue
            a = angle_min + i * angle_inc + lth
            bx, by = lx + r * math.cos(a), ly + r * math.sin(a)
            if dentro_do_robo(bx, by, self.margem):
                continue
            ox, oy = px + c * bx - s * by, py + s * bx + c * by
            self.pts[(int(round(ox / self.cel)), int(round(oy / self.cel)))] = (ox, oy, t)
        # 2) confere cada ponto guardado
        injetar = {}
        for k, (ox, oy, tk) in list(self.pts.items()):
            dx, dy = ox - px, oy - py
            bx, by = c * dx + s * dy, -s * dx + c * dy
            if dentro_do_robo(bx, by, self.margem) or t - tk > self.idade_max:
                del self.pts[k]
                continue
            rx, ry = bx - lx, by - ly
            rl = math.hypot(rx, ry)
            if rl > self.perto + 0.10:
                del self.pts[k]
                continue
            al = math.atan2(ry, rx) - lth
            idx = int(round((math.atan2(math.sin(al - angle_min), math.cos(al - angle_min)) % (2 * math.pi)) / angle_inc))
            if not 0 <= idx < n:
                continue
            if rl > self.cego:
                # visivel: se o LiDAR enxerga alem dele (ou nada) nessa direcao, o obstaculo saiu
                r = ranges[idx]
                if t - tk > 0.5 and (not math.isfinite(r) or r > rl + 0.05):
                    del self.pts[k]
                continue
            # v4: no ponto cego, o feixe que passa pelo ponto e bate mais longe prova que ele saiu
            if t - tk > 0.5 and self._ve_alem(ranges, idx, rl):
                del self.pts[k]
                continue
            # v4: por raio vale a copia mais nova (empate de 0,5 s: a mais perto)
            cur = injetar.get(idx)
            if cur is None or tk > cur[0] + 0.5 or (abs(tk - cur[0]) <= 0.5 and rl < cur[1]):
                injetar[idx] = (tk, rl)
        return sorted((i, v[1]) for i, v in injetar.items())

    def _ve_alem(self, ranges, idx, rl):
        """v4: os 3 raios em volta de idx tem retorno valido mais longe que o ponto (+ folga)."""
        n = len(ranges)
        for j in (idx - 1, idx, idx + 1):
            if not 0 <= j < n:
                return False
            r = ranges[j]
            if not math.isfinite(r) or r <= rl + self.folga_alem:
                return False
        return True


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener

    class Filtro(Node):
        def __init__(self):
            super().__init__('filtro_corpo_scan')
            self.declare_parameter('entrada', '/scan_front')
            self.declare_parameter('saida', '/scan_nav')
            self.declare_parameter('margem', 0.0)
            # MEMORIA_OFF (19/11 noite, pedido do usuario): memoria do ponto cego desligada por padrao.
            # Ela fazia copias da face da mesa/parede aparecerem mais perto que a real (odom no encosto e no
            # deslize) e seguravam o CM (missao19: encosto WS_1 41 s; 12/11: 4,3 cm com a parede a 14).
            # Religar na hora: ros2 param set /filtro_corpo_scan memoria_perto true
            self.declare_parameter('memoria_perto', False)
            self.laser = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.pub = self.create_publisher(LaserScan, self.get_parameter('saida').value,
                                             qos_profile_sensor_data)
            self.create_subscription(LaserScan, self.get_parameter('entrada').value, self.ao_scan,
                                     qos_profile_sensor_data)
            self.n_scans = self.n_com_objeto = 0
            self.memoria = MemoriaPerto()
            self.odom = None
            self.n_injetados = 0
            self.create_subscription(Odometry, '/odom', self.ao_odom, 20)
            self.create_timer(10.0, self.resumo)
            self.get_logger().info('Filtro do corpo: %s -> %s; MEMORIA_PERTO %s' % (
                self.get_parameter('entrada').value, self.get_parameter('saida').value,
                'LIGADA' if self.get_parameter('memoria_perto').value else 'DESLIGADA (MEMORIA_OFF)'))

        def ao_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

        def ao_scan(self, msg):
            if self.laser is None:
                try:
                    tr = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
                    q = tr.transform.rotation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                    self.laser = (tr.transform.translation.x, tr.transform.translation.y, yaw)
                    self.get_logger().info('LiDAR em base_link: x=%.3f y=%.3f yaw=%.1f graus' % (
                        self.laser[0], self.laser[1], math.degrees(yaw)))
                except Exception:
                    return  # sem TF ainda: nao publica (o CM para o robo, que e o seguro)
            ranges, removidos = filtrar(msg.ranges, msg.angle_min, msg.angle_increment, self.laser,
                                        self.get_parameter('margem').value)
            usar_memoria = bool(self.get_parameter('memoria_perto').value)   # MEMORIA_OFF
            if not usar_memoria and self.memoria.pts:
                self.memoria.pts.clear()   # religada depois, comeca sem copias velhas
            if usar_memoria and self.odom is not None:
                import time as _t
                for idx, rl in self.memoria.passo(_t.monotonic(), self.odom, msg.ranges, msg.angle_min,
                                                  msg.angle_increment, self.laser):
                    for j in (idx - 1, idx, idx + 1):   # 3 raios: o CM exige 3 pontos
                        if 0 <= j < len(ranges) and not (math.isfinite(ranges[j]) and ranges[j] < rl):
                            ranges[j] = rl
                    self.n_injetados += 1
                msg.range_min = min(msg.range_min, 0.02)
            msg.ranges = ranges
            self.pub.publish(msg)
            self.n_scans += 1
            if removidos >= 3:
                self.n_com_objeto += 1

        def resumo(self):
            if self.n_injetados:
                self.get_logger().info('MEMORIA_PERTO: %d pontos no ponto cego do LiDAR devolvidos ao /scan_nav '
                                       '(%d guardados)' % (self.n_injetados, len(self.memoria.pts)))
                self.n_injetados = 0
            if self.n_com_objeto:
                self.get_logger().warn('%d de %d scans com objeto EMBAIXO do robo (removido do CM/costmap). '
                                       'Cabo no chao?' % (self.n_com_objeto, self.n_scans))
            self.n_scans = self.n_com_objeto = 0

    rclpy.init()
    no = Filtro()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
