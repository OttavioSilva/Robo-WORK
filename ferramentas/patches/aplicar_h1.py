#!/usr/bin/env python3
"""H1 da missao22 (06/11/2026): nao decidir nem andar com sensor parado. Tres mudancas separadas.

  cm      CM_TIMEOUT: Collision Monitor para com 0,5 s sem scan (era 1,0). Normal: 0,17 s; um scan
          perdido da 0,34 s. Com 0,3 s ele pararia a cada scan perdido (15 vezes na missao22).
  odom    IMU_CONTINUA: yaw da odom continuo quando a IMU volta depois de falhar. Na reconexao a
          placa zera a IMU: o yaw da odom saltou 30,8 -> 0 graus as 08:36:22,20.
  sensor  SENSOR_PARADO: alinhador e destravar param e esperam com scan, odom ou IMU velhos. Na missao22
          a CURVA tomou "sem scan" por "paredes acabaram" e deu SUCESSO (08:35:57-08:36:10).

Uso (a partir daqui mesmo):
    python3 aplicar_h1.py cm|odom|sensor              # aplica (backup antes)
    python3 aplicar_h1.py cm|odom|sensor --desfazer   # volta o backup mais recente
    python3 aplicar_h1.py cm|odom|sensor --raiz <copia do src>   # testa numa copia
Depois: colcon build (cm e odom) e reiniciar o robo.
"""
import argparse
import glob
import os
import shutil
import time

import yaml

A = 'robo_navigation/scripts/alinhador_passagem.py'
D = 'robo_navigation/scripts/destravar_node.py'
P = 'robo_navigation/config/nav2_params.yaml'
O = 'robo_localization/scripts/command_imu_odometry_node.py'

GUARDA = """                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO
                    if pausa > 0.0:
                        with self.lock:
                            od = self.odom
                        %s.retomar(pausa, time.monotonic(), od[1:])
                        bloqueado_desde = None
                        sem_tf_desde = None
                        continue
"""

RETOMAR = """    def retomar(self, pausa, t, pose):
        \"\"\"SENSOR_PARADO: sensores voltaram. O tempo parado nao conta e mede tudo de novo.\"\"\"
        if self.t0 is None or self.estado in ('fim', 'abortar'):
            return
        self.t0 += pausa
        if self.estado == 'aproximar':
            self.t_estado += pausa
            return
        self._mudar('assentar', t, pose)

"""

METODOS = """        def _sensor_parado(self, idade=None):
            \"\"\"SENSOR_PARADO: lista de (sensor, idade) acima do limite. O limite padrao e 0,5 s: um scan
            perdido da 0,34 s (5,9 Hz). A /odom nao basta: sem IMU ela segue a 50 Hz com o yaw congelado.\"\"\"
            lim = self.p('sensor_idade_max') if idade is None else idade
            agora = time.monotonic()
            with self.lock:
                sc, od = self.scan, self.odom
            fontes = (('LiDAR', sc[0] if sc is not None else None),
                      ('odom', od[0] if od is not None else None), ('IMU', self.t_imu))
            return [(n, 99.0 if t is None else agora - t) for n, t in fontes if t is None or agora - t > lim]

        def _esperar_sensores(self, nome):
            \"\"\"Se algum sensor parou: para o robo e espera voltar. Devolve os segundos parados (0 = tudo
            bem). Goal novo levanta Cancelado; sem voltar em sensor_espera_max, levanta SensorParado.\"\"\"
            parados = self._sensor_parado()
            if not parados:
                return 0.0
            t0 = time.monotonic()
            ult, imu_caiu = 0.0, False
            self._parar()
            while parados:
                if self._novo_goal_chegou():
                    raise Cancelado()
                agora = time.monotonic()
                imu_caiu = imu_caiu or any(n == 'IMU' for n, _ in parados)
                if agora - ult >= 2.0:
                    ult = agora
                    self.get_logger().warn('%s: %s parado ha %.1f s: parei' % (nome, parados[0][0], parados[0][1]))
                if agora - t0 > self.p('sensor_espera_max'):
                    raise SensorParado('%s parado ha %.0f s: desisti' % (parados[0][0], agora - t0))
                self._publicar_vel(0.0, 0.0, 0.0)
                time.sleep(0.1)
                parados = self._sensor_parado()
            if imu_caiu and time.monotonic() - t0 >= 1.0:
                # a ponte reconectou: ignora comandos por 5 s (serial_startup_delay_s)
                fim = time.monotonic() + self.p('sensor_espera_ponte')
                self.get_logger().info('%s: IMU voltou: espero a ponte aceitar comandos' % nome)
                while time.monotonic() < fim:
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    self._publicar_vel(0.0, 0.0, 0.0)
                    time.sleep(0.1)
            pausa = time.monotonic() - t0
            self.get_logger().info('%s: sensores voltaram depois de %.1f s: medindo de novo' % (nome, pausa))
            return pausa

"""

MUDANCAS = {
    'cm': dict(marca='CM_TIMEOUT', arq=P, trocas=[(
        "    source_timeout: 1.0\n",
        "    source_timeout: 0.5  # CM_TIMEOUT (06/11): era 1.0; normal 0,17 s, scan perdido 0,34 s\n")]),
    'odom': dict(marca='IMU_CONTINUA', arq=O, trocas=[(
        "        self.raw_imu_yaw = yaw\n        if self.yaw_origin is None:\n",
        "        # IMU_CONTINUA (06/11): a placa reconectou e zerou a IMU (missao22: 30,8 -> 0 graus). Mantem o\n"
        "        # yaw publicado; o SLAM corrige o que girou durante a falha. Olha 3 s, se o zero vier depois.\n"
        "        agora_imu = time.monotonic()\n"
        "        if self.yaw_origin is not None and self.last_imu_time is not None:\n"
        "            if agora_imu - self.last_imu_time > 1.0:\n"
        "                self.imu_janela_fim = agora_imu + 3.0\n"
        "                self.yaw_origin = normalize_angle(yaw - self.current_yaw())\n"
        "                self.get_logger().warning(\n"
        "                    'IMU voltou depois de %.1f s: yaw da odom mantido'\n"
        "                    % (agora_imu - self.last_imu_time))\n"
        "            elif agora_imu < self.imu_janela_fim and \\\n"
        "                    abs(normalize_angle(yaw - self.raw_imu_yaw)) > math.radians(5.0):\n"
        "                self.yaw_origin = normalize_angle(yaw - self.current_yaw())\n"
        "                self.get_logger().warning('IMU zerada depois de voltar: yaw da odom mantido')\n"
        "        self.raw_imu_yaw = yaw\n        if self.yaw_origin is None:\n"), (
        "        self.raw_imu_yaw = 0.0\n",
        "        self.raw_imu_yaw = 0.0\n        self.imu_janela_fim = 0.0  # IMU_CONTINUA\n")]),
    'sensor': dict(marca='SENSOR_PARADO', arq=A, extra=D, trocas=[
        ("    from sensor_msgs.msg import LaserScan\n    from std_msgs.msg import String\n",
         "    from sensor_msgs.msg import Imu, LaserScan  # SENSOR_PARADO\n    from std_msgs.msg import String\n"),
        ("    class Cancelado(Exception):\n",
         "    class SensorParado(Exception):  # SENSOR_PARADO\n        pass\n\n    class Cancelado(Exception):\n"),
        ("            self.declare_parameter('vel_aproximacao', 0.03)",
         "            self.declare_parameter('sensor_idade_max', 0.5)     # SENSOR_PARADO\n"
         "            self.declare_parameter('sensor_espera_max', 60.0)\n"
         "            self.declare_parameter('sensor_espera_ponte', 6.0)\n"
         "            self.declare_parameter('vel_aproximacao', 0.03)"),
        ("            self.create_subscription(Odometry, '/odom', self.ao_odom, 20, callback_group=grupo)\n",
         "            self.t_imu = None   # SENSOR_PARADO\n"
         "            self.create_subscription(Imu, '/imu/data', self.ao_imu, qos_profile_sensor_data,\n"
         "                                     callback_group=grupo)\n"
         "            self.create_subscription(Odometry, '/odom', self.ao_odom, 20, callback_group=grupo)\n"),
        ("        def ao_plano(self, msg):\n",
         "        def ao_imu(self, _msg):   # SENSOR_PARADO\n            self.t_imu = time.monotonic()\n\n"
         "        def ao_plano(self, msg):\n"),
        ("        def _sensores(self, idade_scan=0.6):\n", METODOS + "        def _sensores(self, idade_scan=0.6):\n"),
        # maquinas
        ("    def abortar(self, motivo):\n        self.estado, self.motivo = 'abortar', motivo\n"
         "        self.log('Travessia LiDAR abortada",
         RETOMAR + "    def abortar(self, motivo):\n        self.estado, self.motivo = 'abortar', motivo\n"
         "        self.log('Travessia LiDAR abortada"),
        ("    def abortar(self, motivo):\n        # v3.18:", RETOMAR + "    def abortar(self, motivo):\n        # v3.18:"),
        ("    def abortar(self, motivo):\n        self.estado, self.motivo = 'abortar', motivo\n"
         "        self.log('Encosto abortado",
         RETOMAR + "    def abortar(self, motivo):\n        self.estado, self.motivo = 'abortar', motivo\n"
         "        self.log('Encosto abortado"),
        # lacos
        ("                    if od is None or agora - od[0] > 0.5:\n                        trav.abortar(",
         GUARDA % 'trav' + "                    if od is None or agora - od[0] > 0.5:\n                        trav.abortar("),
        ("                    if od is None or agora - od[0] > 0.5:\n                        tl.abortar(",
         GUARDA % 'tl' + "                    if od is None or agora - od[0] > 0.5:\n                        tl.abortar("),
        ("                    if od is None or agora - od[0] > 0.5:\n                        enc.abortar(",
         (GUARDA % 'enc').replace("                        sem_tf_desde = None\n", "")
         + "                    if od is None or agora - od[0] > 0.5:\n                        enc.abortar("),
        # _andar_reto
        ("            od0, _ = self._sensores()\n            if od0 is None:\n                return 0.0\n",
         "            self._esperar_sensores(nome)   # SENSOR_PARADO\n"
         "            od0, _ = self._sensores()\n            if od0 is None:\n                return 0.0\n"),
        ("                        raise Cancelado()\n                    od, sc = self._sensores()\n"
         "                    if od is None:\n                        break\n",
         "                        raise Cancelado()\n"
         "                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO\n"
         "                    if pausa > 0.0:\n                        t0 += pausa\n                        continue\n"
         "                    od, sc = self._sensores()\n                    if od is None:\n                        break\n"),
        # _girar_seguro
        ("            pm = self._pose_mapa_yaw()\n            od, sc = self._sensores()\n"
         "            if pm is None or od is None:\n                self.get_logger().warn('%s: sem pose/odom/LiDAR para girar'",
         "            self._esperar_sensores(nome)   # SENSOR_PARADO\n"
         "            pm = self._pose_mapa_yaw()\n            od, sc = self._sensores()\n"
         "            if pm is None or od is None:\n                self.get_logger().warn('%s: sem pose/odom/LiDAR para girar'"),
        ("                    od, sc = self._sensores()\n                    if od is None:\n                        self._parar()\n"
         "                        self.get_logger().warn('%s: sem odom/LiDAR recente' % nome)\n",
         "                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO\n"
         "                    if pausa > 0.0:\n                        t0 += pausa\n                        continue\n"
         "                    od, sc = self._sensores()\n                    if od is None:\n                        self._parar()\n"
         "                        self.get_logger().warn('%s: sem odom/LiDAR recente' % nome)\n"),
        # _recuar
        ("                        raise Cancelado()\n                    with self.lock:\n"
         "                        od, cf = self.odom, self.cmd_final\n                    if math.hypot(od[1] - x0",
         "                        raise Cancelado()\n"
         "                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO\n"
         "                    if pausa > 0.0:\n                        t0 += pausa\n"
         "                        bloqueado_desde = None\n                        continue\n"
         "                    with self.lock:\n"
         "                        od, cf = self.odom, self.cmd_final\n                    if math.hypot(od[1] - x0"),
        # _navegar
        ("                self._virar_para_caminho(alvo, nome)\n            req = NavigateToPose.Goal()\n",
         "                self._virar_para_caminho(alvo, nome)\n"
         "            self._esperar_sensores(nome)   # SENSOR_PARADO\n            req = NavigateToPose.Goal()\n"),
        ("                    r = self._esperar(futuro, 0.2)\n                    if r is not None:\n                        break\n",
         "                    r = self._esperar(futuro, 0.2)\n                    if r is not None:\n                        break\n"
         "                    # SENSOR_PARADO: o CM ja segura com 0,5 s; com 1 s tira o goal do Nav2, senao ele\n"
         "                    # gasta a recuperacao sem dados (missao22 08:36:16,94 a 08:36:17,21)\n"
         "                    if self._sensor_parado(1.0):\n"
         "                        h.cancel_goal_async()\n"
         "                        self._esperar(futuro, 5.0)\n"
         "                        self.handle_ativo = None\n"
         "                        self._esperar_sensores(nome)\n"
         "                        return self._navegar(alvo, nome, vigia, aceitar_perto, False, _cisc)\n"),
    ], trocas_extra=[
        ("            self.ult_contorno = None  # CONTORNAR: (t, x, y) na odom\n",
         "            self.ult_contorno = None  # CONTORNAR: (t, x, y) na odom\n"
         "            self.t_aviso_sensor = 0.0  # SENSOR_PARADO\n"),
        ("        def vel(self, vx, vy, wz=0.0):\n",
         "        def vel(self, vx, vy, wz=0.0):\n"
         "            # SENSOR_PARADO (06/11): scan ou odom com mais de 0,5 s: nao manda movimento\n"
         "            agora = time.monotonic()\n"
         "            pts, od = self.pts, self.odom\n"
         "            if (vx or vy or wz) and (pts is None or od is None or agora - pts[0] > 0.5 or agora - od[0] > 0.5):\n"
         "                vx, vy, wz = 0.0, 0.0, 0.0\n"
         "                if agora - self.t_aviso_sensor > 2.0:\n"
         "                    self.t_aviso_sensor = agora\n"
         "                    self.get_logger().warn('DESTRAVAR: sensor parado: parei')\n"),
        ("time.monotonic() - pts[0] > 1.0 or time.monotonic() - od[0] > 0.5:",
         "time.monotonic() - pts[0] > 0.5 or time.monotonic() - od[0] > 0.5:  # SENSOR_PARADO: era 1,0"),
    ]),
}


def aplica(caminho, trocas, marca, chave):
    suf = '.bak_%s_' % chave
    txt = open(caminho).read()
    if marca in txt:
        print('Ja aplicado em %s (achei %s). Nada a fazer.' % (caminho, marca))
        return None
    novo = txt
    for velho, troca in trocas:
        if novo.count(velho) != 1:
            raise RuntimeError('%s: trecho aparece %d vezes (esperado 1): %r' % (caminho, novo.count(velho), velho[:70]))
        novo = novo.replace(velho, troca)
    if caminho.endswith('.py'):
        compile(novo, caminho, 'exec')
    else:
        a, b = yaml.safe_load(txt), yaml.safe_load(novo)
        cm_a, cm_b = a['collision_monitor']['ros__parameters'], b['collision_monitor']['ros__parameters']
        if cm_a.pop('source_timeout') != 1.0 or cm_b.pop('source_timeout') != 0.5 or a != b:
            raise RuntimeError('o yaml mudou mais do que source_timeout')
    return novo, caminho + suf + time.strftime('%Y%m%d_%H%M%S')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mudanca', choices=sorted(MUDANCAS))
    ap.add_argument('--desfazer', action='store_true')
    ap.add_argument('--raiz', default=os.path.expanduser('~/robo_jazzy_ws/src'))
    a = ap.parse_args()
    m = MUDANCAS[a.mudanca]
    alvos = [(os.path.join(a.raiz, m['arq']), m['trocas'])]
    if 'extra' in m:
        alvos.append((os.path.join(a.raiz, m['extra']), m['trocas_extra']))
    if a.desfazer:
        for arq, _ in alvos:
            baks = sorted(glob.glob(arq + '.bak_%s_*' % a.mudanca))
            if not baks:
                print('Nenhum backup para %s.' % arq)
                return 1
            shutil.copy2(baks[-1], arq)
            print('Restaurado de %s' % baks[-1])
        return 0
    try:
        prontos = [(arq, aplica(arq, tr, m['marca'], a.mudanca)) for arq, tr in alvos]
    except Exception as exc:
        print('Nao consegui aplicar (%s). Nada alterado.' % exc)
        return 1
    for arq, r in prontos:
        if r is not None:
            shutil.copy2(arq, r[1])
            with open(arq, 'w') as f:
                f.write(r[0])
            print('Aplicado %s em %s (backup %s)' % (m['marca'], arq, r[1]))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
