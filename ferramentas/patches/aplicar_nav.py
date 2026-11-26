#!/usr/bin/env python3
"""Melhorias da missao22 (06/11/2026). Mudancas separadas, cada uma com parametro para desligar.

  sensor2  SENSOR_IMU2: so a IMU falhando nao para o robo antes de 1,5 s e nao espera a ponte. Com os
           motores girando a IMU falha por 0,6 s ou mais (teste de 06/11); a ponte so reinicia se o
           LiDAR cair junto.
  encosto  ENCOSTO_LINEAR: avanco continuo ate a mesa, sem a fila de pulsos. Os ultimos 8 cm levavam
           6-8 s em ate 11 pulsos (missao22) e o Collision Monitor trava em 9,5 cm.
  fundido  ANGULO_FUNDIDO: na saida lateral, IMU + LiDAR num angulo so e correcao andando.
  entrada  ENTRADA_RAPIDA: sem refino no preparo e sem parar na boca se o erro for pequeno.
  vel      VEL_ZONAS: corredor 0,06 (0,035 se apertado ou perto do fim) e Nav2 ate 0,20.
  uma_parede  UMA_PAREDE: angulo so vale com as 2 paredes a vista (missao23).
  deslize  DESLIZE_SEM_GIRO: sem giro andando na saida lateral; acima de 5 graus gira parado (missao23).
  freio    FREIO_FRENTE: a travessia para pelo LiDAR em qualquer modo (missao23: encostou).
  rota     ROTA_MAPA: obstaculo nao muda a rota: mais curta do mapa (missao23: foi reto em vez da curva).

Uso: python3 aplicar_nav.py <mudanca> [--desfazer] [--raiz <copia do src>]
"""
import argparse
import glob
import os
import shutil
import time

import yaml

A = 'robo_navigation/scripts/alinhador_passagem.py'
P = 'robo_navigation/config/nav2_params.yaml'

MUD = {}
MUD['sensor2'] = ('SENSOR_IMU2', A, [
    ("""            return [(n, 99.0 if t is None else agora - t) for n, t in fontes if t is None or agora - t > lim]
""", """            # SENSOR_IMU2: a IMU falha ~0,6 s com os motores girando; o yaw congela pouco tempo
            return [(n, 99.0 if t is None else agora - t) for n, t in fontes
                    if t is None or agora - t > (max(lim, 1.5) if n == 'IMU' else lim)]
"""),
    ("""            ult, imu_caiu = 0.0, False
""", """            ult, imu_caiu = 0.0, False  # SENSOR_IMU2: "imu_caiu" = IMU e LiDAR juntos: serial caiu
"""),
    ("""                imu_caiu = imu_caiu or any(n == 'IMU' for n, _ in parados)
""", """                nomes = [n for n, _ in parados]
                imu_caiu = imu_caiu or ('IMU' in nomes and 'LiDAR' in nomes)
"""),
])
MUD['encosto'] = ('ENCOSTO_LINEAR', A, [
    ("""                 tempo_max=60.0, log=print):
        self.alvo = distancia_alvo
""", """                 tempo_max=60.0, log=print, linear=False):
        self.linear = linear   # ENCOSTO_LINEAR
        self.alvo = distancia_alvo
"""),
    ("""        self.lat_continuo_min = 0.04   # desvio (m) a partir do qual desliza sem parar
""", """        self.lat_continuo_min = 0.02 if linear else 0.04   # desvio (m) a partir do qual desliza sem parar
"""),
    ("""            if previsto <= self.alvo + 0.08 or t - self.t_estado > 15.0:
""", """            # ENCOSTO_LINEAR: vai direto ao alvo (0,10 m/s reais; a previsao para ~1 cm depois do alvo)
            if previsto <= self.alvo + (0.0 if self.linear else 0.08) or t - self.t_estado > 15.0:
"""),
    ("""            if falta <= 0.012 or self.n_pulsos >= 12:
""", """            if falta <= (0.02 if self.linear else 0.012) or self.n_pulsos >= (2 if self.linear else 12):
"""),
    ("""            self.declare_parameter('distancia_encosto', 0.08)   # para-choque ate a face (m)
""", """            self.declare_parameter('distancia_encosto', 0.09)   # ENCOSTO_LINEAR: era 0.08; o CM trava em 9,5 cm
            self.declare_parameter('encosto_linear', True)
"""),
    ("""            enc = Encosto(distancia_alvo=self.p('distancia_encosto'), para_choque=self.p('para_choque'),
""", """            enc = Encosto(linear=self.p('encosto_linear'),
                          distancia_alvo=self.p('distancia_encosto'), para_choque=self.p('para_choque'),
"""),
])
MUD['fundido'] = ('ANGULO_FUNDIDO', A, [
    ("""                 progresso_reset=0.06, seguranca_max=6, log=print):
        self.lado = 1.0 if lado > 0 else -1.0
""", """                 progresso_reset=0.06, seguranca_max=6, log=print, fundido=False, k_ff=0.2):
        self.fundido, self.k_ff = fundido, k_ff   # ANGULO_FUNDIDO
        self.ref, self.ref_ini, self.gyro = None, None, 0.0
        self.lado = 1.0 if lado > 0 else -1.0
"""),
    ("""        self.meds.append((t, med, pose))
        self.meds = self.meds[-12:]
        self.ult_med_pose = pose
""", """        self.meds.append((t, med, pose))
        self.meds = self.meds[-12:]
        self.ult_med_pose = pose
        self._funde(med, pose)

    def _funde(self, med, pose):
        \"\"\"ANGULO_FUNDIDO: ref = rumo do deslize na odom, medido parado antes de sair. A IMU segue o
        giro e a parede so ajusta devagar (constante ~0,7 s), com as 2 paredes a vista e no maximo 3 graus
        da medida inicial. No replay da missao22 o LiDAR indica 7-9 graus por muitos scans seguidos (em 18
        de 19 paradas a IMU nao confirmou): seguir a parede sem limite giraria a quina contra ela.\"\"\"
        if self.ref is None or self.ref_ini is None or med['n'] < 15 or med['lados'] != 2:
            return
        inov = _ang(_ang(pose[2] + med['angulo']) - self.ref)
        if abs(inov) > math.radians(6.0):
            return
        r = _ang(self.ref + 0.25 * inov)
        dif = _ang(r - self.ref_ini)
        lim = math.radians(3.0)
        self.ref = _ang(self.ref_ini + max(-lim, min(lim, dif)))

    def _deslizar_fundido(self, t, pose, restante):
        \"\"\"Corrige rumo e centro andando. Devolve (vx, vy, wz) ou None se precisa parar e medir.\"\"\"
        erro = _ang(self.ref - pose[2]) if self.ref is not None else 0.0
        des = self._centro_folga(t)
        novas = [m for m in self.meds if m[0] >= t - 0.6]
        if des is None and novas and novas[-1][1]['lados'] == 2:
            des = novas[-1][1]['desvio']
        if abs(erro) > math.radians(12.0) or (des is not None and abs(des) > 0.15):
            if self.correcoes < self.correcoes_max:
                self.correcoes += 1
                self.log('Corrigindo parado: %+.1f graus, centro %s' % (
                    math.degrees(erro), '-' if des is None else '%+.1f cm' % (des * 100)))
                return None
        wz = 0.0
        if abs(erro) > math.radians(0.7):
            wz = _sat(1.5 * erro - 0.3 * self.gyro, 0.022, 0.15)
        vy = self.lado * self.v_lat
        vx = -self.k_ff * vy   # deslize torto medido: k 0,05-0,37, para a frente indo para a esquerda
        if des is not None:
            vx += max(-0.04, min(0.04, 0.4 * des))
        return vx, vy, wz
"""),
    ("""            novas = [m for m in self.meds if m[0] >= self.t_estado + 0.3]
            if novas:
                tm, med, p_scan = novas[-1]
                # sem correcao pela odom""", """            if self.fundido:
                _, dy = self._desloc(self.inicio_estado, pose)
                if abs(dy) > self.sem_parede_max and not [m for m in self.meds if m[0] >= self.t_estado] \\
                        and restante > 0.75:
                    self.abortar('andei %.2f m de lado sem ver parede' % abs(dy))
                    return 0.0, 0.0, 0.0
                cmd = self._deslizar_fundido(t, pose, restante)
                if cmd is None:
                    self._mudar('assentar', t, pose)
                    return 0.0, 0.0, 0.0
                return cmd
            novas = [m for m in self.meds if m[0] >= self.t_estado + 0.3]
            if novas:
                tm, med, p_scan = novas[-1]
                # sem correcao pela odom"""),
    ("""        def ao_imu(self, _msg):   # SENSOR_PARADO
            self.t_imu = time.monotonic()
""", """        def ao_imu(self, msg):   # SENSOR_PARADO
            self.t_imu = time.monotonic()
            self.gyro_z = msg.angular_velocity.z   # ANGULO_FUNDIDO
"""),
    ("""            self.declare_parameter('vel_aproximacao', 0.03)""", """            self.declare_parameter('angulo_fundido', True)      # ANGULO_FUNDIDO
            self.declare_parameter('fundido_k_ff', 0.2)
            self.declare_parameter('vel_aproximacao', 0.03)"""),
    ("""            self.t_imu = None   # SENSOR_PARADO
""", """            self.t_imu = None   # SENSOR_PARADO
            self.gyro_z = 0.0
"""),
    ("""            tl = TravessiaLateral(curva['lado'], v_lateral=self.p('vel_lateral'),
""", """            tl = TravessiaLateral(curva['lado'], fundido=self.p('angulo_fundido'), k_ff=self.p('fundido_k_ff'),
                                  v_lateral=self.p('vel_lateral'),
"""),
    ("""                    vx, vy, wz = tl.passo(agora, od[1:], restante)
""", """                    tl.gyro = self.gyro_z
                    vx, vy, wz = tl.passo(agora, od[1:], restante)
"""),
])
MUD['fundido'][2].append(("""        if e == 'deslizar':
            self.p_desl = pose
""", """        if e == 'deslizar':
            self.p_desl = pose
            self.ref = self.ref_ini = self.yaw_ref if self.yaw_ref is not None else pose[2]   # ANGULO_FUNDIDO
"""))
MUD['entrada'] = ('ENTRADA_RAPIDA', A, [
    ("""            if len(vistas) >= 2:  # duas leituras boas seguidas: nao e ruido
""", """            if len(vistas) >= 2 and self.entrada_rapida and vistas[-1][1].get('desvio') is not None \\
                    and abs(vistas[-1][1]['angulo']) <= math.radians(6.0) and abs(vistas[-1][1]['desvio']) <= 0.08:
                # ENTRADA_RAPIDA: erro pequeno: a travessia corrige andando. A margem e so ~10 cm de cada
                # lado do corpo, por isso 8 cm e 6 graus, e nao 15 cm e 12 graus.
                self.yaw_ref = _ang(vistas[-1][2][2] + vistas[-1][1]['angulo'])
                self.log('Paredes vistas: entrando sem parar (%+.1f graus, centro %+.1f cm)' % (
                    math.degrees(vistas[-1][1]['angulo']), vistas[-1][1]['desvio'] * 100))
                self._mudar('atravessar', t, pose)
                return self.v_aprox, 0.0, 0.0
            if len(vistas) >= 2:  # duas leituras boas seguidas: nao e ruido
"""),
    ("""        self.v_aprox = v_aprox
        self.v_trav = v_travessia
""", """        self.entrada_rapida = False   # ENTRADA_RAPIDA: ligada pelo no
        self.v_aprox = v_aprox
        self.v_trav = v_travessia
"""),
    ("""            self.declare_parameter('vel_aproximacao', 0.03)""", """            self.declare_parameter('entrada_rapida', True)      # ENTRADA_RAPIDA
            self.declare_parameter('vel_aproximacao', 0.03)"""),
    ("""            if not self._girar_seguro(e['yaw'], nome):
                self.get_logger().warn('%s: FALHOU (giro)' % nome)
                return 'falha'
""", """            rapida = self.p('entrada_rapida')   # ENTRADA_RAPIDA: basta 10 cm e 8 graus; a travessia acerta o resto
            pm = self._pose_mapa_yaw() or pm
            if not rapida or abs(_ang(e['yaw'] - pm[2])) > math.radians(8.0):
                if not self._girar_seguro(e['yaw'], nome):
                    self.get_logger().warn('%s: FALHOU (giro)' % nome)
                    return 'falha'
"""),
    ("""            for _ in range(3):
                self._deslocar_ate(e['x'], e['y'], nome)
""", """            for _ in range(1 if rapida else 3):
                pm = self._pose_mapa_yaw()
                if rapida and pm is not None and math.hypot(e['x'] - pm[0], e['y'] - pm[1]) <= 0.10:
                    break
                self._deslocar_ate(e['x'], e['y'], nome)
"""),
    ("""                            if dyaw > math.radians(4.0) or dpos > 0.04:
""", """                            rapida = self.p('entrada_rapida')   # ENTRADA_RAPIDA: era 4 graus e 4 cm
                            if dyaw > math.radians(8.0 if rapida else 4.0) or dpos > (0.10 if rapida else 0.04):
"""),
    ("""            trav = Travessia(v_aprox=self.p('vel_aproximacao'), v_travessia=self.p('vel_travessia'),
""", """            trav = Travessia(v_aprox=self.p('vel_aproximacao'), v_travessia=self.p('vel_travessia'),
"""),
    ("""                        od, sc, sq, cf = self.odom, self.scan, self.scan_seq, self.cmd_final
                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO
                    if pausa > 0.0:
                        with self.lock:
                            od = self.odom
                        trav.retomar(""", """                        od, sc, sq, cf = self.odom, self.scan, self.scan_seq, self.cmd_final
                    trav.entrada_rapida = self.p('entrada_rapida')
                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO
                    if pausa > 0.0:
                        with self.lock:
                            od = self.odom
                        trav.retomar("""),
])
MUD['vel'] = ('VEL_ZONAS', A, [
    ("""            vx = self.v_trav
            if abs(err_yaw) > math.radians(4) or abs(err_lat) > 0.035:""", """            vx = self.v_trav
            # VEL_ZONAS: devagar se apertado ou perto do fim: a parada pelo LiDAR foi medida a ~0,11 m/s
            if restante < 0.40 or (f is not None and min(f[1], f[2]) < 0.08):
                vx = min(vx, 0.035)
            if abs(err_yaw) > math.radians(4) or abs(err_lat) > 0.035:"""),
    ("""            self.declare_parameter('vel_travessia', 0.035)
""", """            self.declare_parameter('vel_travessia', 0.06)   # VEL_ZONAS: era 0.035
"""),
])
MUD['nav20'] = ('NAV_VEL20', P, [
    ("      max_vel_x: 0.16  # NAV_VEL16 (era 0.08)\n", "      max_vel_x: 0.20  # NAV_VEL20 (06/11): era 0.16\n"),
    ("      max_speed_xy: 0.16  # NAV_VEL16 (era 0.08)\n", "      max_speed_xy: 0.20  # NAV_VEL20 (06/11): era 0.16\n"),
    ("    max_velocity:\n    - 0.16  # NAV_VEL16 (era 0.08)\n", "    max_velocity:\n    - 0.20  # NAV_VEL20 (06/11): era 0.16\n"),
])

MUD['uma_parede'] = ('UMA_PAREDE', A, [
    ("""            ang = sorted(m[1]['angulo'] for m in novas)[len(novas) // 2]
            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]
            yaw_scan = novas[-1][2][2]
            self.ciclos += 1
""", """            ang = sorted(m[1]['angulo'] for m in novas)[len(novas) // 2]
            if self.uma_parede and novas[-1][1]['lados'] == 1:
                ang = 0.0   # UMA_PAREDE: com uma parede so o angulo oscilou +6/-6 graus e gastou 22 s (missao23)
            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]
            yaw_scan = novas[-1][2][2]
            self.ciclos += 1
"""),
    ("""        self.entrada_rapida = False   # ENTRADA_RAPIDA: ligada pelo no
""", """        self.entrada_rapida = False   # ENTRADA_RAPIDA: ligada pelo no
        self.uma_parede = False       # UMA_PAREDE: ligada pelo no
"""),
    ("""                    trav.entrada_rapida = self.p('entrada_rapida')
""", """                    trav.entrada_rapida = self.p('entrada_rapida')
                    trav.uma_parede = self.p('uma_parede')
"""),
    ("""            ang = sorted(m[1]['angulo'] for m in novas)[len(novas) // 2]
            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]
            yaw_scan = novas[-1][2][2]
            fr = [m[1]['frente'] for m in novas if m[1]['frente'] is not None]
""", """            ang = sorted(m[1]['angulo'] for m in novas)[len(novas) // 2]
            # UMA_PAREDE: angulo so com as 2 paredes. Na missao22, todo angulo > 5 graus veio de uma parede
            # so (298 de 521); na missao23 esses giros, travados pelo CM, gastaram os primeiros 8 s.
            duas = [m for m in novas if m[1]['lados'] == 2]
            if self.fundido:
                ang = sorted(m[1]['angulo'] for m in duas)[len(duas) // 2] if duas else 0.0
            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]
            yaw_scan = novas[-1][2][2]
            fr = [m[1]['frente'] for m in novas if m[1]['frente'] is not None]
"""),
    ("""                if novas[-1][1]['n'] >= 12:
                    self.yaw_ref = _ang(yaw_scan + ang)
                self._mudar('deslizar', t, pose)
""", """                if duas if self.fundido else novas[-1][1]['n'] >= 12:
                    self.yaw_ref = _ang(yaw_scan + ang)
                elif self.fundido:
                    self.yaw_ref = pose[2]
                self._mudar('deslizar', t, pose)
"""),
    ("""            self.declare_parameter('vel_aproximacao', 0.03)""", """            self.declare_parameter('uma_parede', True)          # UMA_PAREDE
            self.declare_parameter('vel_aproximacao', 0.03)"""),
])
MUD['deslize'] = ('DESLIZE_SEM_GIRO', A, [
    ("""        if abs(erro) > math.radians(12.0) or (des is not None and abs(des) > 0.15):
            if self.correcoes < self.correcoes_max:
                self.correcoes += 1
                self.log('Corrigindo parado: %+.1f graus, centro %s' % (
                    math.degrees(erro), '-' if des is None else '%+.1f cm' % (des * 100)))
                return None
        wz = 0.0
        if abs(erro) > math.radians(0.7):
            wz = _sat(1.5 * erro - 0.3 * self.gyro, 0.022, 0.15)
        vy = self.lado * self.v_lat
        vx = -self.k_ff * vy   # deslize torto medido: k 0,05-0,37, para a frente indo para a esquerda
        if des is not None:
            vx += max(-0.04, min(0.04, 0.4 * des))
        return vx, vy, wz
""", """        # DESLIZE_SEM_GIRO: na missao23 o giro andando nao mudou o rumo (-0,35 graus/s com qualquer comando) e
        # fez a frente encolher mais rapido (1,4 -> 2,4-4 cm/s). O rumo fica com a ponte; passou de 5 graus, gira parado.
        if abs(erro) > math.radians(5.0) and t - self.t_estado > 0.3:
            pode = self.correcoes < self.correcoes_max
            if pode or abs(erro) > math.radians(7.0):
                if pode:
                    self.correcoes += 1
                else:
                    self.n_seg += 1
                    if self.n_seg > self.seguranca_max:
                        self.abortar('rumo saindo demais (%d vezes)' % self.n_seg)
                        return 0.0, 0.0, 0.0
                self.alvo = self.ref
                self.pulso = min(0.40, max(0.05, abs(erro) / self.w_real - 0.06))
                self.log('Rumo saiu %+.1f graus: corrigindo parado' % math.degrees(-erro))
                self._mudar('girar', t, pose)
                return 0.0, 0.0, 0.0
        if des is not None and abs(des) > 0.15 and self.correcoes < self.correcoes_max:
            self.correcoes += 1
            self.log('Corrigindo parado: centro %+.1f cm' % (des * 100))
            return None
        vy = self.lado * self.v_lat
        vx = -self.k_ff * vy   # deslize torto medido: k 0,05-0,37, para a frente indo para a esquerda
        if des is not None:
            vx += max(-0.04, min(0.04, 0.4 * des))
        else:
            # uma parede so: afasta pela folga real
            ff, ft = self._folgas(t)
            if ff is not None and ff < 0.09:
                vx -= 0.03
            elif ft is not None and ft < 0.09:
                vx += 0.03
        if abs(vx) < 0.004:
            vx = 0.0
        elif abs(vx) < 0.016:
            vx = math.copysign(0.016, vx)   # o velocity_smoother zera abaixo de 0,015
        return vx, vy, 0.0
"""),
])
MUD['freio'] = ('FREIO_FRENTE', A, [
    ("""    def nova_folgas(self, t, esq, dir_):
""", """    def nova_frente(self, t, folga):   # FREIO_FRENTE
        self.f_corpo = (t, folga)

    def nova_folgas(self, t, esq, dir_):
"""),
    ("""        self.uma_parede = False       # UMA_PAREDE: ligada pelo no
""", """        self.uma_parede = False       # UMA_PAREDE: ligada pelo no
        self.f_corpo = None           # FREIO_FRENTE: (t, folga a frente na largura do corpo)
"""),
    ("""        if e == 'atravessar':
            if self.parada_lidar:
""", """        if e == 'atravessar':
            # FREIO_FRENTE: em qualquer modo. Na missao23 ele seguia para um ponto alem do cruzamento, sem
            # parada pelo LiDAR, e encostou (10:11:03): quem parou foi o CM, ja com a frente a 0 cm.
            fc = self.f_corpo
            if fc is not None and t - fc[0] < 0.5 and fc[1] < 0.10:
                self.abortar('algo a %.0f cm do para-choque' % (fc[1] * 100))
                return 0.0, 0.0, 0.0
            if self.parada_lidar:
"""),
    ("""            if restante < 0.40 or (f is not None and min(f[1], f[2]) < 0.08):
                vx = min(vx, 0.035)
""", """            if restante < 0.40 or (f is not None and min(f[1], f[2]) < 0.08) or (
                    fc is not None and fc[1] < 0.30):
                vx = min(vx, 0.035)
"""),
    ("""                        trav.nova_folgas(sc[0], fe, fd)
""", """                        trav.nova_folgas(sc[0], fe, fd)
                        # FREIO_FRENTE: pontos do corpo ficam em x ~0,31 e y ~0,33: so alem do para-choque
                        trav.nova_frente(sc[0], min([x - 0.365 for x, y in sc[1] if x > 0.385 and abs(y) < 0.30] or [9.0]))
"""),
])
MUD['rota'] = ('ROTA_MAPA', A, [
    ("""import math
""", """import heapq  # ROTA_MAPA
import math
"""),
    ("""def _tangentes(pontos):
""", """def rota_no_mapa(mapa, ini, fim, folga=0.30, folga_ponta=0.12, raio_ponta=0.60):
    \"\"\"ROTA_MAPA: caminho mais curto so pelo mapa, sem obstaculos do momento, onde o corpo cabe. Devolve
    [(x, y), ...] a cada celula, ou None. Celula solta (ruido do SLAM) nao conta como parede.\"\"\"
    w, h, res = mapa.w, mapa.h, mapa.res
    occ = [1 if v >= mapa.limiar else 0 for v in mapa.dados]
    desc = [v < 0 for v in mapa.dados]
    inf = 1e9
    dist = [inf] * (w * h)
    for j in range(1, h - 1):
        for i in range(1, w - 1):
            k = j * w + i
            if occ[k] and sum(occ[k + a * w + b] for a in (-1, 0, 1) for b in (-1, 0, 1)) >= 3:
                dist[k] = 0.0
    d2 = math.sqrt(2.0)
    for j in range(1, h):
        for i in range(1, w - 1):
            k = j * w + i
            dist[k] = min(dist[k], dist[k - 1] + 1.0, dist[k - w] + 1.0, dist[k - w - 1] + d2, dist[k - w + 1] + d2)
    for j in range(h - 2, -1, -1):
        for i in range(w - 2, 0, -1):
            k = j * w + i
            dist[k] = min(dist[k], dist[k + 1] + 1.0, dist[k + w] + 1.0, dist[k + w + 1] + d2, dist[k + w - 1] + d2)

    def celula(x, y):
        dx, dy = x - mapa.ox, y - mapa.oy
        return int(math.floor((mapa.c * dx + mapa.s * dy) / res)), int(math.floor((-mapa.s * dx + mapa.c * dy) / res))

    def mundo(i, j):
        gx, gy = (i + 0.5) * res, (j + 0.5) * res
        return mapa.ox + mapa.c * gx - mapa.s * gy, mapa.oy + mapa.s * gx + mapa.c * gy

    i0, j0 = celula(*ini)
    i1, j1 = celula(*fim)
    if not (0 <= i0 < w and 0 <= j0 < h and 0 <= i1 < w and 0 <= j1 < h):
        return None
    lim, lim_p, r_p = folga / res, folga_ponta / res, raio_ponta / res
    custo = {(i0, j0): 0.0}
    veio = {}
    fila = [(0.0, i0, j0)]
    while fila:
        c, i, j = heapq.heappop(fila)
        if (i, j) == (i1, j1):
            rota = [mundo(i, j)]
            while (i, j) in veio:
                i, j = veio[(i, j)]
                rota.append(mundo(i, j))
            rota.reverse()
            return rota
        if c > custo.get((i, j), inf):
            continue
        for a in (-1, 0, 1):
            for b in (-1, 0, 1):
                ni, nj = i + a, j + b
                if (a == 0 and b == 0) or not (1 <= ni < w - 1 and 1 <= nj < h - 1):
                    continue
                k = nj * w + ni
                ponta = math.hypot(ni - i0, nj - j0) <= r_p or math.hypot(ni - i1, nj - j1) <= r_p
                if desc[k] or dist[k] < (lim_p if ponta else lim):
                    continue
                centro = 1.0 + 1.5 * max(0.0, 1.0 - dist[k] * res / 0.45)   # prefere o meio do corredor
                n = c + math.hypot(a, b) * centro
                if n < custo.get((ni, nj), inf):
                    custo[(ni, nj)] = n
                    veio[(ni, nj)] = (i, j)
                    heapq.heappush(fila, (n, ni, nj))
    return None


def _comp_rota(p):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))


def _tangentes(pontos):
"""),
    ("""            if mapa is None or pontos is None:
                return [], 'sem mapa ou sem caminho'
""", """            # ROTA_MAPA (missao23): com um objeto no corredor norte, o planner mandou seguir reto e dar a volta
            # pelo leste; o robo encostou. Obstaculo nao muda a rota: ele vai pelo caminho mais curto do mapa e
            # contorna la. So na 3a tentativa para o mesmo destino aceita o desvio do planner.
            pm = self._pose_mapa()
            if goal is not None and mapa is not None and pm is not None and self.p('rota_mapa') \\
                    and self.rota_tent.get((round(gx, 1), round(gy, 1)), 0) <= 2:
                rota = rota_no_mapa(mapa, pm, (gx, gy))
                if rota is not None and (pontos is None or _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5):
                    self.get_logger().warn('Planner: %s; mapa: %.1f m: sigo a rota do mapa' % (
                        'sem caminho' if pontos is None else '%.1f m' % _comp_rota(pontos), _comp_rota(rota)))
                    pontos = rota
            if mapa is None or pontos is None:
                return [], 'sem mapa ou sem caminho'
"""),
    ("""        def executar(self, goal, area=None):
            gx, gy = goal.pose.position.x, goal.pose.position.y
""", """        def executar(self, goal, area=None):
            gx, gy = goal.pose.position.x, goal.pose.position.y
            ch = (round(gx, 1), round(gy, 1))   # ROTA_MAPA: tentativas para o mesmo destino
            self.rota_tent = {ch: self.rota_tent.get(ch, 0) + 1}
"""),
    ("""        def _publicar_resultado(self, nome, ok, texto):
""", """        def _publicar_resultado(self, nome, ok, texto):
            if ok:
                self.rota_tent = {}   # ROTA_MAPA
"""),
    ("""            self.t_imu = None   # SENSOR_PARADO
""", """            self.rota_tent = {}   # ROTA_MAPA
            self.t_imu = None   # SENSOR_PARADO
"""),
    ("""            self.declare_parameter('vel_aproximacao', 0.03)""", """            self.declare_parameter('rota_mapa', True)           # ROTA_MAPA
            self.declare_parameter('vel_aproximacao', 0.03)"""),
])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mudanca', choices=sorted(MUD))
    ap.add_argument('--desfazer', action='store_true')
    ap.add_argument('--raiz', default=os.path.expanduser('~/robo_jazzy_ws/src'))
    a = ap.parse_args()
    marca, rel, trocas = MUD[a.mudanca]
    arq = os.path.join(a.raiz, rel)
    suf = '.bak_%s_' % a.mudanca
    if a.desfazer:
        baks = sorted(glob.glob(arq + suf + '*'))
        if not baks:
            print('Nenhum backup %s*.' % suf)
            return 1
        shutil.copy2(baks[-1], arq)
        print('Restaurado de %s' % baks[-1])
        return 0
    txt = open(arq).read()
    if marca in txt:
        print('Ja aplicado (%s).' % marca)
        return 0
    novo = txt
    try:
        for velho, troca in trocas:
            if velho == troca:
                continue
            if novo.count(velho) != 1:
                raise RuntimeError('trecho aparece %d vezes: %r' % (novo.count(velho), velho[:70]))
            novo = novo.replace(velho, troca)
        if arq.endswith('.py'):
            compile(novo, arq, 'exec')
        else:
            yaml.safe_load(novo)
    except Exception as exc:
        print('Nao consegui aplicar (%s). Nada alterado.' % exc)
        return 1
    bak = arq + suf + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(arq, bak)
    with open(arq, 'w') as f:
        f.write(novo)
    print('Aplicado %s (backup %s)' % (marca, os.path.basename(bak)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
