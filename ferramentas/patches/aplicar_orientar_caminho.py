#!/usr/bin/env python3
"""ORIENTAR_CAMINHO (13/11/2026) - destravar: giro de ORIENTACAO para o caminho, separado da varredura.

Bag arena_start_20261113_112926 (11:41:14-11:41:46): robo em (2,1; 5,1) virado para +90, goal atras. O destravar
decidiu "giro +40 graus para o caminho (+162)": o giro era o do smart_spin (VARREDURA: max 40 graus e para na 1a
abertura frontal), que parou com 7 graus (IMU +9). Depois recusou "retomar: caminho para tras (+162): o DWB nao da
re" e falhou. O mesmo aconteceu as 11:41:53 (+7 graus). O robo nunca ficou de frente para o caminho.

Agora existem dois giros diferentes:
  - 'orientar' (novo; so com caminho ATUAL do planner): gira o angulo que falta para ficar de frente para o caminho (ate 180 pelo lado curto; se o
    lado curto esta barrado e o outro livre, pelo outro lado, giro acumulado ate giro_orientar_max = 360). Antes de
    girar: area varrida inteira livre pelo LiDAR (margem 4,5 cm) e observada >= cobertura_giro, poligono 'stopped'
    do CM livre. Durante: o proprio destravar gira (0,20 rad/s; 0,15 perto do fim ou com pouca folga), reavalia o
    arco dos proximos 30 graus a cada scan novo e para no CM, cancelamento, comando estrangeiro, sensor parado ou
    IMU sem progresso. Nunca inverte o sentido. Resultado medido pelo yaw da IMU (acumulado: passa de 180).
  - 'giro' (como antes): varredura pelo smart_spin. So e escolhido se o 'orientar' nao couber.
Lateral, mover e retomar continuam ANTES do giro (contorno convencional primeiro). Ninguem anda mais rapido: o giro
usa 0,15-0,20 rad/s (o mesmo do DWB; o parametro e limitado a esse intervalo).
v2 ORIENTAR_ROBUSTO (teste 13/11 13:17:33, sem bag; logs do no): o destravar escolheu "orientar +164" mas a checagem
DURANTE o giro (1 ponto num scan tirado girando) parou com 6 graus, embora a foto parada desse a area livre. Agora,
quando a checagem em movimento acusa algo: para, tira foto PARADA e reavalia o arco que falta com o criterio do
inicio; livre -> continua (ate giro_orientar_reavaliar = 3 vezes), barrado -> para. O CM continua parando na hora.
Em tempo de execucao: ros2 param set /destravar giro_orientar false   (volta ao comportamento anterior)
                      ros2 param set /destravar giro_orientar_reavaliar 0   (v1: aborta no 1o ponto em movimento)
Uso: python3 aplicar_orientar_caminho.py [--raiz <copia do workspace>] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_multi import rodar  # noqa: E402

MARCA = 'ORIENTAR_CAMINHO (13/11)'
RA = 'src/robo_navigation/scripts/recuperacao_avancada.py'
NO = 'src/robo_navigation/scripts/destravar_node.py'

TROCAS_RA = [
    # ---- Config
    ("    continuacao: float = 0.15          # o corpo precisa andar isto na direcao do caminho depois\n",
     "    continuacao: float = 0.15          # o corpo precisa andar isto na direcao do caminho depois\n"
     "    # ORIENTAR_CAMINHO (13/11): giro de orientacao para o caminho (False = como antes: so a varredura de 40 graus)\n"
     "    giro_orientar: bool = False\n"
     "    giro_orientar_max: float = 2 * math.pi      # giro acumulado maximo (pelo outro lado: 360 - |angulo|)\n"
     "    giro_orientar_wz: float = 0.20              # rad/s (o DWB usa ate 0,20; o motor gira parado no minimo ~0,15)\n"
     "    giro_orientar_tol: float = math.radians(20.0)   # sobra aceita no fim (o DWB acerta o resto andando)\n"
     "    giro_orientar_reavaliar: int = 0            # v2 ORIENTAR_ROBUSTO: paradas p/ reavaliar o arco (0 = v1: aborta)\n"),
    # ---- decidir: antes da varredura
    ("    # ---- ETAPA F: giro (pelo smart_spin)\n",
     "    # ---- ORIENTAR_CAMINHO (13/11): giro de ORIENTACAO (ficar de frente para o caminho). A varredura do smart_spin\n"
     "    # (abaixo) para na 1a abertura frontal e no maximo 40 graus: no bag de 13/11 11:41 girou 7 de 162 graus.\n"
     "    # So com caminho ATUAL do planner (skill robo-work-nav-melhoria: nada de giro por rota nao validada).\n"
     "    if cfg.giro_orientar and cfg.permitir_giro and ang is not None and abs(ang) >= math.radians(45):\n"
     "        if not es.planner_ok:\n"
     "            rec.append(('orientar', 'planner sem caminho agora: nao giro por direcao antiga'))\n"
     "        elif 'orientar' in es.bloqueadas:\n"
     "            rec.append(('orientar', 'ja tentei orientar aqui sem resultado'))\n"
     "        elif cm_barraria(pts, 'stopped'):\n"
     "            rec.append(('orientar', 'o Collision Monitor barraria (ponto a < 3 cm do corpo)'))\n"
     "        else:\n"
     "            mot = []\n"
     "            longo = ang - math.copysign(2 * math.pi, ang)\n"
     "            for g, lado in ((ang, 'pelo lado curto'), (longo, 'pelo outro lado')):\n"
     "                if abs(g) > cfg.giro_orientar_max + 1e-9:\n"
     "                    mot.append('%+.0f graus %s: acima do limite de %.0f' % (\n"
     "                        math.degrees(g), lado, math.degrees(cfg.giro_orientar_max)))\n"
     "                    continue\n"
     "                av = avaliar(es.raios, [(0.0, 0.0, g)], margem=0.045, cobertura_min=cfg.cobertura_giro)\n"
     "                if av.livre:\n"
     "                    return Decisao('orientar', (0.0, 0.0, g), 'gira %+.0f graus %s para ficar de frente para o '\n"
     "                                   'caminho (%+.0f); area %.0f%% observada' % (\n"
     "                                       math.degrees(g), lado, math.degrees(ang), 100 * av.cobertura), rec)\n"
     "                mot.append('%+.0f graus %s: %s' % (math.degrees(g), lado, av.motivo))\n"
     "            rec.append(('orientar', '; '.join(mot)))\n"
     "\n"
     "    # ---- ETAPA F: giro (pelo smart_spin)\n"),
    # ---- primitiva de giro
    ("    # ---------------- a chamada\n",
     "    def girar_orientar(self, g, t_fim):\n"
     "        \"\"\"ORIENTAR_CAMINHO (13/11): gira g rad parado (sentido fixo), medindo pelo yaw da IMU ACUMULADO.\n"
     "        A cada scan novo confere o arco dos proximos 30 graus (corpo + 3 cm) pelo LiDAR. Para por: chegou (falta\n"
     "        < 3 graus), cancelamento, tempo, sensor parado, comando estrangeiro, controlador comandando, Collision\n"
     "        Monitor (na hora), LiDAR no arco seguinte ou IMU sem progresso por 2 s. -> (motivo, girou_rad)\"\"\"\n"
     "        io, cfg = self.io, self.cfg\n"
     "        sentido = 1.0 if g > 0 else -1.0\n"
     "        yaw_ant = io.odom()[3]\n"
     "        girou, ult_scan, motivo = 0.0, None, 'chegou'\n"
     "        reav = 0                           # v2 ORIENTAR_ROBUSTO: paradas para reavaliar o arco\n"
     "        wz = 0.15\n"
     "        t_ini = io.agora()\n"
     "        t_prog = t_ini\n"
     "        t_lim = min(t_ini + 3.0 + abs(g) / 0.10, t_fim + 10.0)\n"
     "        while True:\n"
     "            agora = io.agora()\n"
     "            od = io.odom()\n"
     "            d = ang_norm(od[3] - yaw_ant)\n"
     "            yaw_ant = od[3]\n"
     "            girou += d\n"
     "            if abs(d) > 0.003:\n"
     "                t_prog = agora\n"
     "            falta = g - girou\n"
     "            if falta * sentido <= math.radians(3.0):\n"
     "                break\n"
     "            if io.cancelado():\n"
     "                motivo = 'cancelado'\n"
     "                break\n"
     "            if agora >= t_lim:\n"
     "                motivo = 'tempo'\n"
     "                break\n"
     "            if not self._sensores_ok():\n"
     "                motivo = 'sensor parado'\n"
     "                break\n"
     "            e = io.ultimo_estrangeiro()\n"
     "            if e is not None and agora - e < 0.3:\n"
     "                motivo = 'comando estrangeiro em /cmd_vel_nav'\n"
     "                break\n"
     "            c = io.ultimo_ctrl()\n"
     "            if c is not None and agora - c < 0.3:\n"
     "                motivo = 'controlador comandando'\n"
     "                break\n"
     "            if io.cm_stop():\n"
     "                motivo = 'Collision Monitor parou'\n"
     "                break\n"
     "            if agora - t_prog > 2.0 and agora - t_ini > 2.0:\n"
     "                motivo = 'IMU sem progresso angular (giro nao comecou ou travou)'\n"
     "                break\n"
     "            sc = io.scan()\n"
     "            if sc.t != ult_scan:\n"
     "                ult_scan = sc.t\n"
     "                prox = sentido * min(abs(falta), math.radians(30.0))\n"
     "                av = avaliar(sc, [(0.0, 0.0, prox)], margem=0.03)\n"
     "                if not av.livre and reav < cfg.giro_orientar_reavaliar:\n"
     "                    # v2 ORIENTAR_ROBUSTO (teste 13/11 13:17:34): 1 ponto num scan tirado GIRANDO derrubou o giro\n"
     "                    # com 6 de 164 graus, embora a foto parada desse a area livre. Agora: para, foto PARADA e\n"
     "                    # reavalia o arco que FALTA com o mesmo criterio do inicio. O CM continua parando na hora.\n"
     "                    reav += 1\n"
     "                    io.parar()\n"
     "                    foto = self.foto(t_lim)\n"
     "                    od = io.odom()\n"
     "                    girou += ang_norm(od[3] - yaw_ant)\n"
     "                    yaw_ant = od[3]\n"
     "                    falta = g - girou\n"
     "                    if foto is None:\n"
     "                        motivo = 'LiDAR: %s no arco seguinte (sem foto parada para reavaliar)' % av.motivo\n"
     "                        break\n"
     "                    av2 = avaliar(foto, [(0.0, 0.0, falta)], margem=0.045, cobertura_min=cfg.cobertura_giro)\n"
     "                    io.log('info' if av2.livre else 'warn',\n"
     "                           'ORIENTAR_ROBUSTO %d: girando vi %s; parado, o arco que falta (%+.0f graus) %s' % (\n"
     "                               reav, av.motivo, math.degrees(falta),\n"
     "                               'esta livre: continuo' if av2.livre else 'NAO esta livre (%s): paro' % av2.motivo))\n"
     "                    if not av2.livre:\n"
     "                        motivo = 'LiDAR (parado): %s no arco que falta' % av2.motivo\n"
     "                        break\n"
     "                    ult_scan, t_prog, t_ini = foto.t, io.agora(), io.agora()\n"
     "                    continue\n"
     "                if not av.livre:\n"
     "                    motivo = 'LiDAR: %s no arco seguinte' % av.motivo\n"
     "                    break\n"
     "                folgado = avaliar(sc, [(0.0, 0.0, prox)], margem=0.07).livre\n"
     "                wz = cfg.giro_orientar_wz if folgado and abs(falta) > math.radians(15.0) else 0.15\n"
     "            io.enviar(0.0, 0.0, sentido * wz)\n"
     "            io.dormir(0.05)\n"
     "        io.parar()\n"
     "        io.dormir(0.3)                     # o motor ainda gira um pouco depois do zero\n"
     "        girou += ang_norm(io.odom()[3] - yaw_ant)\n"
     "        return motivo, girou\n"
     "\n"
     "    # ---------------- a chamada\n"),
    # ---- execucao
    ("            if dec.acao == 'giro':\n"
     "                ok_mov, txt_mov = io.smart_spin(dec.mov[2], max(3.0, min(12.0, t_fim - io.agora() - 2.0)))\n",
     "            if dec.acao == 'orientar':      # ORIENTAR_CAMINHO (13/11): o proprio destravar gira\n"
     "                motivo, self._orientar_girou = self.girar_orientar(dec.mov[2], t_fim)\n"
     "                ok_mov = motivo == 'chegou'\n"
     "                motivo = 'orientar: ' + motivo\n"
     "            elif dec.acao == 'giro':\n"
     "                ok_mov, txt_mov = io.smart_spin(dec.mov[2], max(3.0, min(12.0, t_fim - io.agora() - 2.0)))\n"),
    ("            foto1 = self.foto(t_fim + 2.0)\n",
     "            foto1 = self.foto(t_fim + 2.0 if dec.acao != 'orientar' else max(t_fim, io.agora()) + 2.0)\n"),
    # ---- verificacao (IMU acumulada; o ang_norm de o1-o0 perde o que passa de 180 graus)
    ("            if dec.acao == 'giro':\n"
     "                dyaw = ang_norm(o1[3] - o0[3])\n",
     "            if dec.acao == 'orientar':\n"
     "                girou = self._orientar_girou\n"
     "                falta = dec.mov[2] - girou\n"
     "                ok = girou * dec.mov[2] > 0 and abs(falta) <= cfg.giro_orientar_tol\n"
     "                evid = 'IMU: girou %+.0f de %+.0f graus (falta %+.0f)' % (\n"
     "                    math.degrees(girou), math.degrees(dec.mov[2]), math.degrees(falta))\n"
     "            elif dec.acao == 'giro':\n"
     "                dyaw = ang_norm(o1[3] - o0[3])\n"),
]

TROCAS_NO = [
    # Ancoras proprias: NAO tocar nas linhas de RE_LIBERADA/RE_LONGA (o --desfazer deles precisa achar o texto)
    ("                self.declare_parameter(nome, valor)\n"
     "            self.pts = None\n",
     "                self.declare_parameter(nome, valor)\n"
     "            # ORIENTAR_CAMINHO (13/11): giro de orientacao para o caminho (false = como antes)\n"
     "            for nome, valor in (('giro_orientar', True), ('giro_orientar_max_graus', 360.0),\n"
     "                                ('giro_orientar_wz', 0.20),\n"
     "                                # v2 ORIENTAR_ROBUSTO: paradas para reavaliar o arco (0 = v1: aborta no 1o ponto)\n"
     "                                ('giro_orientar_reavaliar', 3)):\n"
     "                self.declare_parameter(nome, valor)\n"
     "            self.pts = None\n"),
    ("            self.gh_atual = gh\n"
     "            res = DriveOnHeading.Result()\n",
     "            # ORIENTAR_CAMINHO (13/11): wz limitado a 0,15-0,20 (nunca mais rapido que o DWB)\n"
     "            cfg.giro_orientar = bool(self.get_parameter('giro_orientar').value)\n"
     "            cfg.giro_orientar_max = math.radians(max(0.0, min(360.0, float(\n"
     "                self.get_parameter('giro_orientar_max_graus').value))))\n"
     "            cfg.giro_orientar_wz = max(0.15, min(0.20, float(self.get_parameter('giro_orientar_wz').value)))\n"
     "            cfg.giro_orientar_reavaliar = max(0, min(5, int(self.get_parameter('giro_orientar_reavaliar').value)))\n"
     "            self.gh_atual = gh\n"
     "            res = DriveOnHeading.Result()\n"),
]

if __name__ == '__main__':
    rodar('orientar_caminho', MARCA, [(RA, TROCAS_RA), (NO, TROCAS_NO)], doc=__doc__)
