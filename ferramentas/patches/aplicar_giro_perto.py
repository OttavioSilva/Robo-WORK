#!/usr/bin/env python3
"""GIRO_PERTO (25/11/2026): goal SEM AREA (RViz) a <= so_girar_perto do robo: so gira, nao anda.

Caso 11:06:13: goal do RViz (0,20; 0,07) yaw -96 com o robo em ~(0,05; 0,04). A intencao era
girar no lugar, mas o ponto ficou 14 cm ao lado: o TRAJETO CURTO girou e depois deslizou de lado
ate o ponto (operador apertou a emergencia).
Agora, so para goal sem area (area None) e a <= so_girar_perto (padrao 0,15 m): _girar_seguro
para o yaw do goal e termina. Mesas, START/FINISH e docking NAO mudam. O _girar_seguro continua
podendo andar o minimo se a quina nao passa no giro (protecao do CM preservada).
Parametro: so_girar_perto (m; 0 = como antes).  ros2 param set /alinhador_passagem so_girar_perto 0.0

Uso: aplicar_giro_perto.py [--arquivo X] [--desfazer]
"""
import argparse
import glob
import os
import py_compile
import shutil
import sys
import time

MARCA = 'GIRO_PERTO'
PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    ("            self.declare_parameter('curto_max', 1.0)           # v3.23: trajeto curto sem Nav2 (m; 0 desliga)\n",
     "            self.declare_parameter('curto_max', 1.0)           # v3.23: trajeto curto sem Nav2 (m; 0 desliga)\n"
     "            # GIRO_PERTO (25/11): goal sem area a <= isto do robo so gira (RViz); 0 = como antes\n"
     "            self.declare_parameter('so_girar_perto', 0.15)\n"),
    ("                resultado = self._trajeto_curto(goal, livre)\n",
     "                # GIRO_PERTO (25/11): goal sem area colado no robo = intencao de girar no lugar\n"
     "                pm_gp = self._pose_mapa_yaw()\n"
     "                lim_gp = float(self.p('so_girar_perto'))\n"
     "                if area is None and lim_gp > 0.0 and pm_gp is not None and \\\n"
     "                        math.hypot(gx - pm_gp[0], gy - pm_gp[1]) <= lim_gp:\n"
     "                    yaw_gp = yaw_de(goal.pose.orientation)\n"
     "                    self.get_logger().info('GIRO_PERTO: goal a %.0f cm (<= %.0f): so giro para %.0f graus, sem andar'\n"
     "                                           % (100 * math.hypot(gx - pm_gp[0], gy - pm_gp[1]), 100 * lim_gp,\n"
     "                                              math.degrees(yaw_gp)))\n"
     "                    ok_gp = self._girar_seguro(yaw_gp, 'GIRO_PERTO')\n"
     "                    self.get_logger().info('GIRO_PERTO: %s' % ('SUCESSO' if ok_gp else 'nao terminou o giro'))\n"
     "                    return\n"
     "                resultado = self._trajeto_curto(goal, livre)\n"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arquivo', default=PADRAO)
    ap.add_argument('--desfazer', action='store_true')
    a = ap.parse_args()
    f = a.arquivo
    if a.desfazer:
        baks = sorted(glob.glob(f + '.bak_giro_perto_*'))
        if not baks:
            sys.exit('sem backup para desfazer')
        shutil.copy2(baks[-1], f)
        print('restaurado', baks[-1])
        return
    txt = open(f).read()
    if MARCA in txt:
        print('ja aplicado (%s): nada feito' % MARCA)
        return
    novo = txt
    for antes, depois in TROCAS:
        if novo.count(antes) != 1:
            sys.exit('trecho nao encontrado exatamente 1x: %r' % antes[:70])
        novo = novo.replace(antes, depois)
    tmp = f + '.tmp_giro_perto'
    open(tmp, 'w').write(novo)
    py_compile.compile(tmp, doraise=True)
    bak = f + '.bak_giro_perto_' + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(f, bak)
    shutil.copymode(f, tmp)
    os.replace(tmp, f)
    print('aplicado; backup', bak)


if __name__ == '__main__':
    main()
