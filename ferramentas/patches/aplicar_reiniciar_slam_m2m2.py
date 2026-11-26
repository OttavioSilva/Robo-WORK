#!/usr/bin/env python3
"""REINICIAR_SLAM_M2M2 (25/11/2026): Y 2x do teleop com M2M2 so como sensor + slam_toolbox.

Antes: se o servico /m2m2_bridge/limpar_mapa existia, o Y SEMPRE ia pelo mapa do M2M2.
Com base_m2m2_slam.launch.py (publish_map=false, mapa do slam_toolbox) isso tentava recarregar
um .stcm ("failed to open input file") e o mapa do slam_toolbox nunca reiniciava.
Agora: so usa o caminho do M2M2 se a ponte tem publish_map=true; senao segue o caminho do
slam_toolbox (mapping: zera odometria + reset; localization: deserialize).

Uso: aplicar_reiniciar_slam_m2m2.py [--arquivo X] [--desfazer]
"""
import argparse
import glob
import os
import py_compile
import shutil
import sys
import time

MARCA = 'REINICIAR_SLAM_M2M2'
PADRAO = os.path.expanduser('~/robo_jazzy_ws/ferramentas/teleop/teleop_mapeamento_joy.py')

TROCAS = [
    ("        req.names = ['mapeando', 'arquivo_mapa']\n",
     "        req.names = ['mapeando', 'arquivo_mapa']\n"),  # ancora de conferencia (sem mudanca)
    ("    def reiniciar():\n        parar_robo()\n        if cli_m2_limpar.wait_for_service(timeout_sec=0.5):\n",
     "    def m2m2_dono_do_mapa():\n"
     "        # REINICIAR_SLAM_M2M2 (25/11): era so 'servico limpar_mapa existe'. Com publish_map=false\n"
     "        # (base_m2m2_slam.launch.py) o mapa e do slam_toolbox.\n"
     "        if not cli_m2_limpar.wait_for_service(timeout_sec=0.5):\n"
     "            return False\n"
     "        req = GetParameters.Request()\n"
     "        req.names = ['publish_map']\n"
     "        r, _ = chamar(cli_m2_param, req, 3.0)\n"
     "        return bool(r.values[0].bool_value) if r is not None and r.values else True\n"
     "\n"
     "    def reiniciar():\n        parar_robo()\n        if m2m2_dono_do_mapa():\n"),
    # REINICIAR_SLAM_M2M2 v2: o costmap global do Nav2 guardava os obstaculos do mapa velho (RViz "sujo")
    ("        log('REINICIADO: mapa novo com origem aqui; START = (0, 0, 0); mesas e FINISH apagados.')\n",
     "        log('REINICIADO: mapa novo com origem aqui; START = (0, 0, 0); mesas e FINISH apagados.')\n"
     "        # REINICIAR_SLAM_M2M2 v2 (25/11): limpa os costmaps (obstaculos do mapa velho)\n"
     "        from nav2_msgs.srv import ClearEntireCostmap\n"
     "        for c in ('global', 'local'):\n"
     "            cli_c = node.create_client(ClearEntireCostmap, '/%s_costmap/clear_entirely_%s_costmap' % (c, c))\n"
     "            rc, errc = chamar(cli_c, ClearEntireCostmap.Request(), 5.0)\n"
     "            log('REINICIAR: costmap %s %s' % (c, 'limpo' if rc is not None else 'NAO limpo (%s)' % errc))\n"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arquivo', default=PADRAO)
    ap.add_argument('--desfazer', action='store_true')
    a = ap.parse_args()
    f = a.arquivo
    if a.desfazer:
        baks = sorted(glob.glob(f + '.bak_reiniciar_slam_m2m2_*'))
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
            sys.exit('trecho nao encontrado exatamente 1x: %r' % antes[:60])
        novo = novo.replace(antes, depois)
    tmp = f + '.tmp_patch'
    open(tmp, 'w').write(novo)
    try:
        py_compile.compile(tmp, doraise=True)
    finally:
        pass
    bak = f + '.bak_reiniciar_slam_m2m2_' + time.strftime('%Y%m%d_%H%M%S')
    shutil.copy2(f, bak)
    shutil.copymode(f, tmp)
    os.replace(tmp, f)
    print('aplicado; backup', bak)


if __name__ == '__main__':
    main()
