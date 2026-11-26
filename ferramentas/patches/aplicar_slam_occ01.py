#!/usr/bin/env python3
"""SLAM_OCC01 (11/11/2026) - slam_mapping.yaml: occupancy_threshold 0.2 -> 0.1 (padrao do slam_toolbox).
Relato do usuario no remapeamento de 11/11: "demora muito a considerar um ponto". Em 03/11 o limiar foi de 0.1 para 0.2
(backup slam_mapping.yaml.bak_occ01). A celula so vira ocupada quando >= limiar das passagens de raio por ela foram
batidas (com min_pass_through 2). Com 31-38% dos raios do Delta-2D em inf e o robo derrapando, as paredes demoram a
aparecer com 0.2.
So muda o /map gerado (o casamento de scan nao usa esse limiar). Como o grid e refeito a partir dos scans do posegraph,
vale tambem para mapas ja salvos quando o SLAM sobe de novo. Efeito colateral: mais pontinhos de coisas que passaram.
Depois: colcon build --packages-select robo_navigation e subir o SLAM de novo.
Uso: python3 aplicar_slam_occ01.py [--arquivo copia.yaml] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/slam_mapping.yaml')
TROCAS = [("""    occupancy_threshold: 0.2
""", """    occupancy_threshold: 0.1  # SLAM_OCC01 (11/11): era 0.2 (desde 03/11); 0.1 = padrao do slam_toolbox
""")]


def conferir(velho, novo):
    import yaml
    a = yaml.safe_load(velho)['slam_toolbox']['ros__parameters']
    b = yaml.safe_load(novo)['slam_toolbox']['ros__parameters']
    dif = {k for k in set(a) | set(b) if a.get(k) != b.get(k)}
    if dif != {'occupancy_threshold'} or b['occupancy_threshold'] != 0.1:
        sys.exit('ERRO: mudou mais do que occupancy_threshold: %s' % sorted(dif))


if __name__ == '__main__':
    rodar('slam_occ01', 'SLAM_OCC01 (11/11)', ARQ, TROCAS, conferir=conferir, doc=__doc__)
