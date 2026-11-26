#!/usr/bin/env python3
"""SMAC_LATTICE (24/11/2026): planner global que confere o CORPO inteiro (troca o NavFn).

Simulacao (24/11, pontos de toque): o robo cortava quinas (mesa central, ilhas, shelf a 45 graus,
toco do START) porque o NavFn so ve o raio inscrito (0,215 m) e traca rente; o canto do corpo
0,59 x 0,65 varria a quina na curva. Pesos de custo (QUINA_FOLGA no global, DWB_QUINA no local)
pioraram outros trechos e foram desfeitos.
Aqui: plugin GridBased (mesmo nome: alinhador, destravar e arvore continuam iguais) passa a ser o
nav2_smac_planner::SmacPlannerLattice com as primitivas OMNI de 5 cm (mecanum anda de lado) e
checagem do footprint retangular do costmap global em cada pose. Sem re na expansao.
Uso: aplicar_smac_lattice.py | --desfazer | --arquivo <copia>   (depois: colcon build robo_navigation)
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml')
PRIMITIVAS = '/opt/ros/jazzy/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json'

TROCAS = [
    ("""    GridBased:
      plugin: nav2_navfn_planner::NavfnPlanner
      tolerance: 0.1
      use_astar: false
      allow_unknown: true
""",
     """    GridBased:  # SMAC_LATTICE (24/11): era nav2_navfn_planner::NavfnPlanner (tolerance 0.1, use_astar false)
      plugin: nav2_smac_planner::SmacPlannerLattice
      allow_unknown: true
      tolerance: 0.10
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 3.0
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      analytic_expansion_max_cost: 200.0
      analytic_expansion_max_cost_override: false
      reverse_penalty: 2.0
      change_penalty: 0.05
      non_straight_penalty: 1.05
      cost_penalty: 2.0
      rotation_penalty: 5.0
      retrospective_penalty: 0.015
      lattice_filepath: "%s"
      lookup_table_size: 20.0
      cache_obstacle_heuristic: false
      allow_reverse_expansion: false
      smooth_path: true
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
        do_refinement: true
        refinement_num: 2
""" % PRIMITIVAS),
]


def conferir(velho, novo):
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    gb = b['planner_server']['ros__parameters']['GridBased']
    assert gb['plugin'] == 'nav2_smac_planner::SmacPlannerLattice'
    assert os.path.isfile(gb['lattice_filepath']), gb['lattice_filepath']
    b['planner_server'] = a['planner_server']
    assert a == b, 'mudou fora do planner_server'


if __name__ == '__main__':
    rodar('smac_lattice', 'SMAC_LATTICE', PADRAO, TROCAS, conferir, doc=__doc__)
