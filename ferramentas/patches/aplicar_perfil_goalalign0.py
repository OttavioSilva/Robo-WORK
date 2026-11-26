#!/usr/bin/env python3
"""GOALALIGN0 (11/11/2026) - Patch 05. Gera config/nav2_params_goalalign0.yaml = nav2_params.yaml com UMA
diferenca: controller_server FollowPath GoalAlign.scale 18 -> 0. Nao mexe no nav2_params.yaml, na arvore
(continua a navigate_to_pose_smooth.xml integrada) nem em outra critica. O perfil early_test existente NAO
serve para este A/B: muda tambem sim_time (0,8 -> 1,6) e usa a arvore reduzida (sem recuperacao).
Ativar (janela controlada, Nav2 parado):  ver PATCHES.md (params_file:=.../nav2_params_goalalign0.yaml)
Uso: python3 aplicar_perfil_goalalign0.py [--desfazer]   (desfazer = apaga o perfil gerado)"""
import copy, os, sys, yaml
NAV = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config')
ORIG, PERFIL = os.path.join(NAV, 'nav2_params.yaml'), os.path.join(NAV, 'nav2_params_goalalign0.yaml')
MARCA = '# GOALALIGN0 (11/11): perfil A/B gerado de nav2_params.yaml; so muda GoalAlign.scale 18 -> 0\n'
if '--desfazer' in sys.argv:
    if os.path.exists(PERFIL) and open(PERFIL).read().startswith(MARCA):
        os.remove(PERFIL); print('apagado', PERFIL)
    else:
        print('perfil nao existe (ou nao foi gerado por este script): nada feito')
    sys.exit(0)
if os.path.exists(PERFIL):
    sys.exit('ja existe %s: nada feito (use --desfazer antes para regenerar)' % PERFIL)
txt = open(ORIG).read()
linha = '      GoalAlign.scale: 18.0\n'
if txt.count(linha) != 1:
    sys.exit('ERRO: GoalAlign.scale 18.0 nao encontrado uma vez no nav2_params.yaml')
novo = MARCA + txt.replace(linha, '      GoalAlign.scale: 0.0  # GOALALIGN0 (11/11): era 18.0\n')
a, b = yaml.safe_load(txt), yaml.safe_load(novo)
esp = copy.deepcopy(a)
esp['controller_server']['ros__parameters']['FollowPath']['GoalAlign.scale'] = 0.0
if b != esp:
    sys.exit('ERRO: o perfil difere em mais do que GoalAlign.scale')
open(PERFIL, 'w').write(novo)
print('gerado', PERFIL)
