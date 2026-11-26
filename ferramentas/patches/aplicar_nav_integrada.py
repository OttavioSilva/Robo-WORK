#!/usr/bin/env python3
"""NAV_INTEGRADA (10/11/2026): integra a navegacao antecipativa e a recuperacao avancada em src/robo_navigation.

Muda (cada arquivo com backup <arquivo>.bak_navintegrada_<data>):
  config/nav2_params.yaml       FollowPath: RotationShim -> dwb_core::DWBLocalPlanner direto (tira so as 9
                                chaves do Shim; velocidades, aceleracoes, criticos, goal/progress checker,
                                footprint e CM iguais); global_costmap: remove o 1o bloco obstacle_layer
                                duplicado (valia o ultimo: 3,5/4,0 m; o efetivo nao muda).
  behavior_trees/navigate_to_pose_smooth.xml   arvore integrada (copia da anterior em
                                navigate_to_pose_smooth_anterior_20261110.xml).
  scripts/destravar_node.py     DESTRAVAR v4 (usa scripts/recuperacao_avancada.py, ja no src).
  test/test_navigation_profiles.py  o teste do perfil dwb_test passa a comparar com o principal (ja DWB).
  CMakeLists.txt / package.xml  testes novos e dependencias de execucao do destravar v4.

Uso:
  python3 aplicar_nav_integrada.py                    # aplica em ~/robo_jazzy_ws/src/robo_navigation
  python3 aplicar_nav_integrada.py --raiz <copia>     # aplica numa copia (teste do patch)
  python3 aplicar_nav_integrada.py --desfazer [--raiz <dir>]
  python3 aplicar_nav_integrada.py --conferir         # so diz se esta aplicado
Recusa: se a marca ja existe; se um Nav2 rodando usa o nav2_params.yaml principal (missao em curso).
"""
import argparse
import copy
import glob
import os
import py_compile
import shutil
import sys
import time
import xml.etree.ElementTree as ET

import yaml

MARCA = 'NAV_INTEGRADA'
SUF = 'navintegrada'
AQUI = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(AQUI, '..', '..'))
ARQ = os.path.join(WS, 'entregas', 'nav_integrada_20261110', 'arquivos')
RAIZ_PADRAO = os.path.join(WS, 'src', 'robo_navigation')
ANTERIOR = 'behavior_trees/navigate_to_pose_smooth_anterior_20261110.xml'
ALVOS = ['config/nav2_params.yaml', 'behavior_trees/navigate_to_pose_smooth.xml',
         'scripts/destravar_node.py', 'test/test_navigation_profiles.py', 'CMakeLists.txt', 'package.xml']
SHIM = ['primary_controller', 'angular_dist_threshold', 'angular_disengage_threshold',
        'forward_sampling_distance', 'rotate_to_heading_angular_vel', 'max_angular_accel',
        'simulate_ahead_time', 'closed_loop', 'rotate_to_heading_once']


def ler(p):
    with open(p) as f:
        return f.read()


def gravar(p, txt):
    tmp = p + '.tmp_' + SUF
    with open(tmp, 'w') as f:
        f.write(txt)
    os.replace(tmp, p)


def novo_yaml(txt):
    linhas = txt.split('\n')
    out, i, feito_shim, feito_dup = [], 0, False, False
    em_global, blocos_obst = False, 0
    while i < len(linhas):
        ln = linhas[i]
        s = ln.strip()
        if ln.startswith('global_costmap:'):
            em_global = True
        elif ln and not ln.startswith(' ') and not ln.startswith('global_costmap'):
            em_global = False
        if s.startswith('plugin: "nav2_rotation_shim_controller::RotationShimController"'):
            ind = ln[:len(ln) - len(ln.lstrip())]
            out.append(ind + 'plugin: "dwb_core::DWBLocalPlanner"  # NAV_INTEGRADA (10/11): DWB direto; era '
                       'RotationShim (backup .bak_navintegrada_*)')
            i += 1
            while i < len(linhas) and linhas[i].strip().split(':')[0] in SHIM:
                i += 1
            feito_shim = True
            continue
        if em_global and s.startswith('obstacle_layer:') and ln.startswith('      obstacle_layer:'):
            blocos_obst += 1
            if blocos_obst == 1:
                # conta quantos blocos obstacle_layer de mesmo nivel existem no global
                resto = linhas[i + 1:]
                if any(r.startswith('      obstacle_layer:') for r in resto[:80]):
                    j = i + 1
                    while j < len(linhas) and (linhas[j].startswith('        ') or not linhas[j].strip()):
                        j += 1
                    out.append('      # NAV_INTEGRADA (10/11): tirado o 1o bloco obstacle_layer duplicado '
                               '(5,0/5,5 m); valia o ultimo')
                    i = j
                    feito_dup = True
                    continue
        out.append(ln)
        i += 1
    return '\n'.join(out), feito_shim, feito_dup


def esperado_yaml(antes):
    d = copy.deepcopy(antes)
    fp = d['controller_server']['ros__parameters']['FollowPath']
    fp['plugin'] = 'dwb_core::DWBLocalPlanner'
    for k in SHIM:
        del fp[k]
    return d


def novo_teste(txt):
    a = txt.index('def test_dwb_stage_preserves_parameters_and_all_existing_protections')
    b = txt.index('def test_test_tree_fails_without_automatic_motion_recovery')
    novo = '''def test_dwb_stage_preserves_parameters_and_all_existing_protections():
    # NAV_INTEGRADA (10/11): o principal ja e DWB direto; o perfil dwb_test ficou igual a ele.
    original=yaml.safe_load((NAV/'config/nav2_params.yaml').read_text())
    follow=original['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin']=='dwb_core::DWBLocalPlanner'
    for key in ['primary_controller','angular_dist_threshold','angular_disengage_threshold','forward_sampling_distance','rotate_to_heading_angular_vel','max_angular_accel','simulate_ahead_time','closed_loop','rotate_to_heading_once']:assert key not in follow
    stage=yaml.safe_load((NAV/'config/nav2_params_dwb_test.yaml').read_text())
    assert stage==original
    early=yaml.safe_load((NAV/'config/nav2_params_early_test.yaml').read_text())
    follow['GoalAlign.scale']=0.0;follow['sim_time']=1.6
    assert early==original


'''
    return txt[:a] + novo + txt[b:]


def novo_cmake(txt):
    alvo = '  ament_add_pytest_test(giro_integrado test/test_integrado_t.py TIMEOUT 120)\n'
    if alvo not in txt:
        raise SystemExit('CMakeLists.txt: linha de referencia nao encontrada')
    add = ('  # NAV_INTEGRADA (10/11): recuperacao avancada e arvore integrada (offline)\n'
           '  ament_add_pytest_test(recuperacao_avancada test/test_recuperacao_avancada.py TIMEOUT 600)\n'
           '  ament_add_pytest_test(arvore_integrada test/test_arvore_integrada.py TIMEOUT 120)\n'
           '  ament_add_pytest_test(config_integrada test/test_config_integrada.py TIMEOUT 120)\n'
           '  ament_add_pytest_test(destravar_ros_isolado test/test_destravar_ros_isolado.py TIMEOUT 300)\n')
    return txt.replace(alvo, alvo + add)


def novo_pkg(txt):
    alvo = '  <exec_depend>python3-yaml</exec_depend>\n'
    if alvo not in txt:
        raise SystemExit('package.xml: linha de referencia nao encontrada')
    add = ('  <!-- NAV_INTEGRADA (10/11): destravar v4 -->\n'
           + ''.join('  <exec_depend>%s</exec_depend>\n' % d
                     for d in ('python3-numpy', 'nav2_msgs', 'nav_msgs', 'sensor_msgs', 'sensor_msgs_py', 'tf2_ros'))
           + '  <test_depend>python3-pytest</test_depend>\n')
    return txt.replace(alvo, alvo + add)


def nav2_principal_rodando():
    """Algum processo do Nav2 carregou o nav2_params.yaml principal? (missao em curso)"""
    usam = []
    for d in glob.glob('/proc/[0-9]*/cmdline'):
        try:
            c = open(d, 'rb').read().replace(b'\0', b' ').decode(errors='ignore')
        except Exception:
            continue
        if '/nav2_params.yaml' in c and ('controller_server' in c or 'bt_navigator' in c):
            usam.append(c[:140])
    return usam


def processos_destravar():
    out = []
    for d in glob.glob('/proc/[0-9]*/cmdline'):
        try:
            c = open(d, 'rb').read().replace(b'\0', b' ').decode(errors='ignore')
        except Exception:
            continue
        if 'destravar_node.py' in c and 'python' in c:
            out.append(d.split('/')[2])
    return out


def aplicar(raiz):
    for rel in ALVOS:
        p = os.path.join(raiz, rel)
        if not os.path.exists(p):
            raise SystemExit('nao achei %s' % p)
        if MARCA in ler(p):
            raise SystemExit('%s ja tem a marca %s: nada a fazer (use --desfazer antes)' % (rel, MARCA))
    if not os.path.exists(os.path.join(raiz, 'scripts', 'recuperacao_avancada.py')):
        raise SystemExit('falta scripts/recuperacao_avancada.py no src')
    for f in ('navigate_to_pose_smooth.xml', 'destravar_node.py'):
        if MARCA not in ler(os.path.join(ARQ, f)) and f.endswith('.xml'):
            raise SystemExit('arquivo preparado sem a marca: %s' % f)

    # ---- monta tudo em memoria e confere antes de gravar
    novos = {}
    y0 = ler(os.path.join(raiz, 'config/nav2_params.yaml'))
    y1, ok_shim, ok_dup = novo_yaml(y0)
    if not (ok_shim and ok_dup):
        raise SystemExit('YAML: nao achei o RotationShim (%s) ou o bloco duplicado (%s)' % (ok_shim, ok_dup))
    antes, depois = yaml.safe_load(y0), yaml.safe_load(y1)
    if depois != esperado_yaml(antes):
        raise SystemExit('YAML: o resultado mudou mais do que o FollowPath; recusado')
    g0 = antes['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
    g1 = depois['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
    if g0 != g1 or g1['scan']['obstacle_max_range'] != 3.5:
        raise SystemExit('YAML: o obstacle_layer efetivo do global mudaria; recusado')
    if y1.count('      obstacle_layer:') != 2:   # um no local, um no global
        raise SystemExit('YAML: esperava 1 obstacle_layer no local e 1 no global')
    novos['config/nav2_params.yaml'] = y1

    x1 = ler(os.path.join(ARQ, 'navigate_to_pose_smooth.xml'))
    ET.fromstring(x1)
    novos['behavior_trees/navigate_to_pose_smooth.xml'] = x1

    d1 = ler(os.path.join(ARQ, 'destravar_node.py'))
    tmpd = os.path.join('/tmp', 'destravar_check_%d.py' % os.getpid())
    with open(tmpd, 'w') as f:
        f.write(d1)
    py_compile.compile(tmpd, doraise=True)
    os.remove(tmpd)
    novos['scripts/destravar_node.py'] = d1

    novos['test/test_navigation_profiles.py'] = novo_teste(ler(os.path.join(raiz, 'test/test_navigation_profiles.py')))
    novos['CMakeLists.txt'] = novo_cmake(ler(os.path.join(raiz, 'CMakeLists.txt')))
    p1 = novo_pkg(ler(os.path.join(raiz, 'package.xml')))
    ET.fromstring(p1)
    novos['package.xml'] = p1

    # ---- grava com backup
    data = time.strftime('%Y%m%d_%H%M%S')
    ant = os.path.join(raiz, ANTERIOR)
    if not os.path.exists(ant):
        shutil.copy2(os.path.join(raiz, 'behavior_trees/navigate_to_pose_smooth.xml'), ant)
    for rel, txt in novos.items():
        p = os.path.join(raiz, rel)
        shutil.copy2(p, '%s.bak_%s_%s' % (p, SUF, data))
        modo = os.stat(p).st_mode
        gravar(p, txt)
        os.chmod(p, modo)
    print('NAV_INTEGRADA aplicado em %s (backups .bak_%s_%s)' % (raiz, SUF, data))
    print('  arvore anterior preservada em %s' % ANTERIOR)


def desfazer(raiz):
    feitos = 0
    for rel in ALVOS:
        p = os.path.join(raiz, rel)
        baks = sorted(glob.glob('%s.bak_%s_*' % (p, SUF)))
        if not baks:
            print('  %s: sem backup' % rel)
            continue
        if MARCA not in ler(p):
            print('  %s: sem a marca, deixo como esta' % rel)
            continue
        shutil.copy2(baks[-1], p)
        feitos += 1
        print('  %s <- %s' % (rel, os.path.basename(baks[-1])))
    print('NAV_INTEGRADA desfeito em %d arquivo(s). A copia %s e scripts/recuperacao_avancada.py ficam '
          '(nao sao usados pela versao antiga).' % (feitos, ANTERIOR))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--raiz', default=RAIZ_PADRAO)
    ap.add_argument('--desfazer', action='store_true')
    ap.add_argument('--conferir', action='store_true')
    ap.add_argument('--forcar', action='store_true', help='aplica mesmo com Nav2 principal rodando')
    a = ap.parse_args()
    raiz = os.path.abspath(a.raiz)
    if a.conferir:
        for rel in ALVOS:
            p = os.path.join(raiz, rel)
            print('%-45s %s' % (rel, 'APLICADO' if os.path.exists(p) and MARCA in ler(p) else 'original'))
        return
    if raiz == RAIZ_PADRAO and not a.forcar:
        usam = nav2_principal_rodando()
        if usam:
            raise SystemExit('RECUSADO: o Nav2 em execucao usa o nav2_params.yaml principal (missao em curso?):\n  '
                             + '\n  '.join(usam) + '\nPare a navigacao antes, ou use --forcar.')
    if a.desfazer:
        desfazer(raiz)
    else:
        aplicar(raiz)
    if raiz == RAIZ_PADRAO:
        pids = processos_destravar()
        if pids:
            print('AVISO: destravar_node.py em execucao (pid %s) continua com o codigo antigo ate reiniciar '
                  'o lancamento.' % ', '.join(pids))
        print('Depois: cd ~/robo_jazzy_ws && colcon build --packages-select robo_navigation && source install/setup.bash')


if __name__ == '__main__':
    sys.exit(main())
