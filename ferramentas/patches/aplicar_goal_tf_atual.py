#!/usr/bin/env python3
"""GOAL_TF_ATUAL (11/11/2026) - correcao do giro travado no goal (bag 2nav 12:37:22-41). NAO APLICADO por padrao.

Uma mudanca so: o FollowPath passa a receber o caminho com carimbo de tempo zero (no CaminhoSemCarimbo,
biblioteca robo_caminho_sem_carimbo_bt_node). Efeito: o goal checker do controller_server converte o fim do
caminho com o map->odom MAIS RECENTE, igual ao DWB. Nenhum parametro do Nav2 muda.
Arquivos: behavior_trees/navigate_to_pose_smooth.xml e config/nav2_params.yaml (bt_navigator.plugin_lib_names).
Uso:
  python3 aplicar_goal_tf_atual.py                       # aplica no workspace
  python3 aplicar_goal_tf_atual.py --desfazer            # troca reversa
  python3 aplicar_goal_tf_atual.py --arquivo X.xml --yaml Y.yaml   # testar numa copia
"""
import argparse, copy, datetime, re, shutil, sys
import xml.etree.ElementTree as ET
import os, yaml
MARCA = 'GOAL_TF_ATUAL (11/11)'
WS = os.path.expanduser('~/robo_jazzy_ws')
XML = os.path.join(WS, 'src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml')
YML = os.path.join(WS, 'src/robo_navigation/config/nav2_params.yaml')
LIB = 'robo_caminho_sem_carimbo_bt_node'
VELHO = '        <FollowPath path="{smoothed_path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}" />\n'
NOVO = ('        <!-- GOAL_TF_ATUAL (11/11): o FollowPath recebe o caminho com carimbo zero; o goal checker do\n'
        '             controller_server passa a usar o map->odom mais recente, igual ao DWB (giro travado 12:37). -->\n'
        '        <CaminhoSemCarimbo name="CaminhoParaOControlador" input_path="{smoothed_path}" output_path="{caminho_controlador}" />\n'
        '        <FollowPath path="{caminho_controlador}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}" />\n')
COM_LISTA = re.compile(r'(    plugin_lib_names:\n(?:    - .*\n)+)')
SEM_LISTA = '    wait_for_service_timeout: 1000\n'


def bak(p):
    d = '%s.bak_goal_tf_atual_%s' % (p, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(p, d)
    return d


def conferir_yaml(a_txt, b_txt, esperado_lista):
    a, b = yaml.safe_load(a_txt), yaml.safe_load(b_txt)
    e = copy.deepcopy(a)
    e['bt_navigator']['ros__parameters']['plugin_lib_names'] = esperado_lista
    if not esperado_lista:
        e['bt_navigator']['ros__parameters'].pop('plugin_lib_names')
    if b != e:
        sys.exit('ERRO: o yaml mudou alem de bt_navigator.plugin_lib_names')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arquivo', default=XML)
    ap.add_argument('--yaml', default=YML)
    ap.add_argument('--desfazer', action='store_true')
    a = ap.parse_args()
    tx, ty = open(a.arquivo).read(), open(a.yaml).read()
    libs = (yaml.safe_load(ty)['bt_navigator']['ros__parameters'].get('plugin_lib_names') or [])
    if a.desfazer:
        if MARCA not in tx:
            print('GOAL_TF_ATUAL nao aplicado: nada feito'); return
        if tx.count(NOVO) != 1:
            sys.exit('ERRO: trecho do patch alterado; volte o .bak_goal_tf_atual_* a mao')
        nx = tx.replace(NOVO, VELHO)
        resto = [l for l in libs if l != LIB]
        ny = ty.replace('    - %s  # GOAL_TF_ATUAL (11/11)\n' % LIB, '')
        if not resto:
            ny = ny.replace('    # GOAL_TF_ATUAL (11/11): era sem plugin_lib_names\n    plugin_lib_names:\n', '')
        conferir_yaml(ty, ny, resto)
        ET.fromstring(nx)
        for p, n in ((a.arquivo, nx), (a.yaml, ny)):
            print('estado anterior:', bak(p)); open(p, 'w').write(n)
        print('desfeito', MARCA); return
    if MARCA in tx or MARCA in ty:
        print('GOAL_TF_ATUAL ja aplicado (marca encontrada): nada feito'); return
    if tx.count(VELHO) != 1:
        sys.exit('ERRO: FollowPath esperado nao encontrado uma vez na arvore')
    nx = tx.replace(VELHO, NOVO)
    if any('--' in c for c in re.findall(r'<!--(.*?)-->', nx, re.S)):
        sys.exit("ERRO: '--' em comentario XML")
    from collections import Counter
    ca = Counter(e.tag for e in ET.fromstring(tx).iter()); cb = Counter(e.tag for e in ET.fromstring(nx).iter())
    if cb - ca != Counter({'CaminhoSemCarimbo': 1}) or ca - cb:
        sys.exit('ERRO: nos alterados diferentes do previsto')
    m = COM_LISTA.search(ty)
    if m:
        ny = ty.replace(m.group(1), m.group(1) + '    - %s  # GOAL_TF_ATUAL (11/11)\n' % LIB)
    elif ty.count(SEM_LISTA) == 1:
        ny = ty.replace(SEM_LISTA, SEM_LISTA + '    # GOAL_TF_ATUAL (11/11): era sem plugin_lib_names\n'
                        '    plugin_lib_names:\n    - %s  # GOAL_TF_ATUAL (11/11)\n' % LIB)
    else:
        sys.exit('ERRO: bt_navigator do yaml diferente do esperado')
    conferir_yaml(ty, ny, libs + [LIB])
    for p, n in ((a.arquivo, nx), (a.yaml, ny)):
        print('backup:', bak(p)); open(p, 'w').write(n); print('gravado:', p)
    print('Compile: colcon build --packages-select robo_bt_plugins robo_navigation; reinicie o Nav2 so numa janela controlada.')


if __name__ == '__main__':
    main()
