#!/usr/bin/env python3
"""CAMINHO_ORIGINAL (11/11/2026) - Patch 01 do refinamento da navegacao.

Quando o SmoothPath reprova o caminho suavizado, a arvore passava o caminho ORIGINAL do planner ao FollowPath
sem conferir (ORIGINAL_SEM_GATE). Este patch poe a condicao CaminhoOriginalSeguro (pacote robo_bt_plugins)
antes do SetBlackboard. Ela so reprova bloqueio de fato (footprint na tangente sobre LETHAL e nenhum
deslocamento lateral de ate 10 cm livra), fora do footprint atual, dos 0,30 m iniciais e dos 0,60 m finais.
Reprovado: marca caminho_bloqueado e a navegacao vai para a recuperacao avaliada (destravar v4).

INSTALA EM MODO "registrar" (so escreve no log; comportamento igual ao de hoje). Para o teste A/B:
  ros2 topic pub --once --qos-durability transient_local /caminho_original/modo std_msgs/msg/String "{data: aplicar}"
  (volta: o mesmo com "{data: registrar}"; vale ate reiniciar o Nav2, que volta ao modo da arvore)

Arquivos: behavior_trees/navigate_to_pose_smooth.xml e config/nav2_params.yaml (bt_navigator.plugin_lib_names).
Uso:
  python3 aplicar_caminho_original.py                  # aplica no workspace
  python3 aplicar_caminho_original.py --desfazer       # volta os backups mais recentes deste patch
  python3 aplicar_caminho_original.py --arquivo X.xml --yaml Y.yaml   # testar numa copia
"""
import argparse, copy, datetime, glob, os, shutil, sys
import xml.etree.ElementTree as ET
import yaml

MARCA = 'CAMINHO_ORIGINAL (11/11)'
NOME = 'caminho_original'
WS = os.path.expanduser('~/robo_jazzy_ws')
XML = os.path.join(WS, 'src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml')
YML = os.path.join(WS, 'src/robo_navigation/config/nav2_params.yaml')
PLUGIN = 'robo_caminho_original_seguro_bt_node'

GATE = ('<CaminhoOriginalSeguro name="{nome}" path="{{path}}" goal="{{goal}}" modo="registrar" '
        'folga_inicio="0.30" folga_fim="0.60" tolerancia_lateral="0.10" />')

TROCAS_XML = [
    # 1. zera a marca no inicio de cada goal
    ('<Script name="CaminhoAindaNao" code="caminho_do_goal := false" />',
     '<Script name="CaminhoAindaNao" code="caminho_do_goal := false" />\n'
     '    <Script name="SemBloqueioAinda" code="caminho_bloqueado := false" />'),
    # 2. fallback do planejamento normal
    ('''                <Fallback name="SuavizadoOuOriginal">
                  <SmoothPath unsmoothed_path="{path}" smoothed_path="{smoothed_path}" smoother_id="simple_smoother" max_smoothing_duration="2.0" check_for_collisions="true" error_code_id="{smoothing_path_error_code}" />
                  <SetBlackboard name="UsarOriginalDoPlanner" output_key="smoothed_path" value="{path}" />
                </Fallback>''',
     '''                <!-- CAMINHO_ORIGINAL (11/11): o original so vai ao FollowPath se a condicao CaminhoOriginalSeguro
                     (robo_bt_plugins) nao achar BLOQUEIO no trecho restante: footprint na tangente sobre LETHAL que
                     nenhum deslocamento lateral de ate 10 cm livra (rasante de parede passa). Instalado em
                     modo="registrar" (so log); A/B: publicar "aplicar" no topico /caminho_original/modo (comando
                     em ferramentas/patches/aplicar_caminho_original.py). Reprovado: caminho_bloqueado e recuperacao
                     avaliada. -->
                <Fallback name="SuavizadoOuOriginal">
                  <SmoothPath unsmoothed_path="{path}" smoothed_path="{smoothed_path}" smoother_id="simple_smoother" max_smoothing_duration="2.0" check_for_collisions="true" error_code_id="{smoothing_path_error_code}" />
                  <Sequence name="OriginalConferido">
                    ''' + GATE.format(nome='ConferirOriginal') + '''
                    <SetBlackboard name="UsarOriginalDoPlanner" output_key="smoothed_path" value="{path}" />
                  </Sequence>
                  <Sequence name="OriginalBloqueado">
                    <Script name="MarcarBloqueio" code="caminho_bloqueado := true" />
                    <AlwaysFailure />
                  </Sequence>
                </Fallback>'''),
    # 3. caminho aceito limpa a marca
    ('<Script name="CaminhoDesteGoal" code="caminho_do_goal := true" />',
     '<Script name="CaminhoDesteGoal" code="caminho_do_goal := true" />\n'
     '                <Script name="CaminhoLiberado" code="caminho_bloqueado := false" />'),
    # 4. replanejamento depois da recuperacao: mesma conferencia (reprovado -> a pipeline replaneja)
    ('''                  <SetBlackboard output_key="smoothed_path" value="{path}" />''',
     '''                  <Sequence name="OriginalConferidoRecuperacao">
                    ''' + GATE.format(nome='ConferirOriginalRecuperacao') + '''
                    <SetBlackboard output_key="smoothed_path" value="{path}" />
                  </Sequence>'''),
    # 5. bloqueio do original tambem justifica a recuperacao
    ('''          <WouldASmootherRecoveryHelp error_code="{smoothing_path_error_code}" />''',
     '''          <WouldASmootherRecoveryHelp error_code="{smoothing_path_error_code}" />
          <ScriptCondition name="OriginalBloqueadoPedeRecuperacao" code="caminho_bloqueado" />'''),
]


def backup(path):
    dst = '%s.bak_%s_%s' % (path, NOME, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(path, dst)
    return dst


def assinatura(el):
    return [(e.tag, tuple(sorted(e.attrib.items()))) for e in el.iter()]


def patch_xml(txt):
    novo = txt
    for velho, rep in TROCAS_XML:
        if novo.count(velho) != 1:
            raise SystemExit('ERRO: trecho esperado aparece %d vezes na arvore (versao diferente?):\n%s'
                             % (novo.count(velho), velho[:120]))
        novo = novo.replace(velho, rep)
    # conferencia: carrega (expat estrito, como o pytest), sem '--' em comentario, so entraram os nos previstos
    import re
    if any('--' in c for c in re.findall(r'<!--(.*?)-->', novo, re.S)):
        raise SystemExit("ERRO: '--' dentro de comentario XML")
    a, b = ET.fromstring(txt), ET.fromstring(novo)
    tags_a = [t for t, _ in assinatura(a)]
    tags_b = [t for t, _ in assinatura(b)]
    from collections import Counter
    dif = Counter(tags_b) - Counter(tags_a)
    esperado = Counter({'CaminhoOriginalSeguro': 2, 'Sequence': 3, 'Script': 3, 'AlwaysFailure': 1,
                        'ScriptCondition': 1})
    if dif != esperado or Counter(tags_a) - Counter(tags_b):
        raise SystemExit('ERRO: nos alterados diferentes do previsto: %s' % dict(dif))
    return novo


def patch_yaml(txt):
    ancora = '    wait_for_service_timeout: 1000\n'
    if txt.count(ancora) != 1 or 'plugin_lib_names' in txt:
        raise SystemExit('ERRO: bt_navigator do nav2_params.yaml diferente do esperado')
    novo = txt.replace(ancora, ancora + '    # CAMINHO_ORIGINAL (11/11): era sem plugin_lib_names (so os plugins do Nav2)\n'
                       '    plugin_lib_names:\n    - ' + PLUGIN + '\n')
    a, b = yaml.safe_load(txt), yaml.safe_load(novo)
    esperado = copy.deepcopy(a)
    esperado['bt_navigator']['ros__parameters']['plugin_lib_names'] = [PLUGIN]
    if b != esperado:
        raise SystemExit('ERRO: o yaml mudou alem de bt_navigator.plugin_lib_names')
    return novo


def desfazer(arquivos):
    for p in arquivos:
        baks = sorted(glob.glob(p + '.bak_%s_*' % NOME))
        if not baks:
            print('sem backup deste patch para', p)
            continue
        shutil.copy2(baks[-1], p)
        print('voltei', p, '<-', baks[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arquivo', default=XML, help='arvore (para testar numa copia)')
    ap.add_argument('--yaml', default=YML)
    ap.add_argument('--desfazer', action='store_true')
    a = ap.parse_args()
    if a.desfazer:
        desfazer([a.arquivo, a.yaml])
        return
    tx, ty = open(a.arquivo).read(), open(a.yaml).read()
    if MARCA in tx or MARCA in ty:
        print('CAMINHO_ORIGINAL ja aplicado (marca encontrada): nada feito')
        return
    nx, ny = patch_xml(tx), patch_yaml(ty)
    for p, n in ((a.arquivo, nx), (a.yaml, ny)):
        print('backup:', backup(p))
        open(p, 'w').write(n)
        print('gravado:', p)
    print('Pronto. Compile: cd ~/robo_jazzy_ws && colcon build --packages-select robo_bt_plugins robo_navigation'
          ' && source install/setup.bash ; reinicie o ~/subir_robo.sh numa janela controlada.')


if __name__ == '__main__':
    main()
