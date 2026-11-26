#!/usr/bin/env python3
"""SEM_SUAVIZADOR (11/11/2026) - pedido do operador. Tira o SmoothPath (simple_smoother) do planejamento: o FollowPath
passa a seguir o caminho do NavFn. So saem os 2 nos SmoothPath da arvore principal (planejamento normal e depois da
recuperacao); o Fallback seguinte (CaminhoOriginalSeguro em 'registrar' + SetBlackboard smoothed_path={path}) passa a
ser o caminho normal. A chave {smoothed_path} continua com o mesmo nome (IsPathValid, TruncatePathLocal e
CaminhoSemCarimbo leem dela). smoother_server segue no launch, sem uso.
Uso: python3 aplicar_sem_suavizador.py [--arquivo copia.xml] [--desfazer]"""
import os, sys, re
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
import xml.etree.ElementTree as ET
from collections import Counter
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml')
S1 = '                  <SmoothPath unsmoothed_path="{path}" smoothed_path="{smoothed_path}" smoother_id="simple_smoother" max_smoothing_duration="2.0" check_for_collisions="true" error_code_id="{smoothing_path_error_code}" />\n'
S2 = '                  <SmoothPath name="CheckRecoveredPath" unsmoothed_path="{path}" smoothed_path="{smoothed_path}" smoother_id="simple_smoother" max_smoothing_duration="2.0" check_for_collisions="true" error_code_id="{smoothing_path_error_code}" />\n'
M1 = '                  <!-- SEM_SUAVIZADOR (11/11): aqui havia o SmoothPath (simple_smoother); o FollowPath segue o caminho do NavFn -->\n'
M2 = '                  <!-- SEM_SUAVIZADOR (11/11): aqui havia o SmoothPath CheckRecoveredPath -->\n'
TROCAS = [(S1, M1), (S2, M2)]
def conferir(velho, novo):
    a, b = Counter(e.tag for e in ET.fromstring(velho).iter()), Counter(e.tag for e in ET.fromstring(novo).iter())
    if a - b != Counter({'SmoothPath': 2}) or b - a:
        sys.exit('ERRO: nos alterados diferentes do previsto: %s / %s' % (dict(a - b), dict(b - a)))
    if any('--' in c for c in re.findall(r'<!--(.*?)-->', novo, re.S)):
        sys.exit("ERRO: '--' em comentario XML")
if __name__ == '__main__':
    rodar('sem_suavizador', 'SEM_SUAVIZADOR (11/11)', ARQ, TROCAS, conferir=conferir, doc=__doc__)
