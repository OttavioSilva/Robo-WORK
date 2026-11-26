#!/usr/bin/env python3
"""BLOQUEIO_LOCAL (05/11/2026): o bloqueio virtual do destravar nao derruba mais o planejador.

Por que (missao21): em 23 das 40 falhas do planner, as marcas do bloqueio no costmap GLOBAL fechavam
a unica passagem (obstaculo entre as mesas), e o robo ficou ~49 s sem caminho.

Mudancas:
  nav2_params.yaml (costmap global, os 2 blocos): observation_sources "scan bloqueio" -> "scan".
      O costmap local continua com o bloqueio: o DWB desvia na hora e o planejador nao ve a marca.
  destravar_node.py:
      - bloqueio dura 10 s (era 30); ao vencer, limpa o costmap local (as marcas nao somem sozinhas);
      - toda chamada pede um caminho; se o planner responder SEM caminho: apaga os bloqueios,
        publica a nuvem vazia, limpa os costmaps global e local e planeja de novo.

Uso:
    python3 aplicar_bloqueio_local.py              # aplica (backup antes)
    python3 aplicar_bloqueio_local.py --desfazer   # volta os backups mais recentes
    python3 aplicar_bloqueio_local.py --arquivo <copia_destravar> --params <copia_yaml>   # numa copia
Depois: reiniciar o ~/subir_robo.sh (o install do yaml e symlink; o script roda direto do src).
"""
import argparse
import glob
import os
import shutil
import sys
import time

BASE = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation')
MARCA = 'BLOQUEIO_LOCAL'
SUF = '.bak_bloqueiolocal_'

TROCAS_NODE = [
    ("""Com a frente a < 15 cm marca o BLOQUEIO virtual a frente (30 s) e PEDE UM CAMINHO NOVO ao planner
(que ja contorna o vao bloqueado) antes de decidir para onde ir.
""", """Com a frente a < 15 cm marca o BLOQUEIO virtual a frente. BLOQUEIO_LOCAL (05/11): vale so no costmap
local e por 10 s (era global e local, 30 s). Toda chamada pede um caminho novo; se o planner responder
sem caminho, apaga os bloqueios, limpa os costmaps e planeja de novo.
"""),
    ("""    from nav2_msgs.action import ComputePathToPose, DriveOnHeading
""", """    from nav2_msgs.action import ComputePathToPose, DriveOnHeading
    from nav2_msgs.srv import ClearEntireCostmap  # BLOQUEIO_LOCAL
"""),
    ("""            self.cli_plano = ActionClient(self, ComputePathToPose, 'compute_path_to_pose', callback_group=g)
""", """            self.cli_plano = ActionClient(self, ComputePathToPose, 'compute_path_to_pose', callback_group=g)
            # BLOQUEIO_LOCAL: limpar as marcas quando o bloqueio vence ou e apagado
            self.limpar_local_pendente = False
            self.cli_limpar_global = self.create_client(
                ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap', callback_group=g)
            self.cli_limpar_local = self.create_client(
                ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap', callback_group=g)
"""),
    ('''            """Pede um caminho novo ao planner (com o bloqueio ja no costmap). True se veio."""
''', '''            """Pede um caminho novo ao planner. True: veio; False: o planner respondeu sem caminho;
            None: nao deu para perguntar (sem goal, sem servidor ou sem resposta a tempo)."""
'''),
    ("""            if goal is None or not self.cli_plano.wait_for_server(timeout_sec=1.0):
                return False
""", """            if goal is None or not self.cli_plano.wait_for_server(timeout_sec=1.0):
                return None
"""),
    ("""            if not fut.done() or not fut.result().accepted:
                return False
""", """            if not fut.done() or not fut.result().accepted:
                return None
"""),
    ("""            if not fr.done() or not fr.result().result.path.poses:
                return False
""", """            if not fr.done():
                return None
            if not fr.result().result.path.poses:
                return False
"""),
    ("""            self.get_logger().info('DESTRAVAR: caminho novo com o bloqueio (%d pontos)' % len(pts))
""", """            self.get_logger().info('DESTRAVAR: caminho novo (%d pontos)' % len(pts))
"""),
    ("""        def marcar_frente(self, profundidade=0.30, validade=30.0):
""", """        def marcar_frente(self, profundidade=0.30, validade=10.0):  # BLOQUEIO_LOCAL: era 30 s
"""),
    ("""            agora = time.monotonic()
            self.bloqueios = [b for b in self.bloqueios if b[0] > agora]
""", """            agora = time.monotonic()
            if self.limpar_local_pendente:  # BLOQUEIO_LOCAL: a nuvem vazia ja saiu no ciclo anterior
                self.limpar_local_pendente = False
                if self.cli_limpar_local.service_is_ready():
                    self.cli_limpar_local.call_async(ClearEntireCostmap.Request())
            havia = bool(self.bloqueios)
            self.bloqueios = [b for b in self.bloqueios if b[0] > agora]
            if havia and not self.bloqueios:
                self.limpar_local_pendente = True
"""),
    ("""        # ---------------- movimentos ----------------
""", """        def soltar_bloqueio(self):
            \"\"\"BLOQUEIO_LOCAL: o planner nao achou caminho: tira as marcas e limpa os costmaps.\"\"\"
            n = len(self.bloqueios)
            self.bloqueios = []
            self.limpar_local_pendente = False
            self.publicar_bloqueio()
            time.sleep(0.6)  # o costmap guarda a ultima nuvem: limpar antes marcaria tudo de novo
            for cli in (self.cli_limpar_global, self.cli_limpar_local):
                if cli.service_is_ready():
                    fut = cli.call_async(ClearEntireCostmap.Request())
                    t1 = time.monotonic() + 1.0
                    while not fut.done() and time.monotonic() < t1:
                        time.sleep(0.05)
            time.sleep(0.6)  # o costmap global atualiza a 2 Hz
            self.get_logger().info('BLOQUEIO: sem caminho: apaguei %d bloqueio(s), limpei os costmaps e planejo de novo' % n)

        # ---------------- movimentos ----------------
"""),
    ("""                self.marcar_frente()
                time.sleep(1.2)          # o costmap global marca o bloqueio
                self.replanejar()        # a direcao do caminho ja contorna o vao bloqueado
""", """                self.marcar_frente()
                time.sleep(0.4)          # BLOQUEIO_LOCAL: o costmap local atualiza a 5 Hz
            if self.replanejar() is False:  # BLOQUEIO_LOCAL: sem caminho: apaga, limpa e planeja sem ele
                self.soltar_bloqueio()
                self.replanejar()
"""),
]
ANTES_YAML = 'observation_sources: scan bloqueio  # BLOQUEIO_VIRTUAL'
DEPOIS_YAML = 'observation_sources: scan  # BLOQUEIO_LOCAL (05/11): era "scan bloqueio"; no global fechava a passagem'


def patch_yaml(txt):
    """Troca so os blocos do global_costmap (o local fica antes de 'global_costmap:')."""
    i = txt.index('\nglobal_costmap:')
    f = txt.index('\nplanner_server:')
    trecho = txt[i:f]
    if trecho.count(ANTES_YAML) != 2:
        raise RuntimeError('esperava 2 blocos no global, achei %d' % trecho.count(ANTES_YAML))
    return txt[:i] + trecho.replace(ANTES_YAML, DEPOIS_YAML) + txt[f:]


def conferir_yaml(velho, novo):
    import yaml
    a, b = yaml.safe_load(velho), yaml.safe_load(novo)
    ga = a['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
    gb = b['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']
    assert gb['observation_sources'] == 'scan', 'global ainda com o bloqueio'
    la = a['local_costmap']['local_costmap']['ros__parameters']
    assert la['obstacle_layer' if 'obstacle_layer' in la else 'voxel_layer']['observation_sources'] == 'scan bloqueio', 'local mudou'
    ga['observation_sources'] = 'scan'
    assert a == b, 'o yaml mudou mais do que o pretendido'
    dif = [(x, y) for x, y in zip(velho.split('\n'), novo.split('\n')) if x != y]
    assert len(dif) == 2 and len(velho.split('\n')) == len(novo.split('\n')), 'esperava 2 linhas alteradas'


def main():
    ap = argparse.ArgumentParser(description='BLOQUEIO_LOCAL')
    ap.add_argument('--desfazer', action='store_true')
    ap.add_argument('--arquivo', default=os.path.join(BASE, 'scripts/destravar_node.py'))
    ap.add_argument('--params', default=os.path.join(BASE, 'config/nav2_params.yaml'))
    a = ap.parse_args()
    if a.desfazer:
        baks = [sorted(glob.glob(p + SUF + '*')) for p in (a.arquivo, a.params)]
        if not all(baks):
            print('Backup %s* nao encontrado para os 2 arquivos. Nada alterado.' % SUF)
            return 1
        for p, b in zip((a.arquivo, a.params), baks):
            shutil.copy2(b[-1], p)
            print('Restaurado de %s' % b[-1])
        return 0
    node, params = open(a.arquivo).read(), open(a.params).read()
    if MARCA in node or MARCA in params:
        print('Ja aplicado (achei a marca %s). Nada a fazer.' % MARCA)
        return 0
    try:
        novo_node = node
        for velho, novo in TROCAS_NODE:
            if novo_node.count(velho) != 1:
                raise RuntimeError('trecho aparece %d vezes (esperado 1): %s' % (novo_node.count(velho), velho[:60]))
            novo_node = novo_node.replace(velho, novo)
        compile(novo_node, a.arquivo, 'exec')
        novo_params = patch_yaml(params)
        conferir_yaml(params, novo_params)
    except Exception as exc:
        print('Nao consegui aplicar (%s). Nada alterado.' % exc)
        return 1
    data = time.strftime('%Y%m%d_%H%M%S')
    for p, txt in ((a.arquivo, novo_node), (a.params, novo_params)):
        shutil.copy2(p, p + SUF + data)
        open(p, 'w').write(txt)
        print('Backup: %s' % (p + SUF + data))
    print('Aplicado. Reinicie o ~/subir_robo.sh. Conferir: ros2 param get '
          '/global_costmap/global_costmap obstacle_layer.observation_sources   # scan')
    return 0


if __name__ == '__main__':
    sys.exit(main())
