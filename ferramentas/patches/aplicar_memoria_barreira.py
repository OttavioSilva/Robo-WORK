#!/usr/bin/env python3
"""MEMORIA_BARREIRA (25/11/2026): barreira vista nao e esquecida; limpeza nao apaga o mapa; decisao rapida.

Sim de barreiras (sim_arena/cenarios_barreiras.yaml, linha de base 25/11 01:43): b01 COLIDIU com a barreira
(3 aproximacoes), b03 ficou em REINSISTIR 7/8. Causas:
  - REINSISTIR_LIMPA e PASSAGEM_IMPOSSIVEL limpavam o costmap GLOBAL: a barreira era esquecida e, logo apos
    a limpeza, o global ainda vazio deixava o planner passar ATRAVES da mesa ("passagem de 0.00 m no mapa");
  - PASSAGEM_IMPOSSIVEL fazia 5 rodadas de limpa+1,5 s+replaneja a cada REINSISTIR (minutos parado).
Agora (param 'memoria_barreira', false = como antes):
  - antes de qualquer limpeza o alinhador chama /bloqueios/marcar (bloqueios_rota.py): o que o LiDAR ve fora
    do mapa vira StaticLayer do global, que a limpeza nao apaga (expira sozinha);
  - REINSISTIR e PASSAGEM_IMPOSSIVEL limpam so o costmap LOCAL;
  - PASSAGEM_IMPOSSIVEL com largura ~0 (caminho pela parede = global recem-limpo) so espera e replaneja;
  - a rota do mapa (PLANNER_LIVRE/ROTA_MAPA/VIRAR_REPLANEJA) respeita /bloqueios_mapa.
Uso: aplicar_memoria_barreira.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    # funcao de modulo (funciona com qualquer no, inclusive os falsos dos testes)
    ('''def rota_cruza_bloqueio(pontos, bloqueios):
''',
     '''def com_barreiras(no, mapa):
    """MEMORIA_BARREIRA (25/11): o mapa com as barreiras lembradas (/bloqueios_mapa), para as rotas do alinhador."""
    b, m = getattr(no, 'barreiras_msg', None), getattr(no, 'mapa_msg', None)
    if mapa is None or b is None or m is None or not no.p('memoria_barreira') or \\
            b.info.width != mapa.w or b.info.height != mapa.h or not any(v > 0 for v in b.data):
        return mapa
    i, q = m.info, m.info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return Mapa(i.resolution, i.width, i.height, i.origin.position.x, i.origin.position.y, yaw,
                [100 if bv > 0 else mv for mv, bv in zip(m.data, b.data)])


def rota_cruza_bloqueio(pontos, bloqueios):
'''),
    # parametro
    ("""            self.declare_parameter('planner_livre', True)       # PLANNER_LIVRE (25/11)
""",
     """            self.declare_parameter('planner_livre', True)       # PLANNER_LIVRE (25/11)
            self.declare_parameter('memoria_barreira', True)    # MEMORIA_BARREIRA (25/11)
"""),
    ("""            self.declare_parameter('passagem_replanejar_max', 5)      # PASSAGEM_IMPOSSIVEL
""",
     """            self.declare_parameter('passagem_replanejar_max', 2)      # PASSAGEM_IMPOSSIVEL; MEMORIA_BARREIRA: era 5
"""),
    # assinatura da memoria
    ("""            self.create_subscription(OccupancyGrid, '/global_costmap/obstacle_layer', self.ao_obs_global,
                                     qos_mapa, callback_group=grupo)
""",
     """            self.create_subscription(OccupancyGrid, '/global_costmap/obstacle_layer', self.ao_obs_global,
                                     qos_mapa, callback_group=grupo)
            self.barreiras_msg = None   # MEMORIA_BARREIRA (25/11): /bloqueios_mapa do bloqueios_rota.py
            self.create_subscription(OccupancyGrid, '/bloqueios_mapa',
                                     lambda m: setattr(self, 'barreiras_msg', m), qos_mapa, callback_group=grupo)
"""),
    # limpeza: marca antes, e pode limpar so o local
    ("""        def _limpar_costmaps(self):
            \"\"\"v3.27: limpa os costmaps local e global (os obstaculos que ainda existem voltam no scan).\"\"\"
            try:
                from nav2_msgs.srv import ClearEntireCostmap
            except Exception:
                return
            if not hasattr(self, 'cli_limpar'):
                self.cli_limpar = {}
            for srv in ('/local_costmap/clear_entirely_local_costmap',
                        '/global_costmap/clear_entirely_global_costmap'):
""",
     """        def _marcar_barreiras(self):
            \"\"\"MEMORIA_BARREIRA (25/11): grava o que o LiDAR ve fora do mapa no /bloqueios_mapa (sobrevive a limpeza).\"\"\"
            if not self.p('memoria_barreira'):
                return
            from std_srvs.srv import Trigger
            if getattr(self, 'cli_marcar', None) is None:
                self.cli_marcar = self.create_client(Trigger, '/bloqueios/marcar')
            if self.cli_marcar.wait_for_service(timeout_sec=0.5):
                r = self._esperar(self.cli_marcar.call_async(Trigger.Request()), 2.0)
                if r is not None:
                    self.get_logger().info('MEMORIA_BARREIRA: %s' % r.message)

        def _limpar_costmaps(self, so_local=False):
            \"\"\"v3.27: limpa os costmaps local e global (os obstaculos que ainda existem voltam no scan).
            MEMORIA_BARREIRA (25/11): marca antes o que o LiDAR ve; so_local=True nao toca o global.\"\"\"
            self._marcar_barreiras()
            try:
                from nav2_msgs.srv import ClearEntireCostmap
            except Exception:
                return
            if not hasattr(self, 'cli_limpar'):
                self.cli_limpar = {}
            for srv in ('/local_costmap/clear_entirely_local_costmap',) + \\
                    (() if so_local else ('/global_costmap/clear_entirely_global_costmap',)):
"""),
    # REINSISTIR: so o local
    ("""                        self._limpar_costmaps()   # REINSISTIR_LIMPA (25/11): marcas velhas fora antes de replanejar
""",
     """                        self._limpar_costmaps(so_local=bool(self.p('memoria_barreira')))   # REINSISTIR_LIMPA (25/11); MEMORIA_BARREIRA
"""),
    # PASSAGEM_IMPOSSIVEL: so o local; largura ~0 = global recem-limpo, so espera
    ("""                self._limpar_costmaps()
                time.sleep(1.5)
                if self._novo_goal_chegou():
                    return [], 'PASSAGEM_IMPOSSIVEL: goal novo'
""",
     """                if not (self.p('memoria_barreira') and info['largura_mapa'] < 0.05):   # MEMORIA_BARREIRA
                    self._limpar_costmaps(so_local=bool(self.p('memoria_barreira')))
                time.sleep(1.5)
                if self._novo_goal_chegou():
                    return [], 'PASSAGEM_IMPOSSIVEL: goal novo'
"""),
    # rotas do mapa com as barreiras
    ("""            rota = rota_no_mapa(mapa, pm, (alvo.pose.position.x, alvo.pose.position.y))
""",
     """            rota = rota_no_mapa(com_barreiras(self, mapa), pm, (alvo.pose.position.x, alvo.pose.position.y))   # MEMORIA_BARREIRA
"""),
    ("""            rota = rota_no_mapa(mapa, pm, (gx, gy), bloqueios=self._bloqueios_ativos()) \\
""",
     """            rota = rota_no_mapa(com_barreiras(self, mapa), pm, (gx, gy), bloqueios=self._bloqueios_ativos()) \\
"""),
    ("""                rota = rota_no_mapa(mapa, pm, (gx, gy), bloqueios=bloq)
""",
     """                rota = rota_no_mapa(com_barreiras(self, mapa), pm, (gx, gy), bloqueios=bloq)   # MEMORIA_BARREIRA
"""),
    ("""                    rota = rota_no_mapa(mapa, pm, (gx, gy))
""",
     """                    rota = rota_no_mapa(com_barreiras(self, mapa), pm, (gx, gy))   # MEMORIA_BARREIRA
"""),
]


def conferir(velho, novo):
    assert novo.count('MEMORIA_BARREIRA') >= 10
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('memoria_barreira', 'MEMORIA_BARREIRA', PADRAO, TROCAS, conferir, doc=__doc__)
