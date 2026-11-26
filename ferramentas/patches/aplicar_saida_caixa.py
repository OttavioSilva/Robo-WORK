#!/usr/bin/env python3
"""SAIDA_CAIXA (25/11/2026): saindo do START/FINISH, primeiro anda reto para fora da caixa.

Sim da arena Fusion (caixa 0,67 x 0,69, robo 0,59 x 0,65): no START o robo tem ~1 cm de cada lado; o costmap
de 5 cm poe o pilar/parede dentro do footprint, o caminho do NavFn sai na diagonal e o DWB nao acha trajetoria
("frente 0 cm"): 3 areas falharam com 0,0 m andados. Operador: anda reto para fora e so depois navega.
Agora (param 'saida_caixa' = distancia em m, 0 = como antes): no inicio de um goal, se o robo esta a <= 15 cm e
<= 25 graus de um ponto START*/FINISH* do pontos.yaml, anda reto para a frente (_andar_reto, para com folga
< 7 cm) essa distancia. Log 'SAIDA_CAIXA:'.
Uso: aplicar_saida_caixa.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    ("""            self.declare_parameter('memoria_barreira', True)    # MEMORIA_BARREIRA (25/11)
""",
     """            self.declare_parameter('memoria_barreira', True)    # MEMORIA_BARREIRA (25/11)
            self.declare_parameter('saida_caixa', 0.70)         # SAIDA_CAIXA (25/11): m reto para fora do START; 0 desliga
"""),
    ("""                    self.res_area = None
                    self.executar(goal, area)
""",
     """                    self.res_area = None
                    self._saida_caixa(area)   # SAIDA_CAIXA (25/11)
                    self.executar(goal, area)
"""),
    ("""        def _marcar_barreiras(self):
""",
     """        def _saida_caixa(self, area):
            \"\"\"SAIDA_CAIXA (25/11): robo encaixado num START/FINISH anda reto para fora antes de navegar.\"\"\"
            dist = float(self.p('saida_caixa'))
            if dist <= 0.0 or str(area or '').startswith(('START', 'FINISH')):
                return
            pm = self._pose_mapa_yaw()
            if pm is None:
                return
            try:
                with open(self.p('arquivo_pontos')) as fp:
                    pontos = (yaml.safe_load(fp) or {}).get('pontos') or {}
            except Exception:
                return
            for n, pt in pontos.items():
                if not str(n).startswith(('START', 'FINISH')):
                    continue
                dyaw = abs(math.atan2(math.sin(pm[2] - pt['yaw']), math.cos(pm[2] - pt['yaw'])))
                if math.hypot(pm[0] - pt['x'], pm[1] - pt['y']) <= 0.15 and dyaw <= math.radians(25):
                    self.get_logger().info('SAIDA_CAIXA: robo no %s: ando %.0f cm reto para fora' % (n, dist * 100))
                    self._andar_reto('frente', dist, 'SAIDA_CAIXA')
                    return

        def _marcar_barreiras(self):
"""),
]


def conferir(velho, novo):
    assert novo.count('SAIDA_CAIXA') >= 5
    compile(novo, 'alinhador', 'exec')


if __name__ == '__main__':
    rodar('saida_caixa', 'SAIDA_CAIXA', PADRAO, TROCAS, conferir, doc=__doc__)
