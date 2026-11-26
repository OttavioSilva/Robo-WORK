#!/usr/bin/env python3
"""ROTA_MAPA_OBSTACULO (11/11/2026) - Patch 04. Parametro novo do alinhador `rota_mapa_obstaculo`:
  'ignorar'   (PADRAO: igual a hoje, nem le o costmap)
  'registrar' (so log: quando o ROTA_MAPA troca o planner pela rota do mapa, conta celulas LETHAL do
               obstacle_layer global livres no /map e a > 10 cm de parede do mapa, a < 0,35 m da rota)
  'recusar'   (com objeto novo na rota do mapa, NAO troca: fica o caminho do planner)
Por que nao 'recusar' direto: no bag 2nav (12:33:39) a rota do mapa tinha 8 celulas "novas" junto a parede
do corredor e o robo atravessou; o criterio ainda nao separa parede vista deslocada de objeto. Medir antes.
A/B sem reiniciar:  ros2 param set /alinhador_passagem rota_mapa_obstaculo registrar
Uso: python3 aplicar_rota_mapa_obstaculo.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
FUNCAO = '''

# ROTA_MAPA_OBSTACULO (11/11): objeto do momento na rota do mapa (Patch 04)
def obstaculo_novo_na_rota(mapa, obs, rota, raio=0.35, dist_parede=0.10):
    """Celulas ocupadas do `obs` (Mapa do obstacle_layer global, ocupado = 100) que estao livres no `mapa`,
    a mais de `dist_parede` de qualquer parede do mapa e a menos de `raio` de algum ponto da `rota`.
    Devolve [(x, y), ...]. Sem obs ou sem rota: []."""
    if obs is None or not rota:
        return []
    achados = []
    n = int(math.ceil(dist_parede / mapa.res))
    xs = [p[0] for p in rota]; ys = [p[1] for p in rota]
    x0, x1, y0, y1 = min(xs) - raio, max(xs) + raio, min(ys) - raio, max(ys) + raio
    for j in range(obs.h):
        for i in range(obs.w):
            if obs.dados[j * obs.w + i] < obs.limiar:
                continue
            gx, gy = (i + 0.5) * obs.res, (j + 0.5) * obs.res
            x, y = obs.ox + obs.c * gx - obs.s * gy, obs.oy + obs.s * gx + obs.c * gy
            if x < x0 or x > x1 or y < y0 or y > y1 or mapa.estado(x, y) != 0:
                continue
            if any(mapa.estado(x + a * mapa.res, y + b * mapa.res) == 1
                   for a in range(-n, n + 1) for b in range(-n, n + 1) if math.hypot(a, b) <= n):
                continue
            if any(math.hypot(x - p[0], y - p[1]) < raio for p in rota):
                achados.append((x, y))
    return achados
'''
TROCAS = [
    ("\n\ndef _raio(mapa, x, y, ux, uy, alcance):", FUNCAO + "\n\ndef _raio(mapa, x, y, ux, uy, alcance):"),
    ("            self.declare_parameter('rota_mapa', True)           # ROTA_MAPA\n",
     "            self.declare_parameter('rota_mapa', True)           # ROTA_MAPA\n"
     "            # ROTA_MAPA_OBSTACULO (11/11): ignorar | registrar | recusar (padrao ignorar = como antes)\n"
     "            self.declare_parameter('rota_mapa_obstaculo', 'ignorar')\n"),
    ("            self.create_subscription(Path, '/plan', self.ao_plano, 5, callback_group=grupo)\n",
     "            self.create_subscription(Path, '/plan', self.ao_plano, 5, callback_group=grupo)\n"
     "            self.obs_msg = None   # ROTA_MAPA_OBSTACULO (11/11)\n"
     "            self.create_subscription(OccupancyGrid, '/global_costmap/obstacle_layer', self.ao_obs_global,\n"
     "                                     qos_mapa, callback_group=grupo)\n"),
    ("        def ao_mapa(self, msg):\n            with self.lock:\n                self.mapa_msg = msg\n",
     "        def ao_mapa(self, msg):\n            with self.lock:\n                self.mapa_msg = msg\n\n"
     "        def ao_obs_global(self, msg):   # ROTA_MAPA_OBSTACULO (11/11)\n"
     "            with self.lock:\n                self.obs_msg = msg\n\n"
     "        def _obs_global(self):\n"
     "            with self.lock:\n                m = self.obs_msg\n"
     "            if m is None:\n                return None\n"
     "            i = m.info\n"
     "            return Mapa(i.resolution, i.width, i.height, i.origin.position.x, i.origin.position.y,\n"
     "                        yaw_de(i.origin.orientation), m.data, limiar_ocupado=100)\n"),
    ("""                if rota is not None and (pontos is None or _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5):
                    self.get_logger().warn('Planner: %s; mapa: %.1f m: sigo a rota do mapa' % (
                        'sem caminho' if pontos is None else '%.1f m' % _comp_rota(pontos), _comp_rota(rota)))
                    pontos = rota
""", """                if rota is not None and (pontos is None or _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5):
                    modo_obs = str(self.p('rota_mapa_obstaculo'))   # ROTA_MAPA_OBSTACULO (11/11)
                    novos = obstaculo_novo_na_rota(mapa, self._obs_global(), rota) if modo_obs in (
                        'registrar', 'recusar') else []
                    if novos:
                        self.get_logger().warn('ROTA_MAPA_OBSTACULO (%s): %d celula(s) de objeto do momento a '
                                               '< 35 cm da rota do mapa, a 1a em (%.2f, %.2f)' % (
                                                   modo_obs, len(novos), novos[0][0], novos[0][1]))
                    if novos and modo_obs == 'recusar' and pontos is not None:
                        self.get_logger().warn('ROTA_MAPA_OBSTACULO: fico com o planner (%.1f m)' % _comp_rota(pontos))
                    else:
                        self.get_logger().warn('Planner: %s; mapa: %.1f m: sigo a rota do mapa' % (
                            'sem caminho' if pontos is None else '%.1f m' % _comp_rota(pontos), _comp_rota(rota)))
                        pontos = rota
"""),
]
if __name__ == '__main__':
    rodar('rota_mapa_obstaculo', 'ROTA_MAPA_OBSTACULO (11/11)', ARQ, TROCAS, doc=__doc__)
