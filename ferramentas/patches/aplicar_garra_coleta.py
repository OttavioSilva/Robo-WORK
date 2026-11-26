#!/usr/bin/env python3
"""GARRA_COLETA (23/11/2026): executor_prova.py chama a garra (repo Arthur-Demetrio/garra) com --garra.

Sem --garra o executor fica IGUAL a antes (pegar/soltar simulados).
Com --garra:
  - plano com capacidade 1 (a garra segura um cubo so; o robo nao tem deposito);
  - PEGAR: publica /coleta/pedido (JSON area+objetos) e espera /coleta/resultado do alcance_cubo.py
    (camera -> ajuste da base/plataforma Y -> pegar_cubo.py), ate --tempo-coleta s;
  - SOLTAR: publica /entregar_cubo (Point x=--entrega-x, z=altura da mesa + 0,021) e espera
    /entregar_cubo/resultado do pegar_cubo.py, ate --tempo-entrega s;
  - controla o que esta na garra (resultado + chave dos dedos /garra/cubo_preso, se publicada):
    com cubo na garra nao pega outro (abrir os dedos derrubaria o cubo na mesa errada);
    sem cubo, nao tenta soltar;
  - falha de pegar/soltar NAO interrompe a prova (segue para a proxima mesa).
Uso: aplicar_garra_coleta.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/executor_prova.py')

TROCAS = [
    # --- documentacao
    ("""    python3 executor_prova.py tarefa.yaml --manipulacao 5 --confirmar
\"\"\"
""",
     """    python3 executor_prova.py tarefa.yaml --manipulacao 5 --confirmar
    # GARRA_COLETA (23/11): pegar/soltar pela garra (subir antes ~/robo_jazzy_ws/ferramentas/subida_unica/subir_garra.sh)
    python3 executor_prova.py tarefa.yaml --garra
\"\"\"
"""),
    ("""import argparse
import math
""",
     """import argparse
import json  # GARRA_COLETA
import math
"""),
    # --- constantes e altura da mesa
    ("""AREAS = os.path.expanduser('~/mapas/service_areas.yaml')
""",
     """AREAS = os.path.expanduser('~/mapas/service_areas.yaml')
# GARRA_COLETA (23/11): a garra segura 1 cubo; entrega com o centro do cubo a ENTREGA_X de robo_base
# (0,445 = 17 cm a frente do para-choque, ponto conferido pelo time da garra) e meia aresta acima do tampo.
CAPACIDADE_GARRA = 1
MEIO_CUBO = 0.021
ALTURA_MESA_PADRAO = 0.10


def altura_mesa(area):
    \"\"\"GARRA_COLETA: table.height da mesa em service_areas.yaml (padrao 0,10 m).\"\"\"
    try:
        a = ((yaml.safe_load(open(AREAS)) or {}).get('service_areas') or {}).get(area) or {}
        return float((a.get('table') or {}).get('height', ALTURA_MESA_PADRAO))
    except (OSError, TypeError, ValueError):
        return ALTURA_MESA_PADRAO
"""),
    # --- no ROS: topicos da garra
    ("""            self.create_subscription(String, '/ir_para_area/resultado', self.ao_res, 10)
            self.res = []

        def ao_res(self, m):
            self.res.append(m.data)
""",
     """            self.create_subscription(String, '/ir_para_area/resultado', self.ao_res, 10)
            self.res = []
            # GARRA_COLETA (23/11): coleta (alcance_cubo.py) e entrega (pegar_cubo.py)
            self.garra = args.garra
            if self.garra:
                self.pub_coleta = self.create_publisher(String, '/coleta/pedido', 10)
                self.create_subscription(String, '/coleta/resultado',
                                         lambda m: self.res_coleta.append(m.data), 10)
                self.pub_entrega = self.create_publisher(Point, '/entregar_cubo', 10)
                self.create_subscription(String, '/entregar_cubo/resultado',
                                         lambda m: self.res_entrega.append(m.data), 10)
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(Bool, '/garra/cubo_preso',
                                         lambda m: setattr(self, 'cubo_preso', m.data), qos)
            self.res_coleta, self.res_entrega, self.cubo_preso = [], [], None

        def ao_res(self, m):
            self.res.append(m.data)

        def _pedir(self, pub, msg, respostas, timeout, quem):
            \"\"\"GARRA_COLETA: publica e espera a 1a resposta. (True/False/None, texto).\"\"\"
            if not self.esperar(5.0, lambda: pub.get_subscription_count() > 0):
                return None, 'ninguem escutando %s (%s rodando?)' % (pub.topic_name, quem)
            del respostas[:]
            pub.publish(msg)
            if not self.esperar(timeout, lambda: bool(respostas)):
                return None, 'sem resposta em %.0f s' % timeout
            return respostas[0] == 'ok', respostas[0]

        def coletar(self, area, objetos, timeout):
            return self._pedir(self.pub_coleta, String(data=json.dumps(
                {'area': area, 'objetos': [str(o) for o in objetos]})),
                self.res_coleta, timeout, 'alcance_cubo.py')

        def entregar(self, x, z, timeout):
            return self._pedir(self.pub_entrega, Point(x=float(x), y=0.0, z=float(z)),
                               self.res_entrega, timeout, 'pegar_cubo.py')
"""),
    ("""    from std_msgs.msg import String

    class Exec(Node):
""",
     """    from std_msgs.msg import String
    from geometry_msgs.msg import Point  # GARRA_COLETA
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # GARRA_COLETA
    from std_msgs.msg import Bool  # GARRA_COLETA

    class Exec(Node):
"""),
    # --- laco: pegar/soltar pela garra
    ("""    feitos, codigo = [], 0
    try:
""",
     """    feitos, codigo = [], 0
    na_garra = []  # GARRA_COLETA: objetos que a garra esta segurando
    try:
"""),
    ("""            else:
                acao = 'PEGAR' if p[0] == 'pegar' else 'SOLTAR'
                log.info('[%d/%d] %s objeto(s) %s em %s (simulado: sem braco)' % (
""",
     """            elif args.garra:  # GARRA_COLETA (23/11)
                objs = ', '.join(str(o) for o in p[2])
                if p[0] == 'pegar' and na_garra:
                    ok, txt = False, 'garra ocupada com %s: nao pega (derrubaria o cubo aqui)' % na_garra
                elif p[0] == 'soltar' and not na_garra:
                    ok, txt = False, 'nada na garra para soltar'
                elif p[0] == 'pegar':
                    log.info('[%d/%d] PEGAR %s em %s pela garra' % (i, len(passos), objs, p[1]))
                    ok, txt = no.coletar(p[1], p[2], args.tempo_coleta)
                    if ok or no.cubo_preso:
                        na_garra = list(p[2])
                else:
                    z = altura_mesa(p[1]) + MEIO_CUBO
                    log.info('[%d/%d] SOLTAR %s em %s pela garra (x=%.3f z=%.3f)' % (
                        i, len(passos), objs, p[1], args.entrega_x, z))
                    ok, txt = no.entregar(args.entrega_x, z, args.tempo_entrega)
                    if ok or no.cubo_preso is False:
                        na_garra = []
                if not ok:
                    log.warning('%s %s em %s: %s %s (a prova segue)' % (
                        p[0].upper(), objs, p[1], 'FALHOU' if ok is False else 'SEM RESPOSTA', txt))
                feitos.append((p, ok, txt))
            else:
                acao = 'PEGAR' if p[0] == 'pegar' else 'SOLTAR'
                log.info('[%d/%d] %s objeto(s) %s em %s (simulado: sem braco)' % (
"""),
    # --- argumentos e capacidade
    ("""    ap.add_argument('--tentativas', type=int, default=2)
""",
     """    # GARRA_COLETA (23/11)
    ap.add_argument('--garra', action='store_true',
                    help='pegar/soltar pela garra (alcance_cubo.py + pegar_cubo.py); capacidade 1')
    ap.add_argument('--tempo-coleta', type=float, default=120.0, help='s maximos por PEGAR com --garra')
    ap.add_argument('--tempo-entrega', type=float, default=90.0, help='s maximos por SOLTAR com --garra')
    ap.add_argument('--entrega-x', type=float, default=0.445,
                    help='x (m, robo_base) do centro do cubo ao soltar na mesa (padrao 0.445)')
    ap.add_argument('--tentativas', type=int, default=2)
"""),
    ("""    passos = montar_plano(tarefa, pos)
""",
     """    passos = montar_plano(tarefa, pos, capacidade=CAPACIDADE_GARRA if args.garra else CAPACIDADE)  # GARRA_COLETA
"""),
]

if __name__ == '__main__':
    rodar('garra_coleta', 'GARRA_COLETA', PADRAO, TROCAS, doc=__doc__)
