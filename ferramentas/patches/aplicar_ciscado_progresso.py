#!/usr/bin/env python3
"""CISCADO_PROGRESSO (11/11/2026) - alinhador, ANTI_CISCADO.
Antes: ciscou -> cancela, chama o destravar e manda o mesmo goal de novo, ate ciscado_max (3) vezes, mesmo que o
destravar tenha falhado todas as vezes (mesma correcao repetida sem progresso).
O que muda: _chamar_destravar devolve se o destravar terminou com sucesso. A partir da 2a intervencao no mesmo
goal, se o destravar falhou, nao reenvia: devolve 'falha' (o executor/operador decide). A 1a intervencao continua
reenviando como antes. ciscado_exige_destravar False = como antes.
Uso: python3 aplicar_ciscado_progresso.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""            self.declare_parameter('ciscado_max', 3)          # intervencoes por goal
""", """            self.declare_parameter('ciscado_max', 3)          # intervencoes por goal
            # CISCADO_PROGRESSO (11/11): da 2a intervencao em diante, so reenvia se o destravar resolveu
            self.declare_parameter('ciscado_exige_destravar', True)
"""),
    ("""            if not self.destravar_cli.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn('%s: destravar indisponivel' % nome)
                return
""", """            if not self.destravar_cli.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn('%s: destravar indisponivel' % nome)
                return False   # CISCADO_PROGRESSO (11/11): devolve se resolveu
"""),
    ("""            if h is None or not h.accepted:
                self.get_logger().warn('%s: destravar recusou' % nome)
                return
            r = self._esperar(h.get_result_async(), 35.0)
            ok = r is not None and r.status == GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('%s: destravar %s' % (nome, 'terminou' if ok else 'falhou'))
""", """            if h is None or not h.accepted:
                self.get_logger().warn('%s: destravar recusou' % nome)
                return False
            r = self._esperar(h.get_result_async(), 35.0)
            ok = r is not None and r.status == GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('%s: destravar %s' % (nome, 'terminou' if ok else 'falhou'))
            return ok
"""),
    ("""                            self._chamar_destravar('ANTI_CISCADO %d' % (_cisc + 1))
""", """                            destravou = self._chamar_destravar('ANTI_CISCADO %d' % (_cisc + 1))
                            if not destravou and _cisc >= 1 and self.p('ciscado_exige_destravar'):
                                self.get_logger().warn('ANTI_CISCADO %d: %s: o destravar nao resolveu de novo; nao '
                                                       'repito o goal (CISCADO_PROGRESSO)' % (_cisc + 1, nome))
                                return 'falha'
"""),
]
if __name__ == '__main__':
    rodar('ciscado_progresso', 'CISCADO_PROGRESSO (11/11)', ARQ, TROCAS, doc=__doc__)
