#!/usr/bin/env python3
"""CONTROLADOR_ORFAO (11/11/2026) - recuperacao_avancada.py (destravar v4).
Evidencia: bag ensaio_goal_superior (11/11 15:4x): FollowPath recebido 44,90 s, NavigateToPose cancelado 44,95 s sem
'Client requested to cancel' no controller_server (halt antes da aceitacao) -> o controlador seguiu o caminho do goal
anterior com re -0,10 m/s ate 55,6 s ('Failed to make progress'). O destravar chamado nesse meio recusou 2x com
'outro no comandando /cmd_vel_nav: sem controle exclusivo' (recusa correta: havia comando concorrente real).
O que muda: antes da ETAPA A, se nao ha exclusividade e o controlador (/cmd_vel_ctrl) comandou nos ultimos 0,5 s,
pede ao io para cancelar os FollowPath (io.cancelar_controlador, se existir) e espera ate 2 s PARADO (sem publicar)
o comando ficar exclusivo. Depois disso a decisao segue igual: sem exclusividade -> 'falhar' (a protecao nao muda).
Comando estrangeiro que nao e do controlador nunca e cancelado: so espera.
Uso: python3 aplicar_controlador_orfao_recuperacao.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/recuperacao_avancada.py')
TROCAS = [
    ("""    def _exclusivo(self):
        agora = self.io.agora()
        e, c = self.io.ultimo_estrangeiro(), self.io.ultimo_ctrl()
        return (e is None or agora - e > 1.0) and (c is None or agora - c > 0.5)
""", """    def _exclusivo(self):
        agora = self.io.agora()
        e, c = self.io.ultimo_estrangeiro(), self.io.ultimo_ctrl()
        return (e is None or agora - e > 1.0) and (c is None or agora - c > 0.5)

    def _garantir_exclusivo(self, t_fim, espera=2.0):
        \"\"\"CONTROLADOR_ORFAO (11/11): controlador ainda comandando quando a recuperacao comeca = FollowPath orfao
        (a arvore so chama a recuperacao com o FollowPath parado). Cancela o FollowPath e espera a exclusividade
        parado; nunca anda nem ignora o outro comando. Devolve se ficou exclusivo.\"\"\"
        if self._exclusivo():
            return True
        io = self.io
        c = io.ultimo_ctrl()
        if c is not None and io.agora() - c <= 0.5 and hasattr(io, 'cancelar_controlador'):
            aceito = io.cancelar_controlador()
            io.log('warn', 'CONTROLADOR_ORFAO: o controlador ainda comandava com a recuperacao chamada; '
                           'pedi cancelar o FollowPath (%s)' % ('aceito' if aceito else 'sem resposta'))
        t1 = min(t_fim - 1.0, io.agora() + espera)
        t_ini = io.agora()
        while io.agora() < t1:
            if io.cancelado():
                return False
            if self._exclusivo():
                io.log('info', 'CONTROLADOR_ORFAO: comando exclusivo depois de %.1f s' % (io.agora() - t_ini))
                return True
            io.dormir(0.1)
        ok = self._exclusivo()
        if not ok:
            io.log('warn', 'CONTROLADOR_ORFAO: outro no continua comandando depois de %.1f s' % (io.agora() - t_ini))
        return ok
"""),
    ("""        # ---- ETAPA A
        jan = io.janela()
""", """        self._garantir_exclusivo(t_fim)   # CONTROLADOR_ORFAO (11/11): a decisao continua exigindo exclusividade
        # ---- ETAPA A
        jan = io.janela()
"""),
]
if __name__ == '__main__':
    rodar('controlador_orfao', 'CONTROLADOR_ORFAO (11/11)', ARQ, TROCAS, doc=__doc__)
