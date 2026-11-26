#!/usr/bin/env python3
"""CONTROLADOR_ORFAO_NO (11/11/2026) - destravar_node.py: io.cancelar_controlador() usado por CONTROLADOR_ORFAO.
Chama /follow_path/_action/cancel_goal com goal_info vazio (UUID e stamp zero = cancelar TODOS os goals do
FollowPath, padrao do action_msgs/CancelGoal). So e chamado pela recuperacao quando o controlador ainda comanda
durante a recuperacao (a arvore nunca roda FollowPath e destravar ao mesmo tempo). Espera a resposta ate 1 s.
Uso: python3 aplicar_controlador_orfao_destravar.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/destravar_node.py')
TROCAS = [
    ("""            self.cli_cm = self.create_client(GetState, 'collision_monitor/get_state', callback_group=g)
""", """            self.cli_cm = self.create_client(GetState, 'collision_monitor/get_state', callback_group=g)
            # CONTROLADOR_ORFAO_NO (11/11): cancelar FollowPath orfao (todos os goals)
            from action_msgs.srv import CancelGoal
            self.cli_cancel_fp = self.create_client(CancelGoal, 'follow_path/_action/cancel_goal', callback_group=g)
"""),
    ("""        def replanejar(self, timeout=5.0):
            \"\"\"Pede um caminho novo ao planner.""", """        def cancelar_follow_path(self, timeout=1.0):
            \"\"\"CONTROLADOR_ORFAO_NO (11/11): cancela todos os goals do FollowPath. True = servidor respondeu.\"\"\"
            from action_msgs.srv import CancelGoal
            if not self.cli_cancel_fp.wait_for_service(timeout_sec=0.5):
                return False
            r = self._esperar(self.cli_cancel_fp.call_async(CancelGoal.Request()), timeout)
            return r is not None

        def replanejar(self, timeout=5.0):
            \"\"\"Pede um caminho novo ao planner."""),
    ("""        def ultimo_ctrl(self):
            return self.no.ctrl_t
""", """        def ultimo_ctrl(self):
            return self.no.ctrl_t

        def cancelar_controlador(self):   # CONTROLADOR_ORFAO_NO (11/11)
            return self.no.cancelar_follow_path()
"""),
]
if __name__ == '__main__':
    rodar('controlador_orfao_no', 'CONTROLADOR_ORFAO_NO (11/11)', ARQ, TROCAS, doc=__doc__)
