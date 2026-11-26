#!/usr/bin/env python3
"""CONTROLADOR_ORFAO_ALINHADOR (11/11/2026) - alinhador (caminho legado).
Evidencia: bag ensaio_goal_superior: NavigateToPose cancelado 50 ms depois do BT mandar um FollowPath; o
controller_server nao recebeu o cancelamento e seguiu o caminho do goal anterior (re -0,10 m/s por ~10 s;
odom 1,49 -> 1,04 m). O NavigateToPose estava terminal, o FollowPath nao.
O que muda: ao fim de cada NavigateToPose do caminho legado (sucesso, falha, cancelamento ou goal novo), espera
0,35 s; se o controlador (/cmd_vel_ctrl) ainda mandou comando nao nulo depois de 0,2 s do fim, cancela todos os
goals do FollowPath (/follow_path/_action/cancel_goal, goal_info vazio) e espera ate 1,5 s o controlador calar.
No perfil nav2_principal nao faz nada (la o arbitro corta). Parametro: cancelar_controlador_orfao (True).
Uso: python3 aplicar_controlador_orfao_alinhador.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""            self.declare_parameter('ciscado_max', 3)          # intervencoes por goal
""", """            self.declare_parameter('ciscado_max', 3)          # intervencoes por goal
            # CONTROLADOR_ORFAO_ALINHADOR (11/11): FollowPath que sobra depois do NavigateToPose terminar
            self.declare_parameter('cancelar_controlador_orfao', True)
"""),
    ("""            self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
""", """            self.ctrl_nz_t = None   # CONTROLADOR_ORFAO_ALINHADOR: ultimo comando nao nulo do controlador
            self.create_subscription(TwistStamped, '/cmd_vel_ctrl', self.ao_cmd_ctrl, 20, callback_group=grupo)
            from action_msgs.srv import CancelGoal
            self.cli_cancel_fp = self.create_client(CancelGoal, '/follow_path/_action/cancel_goal',
                                                    callback_group=grupo)
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
"""),
    ("""        def _novo_goal_chegou(self):
""", """        def ao_cmd_ctrl(self, msg):   # CONTROLADOR_ORFAO_ALINHADOR
            t = msg.twist
            if abs(t.linear.x) > 1e-3 or abs(t.linear.y) > 1e-3 or abs(t.angular.z) > 1e-3:
                with self.lock:
                    self.ctrl_nz_t = time.monotonic()

        def _verificar_controlador_orfao(self, nome, espera=0.35, calar=1.5):
            \"\"\"CONTROLADOR_ORFAO_ALINHADOR (11/11): NavigateToPose terminal e o controlador ainda comanda ->
            cancela todos os FollowPath e espera calar. Devolve 'ok', 'cancelado' ou 'persistente'.\"\"\"
            if self.p('nav2_principal') or not self.p('cancelar_controlador_orfao'):
                return 'ok'
            t_fim = time.monotonic()
            time.sleep(espera)
            with self.lock:
                c = self.ctrl_nz_t
            if c is None or c < t_fim + 0.2:
                return 'ok'
            from action_msgs.srv import CancelGoal
            self.get_logger().warn('CONTROLADOR_ORFAO: %s terminou mas o controlador segue comandando ha %.1f s: '
                                   'cancelo o FollowPath' % (nome, c - t_fim))
            if self.cli_cancel_fp.wait_for_service(timeout_sec=0.5):
                f = self.cli_cancel_fp.call_async(CancelGoal.Request())
                t1 = time.monotonic() + 1.0
                while not f.done() and time.monotonic() < t1:
                    time.sleep(0.02)
            t_cancel = time.monotonic()
            while time.monotonic() < t_cancel + calar:
                time.sleep(0.05)
                with self.lock:
                    c = self.ctrl_nz_t
                if c is not None and time.monotonic() - c >= 0.3:
                    self.get_logger().info('CONTROLADOR_ORFAO: controlador parado %.1f s depois do cancelamento' % (
                        c - t_cancel if c > t_cancel else 0.0))
                    return 'cancelado'
            self.get_logger().error('CONTROLADOR_ORFAO: o controlador continua comandando depois de cancelar')
            return 'persistente'

        def _novo_goal_chegou(self):
"""),
    ("""            finally:
                self.handle_ativo = None
            ok = r.status == GoalStatus.STATUS_SUCCEEDED
""", """            finally:
                self.handle_ativo = None
                self._verificar_controlador_orfao(nome)   # CONTROLADOR_ORFAO_ALINHADOR (11/11)
            ok = r.status == GoalStatus.STATUS_SUCCEEDED
"""),
]
if __name__ == '__main__':
    rodar('controlador_orfao_alinhador', 'CONTROLADOR_ORFAO_ALINHADOR (11/11)', ARQ, TROCAS, doc=__doc__)
