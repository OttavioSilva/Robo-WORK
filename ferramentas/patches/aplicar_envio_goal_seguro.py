#!/usr/bin/env python3
"""ENVIO_GOAL_SEGURO (11/11/2026) - alinhador, caminho LEGADO (NavigateToPose e destravar).
Defeito (test/test_substituicao_goal_ros.py, ROS real em dominio isolado): se o goal novo chega (ou o timeout de
aceitacao estoura) enquanto o send_goal_async ainda esta pendente, _esperar sai com Cancelado/None antes do handle
existir. ao_goal/_cancelar_nav nao tem handle para cancelar e o goal aceito depois fica rodando no Nav2. O
bt_navigator real so o substitui no proximo envio; ate la ele dirige junto com o pre-giro, o destravar ou as
manobras do alinhador (dois donos do /cmd_vel_nav). O mesmo vale para o goal do destravar.
O que muda: _enviar_goal(cli, req, timeout). Se sair sem handle (Cancelado ou timeout), registra um callback que
cancela o goal assim que ele for aceito. Mesmo mecanismo do 'cancelar_tardio' do perfil nav2_principal.
Uso: python3 aplicar_envio_goal_seguro.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("""        def _novo_goal_chegou(self):
""", """        def _enviar_goal(self, cli, req, timeout):
            \"\"\"ENVIO_GOAL_SEGURO (11/11): envia e espera a aceitacao. Saiu sem handle (goal novo/cancelamento ->
            Cancelado, ou timeout -> None)? O goal aceito depois e cancelado na hora, para nao ficar orfao.\"\"\"
            envio = cli.send_goal_async(req)
            log = self.get_logger()

            def cancelar_tardio(f):
                try:
                    h = f.result()
                except Exception:
                    return
                if h is not None and h.accepted:
                    log.warn('ENVIO_GOAL_SEGURO: goal aceito depois de abandonado: cancelo')
                    h.cancel_goal_async()
            try:
                h = self._esperar(envio, timeout)
            except Cancelado:
                envio.add_done_callback(cancelar_tardio)
                raise
            if h is None:
                envio.add_done_callback(cancelar_tardio)
            return h

        def _novo_goal_chegou(self):
"""),
    ("""            h = self._esperar(self.destravar_cli.send_goal_async(g), 3.0)
""", """            h = self._enviar_goal(self.destravar_cli, g, 3.0)   # ENVIO_GOAL_SEGURO
"""),
    ("""            h = self._esperar(self.nav.send_goal_async(req), 5.0)
            if h is None or not h.accepted:
                self.get_logger().warn('%s: goal recusado' % nome)
""", """            h = self._enviar_goal(self.nav, req, 5.0)   # ENVIO_GOAL_SEGURO (11/11)
            if h is None or not h.accepted:
                self.get_logger().warn('%s: goal %s' % (nome, 'recusado' if h is not None else 'sem aceitacao em 5 s'))
"""),
]
if __name__ == '__main__':
    rodar('envio_goal_seguro', 'ENVIO_GOAL_SEGURO (11/11)', ARQ, TROCAS, doc=__doc__)
