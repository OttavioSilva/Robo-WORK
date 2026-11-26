#!/usr/bin/env python3
"""REINSISTIR (11/11/2026) - missao33. Quando o Nav2 desiste do GOAL FINAL (destravar 'sem alternativa segura' ->
Goal failed), o alinhador repete sozinho o que o operador fazia mandando o goal de novo: reanalisa o caminho,
vira para ele (VIRAR_CAMINHO, com o recuo/lateral minimo que libera o giro) e manda o goal ao Nav2 outra vez.
Evidencia: missao33 10:33:10 Goal failed; 10:33:14 operador manda goal novo -> pre-giro de 120 graus (6 cm de re,
3 cm de lado) -> 10:33:43 Goal succeeded.
Parametro novo do alinhador `reinsistir_goal_final` (padrao 2 tentativas; 0 = como antes). So para falha do Nav2
no GOAL FINAL; goal novo do operador continua interrompendo na hora (Cancelado). Sem mudanca no Nav2/destravar.
Em tempo de execucao: ros2 param set /alinhador_passagem reinsistir_goal_final 0   (desliga)
Uso: python3 aplicar_reinsistir.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("            self.declare_parameter('giro_final_comum', False)\n",
     "            self.declare_parameter('giro_final_comum', False)\n"
     "            # REINSISTIR (11/11): Nav2 desistiu do GOAL FINAL -> reanalisa, vira e manda de novo (0 desliga)\n"
     "            self.declare_parameter('reinsistir_goal_final', 2)\n"),
    ("""                        resultado = self._navegar(alvo, 'GOAL FINAL', vigia_final, aceitar_perto=True)
                        if resultado == 'ok':
                            resultado = self._giro_final(gx, gy, gyaw, 'GOAL FINAL')
                    else:
                        resultado = self._navegar(goal, 'GOAL FINAL', vigia_final, aceitar_perto=livre)
""", """                        resultado = self._navegar(alvo, 'GOAL FINAL', vigia_final, aceitar_perto=True)
                        nav_falhou = resultado == 'falha'   # REINSISTIR (11/11)
                        if resultado == 'ok':
                            resultado = self._giro_final(gx, gy, gyaw, 'GOAL FINAL')
                    else:
                        resultado = self._navegar(goal, 'GOAL FINAL', vigia_final, aceitar_perto=livre)
                        nav_falhou = resultado == 'falha'   # REINSISTIR (11/11)
                    # REINSISTIR (11/11, missao33): o Nav2 desistiu (destravar sem alternativa); antes de dar FALHOU,
                    # faz o que o operador fazia: reanalisa, vira para o caminho e manda o goal de novo
                    if nav_falhou and alerta.get('reinsistir', 0) < int(self.p('reinsistir_goal_final')) \\
                            and not self._novo_goal_chegou():
                        alerta['reinsistir'] = alerta.get('reinsistir', 0) + 1
                        self.get_logger().warn(
                            'REINSISTIR %d/%d: o Nav2 desistiu do GOAL FINAL; reanaliso, viro para o caminho e '
                            'mando de novo' % (alerta['reinsistir'], int(self.p('reinsistir_goal_final'))))
                        fim_espera = time.monotonic() + 1.0
                        while time.monotonic() < fim_espera and not self._novo_goal_chegou():
                            time.sleep(0.05)
                        self.hist_nav = None
                        continue
"""),
]
if __name__ == '__main__':
    rodar('reinsistir', 'REINSISTIR (11/11)', ARQ, TROCAS, doc=__doc__)
