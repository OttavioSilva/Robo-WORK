#!/usr/bin/env python3
"""CURVA_INSISTE (11/11/2026) - missao33/34. A saida lateral do cruzamento (CURVA) abortava ('Collision Monitor
bloqueou 4 vezes' / 'tempo maximo') e o alinhador PARAVA esperando ajuda. Agora:
  1) repete a saida lateral a partir de onde esta, ate `insistir_curva` vezes (padrao 2);
  2) se ainda falhar, NAO para: marca a curva como tratada e segue para o goal pelo Nav2 (com VIRAR_CAMINHO e
     REINSISTIR) - o que funcionou na missao34 quando o operador mandou o goal de novo (11:04:59 -> 11:07:25).
`insistir_curva` 0 + `curva_segue_nav2` false = como antes. Em tempo de execucao: ros2 param set /alinhador_passagem ...
Uso: python3 aplicar_curva_insiste.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("            self.declare_parameter('reinsistir_goal_final', 2)\n",
     "            self.declare_parameter('reinsistir_goal_final', 2)\n"
     "            # CURVA_INSISTE (11/11): saida lateral falhou -> repete ate N vezes e depois segue pelo Nav2\n"
     "            self.declare_parameter('insistir_curva', 2)\n"
     "            self.declare_parameter('curva_segue_nav2', True)\n"),
    ("""                            else:
                                r = self._travessia_lateral(curva, 'CURVA %d' % n)
                            if r == 'ok':
                                if self.p('modo_curva_t') == 'giro_frontal':""",
     """                            else:
                                r = self._travessia_lateral(curva, 'CURVA %d' % n)
                                # CURVA_INSISTE (11/11): na prova nao pode parar; repete a saida lateral daqui
                                k_ins = 0
                                while r != 'ok' and k_ins < int(self.p('insistir_curva')) and \\
                                        not self._novo_goal_chegou():
                                    k_ins += 1
                                    self.get_logger().warn('CURVA_INSISTE %d/%d: a saida lateral falhou; tento de '
                                                           'novo daqui' % (k_ins, int(self.p('insistir_curva'))))
                                    r = self._travessia_lateral(curva, 'CURVA %d (insiste %d)' % (n, k_ins))
                            if r == 'ok':
                                if self.p('modo_curva_t') == 'giro_frontal':"""),
    ("""                            self.get_logger().error(
                                'Nao consegui sair de lado pelo corredor novo (curva %d). Parei; ajude o '
                                'robo e mande o goal de novo.' % n)
                            return""",
     """                            if self.p('curva_segue_nav2') and not self._novo_goal_chegou():   # CURVA_INSISTE
                                self.get_logger().warn('CURVA_INSISTE: saida lateral da curva %d falhou de novo; '
                                                       'nao paro: sigo para o goal pelo Nav2' % n)
                                tratadas.append(curva['entrada_b'])
                                continue
                            self.get_logger().error(
                                'Nao consegui sair de lado pelo corredor novo (curva %d). Parei; ajude o '
                                'robo e mande o goal de novo.' % n)
                            return"""),
]
if __name__ == '__main__':
    rodar('curva_insiste', 'CURVA_INSISTE (11/11)', ARQ, TROCAS, doc=__doc__)
