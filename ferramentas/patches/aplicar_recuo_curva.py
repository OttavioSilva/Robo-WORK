#!/usr/bin/env python3
"""RECUO_CURVA (12/11/2026) - alinhador, TravessiaLateral.bloqueio (saida lateral da curva).

Bag corredor_re_freio_curva_20261112_113352 (11:45:44-11:45:55): a travessia parou no cruzamento ("parede a 9 cm",
o robo ainda embalou ~4 cm) e a saida lateral para a ESQUERDA foi barrada 4x pelo Collision Monitor: o /scan_nav
(fonte do CM, com a memoria do ponto cego) tinha o objeto a 4,6-5 cm do para-choque, dentro do poligono 'left'
(vai ate 5 cm a frente do para-choque). A medicao dizia "centro -4,2 cm a frente": o robo tinha passado ~4 cm.
Antes: so parava e media de novo (4x) e abortava. O usuario: "teria que dar um pouco de re e iniciar o lateral".

Agora: CM segurando a saida lateral (qualquer curva, esquerda ou direita) com a traseira livre (>= 8 cm ou sem
medida): recua ~5 cm (pulso, estado 'ajustar') e desliza de novo; com a traseira curta e a frente medida >= 10 cm,
avanca ~5 cm. v2 (12:07, volta para a DIREITA): o CM barrou com a frente 'medida' a 14 cm no /scan_front e pontos
lembrados a 4,3 cm do para-choque no /scan_nav (zona cega): a frente medida nao serve de criterio perto do corpo. No maximo 2 por curva (nao contam no limite de 4 bloqueios). O CM continua valendo no recuo.
Parametro recuo_curva (true padrao; false = como antes).
Uso: python3 aplicar_recuo_curva.py [--arquivo copia.py] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar

ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')
TROCAS = [
    ("        self.p_correcao = None     # pose na ultima correcao (progresso desde ela)\n",
     "        self.p_correcao = None     # pose na ultima correcao (progresso desde ela)\n"
     "        self.recuo_curva = False   # RECUO_CURVA (12/11): ligado pelo no\n"
     "        self.n_recuo_curva = 0\n"),
    ("        \"\"\"Collision Monitor segurando: para, mede e corrige (afasta da parede) antes de desistir.\"\"\"\n"
     "        self.n_bloq += 1\n",
     "        \"\"\"Collision Monitor segurando: para, mede e corrige (afasta da parede) antes de desistir.\"\"\"\n"
     "        if self.recuo_curva and self.n_recuo_curva < 2 and self.estado not in ('fim', 'abortar', 'ajustar'):\n"
     "            # RECUO_CURVA (12/11): o poligono lateral do CM vai 5 cm alem do para-choque e da traseira;\n"
     "            # parado colado na parede/objeto da frente (ou de tras), anda ~5 cm para longe e desliza de novo\n"
     "            ff, ft = self._folgas(t)\n"
     "            sentido = 0.0\n"
     "            # v2 (12/11 12:07, volta): o /scan_nav do CM lembrava pontos a 4 cm do para-choque com a frente\n"
     "            # 'medida' a 14 cm no /scan_front: perto do corpo a frente e cega. Traseira com espaco: recua.\n"
     "            if ft is None or ft >= 0.08:\n"
     "                sentido = -1.0\n"
     "            elif ff is not None and ff >= 0.10:\n"
     "                sentido = 1.0\n"
     "            if sentido:\n"
     "                self.n_recuo_curva += 1\n"
     "                self.alvo = 0.05 * sentido\n"
     "                self.pulso = min(self.pulso_max, 0.05 / self.v_real)\n"
     "                self.log('RECUO_CURVA: Collision Monitor segurando em \"%s\" (folga frente %s / tras %s cm): '\n"
     "                         '%s 5 cm e deslizo de novo (%d/2)' % (\n"
     "                             self.estado, '-' if ff is None else '%.0f' % (ff * 100),\n"
     "                             '-' if ft is None else '%.0f' % (ft * 100),\n"
     "                             'recuo' if sentido < 0 else 'avanco', self.n_recuo_curva))\n"
     "                self.p_correcao = pose\n"
     "                self._mudar('ajustar', t, pose)\n"
     "                return\n"
     "        self.n_bloq += 1\n"),
    ("            tl.restante_fora = margem - 0.35   # v3.18: corpo inteiro alem da saida do corredor\n",
     "            tl.restante_fora = margem - 0.35   # v3.18: corpo inteiro alem da saida do corredor\n"
     "            try:   # RECUO_CURVA (12/11)\n"
     "                tl.recuo_curva = bool(self.p('recuo_curva'))\n"
     "            except Exception:\n"
     "                pass\n"),
    ("            self.declare_parameter('entrada_rapida', True)      # ENTRADA_RAPIDA\n",
     "            self.declare_parameter('entrada_rapida', True)      # ENTRADA_RAPIDA\n"
     "            self.declare_parameter('recuo_curva', True)   # RECUO_CURVA (12/11): false = como antes\n"),
]
if __name__ == '__main__':
    rodar('recuo_curva', 'RECUO_CURVA (12/11)', ARQ, TROCAS, doc=__doc__)
