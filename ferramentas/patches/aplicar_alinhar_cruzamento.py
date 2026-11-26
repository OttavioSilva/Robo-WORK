#!/usr/bin/env python3
"""ALINHAR_CRUZAMENTO (19/11/2026) - alinhador (TravessiaLateral): no cruzamento, antes do 1o deslize, gira para
ficar paralelo a parede da frente (a parede do corredor NOVO), mesmo vendo uma parede so.

Teste 19/11 (log ~/logs/reinicio_arena_20261119_114527_z4w1qele/nav2_preparo_lateral_120516.log, sem bag):
- os corredores reais do L nao sao a 90 graus: pelo log, o horizontal ficou alinhado em ~+3,5 graus e o vertical em
  ~-95,5 graus no mapa (~9 graus fora do esquadro; o v3.25 ja tinha visto ~10 graus no corredor oeste em 03/11);
- o robo chega no cruzamento alinhado com o corredor ANTIGO e a saida lateral nao gira: com uma parede so o
  angulo e forcado a 0 (ANGULO_FUNDIDO/UMA_PAREDE) -> "Medicao: girar +0.0 graus (1 parede(s))" em todas;
- volta 12:29: deslizou torto, folga frente 7-10 / tras 3-4 cm (reto seriam ~21 cm somados: ~6-8 graus),
  "Perto da parede: afastando +-4 cm" 7x -> "encostando na parede (7 vezes)" -> FALHOU;
- ida 12:26: "folga frente 0" logo no inicio do deslize (mesma inclinacao).

Agora: so no estado 'medir' ANTES do 1o deslize desta saida (o deslize continua como antes, angulo 0 com uma
parede), se nao ha medida das 2 paredes e a parede da FRENTE e comprida (n >= 15 pontos) em pelo menos 3 scans
parados com dispersao <= 1,5 graus, usa a mediana do angulo dela (3 a 12 graus) para girar. Com a frente a menos
de 10 cm, primeiro recua (pulso de ajuste) para o giro nao levar a quina na parede. No maximo 3 giros desses.
O log diz "ALINHAR_CRUZAMENTO: ...". Depois do giro, o rumo do deslize (yaw_ref) e o rumo novo.
Parametro do alinhador: alinhar_cruzamento (padrao true; false = como antes)
  ros2 param set /alinhador_passagem alinhar_cruzamento false
Uso: python3 aplicar_alinhar_cruzamento.py [--raiz <copia do workspace>] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_multi import rodar  # noqa: E402

MARCA = 'ALINHAR_CRUZAMENTO (19/11)'
AL = 'src/robo_navigation/scripts/alinhador_passagem.py'

TROCAS = [
    # 1) atributos
    ("        self.pulso_adapta = False  # PULSO_GIRO_ADAPTA (13/11): ligado pelo no (parametro pulso_giro_adapta)\n",
     "        self.pulso_adapta = False  # PULSO_GIRO_ADAPTA (13/11): ligado pelo no (parametro pulso_giro_adapta)\n"
     "        # ALINHAR_CRUZAMENTO (19/11): antes do 1o deslize, gira pela parede da frente (corredor novo) mesmo com\n"
     "        # uma parede so; ligado pelo no. Corredores reais ~9 graus fora do esquadro no teste de 19/11.\n"
     "        self.alinhar_cruz = False\n"
     "        self.n_cruz = 0\n"),
    # 2) no 'medir': angulo da parede da frente antes do 1o deslize
    ("            if self.fundido:\n"
     "                ang = sorted(m[1]['angulo'] for m in duas)[len(duas) // 2] if duas else 0.0\n"
     "            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]\n",
     "            if self.fundido:\n"
     "                ang = sorted(m[1]['angulo'] for m in duas)[len(duas) // 2] if duas else 0.0\n"
     "            cruz = self._angulo_cruzamento(novas, duas)   # ALINHAR_CRUZAMENTO (19/11)\n"
     "            ff_c, ft_c = self._folgas(t)\n"
     "            if cruz is not None and ff_c is not None and ff_c < 0.10 \\\n"
     "                    and not (ft_c is None or ft_c - (0.12 - ff_c) >= 0.08):\n"
     "                cruz = None   # sem espaco para recuar antes do giro: segue como antes\n"
     "            if cruz is not None:\n"
     "                if ff_c is not None and ff_c < 0.10:\n"
     "                    self.n_cruz += 1\n"
     "                    self.alvo = -(0.12 - ff_c)\n"
     "                    self.pulso = min(self.pulso_max, max(0.08, abs(self.alvo) / self.v_real))\n"
     "                    self.p_correcao = pose\n"
     "                    self.log('ALINHAR_CRUZAMENTO: parede da frente a %+.1f graus mas so %.0f cm a frente: '\n"
     "                             'recuo %.0f cm antes de girar' % (math.degrees(cruz), ff_c * 100, -self.alvo * 100))\n"
     "                    self._mudar('ajustar', t, pose)\n"
     "                    return 0.0, 0.0, 0.0\n"
     "                self.n_cruz += 1\n"
     "                self.correcoes += 1\n"
     "                self.alvo = _ang(novas[-1][2][2] + cruz)\n"
     "                self.pulso = min(0.40, max(0.05, abs(cruz) / self.w_real - 0.06))\n"
     "                self.log('ALINHAR_CRUZAMENTO: parede da frente (corredor novo) a %+.1f graus com uma parede so: '\n"
     "                         'giro antes de deslizar (%d/3)' % (math.degrees(cruz), self.n_cruz))\n"
     "                self._mudar('girar', t, pose)\n"
     "                return 0.0, 0.0, 0.0\n"
     "            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]\n"),
    # 3) metodo auxiliar
    ("    def _avaliar_pulso(self, pose):\n",
     "    def _angulo_cruzamento(self, novas, duas):\n"
     "        \"\"\"ALINHAR_CRUZAMENTO (19/11): angulo (rad) da parede da frente para girar antes do 1o deslize, ou None.\"\"\"\n"
     "        if not self.alinhar_cruz or not self.fundido or duas or self.p_desl is not None or self.n_cruz >= 3:\n"
     "            return None\n"
     "        fr = [m[1]['angulo'] for m in novas\n"
     "              if m[1]['lados'] == 1 and m[1]['frente'] is not None and m[1]['n'] >= 15]\n"
     "        if len(fr) < 3 or max(fr) - min(fr) > math.radians(1.5):\n"
     "            return None\n"
     "        a = sorted(fr)[len(fr) // 2]\n"
     "        if not math.radians(3.0) <= abs(a) <= math.radians(12.0):\n"
     "            return None\n"
     "        return a\n"
     "\n"
     "    def _avaliar_pulso(self, pose):\n"),
    # 4) parametro
    ("            self.declare_parameter('recuo_curva', True)   # RECUO_CURVA (12/11): false = como antes\n",
     "            self.declare_parameter('recuo_curva', True)   # RECUO_CURVA (12/11): false = como antes\n"
     "            # ALINHAR_CRUZAMENTO (19/11): gira pela parede da frente antes do 1o deslize da curva (false = como antes)\n"
     "            self.declare_parameter('alinhar_cruzamento', True)\n"),
    # 5) liga no no
    ("            try:   # PULSO_GIRO_ADAPTA (13/11)\n"
     "                tl.pulso_adapta = bool(self.p('pulso_giro_adapta'))\n"
     "            except Exception:\n"
     "                pass\n",
     "            try:   # PULSO_GIRO_ADAPTA (13/11)\n"
     "                tl.pulso_adapta = bool(self.p('pulso_giro_adapta'))\n"
     "            except Exception:\n"
     "                pass\n"
     "            try:   # ALINHAR_CRUZAMENTO (19/11)\n"
     "                tl.alinhar_cruz = bool(self.p('alinhar_cruzamento'))\n"
     "            except Exception:\n"
     "                pass\n"),
]

if __name__ == '__main__':
    rodar('alinhar_cruzamento', MARCA, [(AL, TROCAS)], doc=__doc__)
