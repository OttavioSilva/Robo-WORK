"""ALINHAR_CRUZAMENTO (19/11): no cruzamento, antes do 1o deslize, gira pela parede da frente (corredor novo)
mesmo vendo uma parede so. Teste 19/11 12:29: volta pelo L com ~6-8 graus de inclinacao, 'girar +0.0 graus
(1 parede(s))' em todas as medicoes e 'encostando na parede (7 vezes)'. Sem ROS."""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robo_navigation/scripts'))
from alinhador_passagem import TravessiaLateral  # noqa: E402

POSE = (0.0, 0.0, 0.0)


def med(ang_graus, n=30, frente=0.20, lados=1):
    return dict(angulo=math.radians(ang_graus), desvio=0.0, frente=frente, lados=lados, n=n,
                largura=0.82 if lados == 2 else None)


def cruzamento(angs, ligado=True, folgas=(0.20, 0.25), **kw):
    """Robo parado no cruzamento, no estado medir, com uma medida por scan."""
    x = TravessiaLateral(-1, fundido=True, log=lambda _: None)
    x.alinhar_cruz = ligado
    x.t0 = 0.0
    x._mudar('medir', 1.0, POSE)
    for i, a in enumerate(angs):
        t = 1.05 + 0.1 * i
        x.nova_medicao(t, med(a, **kw), POSE)
        x.nova_folga(t, *folgas)
    return x


class TestAlinharCruzamento(unittest.TestCase):
    def passo(self, x):
        return x.passo(1.6, POSE, 1.0)

    def test_desligado_e_como_antes(self):
        x = cruzamento([7.0, 7.2, 7.1], ligado=False)
        self.passo(x)
        self.assertNotEqual(x.estado, 'girar')

    def test_parede_da_frente_inclinada_gira_antes_de_deslizar(self):
        x = cruzamento([7.0, 7.2, 7.1])
        self.passo(x)
        self.assertEqual(x.estado, 'girar')
        self.assertAlmostEqual(math.degrees(x.alvo), 7.1, places=1)
        self.assertEqual(x.n_cruz, 1)

    def test_angulo_espalhado_nao_gira(self):   # UMA_PAREDE: angulo de uma parede oscilando (missao23)
        x = cruzamento([4.0, 7.0, 6.0])
        self.passo(x)
        self.assertNotEqual(x.estado, 'girar')

    def test_angulo_pequeno_ou_grande_nao_gira(self):
        for a in (2.0, 13.0):
            x = cruzamento([a, a, a])
            self.passo(x)
            self.assertNotEqual(x.estado, 'girar', a)

    def test_parede_curta_nao_gira(self):
        x = cruzamento([7.0, 7.0, 7.0], n=12)
        self.passo(x)
        self.assertNotEqual(x.estado, 'girar')

    def test_frente_perto_recua_antes(self):
        x = cruzamento([7.0, 7.0, 7.0], folgas=(0.07, 0.28))
        self.passo(x)
        self.assertEqual(x.estado, 'ajustar')
        self.assertAlmostEqual(x.alvo, -0.05, places=3)
        vx, vy, wz = x.passo(1.7, POSE, 1.0)
        self.assertLess(vx, 0.0)
        self.assertEqual((vy, wz), (0.0, 0.0))

    def test_frente_perto_sem_espaco_atras_nao_gira(self):
        x = cruzamento([7.0, 7.0, 7.0], folgas=(0.07, 0.06))
        self.passo(x)
        self.assertNotIn(x.estado, ('girar',))

    def test_depois_do_primeiro_deslize_nao_age(self):
        x = cruzamento([7.0, 7.0, 7.0])
        x.p_desl = POSE
        self.passo(x)
        self.assertNotEqual(x.estado, 'girar')

    def test_no_maximo_tres_giros(self):
        x = cruzamento([7.0, 7.0, 7.0])
        x.n_cruz = 3
        self.passo(x)
        self.assertNotEqual(x.estado, 'girar')

    def test_com_duas_paredes_usa_a_logica_antiga(self):
        x = cruzamento([1.0, 1.0, 1.0], lados=2)
        self.passo(x)
        self.assertEqual(x.n_cruz, 0)

    def test_parametro(self):
        txt = (ROOT / 'src/robo_navigation/scripts/alinhador_passagem.py').read_text()
        self.assertIn("self.declare_parameter('alinhar_cruzamento', True)", txt)



class TestAlinharCruzamentoZero(unittest.TestCase):
    """ALINHAR_CRUZAMENTO_ZERO (19/11 noite): teste 19:3x parou em +5,6 graus (3/3) e deslizou diagonal; o CM
    parou o deslize 4 vezes com folga de 10 cm a frente."""

    def zero(self, angs, **kw):
        x = cruzamento(angs, **kw)
        x.cruz_zero = True
        return x

    def test_angulo_pequeno_gira_no_modo_zero(self):
        x = self.zero([2.0, 2.1, 2.0])
        x.passo(1.6, POSE, 1.0)
        self.assertEqual(x.estado, 'girar')

    def test_abaixo_de_1_grau_nao_gira(self):
        x = self.zero([0.8, 0.8, 0.8])
        x.passo(1.6, POSE, 1.0)
        self.assertNotEqual(x.estado, 'girar')

    def test_pulso_longo_para_o_imu_encerrar(self):
        x = self.zero([7.9, 7.9, 7.9])
        x.passo(1.6, POSE, 1.0)
        self.assertEqual(x.estado, 'girar')
        self.assertGreater(x.pulso, math.radians(7.9) / x.w_real)
        self.assertLessEqual(x.pulso, 1.0)

    def test_ate_6_tentativas(self):
        x = self.zero([5.6, 5.6, 5.6])
        x.n_cruz = 3
        x.passo(1.6, POSE, 1.0)
        self.assertEqual(x.estado, 'girar')
        y = self.zero([5.6, 5.6, 5.6])
        y.n_cruz = 6
        y.passo(1.6, POSE, 1.0)
        self.assertNotEqual(y.estado, 'girar')

    def test_desligado_mantem_3_graus_e_3_tentativas(self):
        x = cruzamento([2.0, 2.0, 2.0])
        x.passo(1.6, POSE, 1.0)
        self.assertNotEqual(x.estado, 'girar')
        y = cruzamento([5.6, 5.6, 5.6])
        y.n_cruz = 3
        y.passo(1.6, POSE, 1.0)
        self.assertNotEqual(y.estado, 'girar')


class TestAlinharCruzamentoZeroV2(unittest.TestCase):
    """v2 (19/11 20:00): volta do L a ~11 graus (folgas 8+3 cm) sem nenhum giro e sem log do motivo."""

    def zero(self, angs, **kw):
        x = cruzamento(angs, **kw)
        x.cruz_zero = True
        logs = []
        x.log = logs.append
        return x, logs

    def test_14_graus_gira_no_modo_zero(self):
        x, _ = self.zero([14.0, 14.2, 13.9])
        x.passo(1.6, POSE, 1.0)
        self.assertEqual(x.estado, 'girar')

    def test_espalhamento_2_graus_gira_no_modo_zero(self):
        x, _ = self.zero([10.0, 11.0, 12.0])
        x.passo(1.6, POSE, 1.0)
        self.assertEqual(x.estado, 'girar')

    def test_sem_giro_registra_o_motivo(self):
        x, logs = self.zero([0.5, 0.5, 0.5])
        x.passo(1.6, POSE, 1.0)
        self.assertNotEqual(x.estado, 'girar')
        self.assertTrue(any('ALINHAR_CRUZAMENTO: sem giro' in l for l in logs), logs)

    def test_desligado_continua_ate_12_graus(self):
        x = cruzamento([14.0, 14.0, 14.0])
        x.passo(1.6, POSE, 1.0)
        self.assertNotEqual(x.estado, 'girar')


class TestVxDeslize(unittest.TestCase):
    """VX_DESLIZE (19/11 noite): deslize para a direita com centragem dava vx +0,023 / vy -0,08 (-74 graus),
    fora do poligono lateral do CM (que agora vai ate 65 graus, CM_LATERAL65)."""

    def test_direcao_do_deslize_fica_no_poligono_lateral(self):
        for lado in (-1, 1):
            for des in (-0.10, -0.018, 0.0, 0.018, 0.10):
                x = TravessiaLateral(lado, fundido=True, k_ff=0.2, log=lambda _: None)
                x.t0 = 0.0
                x.ref = 0.0
                x._mudar('deslizar', 1.0, POSE)
                x._centro_folga = lambda t, d=des: d
                vx, vy, wz = x._deslizar_fundido(2.0, POSE, 0.5)
                ang = abs(math.degrees(math.atan2(vy, vx)))
                self.assertTrue(65.0 <= ang <= 115.0, (lado, des, vx, vy, ang))


class TestRecuoParaMedir(unittest.TestCase):
    """RECUO_PARA_MEDIR (19/11 20:4x): ida 'sem giro: 0 medida(s) (n>=15) | +2.8(n7,8)'; volta sem medida
    nenhuma ('Sem parede do corredor novo agora') com a parede a ~10 cm, deslizou ~9 graus torto."""

    def zero(self, angs, folgas, n=30, zero=True):
        x = cruzamento(angs, folgas=folgas, n=n)
        x.cruz_zero = zero
        return x

    def test_poucos_pontos_perto_recua_para_medir(self):
        x = self.zero([2.8, 2.7, 2.5], folgas=(0.08, None), n=7)
        x.passo(1.6, POSE, 1.0)
        self.assertEqual(x.estado, 'ajustar')
        self.assertAlmostEqual(x.alvo, -0.07, places=3)
        vx, vy, wz = x.passo(1.7, POSE, 1.0)
        self.assertLess(vx, 0.0)

    def test_sem_medida_nenhuma_recua_para_medir(self):
        x = self.zero([], folgas=(0.10, 0.20))
        for i in range(25):
            x.nova_folga(1.05 + 0.1 * i, 0.10, 0.20)
        x.passo(3.2, POSE, 1.0)
        self.assertEqual(x.estado, 'ajustar')
        self.assertAlmostEqual(x.alvo, -0.05, places=3)

    def test_medida_boa_nao_recua(self):   # (o ajustar que sobra e a centragem normal pelas folgas)
        x = self.zero([0.5, 0.5, 0.5], folgas=(0.08, 0.25))
        logs = []
        x.log = logs.append
        x.passo(1.6, POSE, 1.0)
        self.assertFalse(any('RECUO_PARA_MEDIR' in l for l in logs), logs)
        self.assertEqual(getattr(x, 'n_recuo_medir', 0), 0)

    def test_sem_espaco_atras_nao_recua(self):
        x = self.zero([2.8, 2.7, 2.5], folgas=(0.08, 0.10), n=7)
        x.passo(1.6, POSE, 1.0)
        self.assertNotEqual(x.estado, 'ajustar')

    def test_desligado_nao_recua(self):
        x = self.zero([2.8, 2.7, 2.5], folgas=(0.08, None), n=7, zero=False)
        x.passo(1.6, POSE, 1.0)
        self.assertNotEqual(x.estado, 'ajustar')

    def test_no_maximo_2_recuos(self):
        x = self.zero([2.8, 2.7, 2.5], folgas=(0.08, None), n=7)
        x.n_recuo_medir = 2
        x.passo(1.6, POSE, 1.0)
        self.assertNotEqual(x.estado, 'ajustar')


if __name__ == '__main__':
    unittest.main()
