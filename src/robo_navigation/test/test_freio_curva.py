"""FREIO_CURVA (12/11): travessia com curva planejada termina (e segue a saida lateral) quando o FREIO_FRENTE ou o
Collision Monitor param o robo poucos cm ANTES do ponto do cruzamento, com o corpo ja cabendo no corredor novo.
Cenario do teste 12/11 10:46: objeto apos a curva, parado a 10 cm, ~5 cm antes do ponto. Sem ROS."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robo_navigation/scripts'))
from alinhador_passagem import Travessia  # noqa: E402

PATCH = ROOT / 'ferramentas/patches/aplicar_freio_curva.py'
ATUAL = (ROOT / 'src/robo_navigation/scripts/alinhador_passagem.py').read_text()
POSE = (0.0, 0.0, 0.0)


def trav(curva=True, tol=-1.0, largura_b=0.70):
    x = Travessia(parada_lidar=curva, log=lambda _: None)
    x.freio_curva_tol = tol
    x.largura_b = largura_b
    x.t0 = 0.0
    x._mudar('atravessar', 0.0, POSE)
    return x


def freio(x, restante, folga=0.09, t=1.0):
    x.f_corpo = (t, folga)
    return x.passo(t, POSE, restante)


class TestFreioCurva(unittest.TestCase):
    def test_marca_e_parametro(self):
        self.assertIn('FREIO_CURVA (12/11)', ATUAL)
        self.assertIn("self.declare_parameter('freio_curva_tol', -1.0)", ATUAL)

    def test_tolerancia_automatica(self):
        self.assertAlmostEqual(trav(largura_b=0.70).tol_curva(), 0.085)
        self.assertAlmostEqual(trav(largura_b=0.80).tol_curva(), 0.10)    # limite
        self.assertAlmostEqual(trav(largura_b=0.60).tol_curva(), 0.035)
        self.assertIsNone(trav(curva=False).tol_curva())
        self.assertIsNone(trav(tol=0.0).tol_curva())
        self.assertAlmostEqual(trav(tol=0.05).tol_curva(), 0.05)

    def test_cenario_12_11_freio_antes_do_ponto_segue_a_saida(self):
        x = trav()
        self.assertEqual(freio(x, 0.05), (0.0, 0.0, 0.0))
        self.assertEqual(x.estado, 'fim')

    def test_longe_do_ponto_continua_abortando(self):
        x = trav()
        freio(x, 0.20)
        self.assertEqual(x.estado, 'abortar')
        self.assertIn('algo a 9 cm', x.motivo)

    def test_desligado_e_sem_curva_igual_a_antes(self):
        for x in (trav(tol=0.0), trav(curva=False)):
            freio(x, 0.05)
            self.assertEqual(x.estado, 'abortar')

    def test_freio_cruzamento_antigo_inalterado(self):
        x = trav(tol=0.0)
        freio(x, -0.10)
        self.assertEqual(x.estado, 'fim')
        x = trav(curva=False)
        freio(x, -0.10)
        self.assertEqual(x.estado, 'abortar')

    def test_cm_segurando_perto_do_ponto(self):
        x = trav()
        x.ult_restante = 0.06
        x.bloqueio(1.0, POSE)
        self.assertEqual(x.estado, 'fim')

    def test_cm_segurando_longe_ou_desligado_aborta(self):
        for x, r in ((trav(), 0.30), (trav(tol=0.0), 0.06), (trav(curva=False), 0.06)):
            x.ult_restante = r
            x.bloqueio(1.0, POSE)
            self.assertEqual(x.estado, 'abortar')
            self.assertIn('Collision Monitor', x.motivo)

    def test_passo_guarda_restante(self):
        x = trav()
        x.f_corpo = None
        x.ult = None
        try:
            x.passo(1.0, POSE, 0.42)
        except Exception:
            pass
        self.assertEqual(x.ult_restante, 0.42)

    def test_patch_aplicar_recusar_desfazer(self):
        with tempfile.TemporaryDirectory() as d:
            arq = os.path.join(d, 'alinhador_passagem.py')
            Path(arq).write_text(ATUAL)
            subprocess.run([sys.executable, str(PATCH), '--arquivo', arq, '--desfazer'], check=True, capture_output=True)
            orig = Path(arq).read_text()
            self.assertNotIn('FREIO_CURVA', orig)
            subprocess.run([sys.executable, str(PATCH), '--arquivo', arq], check=True, capture_output=True)
            self.assertEqual(Path(arq).read_text(), ATUAL)
            out = subprocess.run([sys.executable, str(PATCH), '--arquivo', arq], check=True, capture_output=True, text=True).stdout
            self.assertIn('ja aplicado', out)


if __name__ == '__main__':
    unittest.main()
