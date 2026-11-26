"""RECUO_CURVA, MANOBRA_FOLGA e RE_LONGA (12/11). Sem ROS.
Cenario do bag corredor_re_freio_curva_20261112_113352 11:45:44: saida lateral barrada pelo CM com o objeto a
4,6 cm do para-choque (na zona cega: folga frente sem medida)."""
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robo_navigation/scripts'))
import alinhador_passagem as ap  # noqa: E402
from alinhador_passagem import TravessiaLateral, escolher_manobra, giro_bloqueado  # noqa: E402

SCRIPTS = ROOT / 'src/robo_navigation/scripts'
PATCHES = ROOT / 'ferramentas/patches'
POSE = (0.0, 0.0, 0.0)


def lateral(recuo=True, folgas=None):
    x = TravessiaLateral(1, fundido=True, log=lambda _: None)
    x.recuo_curva = recuo
    x.t0 = 0.0
    x._mudar('deslizar', 1.0, POSE)
    if folgas is not None:
        x.folga = (1.0,) + folgas
    return x


class TestRecuoCurva(unittest.TestCase):
    def test_parametro(self):
        txt = (SCRIPTS / 'alinhador_passagem.py').read_text()
        self.assertIn("self.declare_parameter('recuo_curva', True)", txt)

    def test_frente_cega_recua_5cm_sem_gastar_bloqueio(self):
        x = lateral()
        x.bloqueio(1.0, POSE)
        self.assertEqual((x.estado, x.alvo, x.n_bloq, x.n_recuo_curva), ('ajustar', -0.05, 0, 1))
        vx, vy, wz = x.passo(1.1, POSE, 1.0)
        self.assertLess(vx, 0.0)
        self.assertEqual((vy, wz), (0.0, 0.0))

    def test_frente_curta_medida_recua(self):
        x = lateral(folgas=(0.045, 0.20))
        x.bloqueio(1.0, POSE)
        self.assertEqual((x.estado, x.alvo), ('ajustar', -0.05))

    def test_volta_12_07_frente_medida_14_cm_recua(self):
        # bag 12:07 (curva para a DIREITA): /scan_front dava frente 14 cm, o /scan_nav do CM lembrava pontos a 4,3 cm
        for lado in (1, -1):
            x = lateral(folgas=(0.14, 0.35))
            x.lado = float(lado)
            x.bloqueio(1.0, POSE)
            self.assertEqual((x.estado, x.alvo), ('ajustar', -0.05))

    def test_traseira_curta_avanca(self):
        x = lateral(folgas=(0.15, 0.04))
        x.bloqueio(1.0, POSE)
        self.assertEqual((x.estado, x.alvo), ('ajustar', 0.05))

    def test_os_dois_lados_curtos_nao_move(self):
        x = lateral(folgas=(0.05, 0.05))
        x.bloqueio(1.0, POSE)
        self.assertEqual((x.estado, x.n_bloq), ('assentar', 1))

    def test_no_maximo_dois_recuos_depois_como_antes(self):
        x = lateral()
        for _ in range(2):
            x.bloqueio(1.0, POSE)
            x._mudar('deslizar', 1.0, POSE)
        x.bloqueio(1.0, POSE)
        self.assertEqual((x.estado, x.n_bloq, x.n_recuo_curva), ('assentar', 1, 2))
        for _ in range(3):
            x.bloqueio(1.0, POSE)
        self.assertEqual(x.estado, 'abortar')

    def test_desligado_igual_antes(self):
        x = lateral(recuo=False)
        x.bloqueio(1.0, POSE)
        self.assertEqual((x.estado, x.n_bloq), ('assentar', 1))

    def test_pulso_de_5cm(self):
        x = lateral()
        x.bloqueio(1.0, POSE)
        self.assertAlmostEqual(x.pulso * x.v_real, 0.05, places=3)


class TestManobraFolga(unittest.TestCase):
    PAREDE = [(0.40, y / 100.0) for y in range(-60, 61, 2)]   # parede colada no para-choque

    def test_anda_alem_do_minimo_com_margem(self):
        ang = math.radians(60)
        self.assertTrue(giro_bloqueado(self.PAREDE, ang))
        m0 = escolher_manobra(self.PAREDE, ang)
        m1 = escolher_manobra(self.PAREDE, ang, margem=0.06)
        self.assertEqual(m0[0], 'mover')
        self.assertEqual((m1[0], m1[1]), ('mover', m0[1]))
        self.assertAlmostEqual(m1[2], m0[2] + 0.06)

    def test_margem_limitada_pela_folga(self):
        ang = math.radians(60)
        atras = [(-0.215 - 0.20, y / 100.0) for y in range(-60, 61, 2)]   # parede 20 cm atras
        pts = self.PAREDE + atras
        m0 = escolher_manobra(pts, ang)
        m1 = escolher_manobra(pts, ang, margem=0.15)
        if m0[0] == 'mover':
            self.assertGreaterEqual(m1[2], m0[2])
            folga = ap.folga_direcao(pts, m1[1])
            self.assertLessEqual(m1[2], folga - ap.FOLGA_MOV + 1e-9)

    def test_parametro(self):
        txt = (SCRIPTS / 'alinhador_passagem.py').read_text()
        self.assertIn("self.declare_parameter('manobra_margem', 0.06)", txt)


class TestReLonga(unittest.TestCase):
    def test_parametros_do_no(self):
        txt = (SCRIPTS / 'destravar_node.py').read_text()
        self.assertIn("('cobertura_re', 0.65)", txt)
        self.assertIn("('re_max', 0.30)", txt)
        self.assertIn("re_max=max(0.10, min(0.40, float(self.get_parameter('re_max').value)))", txt)


class TestPatches(unittest.TestCase):
    def test_aplicar_recusar_desfazer(self):
        for nome, alvo in (('aplicar_recuo_curva.py', 'alinhador_passagem.py'),
                           ('aplicar_manobra_folga.py', 'alinhador_passagem.py'),
                           ('aplicar_re_longa.py', 'destravar_node.py')):
            atual = (SCRIPTS / alvo).read_text()
            with tempfile.TemporaryDirectory() as d:
                arq = os.path.join(d, alvo)
                Path(arq).write_text(atual)
                run = lambda *a: subprocess.run([sys.executable, str(PATCHES / nome), '--arquivo', arq, *a],
                                                check=True, capture_output=True, text=True).stdout
                run('--desfazer')
                self.assertNotEqual(Path(arq).read_text(), atual)
                run()
                self.assertEqual(Path(arq).read_text(), atual, nome)
                self.assertIn('ja aplicado', run())


if __name__ == '__main__':
    unittest.main()
