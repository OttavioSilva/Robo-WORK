"""RECUO_LATERAL (09/11, missao32): antes de deslizar, o Encosto recua reto se o avanco previsto do
deslize (k_lat * |centro|) levaria a face para menos de 15 cm; no deslize, para se a face chegar perto."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robo_navigation/scripts'))

import alinhador_passagem as ap  # noqa: E402

PC = 0.365


class Robo:
    """Robo de mentira: integra o comando do Encosto e gera a medicao da face (sem zona cega).
    k = quanto do deslize lateral vira avanco (para a mesa)."""
    def __init__(self, folga, centro, k=0.8):
        self.folga, self.centro, self.k = folga, centro, k
        self.t = 0.0
        self.x = self.y = 0.0
        self.enc = ap.Encosto(linear=True, distancia_alvo=0.09, para_choque=PC, log=lambda m: None)
        self.estados = []
        self.min_folga = folga

    def rodar(self, tmax=60.0):
        prox_scan = 0.0
        while self.t < tmax and self.enc.estado not in ('fim', 'abortar'):
            if self.t >= prox_scan:
                prox_scan += 1 / 5.9
                self.enc.nova_medicao(self.t, dict(dist=self.folga + PC, ang=0.0, centro=self.centro,
                                                   span=0.89), (self.x, self.y, 0.0))
            vx, vy, _ = self.enc.passo(self.t, (self.x, self.y, 0.0))
            if not self.estados or self.estados[-1] != self.enc.estado:
                self.estados.append(self.enc.estado)
            dt = 0.05
            vx_real = (0.10 if vx > 0.005 else -0.10 if vx < -0.005 else 0.0)
            vy_real = (0.04 if vy > 0.005 else -0.04 if vy < -0.005 else 0.0)
            vx_real += self.k * abs(vy_real)
            self.x += vx_real * dt
            self.y += vy_real * dt
            self.folga -= vx_real * dt
            self.centro -= vy_real * dt
            self.min_folga = min(self.min_folga, self.folga)
            self.t += dt
        return self


class TestRecuoLateral(unittest.TestCase):
    def test_tem_a_marca(self):
        self.assertIn('RECUO_LATERAL', Path(ap.__file__).read_text())

    def test_centro_longe_e_perto_recua_antes_de_deslizar(self):
        r = Robo(folga=0.22, centro=0.30).rodar()
        self.assertIn('recuar', r.estados)
        self.assertLess(r.estados.index('recuar'), r.estados.index('deslocar'))

    def test_perto_e_muito_fora_nao_entra_na_zona_cega_antes_de_centrar(self):
        r = Robo(folga=0.22, centro=0.30, k=0.8).rodar()
        # enquanto ainda havia deslize a fazer, a folga nunca caiu abaixo de ~14 cm
        self.assertEqual(r.enc.estado, 'fim', r.enc.motivo)
        self.assertLessEqual(abs(r.centro), 0.03)

    def test_centro_pequeno_nao_recua(self):
        r = Robo(folga=0.24, centro=0.05).rodar()
        self.assertNotIn('recuar', r.estados)
        self.assertEqual(r.enc.estado, 'fim', r.enc.motivo)

    def test_no_maximo_3_recuos_e_nao_alem_de_45cm(self):
        r = Robo(folga=0.22, centro=0.60, k=1.5).rodar()
        self.assertLessEqual(r.enc.n_recuo, 3)
        self.assertLessEqual(r.enc.recuo_alvo, r.enc.folga_max_recuo + 1e-9)

    def test_k_medido_no_deslize(self):
        r = Robo(folga=0.40, centro=0.20, k=1.0).rodar()
        self.assertGreater(r.enc.k_lat, 0.7)


if __name__ == '__main__':
    unittest.main()
