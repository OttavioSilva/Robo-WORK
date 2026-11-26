"""FREIO_CRUZAMENTO (09/11): obstaculo a frente ja no cruzamento termina a travessia, nao aborta."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robo_navigation/scripts'))
from alinhador_passagem import Travessia


def atravessando(parada_lidar, folga):
    tr = Travessia(parada_lidar=parada_lidar, log=lambda m: None)
    tr.t0 = 0.0
    tr.estado = 'atravessar'
    tr.f_corpo = (10.0, folga)
    return tr


class TestFreioCruzamento(unittest.TestCase):
    def test_missao25_alem_do_cruzamento_termina(self):
        # missao25 12:11: 23 cm alem do ponto, algo a 9 cm
        tr = atravessando(True, 0.09)
        self.assertEqual(tr.passo(10.0, (0.0, 0.0, 0.0), -0.23), (0.0, 0.0, 0.0))
        self.assertEqual(tr.estado, 'fim')

    def test_antes_do_cruzamento_continua_abortando(self):
        tr = atravessando(True, 0.09)
        tr.passo(10.0, (0.0, 0.0, 0.0), 0.40)
        self.assertEqual(tr.estado, 'abortar')

    def test_sem_pose_no_mapa_aborta(self):
        # sem TF map o chamador passa restante = 9.0
        tr = atravessando(True, 0.09)
        tr.passo(10.0, (0.0, 0.0, 0.0), 9.0)
        self.assertEqual(tr.estado, 'abortar')

    def test_passagem_sem_cruzamento_continua_abortando(self):
        tr = atravessando(False, 0.09)
        tr.passo(10.0, (0.0, 0.0, 0.0), -0.23)
        self.assertEqual(tr.estado, 'abortar')


def com_parede(x_parede, x_esp):
    """PAREDE_MAPA: travessia com parada no cruzamento, uma parede transversal medida agora."""
    tr = Travessia(parada_lidar=True, log=lambda m: None)
    tr.t0 = 0.0
    tr.estado = 'atravessar'
    tr.yaw_ref = 0.0
    tr.ult = (0.0, {'lados': 0}, (0.0, 0.0, 0.0))   # medicao de paredes velha: nao corrige
    tr.nova_transversal(10.0, x_parede, (0.0, 0.0, 0.0), x_esp)
    return tr


class TestParedeMapa(unittest.TestCase):
    def test_parede_certa_para_no_cruzamento(self):
        # missao26: a parede leste do corredor novo bateu com o mapa em +-5 cm
        tr = com_parede(0.50, 0.49)
        tr.passo(10.0, (0.0, 0.0, 0.0), 0.0)
        self.assertEqual(tr.estado, 'fim')

    def test_bloco_na_boca_do_corredor_nao_para(self):
        # missao26 1a tentativa: face a 14 cm, mapa esperava 0,9 m adiante -> seguia parando
        tr = com_parede(0.50, 1.42)
        tr.passo(10.0, (0.0, 0.0, 0.0), 0.93)
        self.assertEqual(tr.estado, 'atravessar')

    def test_obstaculo_alem_do_T_para_pelo_mapa(self):
        # missao26 2a tentativa: obstaculo +0,26 m alem do esperado, ja 9 cm alem do ponto
        tr = com_parede(0.66, 0.40)
        tr.passo(10.0, (0.0, 0.0, 0.0), -0.09)
        self.assertEqual(tr.estado, 'fim')

    def test_obstaculo_antes_do_ponto_continua(self):
        tr = com_parede(0.87, 0.65)
        tr.passo(10.0, (0.0, 0.0, 0.0), 0.16)
        self.assertEqual(tr.estado, 'atravessar')


class TestEncostoEspera(unittest.TestCase):
    def test_missao27_avanco_espera_scan_novo(self):
        # ENCOSTO_ESPERA: alinhou com a medicao de 0,05 s atras e mudou para 'avancar';
        # o scan novo so chega ~0,17 s depois. Antes abortava "perdi a face da mesa".
        from alinhador_passagem import Encosto
        enc = Encosto(log=lambda m: None)
        enc.t0 = 0.0
        enc.yaw_ref = 0.0
        med = dict(dist=0.66, ang=0.0, centro=0.0, span=0.8)
        enc.nova_medicao(9.95, med, (0.0, 0.0, 0.0))
        enc.estado, enc.t_estado = 'avancar', 10.0
        self.assertEqual(enc.passo(10.05, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        self.assertEqual(enc.estado, 'avancar')
        enc.nova_medicao(10.12, med, (0.0, 0.0, 0.0))
        vx, _, _ = enc.passo(10.15, (0.0, 0.0, 0.0))
        self.assertEqual(enc.estado, 'avancar')
        self.assertGreater(vx, 0.0)

    def test_sem_scan_novo_a_guarda_de_0_3_s_aborta(self):
        from alinhador_passagem import Encosto
        enc = Encosto(log=lambda m: None)
        enc.t0 = 0.0
        enc.yaw_ref = 0.0
        enc.nova_medicao(9.95, dict(dist=0.66, ang=0.0, centro=0.0, span=0.8), (0.0, 0.0, 0.0))
        enc.estado, enc.t_estado = 'avancar', 10.0
        enc.passo(10.50, (0.0, 0.0, 0.0))   # FALHA_SCAN: guarda passou de 0,3 para 0,45 s
        self.assertEqual(enc.estado, 'abortar')


class TestTravaOdom(unittest.TestCase):
    def test_salto_do_slam_depois_de_travar_nao_perde_a_face(self):
        # TRAVA_ODOM: missao27, map->odom pulou ~8,6 graus com o robo parado e alinhado
        import math
        from face_ws import RastreioFaceWS
        face = dict(x=.8, y=0., yaw=0., largura_medida=.8)
        pts = [(.8, -.4 + .02 * i) for i in range(41)]
        zero = (0., 0., 0.)
        rt = RastreioFaceWS(face)
        self.assertIsNone(rt.medir(pts, 0.00, zero, zero))       # 1a observacao
        self.assertIsNotNone(rt.medir(pts, 0.17, zero, zero))    # travou
        pulo = (0., -.08, math.radians(23))                      # SLAM corrige a pose
        self.assertIsNotNone(rt.medir(pts, 0.34, zero, pulo))
        self.assertIsNotNone(rt.medir(pts, 0.51, zero, pulo))

    def test_sem_trava_o_mapa_ainda_escolhe(self):
        import math
        from face_ws import RastreioFaceWS
        face = dict(x=.8, y=0., yaw=0., largura_medida=.8)
        pts = [(.8, -.4 + .02 * i) for i in range(41)]
        rt = RastreioFaceWS(face)
        self.assertIsNone(rt.medir(pts, 0.0, (0., 0., 0.), (0., -.08, math.radians(23))))
        self.assertIn('fora da referencia', rt.motivo)


class TestEncostoCego(unittest.TestCase):
    Z = (0.0, 0.0, 0.0)

    def enc_final(self, centro_longe, medidas):
        from alinhador_passagem import Encosto
        enc = Encosto(distancia_alvo=0.09, log=lambda m: None)
        enc.t0 = 0.0
        enc.yaw_ref = 0.0
        enc.centro_longe = centro_longe
        for i, m in enumerate(medidas):
            enc.nova_medicao(10.0 + 0.17 * i, m, self.Z)
        enc.t_parado = 9.6   # FACE_FRAGMENTADA: o assentar_final (robo parado) comeca 0,4 s antes
        enc.estado, enc.t_estado = 'medir_final', 10.0
        return enc

    def test_missao28_encostado_fora_do_centro_cego_aceita(self):
        # missao28: encostou a 10,5 cm, -2,8 graus, "centro +5,3" medido com a face partida;
        # de longe estava centrado (+0,9 cm). Antes deslizava de lado e perdia a face.
        import math
        m = dict(dist=0.365 + 0.105, ang=math.radians(-2.8), centro=0.053, span=0.6)
        enc = self.enc_final(0.009, [m] * 4)
        enc.passo(10.55, self.Z)   # 4 medicoes nos ultimos 0,6 s
        self.assertEqual(enc.estado, 'fim')

    def test_cego_centro_de_longe_ruim_aborta(self):
        m = dict(dist=0.365 + 0.10, ang=0.0, centro=0.0, span=0.8)
        enc = self.enc_final(0.045, [m] * 4)
        enc.passo(10.55, self.Z)   # 4 medicoes nos ultimos 0,6 s
        self.assertEqual(enc.estado, 'abortar')

    def test_face_sumiu_encostado_aceita_pela_ultima(self):
        m = dict(dist=0.365 + 0.095, ang=0.0, centro=0.0, span=0.7)
        enc = self.enc_final(0.0, [m])
        enc.meds.clear()          # o rastreio perdeu a face na zona cega
        enc.passo(13.2, self.Z)   # medir_final esperou 3 s sem medicao nova
        self.assertEqual(enc.estado, 'fim')

    def test_face_sumiu_longe_continua_abortando(self):
        m = dict(dist=0.365 + 0.30, ang=0.0, centro=0.0, span=0.85)
        enc = self.enc_final(0.0, [m])
        enc.meds.clear()
        enc.passo(13.2, self.Z)
        self.assertEqual(enc.estado, 'abortar')

    def test_avanco_entra_na_zona_cega_para_e_confere(self):
        from alinhador_passagem import Encosto
        enc = Encosto(distancia_alvo=0.09, log=lambda m: None)
        enc.t0, enc.yaw_ref = 0.0, 0.0
        enc.nova_medicao(10.0, dict(dist=0.365 + 0.12, ang=0.0, centro=0.0, span=0.7), self.Z)
        enc.estado, enc.t_estado = 'avancar', 9.0
        self.assertEqual(enc.passo(10.5, self.Z), (0.0, 0.0, 0.0))   # face velha (0,5 s)
        self.assertEqual(enc.estado, 'assentar_final')


class TestFalhaScan(unittest.TestCase):
    def test_missao29_um_scan_partido_nao_perde_a_face(self):
        # missao29 +29,48/+29,66/+29,83: face inteira, partida (raios inf), inteira
        from face_ws import RastreioFaceWS
        face = dict(x=.8, y=0., yaw=0., largura_medida=.8)
        inteira = [(.8, -.4 + .02 * i) for i in range(41)]
        partida = [p for p in inteira if not -.15 < p[1] < .05]   # buraco de 20 cm no meio
        z = (0., 0., 0.)
        rt = RastreioFaceWS(face)
        rt.medir(inteira, 0.00, z, z)
        self.assertIsNotNone(rt.medir(inteira, 0.17, z, z))
        self.assertIsNone(rt.medir(partida, 0.34, z, z))
        self.assertIsNotNone(rt.medir(inteira, 0.51, z, z))   # antes: "aguardando segunda observacao"

    def test_encosto_um_scan_ruim_nao_aborta_o_deslize(self):
        from alinhador_passagem import Encosto
        enc = Encosto(log=lambda m: None)
        enc.t0, enc.yaw_ref = 0.0, 0.0
        med = dict(dist=0.6, ang=0.0, centro=0.06, span=0.86)
        enc.nova_medicao(10.0, med, (0., 0., 0.))
        enc.estado, enc.t_estado = 'lat_mover', 9.9
        enc.alvo_lat = enc.l_falta = 0.06
        enc.l_yaw, enc.l_tmax = 0.0, 4.0
        enc.nova_medicao(10.17, None, (0., 0., 0.))   # scan partido
        enc.passo(10.2, (0., 0., 0.))
        self.assertEqual(enc.estado, 'lat_mover')
        self.assertEqual(enc.passo(10.5, (0., 0., 0.)), (0.0, 0.0, 0.0))   # falha continua: 0,5 s sem face
        self.assertEqual(enc.estado, 'assentar')     # PERDA_PARA: para e mede de novo


if __name__ == '__main__':
    unittest.main()
