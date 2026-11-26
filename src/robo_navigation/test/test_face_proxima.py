"""Reconhece inteira de longe, acompanha parcial perto sem inventar centro (FACE_FRAGMENTADA)."""
import math
import unittest
from test_lateral_ws import segmento, ZERO
from face_ws import RastreioFaceWS, transformar_ponto
from alinhador_passagem import Encosto

class TestFaceProxima(unittest.TestCase):
    def rastreio(self):
        r = RastreioFaceWS(dict(x=.50,y=0.,yaw=0.,largura_medida=.8))
        pts = segmento(.50, n=81)
        r.medir(pts, 0., ZERO, ZERO)
        self.assertIsNotNone(r.medir(pts, .17, ZERO, ZERO))
        return r, pts

    def test_acompanha_lacuna_sem_inventar_centro(self):
        # FACE_FRAGMENTADA (09/11): antes o centro vinha projetado pela odom (-0,012); agora a face
        # sem as 2 pontas mede so distancia/angulo e o centro fica None (missao32: odom errou 2,8 cm)
        r, pts = self.rastreio()
        pose = (.055,.012,0.)
        parcial = [transformar_ponto(p,ZERO,pose) for p in pts if -.30 < p[1] < .32 and abs(p[1]) > .10]
        m = r.medir(parcial,.34,pose,pose)
        self.assertIsNotNone(m)
        self.assertTrue(m['parcial'])
        self.assertAlmostEqual(m['dist'],.445)
        self.assertIsNone(m['centro'])
        self.assertIsNotNone(r.medir(parcial,.51,pose,pose))

    def test_parcial_nao_reconhece_nem_reassume_apos_perda(self):
        r,pts = self.rastreio()
        partial = [p for p in pts if abs(p[1]) > .1]
        fresh = RastreioFaceWS(r.face)
        self.assertIsNone(fresh.medir(partial,0.,ZERO,ZERO))
        self.assertIsNone(r.medir(partial,1.,ZERO,ZERO))
        self.assertIsNone(r.medir(pts,1.17,ZERO,ZERO))
        self.assertIsNone(r.medir(partial,1.34,ZERO,ZERO))

    def test_nao_aceita_plano_errado_ou_sem_retorno(self):
        for pts in ([],segmento(.40),segmento(.50,a=math.radians(15),width=.3)):
            r,_ = self.rastreio()
            self.assertIsNone(r.medir(pts,.34,ZERO,ZERO))

    def enc(self, centro=0., dist=.445):
        e=Encosto(log=lambda _:None)
        e.t0=e.t_estado=0.
        e.estado='medir_final'
        for t in (3.5,3.67,3.84,4.):
            e.nova_medicao(t,dict(dist=dist,ang=0.,centro=centro,fragmentada=True),ZERO)
        return e

    def test_final_confere_distancia_e_centro(self):
        e=self.enc();self.assertEqual(e.passo(4.,ZERO),(0.,0.,0.));self.assertEqual(e.estado,'fim')
        e=self.enc(.06);e.passo(4.,ZERO);self.assertEqual(e.estado,'abortar')
        e=self.enc(dist=.49);e.passo(4.,ZERO);self.assertEqual(e.estado,'avancar')

    def test_parcial_interrompe_correcao(self):
        # parcial (centro None) numa correcao: para (PERDA_PARA) e mede de novo; nunca desliza/gira
        for state in ('girar','deslocar','lat_mover','lat_pulso'):
            e=Encosto(log=lambda _:None); e.t0=0.; e.alvo_giro=0.
            e._mudar(state,3.9,ZERO)
            e.nova_medicao(4.,dict(dist=.445,ang=0.,centro=None,parcial=True),ZERO)
            self.assertEqual(e.passo(4.,ZERO),(0.,0.,0.))
            self.assertIn(e.estado,('assentar','assentar_final'))

if __name__=='__main__':unittest.main()
