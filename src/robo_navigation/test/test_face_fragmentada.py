"""FACE_FRAGMENTADA (09/11, missao32): fases A/B/C do encosto na WS, sem ROS nem motores.

Cenarios sinteticos usam um LiDAR simulado com a geometria medida (x=0,33, passo 2 graus, nao mede
abaixo de 0,146 m). Os cenarios reais usam scans gravados na missao32 (test/dados/), sem inventar
medicao: a face da WS_1 partida pela zona cega em 14:12:39 e 14:12:15.
"""
import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robo_navigation/scripts'))
import face_ws as fw   # noqa: E402
from face_ws import (RastreioFaceWS, candidatos_face, lacuna_cega, selecionar_face,  # noqa: E402
                     obstaculo_antes_da_face, referencia_ws, JANELA_PARCIAL)
from alinhador_passagem import Encosto   # noqa: E402

Z = (0., 0., 0.)
LIDAR = (0.33, 0.005)
R_MIN = 0.146          # CLAUDE.md: o Delta-2D nao mede nada a menos de ~0,146 m
DADOS = Path(__file__).resolve().parent / 'dados' / 'missao32_ws1_scans.json'


def lidar(segmentos, passo=2.0, r_min=R_MIN, r_max=8.0):
    """Ray casting: segmentos [(p0, p1)] em base_link -> pontos medidos em base_link."""
    pts = []
    for k in range(int(360 / passo)):
        th = math.radians(-180 + k * passo)
        dx, dy = math.cos(th), math.sin(th)
        melhor = None
        for (x0, y0), (x1, y1) in segmentos:
            ex, ey = x1 - x0, y1 - y0
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            ox, oy = x0 - LIDAR[0], y0 - LIDAR[1]
            r = (ox * ey - oy * ex) / den
            u = (ox * dy - oy * dx) / den
            if 0 <= u <= 1 and r > 0 and (melhor is None or r < melhor):
                melhor = r
        if melhor is not None and r_min <= melhor <= r_max:
            pts.append((LIDAR[0] + melhor * dx, LIDAR[1] + melhor * dy))
    return pts


def face(dist, centro=0., largura=.88, a=0.):
    """Face plana: dist ao longo da normal, centro na tangente (mesma convencao do face_ws)."""
    c, s = math.cos(a), math.sin(a)
    px, py = dist * c - centro * s, dist * s + centro * c
    h = largura / 2
    return ((px + s * h, py - c * h), (px - s * h, py + c * h))


def caixa(x, y, lado=.05):
    h = lado / 2
    return [((x - h, y - h), (x - h, y + h)), ((x - h, y + h), (x + h, y + h)),
            ((x + h, y + h), (x + h, y - h)), ((x + h, y - h), (x - h, y - h))]


WS = dict(x=.60, y=0., yaw=0., largura_medida=.88)


def reconhecer(dist=.60, centro=0.):
    """Fase A: 2 observacoes inteiras e coerentes da WS a dist."""
    r = RastreioFaceWS(WS)
    pts = lidar([face(dist, centro)])
    r.medir(pts, 0.0, Z, Z)
    m = r.medir(pts, 0.17, Z, Z)
    assert m is not None and m['centro'] is not None
    return r


def andar(r, ate, centro=0., t=0.17, passo=.02, extra=()):
    """Aproxima de frente (odom exata) ate a face ficar a 'ate'; devolve (t, ultima medicao)."""
    d, m, x = .60, None, 0.
    while d > ate + 1e-9:
        d = max(ate, d - passo); x = .60 - d; t += .17
        m = r.medir(lidar([face(d, centro)] + list(extra)), t, (x, 0., 0.), (x, 0., 0.))
        assert m is not None, (d, r.motivo)
    return t, m


class TestFaseA(unittest.TestCase):
    def test_face_completa_detectada(self):
        r = RastreioFaceWS(WS)
        pts = lidar([face(.60, .03)])
        self.assertIsNone(r.medir(pts, 0., Z, Z))            # 1a observacao nao autoriza
        self.assertEqual(r.motivo, 'aguardando segunda observacao coerente')
        m = r.medir(pts, .17, Z, Z)
        self.assertAlmostEqual(m['dist'], .60, delta=.01)
        self.assertAlmostEqual(m['centro'], .03, delta=.02)
        self.assertFalse(m.get('fragmentada', False))
        self.assertTrue(r.reconhecida)

    def test_fase_a_nao_aceita_face_partida(self):
        # perto da mesa desde o inicio: os pedacos nao reconhecem a WS (exige as 2 pontas inteiras)
        r = RastreioFaceWS(dict(WS, x=.45))
        pts = lidar([face(.45)])
        self.assertGreater(max(p[0] for p in pts), .40)    # ha retornos da face
        for t in (0., .17, .34, .51):
            self.assertIsNone(r.medir(pts, t, Z, Z))
        self.assertFalse(r.reconhecida)
        self.assertIn('2 pontas', r.motivo)


class TestFragmentada(unittest.TestCase):
    def test_lidar_simulado_parte_a_face_perto(self):
        # pre-condicao fisica: a 0,45 m o meio da face cai abaixo de 0,146 m do LiDAR
        ys = [p[1] for p in lidar([face(.45)]) if p[0] > .40]
        self.assertFalse(any(-.05 < y < .05 for y in ys))
        self.assertTrue(any(y < -.3 for y in ys) and any(y > .3 for y in ys))

    def test_dois_segmentos_compativeis_viram_uma_face(self):
        r = reconhecer()
        t, m = andar(r, .45, centro=.02)
        self.assertTrue(m['fragmentada'])
        self.assertAlmostEqual(m['dist'], .45, delta=.01)
        self.assertAlmostEqual(m['centro'], .02, delta=.02)   # centro MEDIDO pelas 2 pontas
        self.assertIn('zona cega', r.motivo)

    def test_segmentos_de_objetos_diferentes_nao_unem(self):
        # mesmo vao da zona cega, mas o pedaco direito esta 6 cm atras: outro objeto
        esq = ((.45, -.44), (.45, -.02)); dir_ = ((.51, .02), (.51, .44))
        pts = lidar([esq, dir_])
        self.assertFalse(any(m.get('fragmentada') for m in candidatos_face(pts, unir_cega=True)))
        # angulo diferente (12 graus)
        a = math.radians(12)
        dir2 = ((.45, .02), (.45 - .42 * math.tan(a), .44))
        pts = lidar([esq, dir2])
        self.assertFalse(any(m.get('fragmentada') for m in candidatos_face(pts, unir_cega=True)))

    def test_lacuna_fora_da_zona_cega_nao_une(self):
        # face longe (0,80) com um buraco de 20 cm (raios inf): o LiDAR veria esse trecho
        a = [(.80, -.44 + .02 * i) for i in range(20)]
        b = [(.80, -.04 + .02 * i + .2) for i in range(20)]
        self.assertIsNone(lacuna_cega(a, b, a + b))
        self.assertFalse(any(m.get('fragmentada') for m in candidatos_face(a + b, unir_cega=True)))

    def test_retorno_dentro_da_lacuna_na_frente_nao_une(self):
        pts = lidar([face(.45)])
        a = [p for p in pts if .40 < p[0] < .50 and p[1] < 0]
        b = [p for p in pts if .40 < p[0] < .50 and p[1] > 0]
        self.assertIsNotNone(lacuna_cega(a, b, pts))
        self.assertIsNone(lacuna_cega(a, b, pts + [(.40, 0.0)]))

    def test_largura_errada_nao_da_centro(self):
        r = reconhecer()
        t, _ = andar(r, .47)
        # ponta esquerda truncada (raios rasantes): largura 8 cm menor -> plano vale, centro nao
        pts = [p for p in lidar([face(.45)]) if p[1] > -.33]
        m = r.medir(pts, t + .17, (.15, 0, 0), (.15, 0, 0))
        self.assertIsNotNone(m, r.motivo)
        self.assertIsNone(m['centro'])
        self.assertTrue(m['parcial'])
        # +26 cm: passa da extensao da WS prevista -> recusa (pode ser outro objeto)
        r = reconhecer()
        t, _ = andar(r, .47)
        self.assertIsNone(r.medir(lidar([face(.45), ((.45, .44), (.45, .70))]), t + .17,
                                  (.15, 0, 0), (.15, 0, 0)))


class TestRastreio(unittest.TestCase):
    def test_uma_ponta_invisivel_temporariamente(self):
        r = reconhecer()
        t, m = andar(r, .50, centro=.02)
        self.assertIsNotNone(m['centro'])
        # ponta direita some (raios inf): so plano, centro None (nao inventa)
        sem_ponta = [p for p in lidar([face(.50, .02)]) if p[1] < .25]
        m = r.medir(sem_ponta, t + .17, (.10, 0, 0), (.10, 0, 0))
        self.assertTrue(m['parcial'])
        self.assertIsNone(m['centro'])
        self.assertAlmostEqual(m['dist'], .50, delta=.01)
        # volta a face inteira: centro medido de novo
        m = r.medir(lidar([face(.50, .02)]), t + .34, (.10, 0, 0), (.10, 0, 0))
        self.assertAlmostEqual(m['centro'], .02, delta=.02)

    def test_parcial_tem_prazo(self):
        r = reconhecer()
        t, _ = andar(r, .50)
        sem_ponta = [p for p in lidar([face(.50)]) if p[1] < .25]
        n, ok = 0, 0
        while n * .17 <= JANELA_PARCIAL + .5:
            n += 1
            ok += r.medir(sem_ponta, t + n * .17, (.10, 0, 0), (.10, 0, 0)) is not None
        self.assertLess(ok, n)                       # parou de aceitar so o plano
        self.assertLessEqual(ok * .17, JANELA_PARCIAL + .01)

    def test_parcial_nao_reconhece(self):
        r = RastreioFaceWS(WS)
        sem_ponta = [p for p in lidar([face(.50)]) if p[1] < .25]
        for t in (0., .17, .34):
            self.assertIsNone(r.medir(sem_ponta, t, Z, Z))

    def test_leitura_antiga_ou_repetida(self):
        r = reconhecer()
        pts = lidar([face(.60)])
        self.assertIsNone(r.medir(pts, .17, Z, Z))          # mesmo instante: repetida
        self.assertEqual(r.motivo, 'observacao repetida/antiga')
        r = reconhecer()
        self.assertIsNone(r.medir(pts, 2.0, Z, Z))          # 1,8 s depois: perdeu a trava
        self.assertFalse(r.reconhecida)
        self.assertIsNone(r.medir(lidar([face(.45)]), 2.17, Z, Z))   # e partida nao reassume

    def test_leitura_inconsistente_salto(self):
        r = reconhecer()
        self.assertIsNone(r.medir(lidar([face(.50)]), .34, Z, Z))    # 10 cm sem a odom andar
        self.assertFalse(r.reconhecida)
        r = reconhecer()
        self.assertIsNone(r.medir(lidar([face(.60)]), .34, (math.nan, 0, 0), Z))
        self.assertFalse(r.reconhecida)

    def test_perda_completa(self):
        r = reconhecer()
        t, _ = andar(r, .46)
        for k in range(1, 6):
            self.assertIsNone(r.medir([], t + .17 * k, (.14, 0, 0), (.14, 0, 0)))
        self.assertFalse(r.reconhecida)
        self.assertIsNone(r.ultima)

    def test_centralizado_e_proximo(self):
        r = reconhecer()
        t, m = andar(r, .445, centro=0.)
        self.assertAlmostEqual(m['dist'] - .365, .08, delta=.01)
        self.assertAlmostEqual(m['centro'], 0., delta=.02)


class TestObstaculo(unittest.TestCase):
    def test_obstaculo_entre_robo_e_bancada(self):
        # caixa de 4 cm a 19 cm do LiDAR (fora da zona cega), 10 cm antes da face
        r = reconhecer()
        pts = lidar([face(.60)] + caixa(.50, .12, .04))
        m = r.medir(pts, .34, Z, Z)
        self.assertIsNotNone(m, r.motivo)
        self.assertFalse(m.get('fragmentada', False))
        g = obstaculo_antes_da_face(pts, m)
        self.assertIsNotNone(g)
        self.assertLess(g, .15)

    def test_obstaculo_na_zona_cega_e_invisivel(self):
        # limite fisico (nao do codigo): a < 0,146 m do LiDAR nada volta; documenta o ENCOSTO_CEGO
        self.assertEqual([p for p in lidar(caixa(.42, .05, .04)) if p[0] > .30], [])


def enc_avancando(med, t=10.0, centro_longe=0.):
    e = Encosto(distancia_alvo=.08, linear=True, log=lambda _: None)
    e.t0 = 0.
    e.yaw_ref = 0.
    e.centro_longe = centro_longe
    e._mudar('avancar', t - .3, Z)
    e.nova_medicao(t - .2, med, Z)
    e.nova_medicao(t, med, Z)
    return e


class TestEncosto(unittest.TestCase):
    def test_parcial_mantem_avanco_reto_ja_autorizado(self):
        med = dict(dist=.53, ang=0., centro=None, span=.88, parcial=True)
        e = enc_avancando(med)
        vx, vy, wz = e.passo(10.05, Z)
        self.assertGreater(vx, 0)
        self.assertEqual(vy, 0.)
        self.assertEqual(e.estado, 'avancar')

    def test_parcial_sem_centro_de_longe_nao_avanca(self):
        med = dict(dist=.53, ang=0., centro=None, span=.88, parcial=True)
        e = enc_avancando(med, centro_longe=None)
        self.assertEqual(e.passo(10.05, Z), (0., 0., 0.))
        self.assertNotEqual(e.estado, 'avancar')

    def test_parcial_nao_autoriza_correcao(self):
        med = dict(dist=.47, ang=0., centro=None, span=.88, parcial=True)
        for estado in ('girar', 'deslocar', 'lat_mover', 'lat_pulso'):
            e = Encosto(distancia_alvo=.08, log=lambda _: None)
            e.t0 = 0.; e.alvo_giro = 0.; e.alvo_lat = .05; e.l_falta = .05; e.l_yaw = 0.
            e.l_dur = .15; e.l_tmax = 4.
            e._mudar(estado, 9.8, Z)
            e.nova_medicao(10., med, Z)
            for k in range(5):
                self.assertEqual(e.passo(10.02 + .01 * k, Z), (0., 0., 0.))
                if e.estado == 'abortar':
                    break
                e.nova_medicao(10.02 + .01 * k, med, Z)
                e._mudar(estado, 10.02 + .01 * k, Z)   # simula voltar a corrigir
            self.assertEqual(e.estado, 'abortar')   # 3 PERDA_PARA e entao falha explicita

    def test_medir_sem_centro_falha_explicita(self):
        e = Encosto(distancia_alvo=.08, log=lambda _: None)
        e.recuo_cego_max = 0   # RECUO_CEGO (24/11): aqui vale o caminho sem recuo
        e.t0 = 0.
        e._mudar('medir', 10., Z)
        for k in range(4):
            e.nova_medicao(10.1 + .1 * k, dict(dist=.50, ang=0., centro=None, span=.88, parcial=True), Z)
            self.assertEqual(e.passo(10.15 + .1 * k, Z), (0., 0., 0.))
        self.assertEqual(e.estado, 'medir')                 # so plano: espera, nao decide
        for k in range(30):
            self.assertEqual(e.passo(10.5 + .1 * k, Z), (0., 0., 0.))
        self.assertEqual(e.estado, 'abortar')
        self.assertIn('centro da face nao medido', e.motivo)

    def test_obstaculo_no_avanco_aborta_parado(self):
        med = dict(dist=.53, ang=0., centro=.0, span=.88)
        e = enc_avancando(med)
        e.obstaculo = .05
        self.assertEqual(e.passo(10.05, Z), (0., 0., 0.))
        self.assertEqual(e.estado, 'abortar')

    def test_distancia_negativa_aborta(self):
        e = enc_avancando(dict(dist=.53, ang=0., centro=0., span=.88))
        e.nova_medicao(10.1, dict(dist=.35, ang=0., centro=0., span=.88), Z)
        self.assertEqual(e.estado, 'abortar')
        self.assertEqual(e.passo(10.15, Z), (0., 0., 0.))

    def test_face_sumiu_depois_de_andar_nao_e_sucesso(self):
        # ultima face valida medida ANDANDO (antes de parar): sumir nao vira sucesso
        e = enc_avancando(dict(dist=.455, ang=0., centro=0., span=.88))
        e._mudar('assentar_final', 10.1, Z)
        e._mudar('medir_final', 10.5, Z)
        e.meds.clear()
        e.passo(13.6, Z)
        self.assertEqual(e.estado, 'abortar')

    def test_face_sumiu_medida_parada_vale(self):
        e = enc_avancando(dict(dist=.455, ang=0., centro=0., span=.88))
        e._mudar('assentar_final', 10.1, Z)
        e.nova_medicao(10.45, dict(dist=.455, ang=0., centro=0., span=.88), Z)
        e._mudar('medir_final', 10.5, Z)
        e.meds.clear()
        e.passo(13.6, Z)
        self.assertEqual(e.estado, 'fim')

    def test_falha_nunca_publica_avanco(self):
        e = enc_avancando(dict(dist=.53, ang=0., centro=0., span=.88))
        e.meds.clear()                       # perdeu tudo longe da zona cega
        saidas = [e.passo(10.05 + .05 * k, Z) for k in range(40)]
        self.assertTrue(all(s == (0., 0., 0.) for s in saidas))
        self.assertEqual(e.estado, 'abortar')


@unittest.skipUnless(DADOS.exists(), 'sem dados da missao32')
class TestMissao32Real(unittest.TestCase):
    """Scans reais da WS_1 (missao32). Antes: 'nenhuma face com as 2 pontas' e encosto abortado."""
    @classmethod
    def setUpClass(cls):
        cls.dados = json.loads(DADOS.read_text())

    def rodar(self, janela):
        r = RastreioFaceWS(self.dados['ws1'])
        out = []
        for s in self.dados['janelas'][janela]:
            pts = [tuple(p) for p in s['pts']]
            m = r.medir(pts, s['t'], tuple(s['odom']), tuple(s['pose_mapa']))
            out.append((s['hora'], m, r.motivo, pts))
        return out

    def test_tentativa4_face_partida_continua_acompanhada(self):
        out = self.rodar('tentativa4')
        finais = [o for o in out if o[0] >= '14:12:39.4']
        self.assertGreater(len(finais), 15)
        for hora, m, motivo, pts in finais:
            self.assertIsNotNone(m, '%s %s' % (hora, motivo))
            self.assertAlmostEqual(m['dist'], .456, delta=.01)
            self.assertLess(abs(math.degrees(m['ang'])), 2.)
        cs = [m['centro'] for _, m, _, _ in finais if m['centro'] is not None]
        self.assertGreater(len(cs), len(finais) // 2)
        # centro medido pelas pontas atuais: 4,7..8 cm (o rastreio de 14:23 dava +2,7 projetado da odom)
        self.assertTrue(all(.04 <= c <= .09 for c in cs), cs)
        self.assertTrue(any(m.get('fragmentada') for _, m, _, _ in finais))

    def test_tentativa4_pedacos_sao_a_mesma_face(self):
        # 14:12:39.59: os 2 segmentos do log (d0.44 w0.30, d0.47 w0.34) sao um plano so
        s = [s for s in self.dados['janelas']['tentativa4'] if s['hora'] == '14:12:39.590'][0]
        pts = [tuple(p) for p in s['pts']]
        sem = candidatos_face(pts)
        self.assertEqual(len([m for m in sem if m['centro'] is None and m['dist'] < .5]), 2)
        com = [m for m in candidatos_face(pts, unir_cega=True) if m.get('fragmentada')]
        self.assertEqual(len(com), 1)
        y1, y2 = com[0]['lacuna']
        self.assertTrue(-.10 < y1 < -.06 and .06 < y2 < .09, (y1, y2))

    def test_tentativa3_deslize_perto_continua_acompanhado(self):
        out = self.rodar('tentativa3')
        finais = [o for o in out if o[0] >= '14:12:15.3']
        self.assertTrue(all(m is not None for _, m, _, _ in finais))
        cs = [m['centro'] for _, m, _, _ in finais if m['centro'] is not None]
        self.assertTrue(all(.13 <= c <= .18 for c in cs), cs)   # ainda 15 cm fora: nao finge centro

    def test_referencia_do_mapa_incoerente_com_a_face(self):
        # Diagnostico (nao corrige): a face medida na WS_1 fica 15-25 graus girada no map em relacao
        # ao cadastro (+3,1 graus). E o motivo de 'FORA:ang' nas tentativas 1 e 2.
        difs = []
        for s in self.dados['janelas']['tentativa4']:
            ref = referencia_ws(self.dados['ws1'], tuple(s['pose_mapa']))
            m = selecionar_face([tuple(p) for p in s['pts']])
            if ref is not None and m is not None and m['centro'] is not None:
                difs.append(abs(math.degrees(fw.ang(m['ang'] - ref['ang']))))
        self.assertGreater(len(difs), 30)
        self.assertGreater(max(difs), 19)   # 14:12:37: ref -21 graus, face medida 0..2
        self.assertGreater(min(difs), 8)


if __name__ == '__main__':
    unittest.main()
