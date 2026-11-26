"""BASELINE_LEGADO (11/11/2026): arquitetura 2D Goal Pose -> alinhador -> Nav2/destravar, encosto como na missao24.

- FACE_LIDAR_M24: fase A do encosto escolhe a face so pelo LiDAR (largura da WS filtra paredes); a referencia do
  mapa (09/11) recusava a face real da missao32 por angulo (d0.62 a+19g vs ref a-1g).
- Subida e launch: legado por padrao; supervisor novo so por opcao; reinsistir_goal_final:=0 pelo launch.
Sem ROS, sem robo.
"""
import ast
import json
import math
import re
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
NAV = ROOT / 'src/robo_navigation'
sys.path.insert(0, str(NAV / 'scripts'))
sys.path.insert(0, str(NAV / 'test'))
from face_ws import RastreioFaceWS, selecionar_face_lidar   # noqa: E402
from test_face_fragmentada import lidar, face, DADOS, Z   # noqa: E402

SUBIR = Path.home() / 'subir_robo.sh'


def rastrear(r, pts, n=4, pose_mapa=Z):
    out = []
    for k in range(n):
        out.append(r.medir(pts, .17 * k, Z, pose_mapa))
    return out


class TestFaceLidarM24(unittest.TestCase):
    WS = dict(x=.60, y=0., yaw=0., largura_medida=.85)

    def test_face_inclinada_19_graus_aceita_sem_mapa_e_recusada_com_mapa(self):
        # missao32 WS_1: face medida d0.62 c+0.03 a+19g w0.85; a referencia do mapa dizia a-1g (diferenca 20g,
        # limite TOL_ANG_REF 20g -> FORA:ang). Aqui: pose do SLAM com yaw +2g -> referencia a-2g, diferenca 21g.
        pts = lidar([face(.62, centro=.03, largura=.85, a=math.radians(19))])
        com_mapa = RastreioFaceWS(self.WS, usar_mapa=True)
        pose_slam = (0., 0., math.radians(2))
        self.assertTrue(all(m is None for m in rastrear(com_mapa, pts, pose_mapa=pose_slam)))
        self.assertIn('FORA:ang', com_mapa.motivo)
        sem_mapa = RastreioFaceWS(self.WS, usar_mapa=False)
        ms = rastrear(sem_mapa, pts)
        self.assertIsNone(ms[0])            # 1a observacao: espera a 2a coerente (igual a antes)
        self.assertTrue(all(m is not None for m in ms[1:]), sem_mapa.motivo)
        self.assertAlmostEqual(math.degrees(ms[-1]['ang']), 19., delta=1.5)
        # face_ws mede dist no eixo x do robo (x + tan(a)*y): normal 0,62 / cos(19g) = 0,656
        self.assertAlmostEqual(ms[-1]['dist'], .62 / math.cos(math.radians(19)), delta=.02)

    def test_sem_mapa_ignora_pose_do_slam(self):
        # pose do SLAM errada/ausente nao muda nada sem mapa
        pts = lidar([face(.60, largura=.85)])
        for pose in (Z, (.3, -.2, math.radians(14)), None):
            r = RastreioFaceWS(self.WS, usar_mapa=False)
            ms = [r.medir(pts, .17 * k, Z, pose) for k in range(3)]
            self.assertIsNotNone(ms[-1], (pose, r.motivo))

    def test_parede_longa_nao_vira_mesa(self):
        parede = lidar([((.70, -1.5), (.70, 1.5))])
        r = RastreioFaceWS(self.WS, usar_mapa=False)
        self.assertTrue(all(m is None for m in rastrear(r, parede)))
        self.assertIn('sem mapa', r.motivo)

    def test_mesa_na_frente_de_parede_escolhe_a_mesa(self):
        pts = lidar([face(.55, largura=.85), ((1.20, -1.5), (1.20, 1.5))])
        m = selecionar_face_lidar(pts, .85)
        self.assertIsNotNone(m)
        self.assertAlmostEqual(m['dist'], .55, delta=.02)

    def test_largura_errada_recusada(self):
        pts = lidar([face(.60, largura=.45)])
        self.assertIsNone(selecionar_face_lidar(pts, .85))

    def test_salto_geometrico_continua_derrubando(self):
        r = RastreioFaceWS(self.WS, usar_mapa=False)
        a = lidar([face(.60, largura=.85)])
        rastrear(r, a, 3)
        b = lidar([face(.75, largura=.85)])   # 15 cm de salto sem a odom ver
        self.assertIsNone(r.medir(b, .51, Z, Z))

    def test_padrao_da_classe_inalterado(self):
        self.assertTrue(RastreioFaceWS(self.WS).usar_mapa)


@unittest.skipUnless(DADOS.exists(), 'sem dados da missao32')
class TestFaceLidarMissao32Real(unittest.TestCase):
    def test_scans_reais_reconhecidos_sem_mapa(self):
        d = json.loads(DADOS.read_text())
        for janela, scans in d['janelas'].items():
            r = RastreioFaceWS(d['ws1'], usar_mapa=False)
            ok = [r.medir([tuple(p) for p in s['pts']], s['t'], tuple(s['odom']), tuple(s['pose_mapa']))
                  for s in scans]
            vistos = [m for m in ok if m is not None]
            self.assertGreater(len(vistos), len(scans) // 2, janela)
            self.assertTrue(all(.35 < m['dist'] < 1.0 for m in vistos))


class TestBaselineConfig(unittest.TestCase):
    def test_alinhador_encosto_sem_mapa_por_padrao(self):
        src = (NAV / 'scripts/alinhador_passagem.py').read_text()
        ast.parse(src)
        self.assertIn("self.declare_parameter('encosto_referencia_mapa', False)", src)
        self.assertIn("RastreioFaceWS(face, usar_mapa=bool(self.p('encosto_referencia_mapa')))", src)
        # codigo do supervisor preservado
        self.assertIn('def _executar_principal', src)
        self.assertTrue((NAV / 'scripts/supervisor_passagem.py').exists())

    def test_launch_legado_por_padrao_e_reinsistir_zero(self):
        src = (NAV / 'launch/navigation.launch.py').read_text()
        self.assertRegex(src, r"DeclareLaunchArgument\('nav2_principal', default_value='false'")
        self.assertIn('"reinsistir_goal_final:=0"', src)
        self.assertIn('"insistir_curva:=0"', src)
        # arbitro so no perfil novo
        self.assertRegex(src, r"arbitro_nav_principal\.py'\)\],\s*condition=IfCondition\(principal\)")

    @unittest.skipUnless(SUBIR.exists(), 'sem ~/subir_robo.sh')
    def test_subida_legada_por_padrao(self):
        s = SUBIR.read_text()
        self.assertIn('NAV_LEGADO=1; SUPERVISOR_MODO=observar', s)
        self.assertRegex(s, r'--supervisor\|--supervisor-observar\) NAV_LEGADO=0')
        self.assertIn('--supervisor-aplicar) NAV_LEGADO=0', s)
        self.assertIn('navigation.launch.py nav2_principal:=false', s)
        # preflight do perfil novo so quando pedido
        self.assertRegex(s, r'if \[ "\$NAV_LEGADO" = 0 \]; then\n\s+bash "\$WS/ferramentas/iniciar_supervisor_validado.sh"')


if __name__ == '__main__':
    unittest.main()
