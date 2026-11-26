import math
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from giro_frontal import (GiroFrontal, ArbitragemT, disco_observado, ZERO,
                          TempoSensor, Observacao, avaliar_varredura, devolver_controle)
from alinhador_passagem import Encosto, TravessiaLateral
from suavizador_nav import comando_corredor


class TestGiro(unittest.TestCase):
    def novo(self, lado=1, timeout=90):
        return GiroFrontal((0, 0), (0, lado), (0, lado*0.5), timeout=timeout)

    def passo(self, g, t, yaw=0, x=0, y=0, **kw):
        args = dict(pose=(x, y, yaw), imu=(t, yaw, 0), scan_t=t,
                    livre=True, parado=True)
        args.update(kw)
        return g.passo(t, **args)

    def girando(self, lado=1):
        g = self.novo(lado)
        self.assertEqual(self.passo(g, 0), ZERO)
        self.assertEqual(self.passo(g, 0.7), ZERO)
        self.assertEqual(g.estado, 'girar')
        return g

    def test_esquerda_direita_e_conclusao(self):
        for lado in (-1, 1):
            with self.subTest(lado=lado):
                g = self.girando(lado)
                self.assertGreater(self.passo(g, 0.8)[2]*lado, 0)
                self.passo(g, 1, yaw=lado*0.8)
                self.assertEqual(self.passo(g, 2, yaw=lado*math.pi/2), ZERO)
                self.assertEqual(self.passo(g, 2.6, yaw=lado*math.pi/2), ZERO)
                self.assertEqual(g.estado, 'avancar')
                cmd = self.passo(g, 2.7, yaw=lado*math.pi/2)
                self.assertGreater(cmd[0], 0)
                self.assertEqual(cmd[1], 0)
                self.assertEqual(self.passo(g, 3, yaw=lado*math.pi/2, y=lado*1.1), ZERO)
                self.assertEqual(g.estado, 'fim')

    def test_sem_espaco(self):
        g = self.girando()
        self.assertEqual(self.passo(g, 1, livre=False), ZERO)
        self.assertEqual(g.estado, 'abortar')

    def test_imu_ausente_invalida_antiga(self):
        for imu in (None, (0, 0, 0), (1, float('nan'), 0), (1, 0, float('inf'))):
            g = self.girando()
            self.assertEqual(self.passo(g, 1, imu=imu), ZERO)
            self.assertEqual(g.estado, 'abortar')

    def test_lidar_ausente_antigo_futuro(self):
        for stamp in (None, 0, 2):
            g = self.girando()
            self.assertEqual(self.passo(g, 1, scan_t=stamp), ZERO)
            self.assertEqual(g.estado, 'abortar')

    def test_timeout_cancelamento_bloqueio(self):
        for kw in ({'cancelado': True}, {'bloqueado': True}):
            g = self.girando()
            self.assertEqual(self.passo(g, 1, **kw), ZERO)
            self.assertEqual(g.estado, 'abortar')
        g = self.novo(timeout=0.5)
        self.passo(g, 0)
        self.assertEqual(self.passo(g, 0.6), ZERO)
        self.assertEqual(g.motivo, 'timeout')

    def test_sem_progresso_no_giro_e_avanco(self):
        g = self.girando()
        self.assertEqual(self.passo(g, 7), ZERO)
        self.assertIn('sem progresso', g.motivo)
        g = self.girando()
        self.passo(g, 1, yaw=math.pi/2)
        self.passo(g, 1.6, yaw=math.pi/2)
        self.passo(g, 1.7, yaw=math.pi/2)
        self.assertEqual(self.passo(g, 7, yaw=math.pi/2), ZERO)
        self.assertIn('sem progresso', g.motivo)

    def test_centro_parada_e_confirmacao(self):
        g = self.novo()
        self.assertEqual(self.passo(g, 0, x=0.1), ZERO)
        self.assertEqual(g.estado, 'abortar')
        g = self.novo()
        for t in (0, 1, 2):
            self.assertEqual(self.passo(g, t, parado=False), ZERO)
        self.assertEqual(g.estado, 'parar')
        g = self.girando()
        self.passo(g, 1, imu=(1, math.pi/2, 0))
        self.passo(g, 1.6, imu=(1.6, math.pi/2, 0))
        self.assertEqual(g.estado, 'abortar')

    def test_giro_independe_origem_da_imu(self):
        g = self.novo()
        self.passo(g, 0, imu=(0, 0.4, 0))
        self.passo(g, 0.7, imu=(0.7, 0.4, 0))
        self.assertAlmostEqual(g.alvo_imu, math.pi/2+0.4)


class TestObservacao(unittest.TestCase):
    def livre(self, ranges, laser=(0, 0, 0), inc=math.pi/360):
        return disco_observado(ranges, -math.pi, inc, 0.05, 10, laser, 0.54)

    def test_disco_total_observado(self):
        self.assertTrue(self.livre([2]*720))
        self.assertTrue(self.livre([2]*720, laser=(0.12, 0.02, 0.1)))

    def test_obstaculo_e_setores_desconhecidos(self):
        self.assertFalse(self.livre([float('inf')]*720))
        self.assertFalse(self.livre([float('nan')]*720))
        self.assertFalse(self.livre([2]*500))
        r = [2]*720
        r[200] = 0.4
        self.assertFalse(self.livre(r))
        r[200:220] = [float('inf')]*20
        self.assertFalse(self.livre(r))

    def test_avanco_cabe_no_corredor_sem_exigir_disco_de_giro(self):
        ranges = []
        for i in range(720):
            a = -math.pi+i*math.pi/360
            ranges.append(min(3.0, 0.41/abs(math.sin(a))) if abs(math.sin(a)) > 1e-9 else 3.0)
        self.assertFalse(self.livre(ranges))
        self.assertTrue(disco_observado(ranges, -math.pi, math.pi/360, 0.05, 10,
                                       (0, 0, 0), 0.54, caixa=(-0.255, 0.535, -0.365, 0.365)))
        ranges[360] = 0.5
        self.assertFalse(disco_observado(ranges, -math.pi, math.pi/360, 0.05, 10,
                                        (0, 0, 0), 0.54, caixa=(-0.255, 0.535, -0.365, 0.365)))


class TestArbitragem(unittest.TestCase):
    def test_passagem_normal_e_exclusividade(self):
        a = ArbitragemT()
        cmd = (0.1, 0, 0)
        self.assertEqual(a.normal(0, cmd), cmd)
        self.assertFalse(a.adquirir(0.1))
        self.assertTrue(a.adquirir(1))
        self.assertFalse(a.adquirir(1))
        self.assertEqual(a.experimental(1.1, cmd), cmd)
        self.assertEqual(a.normal(1.2, cmd), ZERO)
        self.assertEqual(a.experimental(1.3, cmd), ZERO)
        self.assertTrue(a.falha)

    def test_watchdog_zeros_normais_e_comandos_sem_lease(self):
        a = ArbitragemT()
        self.assertEqual(a.experimental(0, (1, 0, 0)), ZERO)
        a.adquirir(1)
        self.assertEqual(a.normal(1.1, ZERO), ZERO)
        self.assertFalse(a.falha)
        a.vigiar(1.3)
        self.assertTrue(a.falha)
        self.assertEqual(a.experimental(1.4, (1, 0, 0)), ZERO)

    def test_falha_trava_ate_reset_manual(self):
        a = ArbitragemT()
        a.adquirir(0)
        a.vigiar(0.3)
        a.ativo = False  # liberar lease nao apaga falha
        self.assertEqual(a.normal(0.4, (0.1, 0, 0)), ZERO)
        self.assertFalse(a.adquirir(1))
        self.assertFalse(a.resetar(0.5))
        self.assertTrue(a.resetar(1.1))
        self.assertEqual(a.normal(1.2, (0.1, 0, 0)), (0.1, 0, 0))


class TestEncosto(unittest.TestCase):
    def encosto(self, estado):
        e = Encosto(log=lambda _: None)
        e.t0, e.t_estado, e.estado = 0, 0, estado
        e.yaw_ref = 0
        return e

    def medida(self, e, t, folga=0.08, ang=0, centro=0):
        e.nova_medicao(t, dict(dist=e.pc+folga, ang=ang, centro=centro), (0, 0, 0))

    def test_sem_face_nao_avanca(self):
        for estado in ('avancar', 'pulso', 'lat_mover'):
            e = self.encosto(estado)
            self.assertEqual(e.passo(1, (0, 0, 0)), ZERO)
            # PERDA_PARA (09/11): no deslize para e mede de novo; no avanco aborta
            self.assertEqual(e.estado, 'assentar' if estado == 'lat_mover' else 'abortar')

    def test_face_perdida_apos_medicao_e_face_antiga(self):
        e = self.encosto('avancar')
        self.medida(e, 0.9, folga=0.5)
        e.nova_medicao(1, None, (0, 0, 0))
        # FALHA_SCAN (09/11): 1 scan sem face e tolerado; 0,45 s sem face aborta
        self.assertEqual(e.estado, 'avancar')
        self.assertEqual(e.passo(1.4, (0, 0, 0)), ZERO)
        self.assertEqual(e.estado, 'abortar')
        e = self.encosto('avancar')
        self.medida(e, 0, folga=0.5)
        self.assertEqual(e.passo(1, (0, 0, 0)), ZERO)
        self.assertEqual(e.estado, 'abortar')

    def test_folga_negativa_nunca_sucesso(self):
        for estado in ('medir_final', 'avancar', 'esperar'):
            e = self.encosto(estado)
            self.medida(e, 1, folga=-0.01)
            self.assertEqual(e.passo(1, (0, 0, 0)), ZERO)
            self.assertEqual(e.estado, 'abortar')
            self.assertIsNone(e.resultado)

    def test_folga_negativa_antes_do_primeiro_passo_fica_travada(self):
        e = Encosto(log=lambda _: None)
        self.medida(e, 0, folga=-0.01)
        self.assertEqual(e.passo(0, (0, 0, 0)), ZERO)
        self.assertEqual(e.estado, 'abortar')

    def test_resultado_final_fora_limites(self):
        for ang, centro in ((0.2, 0), (0, 0.1), (0, None)):
            e = self.encosto('medir_final')
            e.n_lat_final = 2
            e.g_final = True
            for t in (0.7, 0.8, 0.9, 1):
                self.medida(e, t, ang=ang, centro=centro)
            self.assertEqual(e.passo(1, (0, 0, 0)), ZERO)
            self.assertEqual(e.estado, 'abortar')

    def test_sucesso_exige_medicao_final_valida(self):
        e = self.encosto('medir_final')
        for t in (0.7, 0.8, 0.9, 1):
            self.medida(e, t)
        e.passo(1, (0, 0, 0))
        self.assertEqual(e.estado, 'fim')


class TestLateral(unittest.TestCase):
    def test_saida_lateral_antiga_nos_dois_sentidos(self):
        for lado in (-1, 1):
            t = TravessiaLateral(lado, log=lambda _: None)
            t.passo(0, (0, 0, 0), 0.5)
            t.passo(0.5, (0, 0, 0), 0.5)
            t.passo(2.6, (0, 0, 0), 0.5)
            self.assertEqual(t.estado, 'deslizar')
            cmd = t.passo(2.7, (0, 0, 0), 0.5)
            self.assertGreater(cmd[1]*lado, 0)
            self.assertEqual(cmd[0], 0)
            self.assertEqual(t.passo(3, (0, lado*0.5, 0), 0), ZERO)
            self.assertEqual(t.estado, 'fim')

    def test_estados_iniciais_laterais(self):
        for lado in (-1, 1):
            t = TravessiaLateral(lado, log=lambda _: None)
            self.assertEqual(t.passo(0, (0, 0, 0), 2), ZERO)
            self.assertEqual(t.lado, lado)
            self.assertEqual(t.estado, 'assentar')


class TestAdaptadorROS(unittest.TestCase):
    """Executa os metodos reais do no com mensagens/relogio falsos, sem ROS spin."""
    def setUp(self):
        import ast
        import threading
        import time
        from types import SimpleNamespace as NS
        self.NS = NS
        tree = ast.parse((SCRIPTS/'alinhador_passagem.py').read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        cls = next(n for n in main.body if isinstance(n, ast.ClassDef) and n.name == 'Alinhador')
        module = ast.Module(body=[cls], type_ignores=[])
        def twist():
            return NS(header=NS(stamp=None, frame_id=''),
                      twist=NS(linear=NS(x=0, y=0), angular=NS(z=0)))
        ns = dict(Node=object, time=time, math=math, TempoSensor=TempoSensor,
                  json=__import__('json'),
                  TwistStamped=twist, comando_corredor=comando_corredor,
                  yaw_de=lambda q: 2*math.atan2(q.z, q.w))
        exec(compile(module, str(SCRIPTS/'alinhador_passagem.py'), 'exec'), ns)
        self.ns = ns
        self.node = ns['Alinhador'].__new__(ns['Alinhador'])
        self.node.em_preparo = False
        self.node.fator_corredor = 0.5
        self.node.lock = threading.Lock()
        self.node.tempos_t, self.node.ultimos_stamps_t = {}, {}
        self.node.p = lambda name: 0.2 if name in ('limite_vel_angular_real_t','limite_vel_linear_real_t') else 0.0
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000,
                                                         to_msg=lambda: 'stamp'))

    def test_imu_rejeita_covariancia_e_stamp_invalidos(self):
        ns = self.NS
        msg = ns(header=ns(stamp=ns(sec=10, nanosec=0)),
                 orientation=ns(x=0, y=0, z=0, w=1),
                 angular_velocity=ns(z=0), orientation_covariance=[0]*9)
        self.node.ao_imu(msg, experimental=True)
        self.assertIsNotNone(self.node.imu_t)
        msg.orientation_covariance[0] = -1
        self.node.ao_imu(msg, experimental=True)
        self.assertIsNone(self.node.imu_t)
        msg.orientation_covariance[0] = 0
        msg.header.stamp.sec = 9
        self.node.ao_imu(msg, experimental=True)
        self.assertIsNone(self.node.imu_t)

    def test_scan_com_timestamp_antigo_invalida_area(self):
        self.node.scan_t_raw = 'anterior'
        msg = self.NS(header=self.NS(stamp=self.NS(sec=9, nanosec=0)))
        self.node.ao_scan(msg, experimental=True)
        self.assertIsNone(self.node.scan_t_raw)

    def test_rota_exclusiva_de_comandos(self):
        normal, experimental = [], []
        self.node.pub_cmd = self.NS(publish=normal.append)
        self.node.pub_t = self.NS(publish=experimental.append)
        self.node.controle_t_ativo = False
        self.node._publicar_vel(0.1, 0, 0)
        self.node.controle_t_ativo = True
        self.node._publicar_vel(0, 0, 0.1)
        self.assertEqual(len(normal), 1)
        self.assertEqual(len(experimental), 1)
        self.assertEqual(experimental[0].twist.angular.z, 0.1)

    def test_ganho_na_saida_do_alinhador(self):
        enviados = []
        self.node.pub_cmd = self.NS(publish=enviados.append)
        self.node.controle_t_ativo = False
        for preparo, corredor in ((False, False), (True, False), (False, True), (True, True)):
            self.node.em_preparo = preparo
            self.node._publicar_vel(0.06, -0.08, 0.0, corredor=corredor)
            linear = enviados[-1].twist.linear
            self.assertEqual((linear.x, linear.y), (0.03, -0.04) if preparo or corredor else (0.06, -0.08))

    def test_topologia_com_publicador_extra_e_recusada(self):
        self.node.get_name = lambda: 'alinhador_passagem'
        pubs = {'/cmd_vel_nav_seguro': [self.NS(node_name='arbitro_t')],
                '/cmd_vel_nav_t': [self.NS(node_name='alinhador_passagem')],
                '/cmd_vel_smoothed': [self.NS(node_name='velocity_smoother')],
                '/cmd_vel': [self.NS(node_name='collision_monitor')]}
        self.node.get_publishers_info_by_topic = pubs.get
        self.assertTrue(self.node._arbitro_exclusivo())
        pubs['/cmd_vel_nav_seguro'].append(self.NS(node_name='intruso'))
        self.assertFalse(self.node._arbitro_exclusivo())

    def test_falha_temporal_explicitamente_invalida_amostra(self):
        self.node.scan_t_raw, self.node.imu_t = 'scan anterior', 'imu anterior'
        self.node.tempos_t = dict(lidar='anterior', imu='anterior')
        self.node.ao_timing_t(self.NS(data='{"sensor":"lidar","timed_frame":true,"valid":false}'))
        self.assertIsNone(self.node.scan_t_raw)
        self.assertIsNone(self.node.tempos_t['lidar'])
        self.node.ao_timing_t(self.NS(data='{"sensor":"imu","timed_frame":false,"valid":false}'))
        self.assertEqual(self.node.imu_t,'imu anterior')  # legado nao sobrescreve fluxo temporal
        self.node.ao_timing_t(self.NS(data='{"sensor":"imu","timed_frame":true,"valid":false}'))
        self.assertIsNone(self.node.imu_t)

    def test_callback_imu_nao_renova_029_para_058(self):
        self.node.get_clock = lambda: self.NS(now=lambda: self.NS(nanoseconds=10_290_000_000))
        msg = self.NS(header=self.NS(stamp=self.NS(sec=10, nanosec=0)),
                      orientation=self.NS(x=0, y=0, z=0, w=1),
                      angular_velocity=self.NS(z=0), orientation_covariance=[0]*9)
        self.node.ao_imu(msg, experimental=True)
        sample = self.node.tempos_t['imu']
        self.assertIsNotNone(sample)
        self.assertAlmostEqual(sample.mono_rx-self.node.imu_t[0], 0.29)
        self.assertFalse(sample.recente(10_580_000_000, sample.mono_rx+0.29))

    def test_scan_temporal_nao_desconta_duas_vezes_sem_reescrever_header(self):
        self.node.laser_tf = (0, 0, 0)
        self.node.odom, self.node.scan_seq = None, 0
        self.node.get_clock = lambda: self.NS(now=lambda: self.NS(nanoseconds=10_050_000_000))
        msg = self.NS(header=self.NS(stamp=self.NS(sec=10, nanosec=0)),
                      scan_time=0.17, time_increment=0.17/180,
                      angle_min=-math.pi, angle_increment=math.pi/90,
                      range_min=0.05, range_max=10, ranges=[2.0]*180)
        self.node.p = lambda name: 0.02
        self.node.ao_scan(msg, experimental=True)
        sample = self.node.tempos_t['lidar']
        self.assertAlmostEqual(sample.mono_rx-self.node.scan_t_raw[0], 0.05)
        self.assertEqual(msg.header.stamp.sec, 10)
        self.assertEqual(msg.header.stamp.nanosec, 0)
        self.assertEqual(sample.stamp_ns, 10_000_000_000)
        # Janela deslizante pode manter o raio mais antigo: conserva MESMO prazo.
        self.node.ao_scan(msg, experimental=True)
        self.assertEqual(self.node.tempos_t['lidar'],sample)
        self.assertEqual(self.node.scan_t_raw[0],sample.aquisicao_mono)

    def test_precondicao_aceita_180_bins_com_lacuna_limitada_e_exige_tempo(self):
        import time
        stamp = self.NS(sec=10, nanosec=0)
        sample = TempoSensor.receber(stamp, 10_000_000_000, time.monotonic())
        self.node.tempos_t = dict(lidar=sample, imu=sample, odom=sample)
        self.node.scan_t_raw = (sample.aquisicao_mono, [2]*180, -math.pi,
                                math.pi/90, 0.05, 10, (0, 0, 0))
        self.assertTrue(self.node._precondicoes_t())
        self.node.scan_t_raw[1][10] = math.inf
        self.assertFalse(self.node._precondicoes_t())
        self.node.scan_t_raw = (sample.aquisicao_mono, [2]*720, -math.pi,
                                math.pi/360, 0.05, 10, (0, 0, 0))
        self.assertTrue(self.node._precondicoes_t())
        self.node.tempos_t['lidar'] = None
        self.assertFalse(self.node._precondicoes_t())

    def test_rotina_real_nao_retorna_ok_sem_devolucao(self):
        for modo in ('sucesso', 'recusa', 'sem_resposta', 'excecao'):
            with self.subTest(modo=modo):
                a, clock = ArbitragemT(), [0.0]
                def esperar(dt):
                    clock[0] += dt
                def mandar(*cmd):
                    a.experimental(clock[0], cmd)
                def parar():
                    for _ in range(5):
                        mandar(*ZERO)
                        esperar(0.05)
                def completo(*args, **kwargs):
                    g = GiroFrontal(*args, **kwargs)
                    g.estado = 'fim'  # simula somente a chegada, testa finally real
                    return g
                def enviar(req):
                    if req.data:
                        ok = a.adquirir(clock[0])
                        return self.NS(done=lambda: True, result=lambda: self.NS(success=ok))
                    if modo == 'excecao':
                        raise RuntimeError('RPC')
                    if modo == 'sem_resposta':
                        return self.NS(done=lambda: False)
                    ok = a.liberar(clock[0]) if modo == 'sucesso' else False
                    return self.NS(done=lambda: True, result=lambda: self.NS(
                        success=ok, message='parado_confirmado' if ok else 'recusado'))
                self.ns.update(GiroFrontal=completo, rclpy=self.NS(ok=lambda: False),
                               SetBool=self.NS(Request=lambda: self.NS(data=False)),
                               time=self.NS(monotonic=lambda: clock[0], sleep=esperar),
                               devolver_controle=lambda *args: devolver_controle(*args,
                                   relogio=lambda: clock[0], esperar=esperar, timeout=0.1))
                self.node.controle_t = self.NS(wait_for_service=lambda **kw: True, call_async=enviar)
                self.node._arbitro_exclusivo = lambda: True
                self.node._verificar_cm_t = lambda: True
                self.node._estado_t = lambda *args: None
                self.node.handle_ativo = None
                self.node._parar, self.node._publicar_vel = parar, mandar
                self.node.get_logger = lambda: self.NS(info=lambda _: None)
                r = self.node._travessia_giro_frontal(
                    dict(juncao=(0, 0), eixo_b=(0, 1), saida_b=(0, 1)), 'teste')
                self.assertEqual(r, 'ok' if modo == 'sucesso' else 'falha')
                a.vigiar(clock[0]+0.3)
                self.assertFalse(a.ativo)
                self.assertEqual(a.normal(clock[0]+0.4, (0.1, 0, 0)), ZERO)



class TestTempoTotal(unittest.TestCase):
    def stamp(self, value):
        from types import SimpleNamespace
        return SimpleNamespace(sec=int(value), nanosec=round((value-int(value))*1e9))

    def test_recente_preserva_stamp_original(self):
        t = TempoSensor.receber(self.stamp(10), 10_100_000_000, 100)
        self.assertEqual(t.stamp_ns, 10_000_000_000)
        self.assertAlmostEqual(t.aquisicao_mono, 99.9)
        self.assertTrue(t.recente(10_200_000_000, 100.1))

    def test_regressao_058_segundos(self):
        t = TempoSensor.receber(self.stamp(10), 10_290_000_000, 100)
        self.assertIsNotNone(t)
        self.assertFalse(t.recente(10_580_000_000, 100.29))
        g = GiroFrontal((0, 0), (0, 1), (0, 1))
        self.assertEqual(g.passo(100.29, (0, 0, 0), (t.aquisicao_mono, 0, 0),
                                 t.aquisicao_mono, True, True), ZERO)
        self.assertEqual(g.estado, 'abortar')

    def test_atrasado_ja_na_recebida(self):
        self.assertIsNone(TempoSensor.receber(self.stamp(10), 10_310_000_000, 100))

    def test_nulo_futuro_negativo_malformado_repetido(self):
        from types import SimpleNamespace as NS
        for stamp in (None, NS(sec=0, nanosec=0), NS(sec=-1, nanosec=0),
                      NS(sec=10, nanosec=-1), NS(sec=10, nanosec=1_000_000_000),
                      NS(sec=11, nanosec=0)):
            self.assertIsNone(TempoSensor.receber(stamp, 10_000_000_000, 100))
        self.assertIsNone(TempoSensor.receber(self.stamp(10), 10_000_000_000, 100,
                                             anterior=10_000_000_000))
        self.assertIsNone(TempoSensor.receber(self.stamp(10), 10_000_000_000, 100,
                                             anterior=10_010_000_000))

    def test_salto_e_pausa_de_relogio(self):
        t = TempoSensor.receber(self.stamp(10), 10_000_000_000, 100)
        for ros, mono in ((9_900_000_000, 100.1), (10_500_000_000, 100.1),
                          (10_000_000_000, 100.2), (10_000_000_000, 99)):
            self.assertFalse(t.recente(ros, mono))

    def test_driver_publicacao_inclui_varredura_e_fila(self):
        # Scan 0.17 s + fila limitada a 0.02 + entrega 0.05 = idade 0.24.
        t = TempoSensor.receber(self.stamp(10), 10_050_000_000, 100, atraso=0.19)
        self.assertIsNotNone(t)
        self.assertTrue(t.recente(10_090_000_000, 100.04))
        self.assertFalse(t.recente(10_150_000_000, 100.1))
        for atraso in (-1, float('nan'), float('inf'), 0.31):
            self.assertIsNone(TempoSensor.receber(self.stamp(10), 10_000_000_000, 100,
                                                 atraso=atraso))


class TestGeometriaRevisao2(unittest.TestCase):
    def avaliar(self, r, inc=math.pi/360):
        return avaliar_varredura(r, -math.pi, inc, 0.05, 10, (0, 0, 0), 0.54)

    def test_um_raio_invalido_nao_e_preenchido_por_vizinho(self):
        r = [2.0]*720
        for valor in (float('nan'), float('inf'), 0.01, 11.0):
            r[100] = valor
            self.assertEqual(self.avaliar(r), Observacao.DESCONHECIDA)

    def test_180_bins_lacuna_limitada_sem_preencher_raios_invalidos(self):
        self.assertEqual(self.avaliar([2.0]*180, math.pi/90), Observacao.LIVRE)
        r = [float('nan')]*720
        r[::4] = [2.0]*180
        self.assertEqual(self.avaliar(r), Observacao.DESCONHECIDA)

    def test_livre_obstaculo_desconhecido_distintos(self):
        self.assertEqual(self.avaliar([2.0]*720), Observacao.LIVRE)
        r = [float('nan')]*720
        r[300] = 0.4
        self.assertEqual(self.avaliar(r), Observacao.OBSTACULO)
        self.assertEqual(self.avaliar([2.0]*600), Observacao.DESCONHECIDA)
        self.assertEqual(self.avaliar([2.0]*720, 0), Observacao.DESCONHECIDA)

    def test_obstaculo_tem_precedencia_em_scan_de_baixa_resolucao(self):
        r = [2.0]*180
        r[100] = 0.4
        self.assertEqual(self.avaliar(r, math.pi/90), Observacao.OBSTACULO)

    def test_geometria_invalida_e_desconhecida(self):
        self.assertEqual(avaliar_varredura([2]*720, -math.pi, math.pi/360,
                                          0.05, 10, (1, 0, 0), 0.54), Observacao.DESCONHECIDA)


class TestDevolucao(unittest.TestCase):
    def executar(self, modo):
        from types import SimpleNamespace as NS
        a = ArbitragemT()
        a.adquirir(0)
        a.experimental(0, ZERO)
        a.experimental(0.2, ZERO)
        a.experimental(0.3, ZERO)
        relogio = [0.3]
        zeros = []
        def publicar():
            zeros.append(ZERO)
            a.experimental(relogio[0], ZERO)
        def esperar(dt):
            relogio[0] += dt
        class Cliente:
            def call_async(self, req):
                if modo == 'excecao_envio':
                    raise RuntimeError('envio')
                if modo == 'sem_resposta':
                    return NS(done=lambda: False)
                if modo == 'resposta_perdida':
                    a.liberar(relogio[0])
                    return NS(done=lambda: False)
                if modo == 'excecao_futuro':
                    def erro():
                        raise RuntimeError('futuro')
                    return NS(done=lambda: True, result=erro)
                if modo == 'sucesso':
                    ok = a.liberar(relogio[0])
                    return NS(done=lambda: True, result=lambda: NS(success=ok, message='parado_confirmado'))
                return NS(done=lambda: True, result=lambda: NS(success=False, message='recusado'))
        resultado = devolver_controle(Cliente(), NS(data=False), publicar,
                                      relogio=lambda: relogio[0], esperar=esperar, timeout=0.1)
        a.vigiar(relogio[0]+0.3)
        return resultado, a, zeros

    def test_sucesso_somente_apos_estado_parado_confirmado(self):
        (ok, motivo), a, zeros = self.executar('sucesso')
        self.assertTrue(ok)
        self.assertEqual(motivo, '')
        self.assertFalse(a.ativo)
        self.assertTrue(a.estacionado)
        self.assertTrue(zeros)
        self.assertEqual(a.normal(2, (0.1, 0, 0)), ZERO)
        self.assertTrue(a.liberar(2))  # idempotente sem reativar fontes

    def test_falha_timeout_e_excecoes_nao_sao_sucesso(self):
        for modo in ('recusa', 'sem_resposta', 'excecao_envio', 'excecao_futuro', 'resposta_perdida'):
            with self.subTest(modo=modo):
                (ok, motivo), a, zeros = self.executar(modo)
                self.assertFalse(ok)
                self.assertTrue(motivo)
                self.assertTrue(zeros)
                self.assertFalse(a.ativo)
                self.assertEqual(a.normal(3, (0.1, 0, 0)), ZERO)
                self.assertEqual(a.experimental(3, (0.1, 0, 0)), ZERO)

    def test_release_sem_parada_falha_fechado(self):
        a = ArbitragemT()
        a.adquirir(0)
        a.experimental(0.1, (0, 0, 0.1))
        self.assertFalse(a.liberar(0.2))
        self.assertFalse(a.ativo)
        self.assertTrue(a.estacionado)
        self.assertTrue(a.falha)

    def test_reset_nao_libera_manobra_ativa_e_exige_silencio(self):
        a = ArbitragemT()
        a.adquirir(0)
        self.assertFalse(a.resetar(0.1))
        a.experimental(0.1, ZERO)
        a.experimental(0.4, ZERO)
        self.assertTrue(a.liberar(0.4))
        self.assertFalse(a.resetar(0.5))
        self.assertTrue(a.resetar(1.1))
        self.assertEqual(a.normal(1.2, (0.1, 0, 0)), (0.1, 0, 0))


if __name__ == '__main__':
    unittest.main()
