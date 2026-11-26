"""Integracao simulada em memoria. Nao inicia ROS nem publica velocidade."""
import ast
import importlib.util
import json
import math
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/robo_navigation/scripts'))
from giro_frontal import GiroFrontal, ArbitragemT, ZERO, Observacao, avaliar_varredura


def module(name,file):
    spec=importlib.util.spec_from_file_location(name,file)
    result=importlib.util.module_from_spec(spec)
    sys.modules[name]=result
    spec.loader.exec_module(result)
    return result


class TestLacunas(unittest.TestCase):
    def test_180_bins_com_sensor_deslocado(self):
        self.assertEqual(avaliar_varredura([2]*180,-math.pi,math.pi/90,0.08,6,
                         (0.330,0.005,0),math.hypot(.365,.315)+.05), Observacao.LIVRE)

    def test_lacuna_fisica_grande_e_desconhecida(self):
        self.assertEqual(avaliar_varredura([2]*36,-math.pi,math.pi/18,0.08,6,
                         (.33,.005,0),.532),Observacao.DESCONHECIDA)

    def test_autorretorno_mascarado_continua_desconhecido_para_giro(self):
        cfg=yaml.safe_load((ROOT/'src/robo_lidar/config/lidar.yaml').read_text())
        params=cfg['scan_filter']['ros__parameters']
        self.assertTrue(params['body_filter_enabled'])
        lidar=module('scan_filter_m35_integrado',ROOT/'src/robo_lidar/scripts/scan_filter_node.py')
        r=[2.0]*180
        # Motor real a 140 graus / 40 cm; NaN nao vira prova de area livre.
        r[160]=.40
        for i,dist in enumerate(r):
            if lidar.ponto_do_corpo(dist,-math.pi+i*math.pi/90):
                r[i]=math.nan
        self.assertTrue(math.isnan(r[160]))
        self.assertEqual(avaliar_varredura(r,-math.pi,math.pi/90,.08,6,(.33,.005,0),.532),
                         Observacao.DESCONHECIDA)

    def test_movimento_durante_varredura_exige_cobertura_adicional(self):
        self.assertEqual(avaliar_varredura([2.0]*180,-math.pi,math.pi/90,.08,6,(.33,.005,0),.532,
            movimento_angular=.2),Observacao.DESCONHECIDA)
        r=[2.0]*180; r[90]=.59
        self.assertEqual(avaliar_varredura(r,-math.pi,math.pi/90,.08,6,(0,0,0),.54,
            movimento_linear=.03),Observacao.OBSTACULO)

    def test_um_bin_sem_retorno_nunca_e_livre(self):
        for value in (math.inf,math.nan):
            r=[2.0]*180
            r[80]=value
            self.assertEqual(avaliar_varredura(r,-math.pi,math.pi/90,.08,6,(.33,.005,0),.532),
                             Observacao.DESCONHECIDA)

    def test_reserva_de_lacuna_aumenta_folga(self):
        # O mesmo retorno seria alem de raio+2,5 cm; agora a lacuna tambem e reservada.
        r=[2.0]*180
        r[90]=.54+.025+.005
        self.assertEqual(avaliar_varredura(r,-math.pi,math.pi/90,.08,6,(0,0,0),.54),
                         Observacao.OBSTACULO)


class TestIntegracaoSimulada(unittest.TestCase):
    def simular(self,lado,interromper=None):
        g=GiroFrontal((0,0),(0,lado),(0,lado*.2),margem=.2,timeout=40)
        a=ArbitragemT()
        self.assertTrue(a.adquirir(0))
        x=y=yaw=0.0
        velocity=0.0
        ultimo=ZERO
        for i in range(1000):
            t=i*.05
            if interromper and t>2:
                kind=interromper
            else:
                kind=None
            if kind=='conflito':
                self.assertEqual(a.normal(t,(.1,0,0)),ZERO)
            livre=avaliar_varredura([2.0]*180,-math.pi,math.pi/90,.08,6,(.33,.005,0),.532) is Observacao.LIVRE
            imu=(t,yaw,velocity) if kind!='imu' else None
            cmd=g.passo(t,(x,y,yaw),imu,t if kind!='lidar' else t-1,livre and kind!='obstaculo',
                         abs(velocity)<.03 and ultimo==ZERO,
                         cancelado=kind=='cancelamento',bloqueado=a.falha)
            actual=a.experimental(t,cmd)
            a.vigiar(t)
            # Modelo deliberadamente explicito, nao afirmado como calibracao real.
            wanted=math.copysign(max(.157,abs(actual[2])),actual[2]) if actual[2] else 0
            velocity += max(-.8*.05,min(.8*.05,wanted-velocity))
            yaw += velocity*.05
            x += actual[0]*math.cos(yaw)*.05
            y += actual[0]*math.sin(yaw)*.05
            ultimo=actual
            if g.estado in ('fim','abortar'):
                self.assertEqual(cmd,ZERO)
                return g,a,t
        self.fail('simulacao nao terminou')

    def test_curva_esquerda_com_atuador_minimo_simulado(self):
        self.assertEqual(self.simular(1)[0].estado,'fim')

    def test_curva_direita_com_atuador_minimo_simulado(self):
        self.assertEqual(self.simular(-1)[0].estado,'fim')

    def test_interrupcoes_em_toda_cadeia(self):
        for reason in ('imu','lidar','obstaculo','cancelamento','conflito'):
            with self.subTest(reason=reason):
                g,a,t=self.simular(1,reason)
                self.assertEqual(g.estado,'abortar')
                a.vigiar(t+.3)
                self.assertFalse(a.ativo)
                self.assertEqual(a.normal(t+.4,(.1,0,0)),ZERO)

    def test_fonte_dds_exclusiva_e_comando_nao_finito(self):
        from types import SimpleNamespace as NS
        from giro_frontal import fonte_exclusiva
        endpoint=NS(node_name='alinhador_passagem',endpoint_gid=[1,2,3])
        self.assertTrue(fonte_exclusiva([endpoint],[1,2,3]))
        self.assertFalse(fonte_exclusiva([endpoint,endpoint],[1,2,3]))
        self.assertFalse(fonte_exclusiva([endpoint],[9,9,9]))
        self.assertFalse(fonte_exclusiva([],[]))
        for source in ('normal','experimental'):
            a=ArbitragemT(); a.adquirir(0)
            self.assertEqual(getattr(a,source)(.1,(math.nan,0,0)),ZERO)
            self.assertTrue(a.falha)
            self.assertFalse(a.ativo)

    def test_lease_cobre_centro_e_giro_sem_adquirir_duas_vezes(self):
        a=ArbitragemT()
        self.assertTrue(a.adquirir(0))
        self.assertEqual(a.experimental(.1,(.03,0,0)),(.03,0,0))
        self.assertFalse(a.adquirir(.2))
        self.assertEqual(a.experimental(.2,(0,0,.025)),(0,0,.025))
        self.assertEqual(a.normal(.3,(.1,0,0)),ZERO)
        self.assertTrue(a.falha)


class TestCollisionMonitor(unittest.TestCase):
    def setUp(self):
        self.cm=yaml.safe_load((ROOT/'src/robo_navigation/config/nav2_params.yaml').read_text())['collision_monitor']['ros__parameters']

    def test_default_desabilitado_e_protecoes_anteriores_preservadas(self):
        self.assertFalse(self.cm['SweptTurnStop']['enabled'])
        self.assertIn('VelocityPolygonStop',self.cm['polygons'])
        self.assertEqual(self.cm['cmd_vel_in_topic'],'cmd_vel_smoothed')
        self.assertEqual(self.cm['cmd_vel_out_topic'],'cmd_vel')
        self.assertEqual(self.cm['SweptTurnStop']['min_points'],1)

    def test_todos_os_cantos_varridos_dentro_do_envelope(self):
        p=ast.literal_eval(self.cm['SweptTurnStop']['left_turn']['points'])
        def dentro(x,y):
            return all((p[(i+1)%len(p)][0]-a[0])*(y-a[1])-
                       (p[(i+1)%len(p)][1]-a[1])*(x-a[0])>=-1e-6 for i,a in enumerate(p))
        for j in range(361):
            yaw=math.radians(j)
            for x,y in ((.365,.315),(.365,-.315),(-.205,.315),(-.205,-.315)):
                self.assertTrue(dentro(x*math.cos(yaw)-y*math.sin(yaw), x*math.sin(yaw)+y*math.cos(yaw)))

    def test_validador_de_parametro_ativo_recusa_poligono_reduzido(self):
        from giro_frontal import envolve_disco
        radius=math.hypot(.365,.315)+.05
        pts=ast.literal_eval(self.cm['SweptTurnStop']['left_turn']['points'])
        self.assertTrue(envolve_disco(pts,radius))
        self.assertFalse(envolve_disco([[x*.9,y*.9] for x,y in pts],radius))
        self.assertFalse(envolve_disco(list(reversed(pts)),radius))

    def test_cobertura_rotacao_pura_e_combinada(self):
        cm=self.cm['SweptTurnStop']
        for vx,vy in ((0,0),(.03,0),(.2,.16),(-.16,-.16)):
            for wz,name in ((.025,'left_turn'),(.4,'left_turn'),(-.025,'right_turn'),(-.4,'right_turn')):
                p=cm[name]
                self.assertTrue(p['linear_min']<=math.hypot(vx,vy)<=p['linear_max'])
                self.assertTrue(p['theta_min']<=wz<=p['theta_max'])


class TestGravador(unittest.TestCase):
    def test_empacotamento_preserva_brutos_e_snapshot(self):
        recorder=module('gravador_integrado_test',ROOT/'ferramentas/gravar_teste.py')
        with tempfile.TemporaryDirectory() as tmp:
            # Snapshot minimo neste teste; preservacao real de fontes e testada separadamente.
            def sources(folder):
                (folder/'fontes').mkdir()
                (folder/'fontes/status.txt').write_text('simulado')
            with patch.object(recorder,'snapshot_sources',sources):
                self.assertEqual(recorder.main(['teste','simulado','--offline-test','--base',tmp]),0)
            folders=[p for p in Path(tmp).iterdir() if p.is_dir()]
            self.assertEqual(len(folders),1)
            folder=folders[0]
            self.assertTrue((folder/'bag_simulado/NAO_E_UM_ROSBAG.txt').exists())
            self.assertTrue(json.loads((folder/'manifesto.json').read_text())['simulado'])
            with tarfile.open(folder.with_suffix('.tar.gz')) as tar:
                self.assertIn(folder.name+'/fontes/status.txt',tar.getnames())

    def test_encerramento_e_sigint_preservam_bag_com_processos_simulados(self):
        import os, signal
        recorder=module('gravador_ciclo_test',ROOT/'ferramentas/gravar_teste.py')
        for interrupt in (False, True):
            with self.subTest(interrupt=interrupt), tempfile.TemporaryDirectory() as tmp:
                class Processo:
                    returncode=None
                    def poll(self): return self.returncode
                    def send_signal(self,sig): self.returncode=0
                    def wait(self,timeout=None):
                        if self.returncode is None: self.returncode=0
                        return self.returncode
                def popen(cmd,**kwargs):
                    if cmd[0]=='ros2':
                        folder=Path(cmd[cmd.index('-o')+1]);folder.mkdir()
                        (folder/'dados.mcap').write_bytes(b'DADOS SIMULADOS PRESERVADOS')
                        (folder/'metadata.yaml').write_text(yaml.safe_dump(dict(
                            rosbag2_bagfile_information=dict(topics_with_message_count=[dict(
                                topic_metadata=dict(name='/scan_nav'),message_count=1)]))))
                    return Processo()
                def sources(folder): (folder/'fontes').mkdir()
                def ros(folder,phase,offline):
                    (folder/phase).mkdir()
                    if interrupt and phase=='inicio': os.kill(os.getpid(),signal.SIGINT)
                def capture(args,target,**kwargs): target.write_text('simulado');return 0
                with patch.object(recorder,'snapshot_sources',sources), patch.object(recorder,'snapshot_ros',ros), \
                     patch.object(recorder,'capture',capture), patch.object(recorder.subprocess,'Popen',popen):
                    self.assertEqual(recorder.main(['simulado','processos falsos','--base',tmp,'--duration','.01']),0)
                folder=next(p for p in Path(tmp).iterdir() if p.is_dir())
                self.assertEqual((folder/'bag/dados.mcap').read_bytes(),b'DADOS SIMULADOS PRESERVADOS')
                manifest=json.loads((folder/'manifesto.json').read_text())
                self.assertEqual(manifest['retorno_bag'],0)
                self.assertEqual(manifest['mensagens_por_topico']['/scan_nav'],1)
                self.assertTrue(folder.with_suffix('.tar.gz').exists())

    def test_snapshot_inclui_codigo_nao_commitado(self):
        recorder=module('gravador_snapshot_test',ROOT/'ferramentas/gravar_teste.py')
        with tempfile.TemporaryDirectory() as tmp:
            recorder.snapshot_sources(Path(tmp))
            with tarfile.open(Path(tmp)/'fontes/workspace_fontes.tar.gz') as tar:
                self.assertIn('src/robo_navigation/scripts/giro_frontal.py',tar.getnames())
                self.assertIn('firmware/mega_mecanum_temporal/mega_mecanum_temporal.ino',tar.getnames())
            self.assertTrue((Path(tmp)/'fontes/diff.txt').stat().st_size>0)

    def test_grava_todos_topicos_e_inclui_estagios(self):
        recorder=module('gravador_topicos_test',ROOT/'ferramentas/gravar_teste.py')
        for topic in ('/scan_nav','/scan_raw_temporal','/cmd_vel_ctrl','/cmd_vel_nav_seguro',
                      '/arbitro_t/estado','/alinhador/estado_t','/global_costmap/costmap_updates'):
            self.assertIn(topic,recorder.IMPORTANT)
        source=(ROOT/'ferramentas/gravar_teste.py').read_text()
        self.assertIn("'--all','--include-hidden-topics'",source)


class TestPreflight(unittest.TestCase):
    def test_incompleto_nao_apto(self):
        preflight=module('preflight_test',ROOT/'ferramentas/verificar_giro_t.py')
        self.assertEqual(preflight.evaluate({})['parecer'],'NAO APTO')

    def test_snapshot_simulado_completo_e_mascara(self):
        preflight=module('preflight_full_test',ROOT/'ferramentas/verificar_giro_t.py')
        keys=('modo_experimental','cm_envelope','cm_ativo','smoother_ativo',
              'hardware_timing_valido','limites_temporais_configurados','tf_laser_valida',
              'arbitro_disponivel','saida_zero','arbitro_parado','sem_correcao_pos_cm','limites_de_movimento_configurados','autoridade_no_hardware')
        data=dict(checks=dict.fromkeys(keys,True), ages=dict(scan=.1,imu=.1,odom=.1),gyro_abs=0,linear_abs=0,
                  scan=dict(ranges=[2.0]*180,amin=-math.pi,inc=math.pi/90,rmin=.08,rmax=6),
                  publishers={'/cmd_vel_nav_seguro':['arbitro_t'],'/cmd_vel_nav_t':['alinhador_passagem'],
                              '/cmd_vel_smoothed':['velocity_smoother'],'/cmd_vel':['collision_monitor']})
        self.assertEqual(preflight.evaluate(data)['parecer'],'APTO')
        data['scan']['ranges'][100]=math.inf
        self.assertEqual(preflight.evaluate(data)['parecer'],'NAO APTO')
        data['scan']['ranges'][100]=2.0
        data['ages']['scan']=.58
        self.assertEqual(preflight.evaluate(data)['parecer'],'NAO APTO')
        data['ages']['scan']=.1
        data['publishers']['/cmd_vel'].append('teleop')
        self.assertEqual(preflight.evaluate(data)['parecer'],'NAO APTO')


class TestReversao(unittest.TestCase):
    def test_restaura_sem_perder_arquivos_e_recusa_alteracao_posterior(self):
        import hashlib
        restore=module('restaurador_test',ROOT/'ferramentas/restaurar_arquivos_integrado.py')
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'src').mkdir()
            changed=root/'src/antigo.py';changed.write_text('depois')
            new=root/'src/novo.py';new.write_text('novo')
            wrapper=root/'wrapper.sh';wrapper.write_text('depois')
            original=root/'wrapper_antes.sh';original.write_text('antes')
            original_file=root/'original.tmp';original_file.write_text('antes')
            snapshot=root/'snapshot.tar.gz'
            with tarfile.open(snapshot,'w:gz') as tar:tar.add(original_file,arcname='src/antigo.py')
            def entry(p):return dict(path=str(p.relative_to(root)),sha256=restore.digest(p))
            manifest=dict(modificados_existentes=[entry(changed)],novos_codigo=[entry(new)],
                          wrapper_home_sha256=restore.digest(wrapper))
            changed.write_text('outra sessao')
            with self.assertRaises(RuntimeError):restore.restore(root,manifest,snapshot,wrapper,original)
            self.assertEqual(changed.read_text(),'outra sessao')
            changed.write_text('depois')
            backup=restore.restore(root,manifest,snapshot,wrapper,original)
            self.assertEqual(changed.read_text(),'antes')
            self.assertEqual(wrapper.read_text(),'antes')
            self.assertFalse(new.exists())
            self.assertEqual((backup/'novos/src/novo.py').read_text(),'novo')
            with self.assertRaises(ValueError):restore.safe_path(root,'../../etc/passwd')


if __name__=='__main__':
    unittest.main()
