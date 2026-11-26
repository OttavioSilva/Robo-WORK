"""Cenarios geometricos e regressao sem ROS, motores ou navegacao."""
import ast
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import importlib.util
import subprocess
import tarfile
import hashlib
import json
from types import SimpleNamespace as NS
import threading
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/robo_navigation/scripts'))
from face_ws import (selecionar_face, referencia_ws, RastreioFaceWS, transformar_ponto,
                     pose_mapa_no_scan, obstaculo_antes_da_face)
from alinhador_passagem import Encosto, TravessiaLateral, FOOTPRINT
from saida_curva import SaidaCurva
ZERO=(0.,0.,0.)
FACE=dict(x=.8,y=0.,yaw=0.,largura_medida=.8)

def segmento(dist=.8,centro=0.,a=0.,width=.8,n=41):
    # centro medido na tangente; dist e intersecao com eixo x.
    c,s=math.cos(a),math.sin(a);d=dist*c
    return [(d*c-s*(centro+width*(i/(n-1)-.5)),
             d*s+c*(centro+width*(i/(n-1)-.5))) for i in range(n)]

class TestFaceWS(unittest.TestCase):
    def test_bancada_contra_parede_mais_densa(self):
        pts=segmento()+segmento(1.15,width=1.3,n=100)
        self.assertAlmostEqual(selecionar_face(pts,referencia_ws(FACE,ZERO))['dist'],.8)

    def test_ws1_ws2_com_obstaculos_laterais_e_desalinhamento(self):
        areas=[(dict(x=.859,y=3.862,yaw=-3.096,largura_medida=.87),(1.458,3.889,-3.096)),
               (dict(x=2.6,y=1.98,yaw=-.0508,largura_medida=.87),(2.001,2.01,-.0508))]
        for face,pose in areas:
            for erro in (-.2,0,.2):
                p=(pose[0],pose[1],pose[2]+erro)
                ref=referencia_ws(face,p)
                pts=segmento(ref['dist'],ref['centro'],ref['ang'],.87)+[(.45,.6),(.5,-.62)]
                med=selecionar_face(pts,ref)
                self.assertIsNotNone(med)
                self.assertAlmostEqual(med['ang'],ref['ang'],delta=math.radians(.5))
                self.assertIsNone(obstaculo_antes_da_face(pts,med))

    def test_amostragem_angular_real_2_graus(self):
        pts=[(.8,.8*math.tan(math.radians(i))) for i in range(-26,27,2)]
        med=selecionar_face(pts,referencia_ws(FACE,ZERO))
        self.assertIsNotNone(med)
        self.assertAlmostEqual(med['dist'],.8)

    def test_parede_ou_face_incompativel_recusada(self):
        # FACE_ANG20 (09/11): angulo incompativel era .3 rad (17 graus); limite agora 20
        # JANELA_LARGA (09/11): centro incompativel era .32 m; limite agora .35 (missao30: WS_1 a +0,27..0,30)
        for pts in (segmento(1.1),segmento(.8,.45),segmento(.8,a=.40),segmento(.8,width=1.3)):
            self.assertIsNone(selecionar_face(pts,referencia_ws(FACE,ZERO)))

    def test_face_ang20_missao25_aceita(self):
        # FACE_ANG20: missao25, LiDAR via a WS_2 limpa com 13,8 e 15,8 graus de diferenca do mapa
        for graus in (13.8,15.8,-15.8):
            med=selecionar_face(segmento(.8,a=math.radians(graus)),referencia_ws(FACE,ZERO))
            self.assertIsNotNone(med)

    def test_janela_larga_missao30_ws1_30cm_de_lado_aceita(self):
        # JANELA_LARGA: WS_1 inteira a vista de y -0,15 a +0,74 (ponta alem da janela antiga de 0,70)
        med=selecionar_face(segmento(.8,.295,width=.88),referencia_ws(FACE,ZERO))
        self.assertIsNotNone(med)
        self.assertAlmostEqual(med['centro'],.295,delta=.02)

    def test_duas_faces_ambiguas_recusadas(self):
        self.assertIsNone(selecionar_face(segmento(.75)+segmento(.85),referencia_ws(FACE,ZERO)))

    def test_lacuna_nao_inventa_face_unica(self):
        pts=[p for p in segmento(n=81) if abs(p[1])>.15]
        self.assertIsNone(selecionar_face(pts,referencia_ws(FACE,ZERO)))

    def test_rastreio_duas_observacoes_e_perda(self):
        r=RastreioFaceWS(FACE)
        self.assertIsNone(r.medir(segmento(),0,ZERO,ZERO))
        self.assertIsNotNone(r.medir(segmento(),.17,ZERO,ZERO))
        self.assertIsNone(r.medir([], .34,ZERO,ZERO))
        # FALHA_SCAN (09/11): 1 scan sem face nao destrava; a face volta valida no seguinte
        self.assertIsNotNone(r.medir(segmento(),.51,ZERO,ZERO))
        # falha longa (> 0,5 s): destrava e exige 2 observacoes de novo
        for t in (.68,.85,1.02,1.19):self.assertIsNone(r.medir([],t,ZERO,ZERO))
        self.assertIsNone(r.medir(segmento(),1.36,ZERO,ZERO))
        self.assertIsNotNone(r.medir(segmento(),1.53,ZERO,ZERO))

    def test_salto_de_face_nao_rastreado(self):
        r=RastreioFaceWS(FACE);r.medir(segmento(),0,ZERO,ZERO)
        self.assertIsNone(r.medir(segmento(.9),.17,ZERO,ZERO))
        self.assertIn('salto',r.motivo)

    def test_compensacao_do_movimento(self):
        r=RastreioFaceWS(FACE);pts=segmento()
        r.medir(pts,0,ZERO,ZERO)
        nova=(.04,.02,.04)
        med=r.medir([transformar_ponto(p,ZERO,nova) for p in pts],.17,nova,nova)
        self.assertIsNotNone(med)
        self.assertAlmostEqual(med['ang'],-.04)
        self.assertAlmostEqual(med['centro'],referencia_ws(FACE,nova)['centro'])

    def test_pose_mapa_na_epoca_do_scan(self):
        self.assertEqual(pose_mapa_no_scan((10,20,math.pi/2),(1,0,0),(0,0,0)),(10,19,math.pi/2))

    def test_tf_ausente_tempo_repetido_antigo_invalido(self):
        for t,pose in ((0,ZERO),(.6,ZERO),(float('nan'),ZERO),(.17,None)):
            r=RastreioFaceWS(FACE);r.medir(segmento(),0,ZERO,ZERO)
            self.assertIsNone(r.medir(segmento(),t,ZERO,pose))

    def test_config_invalida_rejeitada_antes_de_mover(self):
        for face in ({},None,dict(FACE,x=float('nan')),dict(FACE,largura_medida=5)):
            with self.assertRaises(ValueError):RastreioFaceWS(face)

    def test_obstaculo_antes_da_face_e_lateral(self):
        med=selecionar_face(segmento(),referencia_ws(FACE,ZERO))
        for x in (.366,.5):
            self.assertIsNotNone(obstaculo_antes_da_face(segmento()+[(x,.1)],med))
        self.assertIsNone(obstaculo_antes_da_face(segmento()+[(.5,.6)],med))

class TestEncostoWS(unittest.TestCase):
    def enc(self,e='avancar',med=None):
        x=Encosto(log=lambda _:None);x.t0=x.t_estado=0;x.estado=e;x.yaw_ref=0
        if med is not None:x.nova_medicao(.17,med,ZERO)
        return x

    def test_referencia_perdida_em_todos_movimentos(self):
        for e in ('avancar','pulso','girar','deslocar','lat_mover','lat_pulso'):
            for m in (None,dict(dist=.8,ang=0,centro=None)):
                enc=self.enc(e,m)
                self.assertEqual(enc.passo(.2,ZERO),ZERO)   # nunca se move sem face
                # PERDA_PARA (09/11): correcoes param e medem de novo (3x); avanco aborta
                self.assertEqual(enc.estado,'abortar' if e in ('avancar','pulso') else 'assentar')
                if e not in ('avancar','pulso'):
                    for _ in range(3):
                        enc.estado=e;enc.passo(.2,ZERO)
                    self.assertEqual(enc.estado,'abortar')

    def test_obstaculo_interposto_para_antes_do_comando(self):
        for e in ('avancar','pulso'):
            enc=self.enc(e,dict(dist=.8,ang=0,centro=0))
            enc.obstaculo=.1
            self.assertEqual(enc.passo(.2,ZERO),ZERO)
            self.assertEqual(enc.estado,'abortar')

    def test_desalinhado_corrige_parado_antes_de_avancar(self):
        for a,centro,estado in ((.12,0,'girar'),(0,.08,'deslocar'),(0,0,'avancar')):
            enc=self.enc('medir')
            for t in (.1,.2,.3,.4):enc.nova_medicao(t,dict(dist=.8,ang=a,centro=centro),ZERO)
            self.assertEqual(enc.passo(.4,ZERO),ZERO)
            self.assertEqual(enc.estado,estado)

    def test_integracao_face_perdida_interrompe_avanco(self):
        r=RastreioFaceWS(FACE);r.medir(segmento(),0,ZERO,ZERO)
        med=r.medir(segmento(),.17,ZERO,ZERO)
        enc=self.enc(med=med)
        self.assertGreater(enc.passo(.2,ZERO)[0],0)
        enc.nova_medicao(.34,r.medir([], .34,ZERO,ZERO),ZERO)
        enc.passo(.35,ZERO)
        self.assertEqual(enc.estado,'avancar')   # FALHA_SCAN: 1 scan ruim tolerado
        enc.nova_medicao(.51,r.medir([], .51,ZERO,ZERO),ZERO)
        self.assertEqual(enc.passo(.65,ZERO),ZERO)   # 0,48 s sem face: para
        self.assertEqual(enc.estado,'abortar')

class TestLateralSeguro(unittest.TestCase):
    def test_folga_negativa_imediata_antes_do_primeiro_passo(self):
        x=TravessiaLateral(1,log=lambda _:None)
        x.nova_folga(0,.1,.1);x.nova_folga(.17,-.01,.1)
        self.assertEqual(x.passo(.17,ZERO,.2),ZERO)
        self.assertEqual(x.estado,'abortar')

    def test_falha_perto_da_saida_nao_e_sucesso(self):
        for motivo in ('timeout','sensor ausente','Collision Monitor'):
            x=TravessiaLateral(1,log=lambda _:None);x.restante=.1
            x.abortar(motivo)
            self.assertEqual(x.estado,'abortar')
            self.assertEqual(x.passo(.2,ZERO,.1),ZERO)

    def test_centro_pelas_duas_folgas_sem_reduzir_margens(self):
        x=TravessiaLateral(1,log=lambda _:None)
        x.nova_folga(0,.14,.06)
        self.assertAlmostEqual(x._centro_folga(.1),.04)
        self.assertIsNone(x._centro_folga(1))
        x.nova_folga(1,.14,.06,True,False)
        self.assertIsNone(x._centro_folga(1))
        self.assertEqual(x.folga_min,.045)

class TestAdaptadorLateral(unittest.TestCase):
    def test_tf_perdida_na_rotina_real_para_sem_comando(self):
        tree=ast.parse((ROOT/'src/robo_navigation/scripts/alinhador_passagem.py').read_text())
        func=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_percorrer_de_lado')
        ns=dict(TravessiaLateral=TravessiaLateral,SaidaCurva=SaidaCurva,FOOTPRINT=FOOTPRINT,time=time,rclpy=NS(ok=lambda:True))
        exec(compile(ast.Module(body=[func],type_ignores=[]),'adaptador_lateral','exec'),ns)
        comandos=[];paradas=[]
        node=NS(fator_corredor=.5,p=lambda key:dict(margem_saida_lateral=.6,angulo_fundido=False,fundido_k_ff=.2,
            abertura_saida_curva=.70,vel_lateral=.08,largura_passagem=.9)[key],lock=threading.Lock(),
            odom=(time.monotonic(),0,0,0),scan=None,scan_seq=0,cmd_final=(time.monotonic(),True),
            _pose_mapa=lambda:None,_pose_mapa_yaw=lambda **_:None,
            get_logger=lambda:NS(info=lambda _:None),_esperar_sensores=lambda _:0,
            _conferir_evidencia_passagem=lambda:None,
            _novo_goal_chegou=lambda:False,_publicar_vel=lambda *c:comandos.append(c),
            _parar=lambda:paradas.append(ZERO))
        self.assertEqual(ns['_percorrer_de_lado'](node,dict(eixo_b=(0,1),saida_b=(0,1),lado=1),'T'),'falha')
        # TF_TOLERA (09/11): espera ate 1 s parada, publicando so velocidade zero
        self.assertTrue(all(c==ZERO for c in comandos));self.assertEqual(paradas,[ZERO])

class TestReversao(unittest.TestCase):
    def test_simulada_preserva_arquivos_e_recusa_edicoes_posteriores(self):
        spec=importlib.util.spec_from_file_location('rollback',ROOT/'ferramentas/reverter_prioridade_lateral_ws.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);old=root/'src/antigo.py';new=root/'src/novo.py'
            old.parent.mkdir();old.write_text('antes')
            snap=root/mod.SNAP;snap.parent.mkdir(parents=True)
            with tarfile.open(snap,'w:gz') as tar:tar.add(old,arcname='src/antigo.py')
            old.write_text('depois');new.write_text('novo')
            entry=lambda p:dict(path=str(p.relative_to(root)),sha256=mod.digest(p))
            manifest=dict(modificados_rodada=[entry(old)],novos_codigo_rodada=[entry(new)],snapshot_sha256=mod.digest(snap))
            mod.reverter(root,manifest)
            self.assertEqual(old.read_text(),'depois')
            new.write_text('outra sessao')
            with self.assertRaises(RuntimeError):mod.reverter(root,manifest,True)
            self.assertEqual(old.read_text(),'depois')
            new.write_text('novo');mod.reverter(root,manifest,True)
            self.assertEqual(old.read_text(),'antes')
            saved=list((root/'backups').glob('reversao_lateral_ws_*/novos/src/novo.py'))
            self.assertEqual(len(saved),1);self.assertEqual(saved[0].read_text(),'novo')

class TestConfiguracao(unittest.TestCase):
    def test_launch_lateral_sem_arbitro_e_com_protecoes(self):
        tree=ast.parse((ROOT/'src/robo_navigation/launch/navigation.launch.py').read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)]
        arg=next(n for n in calls if getattr(n.func,'id','')=='DeclareLaunchArgument' and n.args and isinstance(n.args[0],ast.Constant) and n.args[0].value=='modo_curva_t')
        self.assertEqual({k.arg:ast.literal_eval(k.value) for k in arg.keywords},{'default_value':'lateral','choices':['lateral']})
        source=ast.unparse(tree)
        self.assertNotIn('arbitro_t.py',source)
        self.assertNotIn("('cmd_vel', 'cmd_vel_nav_seguro')",source)
        self.assertIn("('cmd_vel', 'cmd_vel_nav')",source)
        self.assertIn('nav2_collision_monitor',source)
        self.assertIn("['modo_curva_t:=', modo_curva_t]",source)

    def test_bt_valida_caminho_seguido(self):
        root=ET.parse(ROOT/'src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml')
        checked=root.find('.//IsPathValid').attrib['path']
        followed=root.find('.//FollowPath').attrib['path']
        # NAV_INTEGRADA (10/11): valida o TRECHO RESTANTE do caminho seguido (TruncatePathLocal)
        for t in root.iter('TruncatePathLocal'):
            if t.attrib.get('output_path')==checked:checked=t.attrib['input_path']
        # GOAL_TF_ATUAL (11/11): o FollowPath segue a COPIA sem carimbo do mesmo caminho (mesma geometria)
        for t in root.iter('CaminhoSemCarimbo'):
            if t.attrib.get('output_path')==followed:followed=t.attrib['input_path']
        self.assertEqual(checked,followed)
        # Obstaculo so no atalho suavizado: validar o bruto nao provocaria replanejamento.
        paths={'{path}':[(0,0),(1,1),(2,0)],'{smoothed_path}':[(0,0),(1,0),(2,0)]}
        valid=lambda path:all(math.dist(p,(1,0))>.2 for p in path)
        self.assertTrue(valid(paths['{path}']))
        self.assertFalse(valid(paths[checked]))

    def test_patch_idempotente_reversivel_em_copia(self):
        spec=importlib.util.spec_from_file_location('patch_lateral',ROOT/'ferramentas/priorizar_lateral_ws.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        for tipo,path in (('launch','launch/navigation.launch.py'),('bt','behavior_trees/navigate_to_pose_smooth.xml')):
            file=ROOT/'src/robo_navigation'/path
            old=file.with_name(file.name+'.antes_lateral_ws').read_text()
            new=mod.transformar(old,tipo)
            atual=file.read_text()
            # NAV2_PRINCIPAL: comparar patch historico com fixture anterior a refatoracao;
            # no launch atual, conferir tambem que reaplicar e estritamente no-op.
            if tipo == 'launch':
                self.assertEqual(mod.transformar(atual, tipo), atual)
                atual = (ROOT/'src/robo_navigation/test/dados/navigation_lateral_ws_20261109.py').read_text()
            if 'NAV_INTEGRADA' in atual:  # NAV_INTEGRADA (10/11): o estado deste patch ficou preservado
                atual=file.with_name('navigate_to_pose_smooth_anterior_20261110.xml').read_text()
            self.assertEqual(new,atual)
            self.assertEqual(mod.transformar(new,tipo),new)
            with tempfile.TemporaryDirectory() as tmp:
                copy=Path(tmp)/file.name;copy.write_text(old)
                cmd=[sys.executable,str(ROOT/'ferramentas/priorizar_lateral_ws.py'),'--tipo',tipo,'--arquivo',str(copy)]
                for _ in range(2):subprocess.run(cmd,check=True,capture_output=True)
                self.assertEqual(copy.read_text(),new)
                subprocess.run(cmd+['--desfazer'],check=True,capture_output=True)
                self.assertEqual(copy.read_text(),old)

if __name__=='__main__':unittest.main()
