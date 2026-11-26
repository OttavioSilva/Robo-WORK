#!/usr/bin/env python3
"""FACE_LIDAR_M24 (11/11/2026) - baseline funcional. Encosto escolhe a face da mesa so pelo LiDAR, como na missao24.
Evidencia: missao24 (06/11, HEAD 7201367) encostou 7/7 vezes que chegou na aproximacao, medindo a face so pelo LiDAR.
Missoes 25-32 (09/11) com a referencia do mapa (face cadastrada vista pela pose do SLAM): ~1 encosto em ~25 pedidos;
missao32 WS_1: face real a d0.62 c+0.03 a+19g w0.85, recusada 'FORA:ang' dezenas de vezes (ref a-1g).
O que muda: RastreioFaceWS(face, usar_mapa=False) seleciona na fase A a maior face com as 2 pontas cuja largura bate
+-12 cm com a largura cadastrada (filtra paredes), a ate 1,0 m e +-35 graus. Fase B (ja reconhecida) continua igual:
referencia = face anterior projetada pela odometria (TRAVA_ODOM), fragmentos/zona cega iguais.
usar_mapa=True (padrao da classe) = comportamento de antes, sem nenhuma mudanca.
Uso: python3 aplicar_face_lidar_m24.py [--arquivo copia.py] [--desfazer]"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar
ARQ = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/face_ws.py')
TROCAS = [
    ("""def motivo_face(pts, referencia, **kwargs):""",
     """def selecionar_face_lidar(pts, largura, tol_larg=.12, dist_max=1.0, ang_max=math.radians(35), **kwargs):
    \"\"\"FACE_LIDAR_M24 (11/11): fase A sem mapa (como a missao24): maior face com as 2 pontas e largura da WS.\"\"\"
    cands=[m for m in candidatos_face(pts,**kwargs) if m['centro'] is not None
           and abs(m['span']-largura)<=tol_larg and m['dist']<=dist_max and abs(m['ang'])<=ang_max]
    return max(cands,key=lambda m:(m['n'],-m['dist'])) if cands else None


def motivo_face_lidar(pts, largura, **kwargs):
    \"\"\"FACE_LIDAR_M24: por que a fase A sem mapa recusou (so para o log).\"\"\"
    cands=candidatos_face(pts,**kwargs)
    return 'sem mapa: nenhuma face inteira com largura %.2f+-0.12 a <=1,0 m e <=35 graus (%s)' % (
        largura,', '.join('d%.2f c%s a%+.0fg w%.2f' % (m['dist'],'nd' if m['centro'] is None else '%+.2f' % m['centro'],
                                                      math.degrees(m['ang']),m['span']) for m in cands[:4]) or 'nenhum segmento')


def motivo_face(pts, referencia, **kwargs):"""),
    ("""    def __init__(self, face):
        if (not isinstance(face,dict) or""",
     """    def __init__(self, face, usar_mapa=True):
        self.usar_mapa=bool(usar_mapa)   # FACE_LIDAR_M24: False = fase A so pelo LiDAR (missao24)
        if (not isinstance(face,dict) or"""),
    ("""        ref=referencia_ws(self.face,pose_mapa)
        recente=self.ultima is not None and 0<=t-self.ultima[0]<=JANELA_FALHA""",
     """        ref=referencia_ws(self.face,pose_mapa) if self.usar_mapa else None
        recente=self.ultima is not None and 0<=t-self.ultima[0]<=JANELA_FALHA"""),
    ("""        if ref is not None and recente:
            pred=projetar_face(self.ultima[1],self.ultima[2],odom)""",
     """        if recente and (ref is not None or not self.usar_mapa):   # FACE_LIDAR_M24: sem mapa a trava segue pela odom
            pred=projetar_face(self.ultima[1],self.ultima[2],odom)"""),
    ("""        m=selecionar_face(pts,ref,unir_cega=fase_b) if ref is not None else None""",
     """        if ref is not None:
            m=selecionar_face(pts,ref,unir_cega=fase_b)
        elif not self.usar_mapa:   # FACE_LIDAR_M24: fase A sem mapa
            m=selecionar_face_lidar(pts,self.face.get('largura_medida',.8))
        else:
            m=None"""),
    ("""            self.motivo='face ausente, ambigua ou fora da referencia WS: '+motivo_face(
                pts,ref,unir_cega=fase_b)   # MOTIVO_FACE""",
     """            self.motivo='face ausente, ambigua ou fora da referencia WS: '+(motivo_face(
                pts,ref,unir_cega=fase_b) if ref is not None or self.usar_mapa else
                motivo_face_lidar(pts,self.face.get('largura_medida',.8)))   # MOTIVO_FACE / FACE_LIDAR_M24"""),
]
if __name__ == '__main__':
    rodar('face_lidar_m24', 'FACE_LIDAR_M24', ARQ, TROCAS, doc=__doc__)
