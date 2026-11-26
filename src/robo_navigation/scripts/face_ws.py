"""Selecao geometrica de WS e rastreamento de uma unica face; sem ROS."""
import math

TOL_ANG_REF = math.radians(20)   # FACE_ANG20 (09/11): era 12 graus
JANELA_FALHA = .5   # FALHA_SCAN (09/11): era .3 s; a 5,9 Hz tolera 1 scan ruim entre 2 bons
# JANELA_LARGA (09/11, missao30): a WS_1 estava inteira a vista (0,88 m, y -0,15..+0,74) mas a ponta
# passava da janela de 0,70 m e o centro (30 cm de lado) passava dos 18 cm: recusava a mesa certa.
# Agora o robo reconhece a mesa ate ~30 cm de lado e desliza ate o centro. A largura (0,60-0,98 m),
# a distancia (15 cm) e o angulo continuam filtrando paredes.
Y_LIM_FACE = .95       # era .70
TOL_CENTRO_REF = .35   # era .18
# FACE_FRAGMENTADA (09/11, missao32 14:12:39): com a face a menos de ~0,48 m do base_link o meio
# dela fica a menos de 0,146 m do LiDAR (x=0,33) e nao volta nenhum raio: a face chega em 2 pedacos
# (lacuna y -0,08..+0,07) e era recusada por "nenhuma face com as 2 pontas". Depois que a face foi
# reconhecida inteira (fase A), 2 pedacos so viram uma face quando sao o mesmo plano E toda a
# lacuna entre eles esta dentro da zona cega do LiDAR (geometria, nao suposicao).
LIDAR_XY = (0.33, 0.005)   # TF base_link -> laser_raw (tf_static da missao32)
R_CEGO = 0.165             # LiDAR nao mede abaixo de ~0,146 m (missao32: menor retorno 0,148 m)
LACUNA_MAX = 0.30          # lacuna maior que isso nao e zona cega (a face estaria a < 2 cm do LiDAR)
TOL_PEDACOS_ANG = math.radians(8)   # angulo entre os 2 pedacos
TOL_PEDACOS_OFF = .02      # desnivel medio entre os 2 pedacos (a face da WS_1 tem degraus de ~1,7 cm)
# Fase B com uma ponta fora de vista: mede so distancia e angulo (centro None) e no maximo por
# este tempo depois da ultima face com as 2 pontas medidas; nunca reconhece nem centraliza.
JANELA_PARCIAL = 2.0


def ang(x):
    return math.atan2(math.sin(x), math.cos(x))


def transformar_ponto(p, origem, destino):
    """Ponto em origem base -> mundo odom -> destino base."""
    c,s=math.cos(origem[2]),math.sin(origem[2])
    wx,wy=origem[0]+c*p[0]-s*p[1],origem[1]+s*p[0]+c*p[1]
    dx,dy=wx-destino[0],wy-destino[1]
    c,s=math.cos(destino[2]),math.sin(destino[2])
    return c*dx+s*dy,-s*dx+c*dy


def pose_mapa_no_scan(mapa_agora, odom_agora, odom_scan):
    dx,dy=transformar_ponto((0,0),odom_scan,odom_agora)
    c,s=math.cos(mapa_agora[2]),math.sin(mapa_agora[2])
    return (mapa_agora[0]+c*dx-s*dy,mapa_agora[1]+s*dx+c*dy,
            ang(mapa_agora[2]+odom_scan[2]-odom_agora[2]))


def referencia_ws(face, pose):
    if pose is None or not all(math.isfinite(v) for v in pose):
        return None
    if not all(math.isfinite(face[k]) for k in ('x','y','yaw')):
        return None
    x,y=transformar_ponto((face['x'],face['y']),(0,0,0),pose)
    a=ang(face['yaw']-pose[2])
    if abs(a)>math.radians(25):
        return None
    width=face.get('largura_medida',0.8)
    if not math.isfinite(width) or not .6<=width<=1.0:
        return None
    return dict(dist=x+math.tan(a)*y,ang=a,
                centro=-math.sin(a)*x+math.cos(a)*y,span=width)


def _ajustar(points, y_lim, tol):
    if len(points)<6:return None
    for _ in range(2):
        m=len(points)
        if m<6:return None
        my=sum(p[1] for p in points)/m;mx=sum(p[0] for p in points)/m
        sy=sum((p[1]-my)**2 for p in points)
        if sy<1e-4:return None
        b=sum((p[1]-my)*(p[0]-mx) for p in points)/sy
        a=mx-b*my
        points=[p for p in points if abs(p[0]-a-b*p[1])<=tol]
    if len(points)<6:return None
    lo=min(p[1] for p in points);hi=max(p[1] for p in points)
    span=hi-lo
    if span<.25:return None
    yc=(lo+hi)/2;xc=a+b*yc
    centro=(yc+xc*b)/math.sqrt(1+b*b) if .60<=span<=.98 and lo>-y_lim+.05 and hi<y_lim-.05 else None
    return dict(dist=a,ang=-math.atan(b),centro=centro,span=span,n=len(points),
                residual=max(abs(p[0]-a-b*p[1]) for p in points),lo=lo,hi=hi)


def _reta(points):
    """Minimos quadrados x = a + b*y; None se os pontos nao tem extensao lateral."""
    m=len(points)
    if m<2:return None
    my=sum(p[1] for p in points)/m;mx=sum(p[0] for p in points)/m
    sy=sum((p[1]-my)**2 for p in points)
    if sy<1e-4:return None
    b=sum((p[1]-my)*(p[0]-mx) for p in points)/sy
    return mx-b*my,b


def lacuna_cega(seg_a, seg_b, todos, lidar=LIDAR_XY, r_cego=R_CEGO):
    """FACE_FRAGMENTADA: (y1, y2) se os 2 pedacos sao um plano so partido pela zona cega; senao None.

    Exige: lacuna entre 10 cm e LACUNA_MAX; os 2 pedacos com o mesmo angulo e sem desnivel no meio
    da lacuna; 90% dos pontos a 2,5 cm da reta unica; TODA a lacuna (menos 2 cm em cada borda) mais
    perto do LiDAR que o alcance minimo; nenhum retorno dentro da lacuna na frente do plano.
    """
    if len(seg_a)<6 or len(seg_b)<6:return None
    if max(p[1] for p in seg_a)>max(p[1] for p in seg_b):seg_a,seg_b=seg_b,seg_a
    y1=max(p[1] for p in seg_a);y2=min(p[1] for p in seg_b)
    if not .10<y2-y1<=LACUNA_MAX:return None
    ra,rb,ru=_reta(seg_a),_reta(seg_b),_reta(seg_a+seg_b)
    if ra is None or rb is None or ru is None:return None
    if (max(p[1] for p in seg_a)-min(p[1] for p in seg_a)<.10 or
            max(p[1] for p in seg_b)-min(p[1] for p in seg_b)<.10):return None
    # os 2 pedacos e a reta unica com o mesmo angulo: 2 objetos paralelos em profundidades
    # diferentes dao uma reta unica inclinada (os residuos medios se anulam)
    angs=[math.atan(r[1]) for r in (ra,rb,ru)]
    if max(angs)-min(angs)>TOL_PEDACOS_ANG:return None
    a,b=ru;uni=seg_a+seg_b
    # desnivel: residuo medio de cada pedaco em relacao a reta unica (extrapolar a reta de um
    # pedaco curto amplifica os degraus de ~1,7 cm da face; o residuo medio nao)
    res=lambda seg:sum(p[0]-a-b*p[1] for p in seg)/len(seg)
    if abs(res(seg_a)-res(seg_b))>TOL_PEDACOS_OFF:return None
    if sum(abs(p[0]-a-b*p[1])<=.025 for p in uni)<.9*len(uni):return None
    k=0
    while y1+.02+.01*k<=y2-.02:
        y=y1+.02+.01*k;k+=1
        if math.hypot(a+b*y-lidar[0],y-lidar[1])>=r_cego:return None
    for x,y in todos:
        if y1+.01<y<y2-.01 and x<a+b*y+.03:return None
    return y1,y2


def candidatos_face(pts,x_min=.30,x_max=1.30,y_lim=None,tol=.02,unir_cega=False):
    """Ate quatro planos; separa segmentos colineares por lacunas >10 cm.

    unir_cega (so na fase B, face ja reconhecida): junta 2 pedacos partidos pela zona cega.
    """
    if y_lim is None:y_lim=Y_LIM_FACE
    remaining=[p for p in pts if len(p)==2 and all(math.isfinite(v) for v in p)
               and x_min<=p[0]<=x_max and abs(p[1])<=y_lim]
    todos=list(remaining)
    pieces=[]
    for _ in range(4):
        if len(remaining)<6:break
        best=None
        for k in range(-50,51):
            th=math.radians(k*.5);c,s=math.cos(th),math.sin(th)
            offsets=sorted((x*c-y*s,(x,y)) for x,y in remaining)
            j=0
            for i in range(len(offsets)):
                while offsets[i][0]-offsets[j][0]>.03:j+=1
                n=i-j+1
                if best is None or n>best[0] or (n==best[0] and offsets[j][0]<best[2]):
                    best=(n,th,offsets[j][0],(offsets[i][0]+offsets[j][0])/2)
        if best[0]<6:break
        c,s=math.cos(best[1]),math.sin(best[1])
        line=[p for p in remaining if abs(p[0]*c-p[1]*s-best[3])<=tol]
        if len(line)<6:break
        taken=set(line);remaining=[p for p in remaining if p not in taken]
        line.sort(key=lambda p:p[1])
        segments=[[]]
        for p in line:
            if segments[-1] and math.dist(p,segments[-1][-1])>.10:segments.append([])
            segments[-1].append(p)
        pieces.extend((seg,None) for seg in segments)
    if unir_cega:
        unido=True
        while unido:
            unido=False
            for i in range(len(pieces)):
                for j in range(i+1,len(pieces)):
                    lac=lacuna_cega(pieces[i][0],pieces[j][0],todos)
                    if lac is not None:
                        seg=sorted(pieces[i][0]+pieces[j][0],key=lambda p:p[1])
                        pieces=[q for k,q in enumerate(pieces) if k not in (i,j)]+[(seg,lac)]
                        unido=True;break
                if unido:break
    result=[]
    for seg,lac in pieces:
        fit=_ajustar(seg,y_lim,tol)
        if fit is not None:
            if lac is not None:fit.update(fragmentada=True,lacuna=lac)
            result.append(fit)
    return result


def selecionar_face(pts, referencia=None, **kwargs):
    candidates=candidatos_face(pts,**kwargs)
    if referencia is None:
        return max(candidates,key=lambda m:(m['n'],-m['dist'])) if candidates else None
    ranked=[]
    for m in candidates:
        if m['centro'] is None:continue
        dd=abs(m['dist']-referencia['dist']); da=abs(ang(m['ang']-referencia['ang']))
        dc=abs(m['centro']-referencia['centro']);dw=abs(m['span']-referencia['span'])
        # FACE_ANG20 (09/11): era 12 graus. Missao25: LiDAR via a WS_2 limpa, mas o
        # yaw do mapa diferia 8-16 graus da face real (cadastro + deriva de lado).
        if dd>.15 or da>TOL_ANG_REF or dc>TOL_CENTRO_REF or dw>.12:continue
        ranked.append((dd/.15+da/TOL_ANG_REF+dc/TOL_CENTRO_REF+dw/.12,m))
    ranked.sort(key=lambda v:v[0])
    # Alternativas geometricamente equivalentes nao autorizam escolher por densidade.
    if len(ranked)>1 and ranked[1][0]-ranked[0][0]<.35:return None
    return ranked[0][1] if ranked else None


def selecionar_face_lidar(pts, largura, tol_larg=.12, dist_max=1.0, ang_max=math.radians(35), **kwargs):
    """FACE_LIDAR_M24 (11/11): fase A sem mapa (como a missao24): maior face com as 2 pontas e largura da WS."""
    cands=[m for m in candidatos_face(pts,**kwargs) if m['centro'] is not None
           and abs(m['span']-largura)<=tol_larg and m['dist']<=dist_max and abs(m['ang'])<=ang_max]
    return max(cands,key=lambda m:(m['n'],-m['dist'])) if cands else None


def motivo_face_lidar(pts, largura, **kwargs):
    """FACE_LIDAR_M24: por que a fase A sem mapa recusou (so para o log)."""
    cands=candidatos_face(pts,**kwargs)
    return 'sem mapa: nenhuma face inteira com largura %.2f+-0.12 a <=1,0 m e <=35 graus (%s)' % (
        largura,', '.join('d%.2f c%s a%+.0fg w%.2f' % (m['dist'],'nd' if m['centro'] is None else '%+.2f' % m['centro'],
                                                      math.degrees(m['ang']),m['span']) for m in cands[:4]) or 'nenhum segmento')


def motivo_face(pts, referencia, **kwargs):
    """MOTIVO_FACE (09/11): por que selecionar_face recusou, com os numeros (so para o log)."""
    if referencia is None:
        return 'sem referencia (pose no mapa/TF)'
    cands=candidatos_face(pts,**kwargs)
    inteiras=[m for m in cands if m['centro'] is not None]
    if not inteiras:
        return 'nenhuma face com as 2 pontas (%d segmentos: %s)' % (len(cands), ', '.join(
            'd%.2f w%.2f' % (m['dist'],m['span']) for m in cands[:4]) or 'nenhum')
    partes=[];ok=[]
    for m in inteiras:
        dd=abs(m['dist']-referencia['dist']); da=abs(ang(m['ang']-referencia['ang']))
        dc=abs(m['centro']-referencia['centro']);dw=abs(m['span']-referencia['span'])
        estoura=[n for n,v,l in (('dist',dd,.15),('ang',da,TOL_ANG_REF),('centro',dc,TOL_CENTRO_REF),('larg',dw,.12)) if v>l]
        partes.append('d%.2f c%+.2f a%+.0fg w%.2f%s' % (m['dist'],m['centro'],math.degrees(m['ang']),m['span'],
                      (' FORA:'+'/'.join(estoura)) if estoura else ''))
        if not estoura:ok.append(m)
    pre='ambigua (%d faces validas)' % len(ok) if len(ok)>1 else 'fora da referencia'
    return '%s | ref d%.2f c%+.2f a%+.0fg | %s' % (pre,referencia['dist'],referencia['centro'],
                                                 math.degrees(referencia['ang']),'; '.join(partes[:3]))


def projetar_face(m,old,new):
    # Ponto medido no centro da face e direcao normal, ambos compensados pelo SE(2).
    a=m['ang'];c,s=math.cos(a),math.sin(a)
    d=m['dist']*c;t=m['centro']
    p=transformar_ponto((d*c-t*s,d*s+t*c),old,new)
    a=ang(a+old[2]-new[2]);c,s=math.cos(a),math.sin(a)
    if c<.5:return None
    return dict(dist=(p[0]*c+p[1]*s)/c,ang=a,centro=-p[0]*s+p[1]*c)


def acompanhar_face_parcial(pts, referencia):
    """Fase B, uma ponta fora de vista: mede AGORA so o plano (distancia e angulo).

    FACE_FRAGMENTADA (09/11): antes o centro vinha da face antiga projetada pela odometria; na
    missao32 a odom disse 2,8 cm de lado e o LiDAR mostrou o robo parado de lado (andou 3,5 cm
    para a frente). Centro projetado nao e medicao: aqui ele volta None e nao autoriza corrigir.
    Nunca reconhece uma WS: exige retornos atuais no plano previsto, dentro da extensao da face.
    """
    if not .365 <= referencia['dist'] <= .60:
        return None
    b = -math.tan(referencia['ang'])
    points = [p for p in pts if len(p) == 2 and all(math.isfinite(v) for v in p)
              and .30 <= p[0] <= .70 and abs(p[1]) <= Y_LIM_FACE
              and abs(p[0] - referencia['dist'] - b*p[1]) <= .04]
    if len(points) < 12:
        return None
    m = _ajustar(points, Y_LIM_FACE, .025)
    if (m is None or m['n'] < 12 or
            abs(m['dist'] - referencia['dist']) > .05 or
            abs(ang(m['ang'] - referencia['ang'])) > math.radians(8)):
        return None
    # o pedaco visto tem de caber na face prevista (8 cm de folga: a odom erra de lado)
    a = m['ang']; c, s = math.cos(a), math.sin(a)
    tang = [-s*p[0] + c*p[1] for p in points if abs(p[0] - m['dist'] + math.tan(a)*p[1]) <= .025]
    meia = referencia['span'] / 2 + .08
    if not tang or min(tang) < referencia['centro'] - meia or max(tang) > referencia['centro'] + meia:
        return None
    m.update(centro=None, span=referencia['span'], parcial=True)
    return m


# FACE_FRAGMENTADA: face partida so da centro se a largura bate +-4 cm com a medida INTEIRA na fase A
# (missao32: a ponta direita da WS_1 pula 0,41..0,54 m perto da mesa; com a largura certa o centro e medido)
TOL_LARG_FRAG = .04


class RastreioFaceWS:
    """Fase A: reconhece a WS inteira (2 pontas) 2 vezes coerentes. Fase B: acompanha a mesma
    face (inteira, partida pela zona cega ou so o plano) com medicoes atuais; perdeu -> fase A."""
    def __init__(self, face, usar_mapa=True):
        self.usar_mapa=bool(usar_mapa)   # FACE_LIDAR_M24: False = fase A so pelo LiDAR (missao24)
        if (not isinstance(face,dict) or
                any(not isinstance(face.get(k), (int,float)) or not math.isfinite(face[k])
                    for k in ('x','y','yaw')) or
                not isinstance(face.get('largura_medida',.8),(int,float)) or
                not .6<=face.get('largura_medida',.8)<=1.0):
            raise ValueError('referencia WS invalida')
        self.face=dict(face);self.ultima=None;self.motivo=''
        self.reconhecida = False
        self.t_completa = None   # ultima medicao com as 2 pontas (centro medido)
        self.larg_ref = None     # largura da ultima face inteira (sem lacuna) acompanhada

    def _perder(self):
        self.ultima=None;self.reconhecida=False;self.t_completa=None;self.larg_ref=None

    def medir(self,pts,t,odom,pose_mapa):
        if (not math.isfinite(t) or len(odom)!=3 or
                not all(math.isfinite(v) for v in odom)):
            self._perder();self.motivo='tempo ou odometria invalida';return None
        ref=referencia_ws(self.face,pose_mapa) if self.usar_mapa else None
        recente=self.ultima is not None and 0<=t-self.ultima[0]<=JANELA_FALHA
        if self.ultima is not None and not recente:
            self._perder()   # trava velha nao vale: volta a fase A (exige a face inteira 2 vezes)
        # TRAVA_ODOM (09/11, missao27): o mapa so escolhe a face na 1a vez. Travado nela, a
        # referencia e a face anterior projetada pela odometria: um salto do SLAM (+8,6 graus e
        # 30 cm em map->odom, robo parado em frente a WS_2) nao derruba mais o encosto.
        if recente and (ref is not None or not self.usar_mapa):   # FACE_LIDAR_M24: sem mapa a trava segue pela odom
            pred=projetar_face(self.ultima[1],self.ultima[2],odom)
            if pred is not None:
                ref=dict(pred,span=self.ultima[1]['span'])
        fase_b=self.reconhecida and recente and ref is not None
        if ref is not None:
            m=selecionar_face(pts,ref,unir_cega=fase_b)
        elif not self.usar_mapa:   # FACE_LIDAR_M24: fase A sem mapa
            m=selecionar_face_lidar(pts,self.face.get('largura_medida',.8))
        else:
            m=None
        if m is not None and m.get('fragmentada') and (
                self.larg_ref is None or abs(m['span']-self.larg_ref)>TOL_LARG_FRAG):
            m=dict(m,centro=None,parcial=True)   # uma ponta duvidosa: so distancia e angulo
        if (m is None and fase_b and self.t_completa is not None
                and 0<=t-self.t_completa<=JANELA_PARCIAL):
            m=acompanhar_face_parcial(pts,ref)
        if m is None:
            # FALHA_SCAN (09/11, missao29): 31-38% dos raios voltam inf e um scan isolado parte a
            # face em pedacos. Nao esquece a face travada por um scan so: o proximo segue pela odometria.
            self.motivo='face ausente, ambigua ou fora da referencia WS: '+(motivo_face(
                pts,ref,unir_cega=fase_b) if ref is not None or self.usar_mapa else
                motivo_face_lidar(pts,self.face.get('largura_medida',.8)))   # MOTIVO_FACE / FACE_LIDAR_M24
            return None
        old=self.ultima
        predicted=projetar_face(old[1],old[2],odom) if old is not None else None
        # sem centro medido, a referencia interna guarda o previsto (so para projetar o proximo)
        interno=m if m['centro'] is not None or predicted is None else dict(m,centro=predicted['centro'])
        if self.larg_ref is not None and (m['centro'] is None or m.get('fragmentada')):
            interno=dict(interno,span=self.larg_ref)   # pedaco nao redefine a largura da WS
        self.ultima=(t,interno,odom)
        if old is None:
            self.reconhecida = False
            self.motivo='aguardando segunda observacao coerente';return None
        if not .03<=t-old[0]<=JANELA_FALHA:
            self.motivo='observacao repetida/antiga';return None
        if (predicted is None or abs(predicted['dist']-m['dist'])>.06
            or (m['centro'] is not None and abs(predicted['centro']-m['centro'])>.06)
            or abs(ang(predicted['ang']-m['ang']))>math.radians(5)):
            self._perder();self.motivo='salto de referencia geometrica';return None
        if m['centro'] is not None:
            self.t_completa=t
            if not m.get('fragmentada'):
                self.larg_ref=m['span']
        self.reconhecida = True
        self.motivo=('face WS parcial: so distancia/angulo' if m['centro'] is None else
                     'face WS partida pela zona cega (medida)' if m.get('fragmentada') else
                     'face WS consistente')
        return m


def obstaculo_antes_da_face(pts,med,para_choque=.365):
    """Um retorno entre para-choque e WS no corredor frontal basta para parar."""
    if med is None:return None
    b=-math.tan(med['ang'])
    gaps=[x-para_choque for x,y in pts if math.isfinite(x) and math.isfinite(y)
          and abs(y)<=.335 and x>para_choque and x<med['dist']+b*y-.05]
    return min(gaps) if gaps else None
