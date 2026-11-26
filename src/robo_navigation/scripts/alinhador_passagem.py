#!/usr/bin/env python3
"""Alinhador de passagens estreitas para o Nav2 (v3.32: travessia pelo LiDAR + ir ate Service Area/START/FINISH, encostar e publicar o resultado).

v3.32 (missao20 05/11) GIRO_DECIDIDO + GIRO_FINAL:
  - "se decidi girar, eu giro": o VIRAR_CAMINHO gira sempre pelo _girar_seguro, inclusive com o
    giro livre (na missao20 12:16:03 o LiDAR dizia livre, o alinhador deixou com o Nav2, o shim nao
    girou e o CM travou 6 s);
  - giro RAPIDO (0,30 rad/s) quando o retangulo maior do CM (7 cm, giro acima de 0,20 rad/s) esta
    livre em todo o arco; senao 0,15 como antes. O rapido para 14 graus antes e o lento acerta;
  - GIRO_FINAL nas mesas: o Nav2 leva so ate a POSICAO de aproximacao, chegando com o rumo do
    caminho (sem giro final do Nav2), e o giro para o angulo da mesa e do alinhador (olha o
    LiDAR, afasta o minimo se a quina pega) + ultimos cm de lado/frente. Na missao20 o giro final
    do Nav2 travou no CM nas 2 rodadas (quina a 1-7 cm da mesa).

v3.31 (missao19 05/11) ANTI_CISCADO: durante um goal do Nav2, se em 4 s o comando do Nav2
(/cmd_vel_nav) inverte de sentido 4 vezes ou mais (giro ou lado) e o robo nao sai de 5 cm nem gira
15 graus, e ciscado: cancela o goal, chama o destravar (o mesmo da arvore) e manda o goal de novo
(que antes passa pelo VIRAR_CAMINHO). No maximo 3 vezes por goal; depois deixa com o Nav2.
Calibrado nos bags: missao19 dispara +4 s depois do comeco nos 3 trechos ruins (o Nav2 so desistiu
com 64 s, 11 s e 29 s) e nunca nos trechos bons; missao18 so dispara em goals que acabaram em
"Failed to make progress".

v3.30 (missao19 05/11) VIRAR_CAMINHO: antes de entregar um goal ao Nav2, o robo vira para o rumo do
caminho (medido 0,5 m a frente) se ele sair a mais de virar_caminho_graus (45) da frente. Se o giro esta
livre pelo LiDAR, deixa o RotationShim girar (0,40 rad/s, mais rapido); se a quina pegaria, usa o
_girar_seguro (anda o minimo de lado/frente/tras que libera o giro e gira).
Na missao19 o Nav2 comecou 3 vezes com o caminho atras (+134, +106, -139 graus) logo depois de
sair da mesa, com uma peca a 0,45-0,50 m da quina (o giro no lugar precisa de 0,50 m): o
RotationShim nao gira (preve colisao), o DWB nao da re e fica mandando giros de +-0,047 rad/s que
o motor executa a 0,15-0,19 (giro minimo do PWM 50): passa do ponto, volta, passa... 64 s, 72 s e
29 s ciscando. O escolher_manobra, nesses 3 momentos, pedia 4-10 cm de lado e depois o giro.
Tambem: o _girar_seguro, com menos de 10 graus para girar, girava sem olhar o LiDAR; se o
Collision Monitor travava, tentava o mesmo giro de novo (5 vezes: FINISH falhou 2 vezes e a prova
parou). Agora, depois de um travamento, sempre consulta o escolher_manobra (pedia 4 cm para a esq).

v3.29 (missao16 05/11): o DESVIO da v3.27 vem DESLIGADO (parametro desvio_lidar, padrao False). Na
missao16 ele disparou 3 vezes saindo da WS_2 (a "peca" era a mesa + a peca, de -55 a +39 cm) e
cada disparo cancelava o goal do Nav2 depois de 6 s: a arvore do Nav2 recomecava do zero e as
recuperacoes dela nunca chegavam a rodar. Destravar agora e trabalho da arvore do Nav2
(recuperacao "destravar": anda para a direcao com mais espaco).

v3.28 (missao15 05/11): a travessia nao se joga mais na parede no fim do corredor. Na volta, ~0,8 m
antes do cruzamento, as paredes comecam a acabar e a medida passou a ver o vao do cruzamento
como parede (largura 1,04-1,15 m, ou uma parede so com o centro "16-33 cm a esquerda"). A
travessia acreditou, parou o avanco para corrigir e deslizou para a esquerda por ~15 s com a
lateral a 0-1 cm da parede, girando 20 graus, ate o Collision Monitor travar. Agora:
  - seguranca pelas folgas REAIS da lateral do corpo (LiDAR): a menos de 4 cm de um lado afasta
    para o outro; nunca desliza para um lado com menos de 6 cm; e quando a medida pede para ir
    contra a parede, ela e ignorada (o robo segue em frente em vez de parar para "corrigir");
  - medida com o centro mais longe do que o robo cabe no corredor (meia largura conhecida - meio
    robo + 5 cm) e descartada.
  Simulador PWM60 (teste_no_curva3, reto / chegando +10 / -10 graus): 17/14/11 -> 18/18/16 de 18.
  Testado e descartado: no ultimo 1 m nao seguir a medida e nao parar para corrigir (10/1/8).

v3.27 (missao14 05/11): DESVIO de obstaculo pelo LiDAR, sem depender do mapa. Indo para uma mesa
com uma peca no caminho, o robo ficou ~1 min parado na frente dela: o planner passava pelo vao
de 0,53 m entre a peca e a parede (o robo tem 0,63 m), o DWB nao achava trajetoria, as
recuperacoes do Nav2 (escape de 3 cm, re de 10 cm, giro) nao mudavam nada e o Nav2 replanejava
o mesmo caminho. Agora, no goal final pelo Nav2: se o robo fica 6 s sem sair do lugar (menos de
5 cm e 15 graus) longe do destino e o LiDAR ve algo na frente do para-choque, o alinhador cancela
o Nav2 e faz: re (se a peca estiver a menos de 15 cm) -> mede a peca pelo LiDAR -> desliza de lado
para o lado que libera a frente com menos deslocamento (e que tem espaco) -> avanca ate o
para-choque passar a peca -> limpa os costmaps -> manda o goal de novo. Ate 3 desvios por goal.

v3.26 (missao13 04/11, ponte PWM60): a ponte agora obedece o comando (real de frente ~0,078 +
0,75 x comando; de lado ~0,6 x isso). Antes qualquer comando dava ~0,2 m/s e o alinhador
contava com isso: na missao13 as saidas laterais levaram 77-85 s (antes 23-52 s), com o mesmo
"corrigindo parado, centro +5,5 cm" repetido: o pulso de ajuste supunha 0,20 m/s e andava
menos da metade.
  - saida lateral: velocidade real do ajuste pela regra do motor (~0,09 m/s) e pulso ate ~6 cm
    (como antes); vel_lateral 0,025 -> 0,08 (real ~0,083 m/s de lado; com 0,025 era ~0,058).
    Simulador com a fisica PWM60 (cruz/sim_lateral60.py): 71/120 -> 118/120.
  - travessia: giro minimo andando 0,016 -> 0,022 rad/s: a ponte ignora giro < 0,02 com
    movimento (heading hold), entao a correcao de rumo nao acontecia, o erro passava de 4 graus
    e o robo parava e girava parado (giro puro = PWM de rotacao 80, ~15 graus/s): era a
    guinada de ~5 graus logo depois de entrar no corredor (missao13 12:47:35).

v3.25 (missao11 04/11): na saida lateral, parede do corredor novo torta ate 16 graus em
relacao ao robo vale como medida (se tiver >= 15 pontos); antes o limite era 8. O corredor
oeste da arena estava ~10 graus torto em relacao ao norte: o robo, alinhado com o corredor
da travessia, jogava fora todas as medidas ("sem parede do corredor novo"), deslizava torto e
a traseira chegava a 0-1 cm da parede (6 paradas de seguranca na volta). A parada de seguranca
afasta pelo menos 3 cm (com 1-2 cm nao sobrava espaco para o giro de alinhamento).

v3.24 (missao10 04/11): o ajuste fino do preparo confere a posicao pelo mapa e repete (ate 3
passes): na volta o SLAM corrigiu o map->odom em 0,95 m e 11 graus durante o ajuste, o robo
entrou 20 cm fora do preparo e inclinado, e o Collision Monitor travou a travessia. Trajeto
curto ate mesa aceita 10 cm (o encosto acerta o centro).

v3.23 (missao9 04/11): destino a ate 1 m (curto_max) em linha reta e livre vai sem Nav2: gira
so se for mesa (giro seguro) e anda de lado e de frente/re ate o ponto. O RotationShim sempre
virava o robo para o rumo do caminho: com o destino atras, girava, andava e girava de novo.
(Testado e descartado: procurar passagens tambem no costmap global; nos replays de missao7-9
os obstaculos/ruido do LiDAR criaram passagens falsas em area aberta.)

v3.22 (missao8 03/11): no cruzamento com curva, o robo para pela largura do corredor NOVO
(onde o corpo fica de comprido) e nao pela do corredor da travessia. Na volta da missao8 a
travessia tinha 0,84 m e o corredor novo ~0,70 m: o alvo saiu 13 cm (certo: ~6 cm), o robo
parou 8 cm fora do centro, a traseira ficou a 1 cm da quina e a saida lateral abortou.

v3.21 (missao6 03/11): na saida lateral, a parada de seguranca sem parede do corredor novo a
vista voltava a deslizar com o para-choque a 0 cm (30 s empurrando a parede da frente, porque o
robo andava na diagonal ao deslizar para a esquerda). Agora afasta antes (conta nas 6 paradas
de seguranca e aborta depois delas).

v3.20 (missao5 29/10): (1) chegada ao PREPARO sem o vai-e-volta: o Nav2 so leva o robo ate a
posicao (chega com o rumo do caminho, aceita a 10 cm) e o giro e feito pelo alinhador: para,
olha o LiDAR e, se a quina pegaria girando (retangulo do Collision Monitor), anda o minimo que
libera o giro -- para tras, para frente OU DE LADO -- e so entao gira, devagar. Os ultimos cm
ate o preparo vao de lado/frente, sem girar. Antes o Nav2 girava com a quina na parede, o
Collision Monitor travava, a recuperacao dava re e o DWB voltava ao mesmo canto (1 min 15 s e
"PREPARO refinado FALHOU"). (2) Largura do corredor medida tambem em perpendiculares giradas
+-15/30 graus: o caminho que entrava no corredor na diagonal nao achava a parede de um lado e
a passagem saia com eixo -122 graus em vez de -94 (2 travessias falhas: "nao vejo as paredes").

v3.19 (missao4 29/10): na volta o cruzamento em L nao foi reconhecido depois do "preparo
refinado": o caminho cortava o canto na diagonal e virava um trecho curto e torto (-134 graus)
entre os dois corredores (efeito colateral da v3.16). Sem cruzamento, a travessia seguia reto
ate a parede do fim do L e o Collision Monitor travava (3 tentativas, 2 min). Agora esse trecho
curto e diagonal e pulado e o cruzamento e achado no corredor seguinte (todos os caminhos
gravados de 27 a 29/10 saem com "curva: sim").

v3.18 (missao2 29/10): na saida lateral da volta a traseira chegou a 1 cm da parede (o robo
deriva para tras ao deslizar para a direita; a folga caiu 6,1 -> 3,7 -> 1,0 cm em 0,4 s).
Parada de seguranca a 4,5 cm (era 3) e valendo ja no scan que cruza o limite quando a queda e
gradual (antes esperava a maior das 2 ultimas leituras, um scan a mais). Se algo abortar a
saida com o corpo ja todo fora do corredor, conta como saida concluida.

v3.17 (missao1 29/10): encosto mais rapido e centrado: desvio lateral >= 4 cm vai de uma vez,
parando pelo LiDAR (antes: pulsos de 0,15-0,3 s, 15 cm em 40 s e encosto abortado); depois
de encostar, se o centro ficou a mais de 3 cm, corrige de lado (WS_2 ficou a +16,5 cm). Saida
lateral termina 0,60 m alem do corredor (era 0,40: o giro seguinte travava no Collision Monitor).

v3.16 (teste curva5 28/10): (1) o caminho que corta o canto de um L na diagonal virava uma
passagem so, com eixo de -147 graus, e o preparo ia para 1 m fora, na area aberta: agora o
trecho estreito termina onde o caminho vira mais de 45 graus. (2) Na saida lateral, a quina
do cruzamento vista de lado (3 pontos) contava como parede de tras e o centro brigava com a
medida pela parede da frente (10 s de vai e vem): o centro pelas folgas so vale com parede
de verdade dos dois lados; a parada de seguranca usa o ponto mais perto (quina inclusive).

v3.15 (teste curva4 28/10): saida lateral centrada pela folga real do para-choque e da
traseira (sem supor a largura do corredor), parada de seguranca a 3 cm que vale sempre
(na volta o robo deslizou com a traseira a 0-1 cm da parede porque as correcoes tinham
acabado), contador de correcoes zera a cada 6 cm de progresso (na ida abortou a 17 cm do
fim, centrado) e, sem correcoes, segue deslizando se as folgas estiverem boas.

v3.14 (teste curva3 12:06): saida lateral sem vai-e-vem (pulsos com 0,2 m/s reais, tolerancia
de 4-6 cm porque as paredes reais nao sao retas), medida so com a parede ao lado do corpo e,
nos ultimos 0,5 m, so termina de sair (perto da saida a medida pegava outras paredes).

v3.13 (teste curva2): saida lateral com ajuste para frente/tras em pulsos curtos medidos pelo
LiDAR (o robo anda ~0,1 m/s com qualquer comando), parede de tras so perto de onde deve estar,
afasta da parede antes de girar, e pose de preparo procurada mais longe/sem margem (na volta
o mapa nao achou preparo e o Nav2 tentou fazer a curva sozinho).

v3.12: no cruzamento com curva o robo para pelo LiDAR (parede a frente ou do lado de la do
corredor novo), e nao pelo mapa: no teste curva1 o mapa errou ~15 cm no eixo do corredor e o
robo encostou na parede da frente.

v3.11: cruzamentos de corredores. Reto (+ ou T atravessado): uma travessia so, do comeco
do primeiro corredor ao fim do ultimo. Com curva (L, T ou +): o robo entra de frente, para
no cruzamento com o corpo centrado no corredor novo e sai DE LADO (mecanum), sem girar,
guiado pela parede da frente (e de tras, se o LiDAR ver).

v3.10: em pontos sem mesa (START/FINISH) o angulo final nao importa: o goal vai com o
angulo de quem chega pelo caminho e o alinhador aceita a chegada quando o robo fica
1,5 s a menos de 10 cm do ponto (cancela o Nav2 em vez de esperar o giro final).

Recebe um goal (topico /goal_pose_alinhado, ferramenta "2D Goal Pose" do RViz
com o topico trocado), pede um caminho ao planner, procura no mapa trechos
estreitos ao longo desse caminho e, antes de cada um, manda o robo (pelo Nav2)
a uma pose de preparo no eixo da passagem.

v3: a TRAVESSIA nao usa mais o DWB. Da pose de preparo o robo avanca devagar ate
o LiDAR enxergar as duas paredes, para na boca, mede o angulo e o centro do
corredor pelas paredes reais (nao pelo mapa), gira/desliza para alinhar e
atravessa reto corrigindo pelo LiDAR. Os comandos vao para /cmd_vel_nav, ou
seja, continuam passando pelo velocity_smoother e pelo Collision Monitor.
Se algo der errado, volta ao comportamento antigo (TRAVESSIA pelo Nav2).

Uso:
    python3 alinhador_passagem.py
    python3 alinhador_passagem.py --ros-args -p modo_travessia:=nav2   (comportamento v2)
"""
from face_ws import selecionar_face, RastreioFaceWS, pose_mapa_no_scan, obstaculo_antes_da_face
import json
import collections
import heapq  # ROTA_MAPA
import math
import os
import threading
import time
from suavizador_nav import comando_corredor
from saida_curva import SaidaCurva, abertura_observada
from giro_frontal import (GiroFrontal, TempoSensor, Observacao, avaliar_varredura,
                          devolver_controle, envolve_disco)

# ---------------------------------------------------------------------------
# Nucleo geometrico (sem ROS: pode ser testado fora do robo)
# ---------------------------------------------------------------------------

FOOTPRINT = [(0.365, 0.315), (0.365, -0.315), (-0.205, -0.315), (-0.205, 0.315)]


class Mapa:
    """Acesso a um OccupancyGrid: 1 = ocupado, 0 = livre, -1 = desconhecido/fora."""

    def __init__(self, resolucao, largura, altura, origem_x, origem_y, origem_yaw, dados,
                 limiar_ocupado=65):
        self.res = resolucao
        self.w = largura
        self.h = altura
        self.ox = origem_x
        self.oy = origem_y
        self.c = math.cos(origem_yaw)
        self.s = math.sin(origem_yaw)
        self.dados = dados
        self.limiar = limiar_ocupado

    def estado(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        gx = self.c * dx + self.s * dy
        gy = -self.s * dx + self.c * dy
        i, j = int(math.floor(gx / self.res)), int(math.floor(gy / self.res))
        if i < 0 or j < 0 or i >= self.w or j >= self.h:
            return -1
        v = self.dados[j * self.w + i]
        if v < 0:
            return -1
        return 1 if v >= self.limiar else 0

    def ocupado_firme(self, x, y, minimo=3):
        """Ocupado e parte de um grupo de >= minimo celulas (8-vizinhanca).

        Ignora pontinhos soltos do SLAM (ex.: o auto-retorno do LiDAR que deixa um
        rastro de celulas isoladas por onde o robo passa).
        """
        if self.estado(x, y) != 1:
            return False
        r = self.res
        vistos = {(0, 0)}
        fila = [(0, 0)]
        while fila:
            i, j = fila.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    k = (i + di, j + dj)
                    if k in vistos or abs(k[0]) > 3 or abs(k[1]) > 3:
                        continue
                    if self.estado(x + k[0] * r, y + k[1] * r) == 1:
                        vistos.add(k)
                        if len(vistos) >= minimo:
                            return True
                        fila.append(k)
        return False


def rota_no_mapa(mapa, ini, fim, folga=0.30, folga_ponta=0.12, raio_ponta=0.60, bloqueios=()):
    """ROTA_MAPA: caminho mais curto so pelo mapa, sem obstaculos do momento, onde o corpo cabe. Devolve
    [(x, y), ...] a cada celula, ou None. Celula solta (ruido do SLAM) nao conta como parede.
    PASSAGEM_BLOQUEADA (11/11): bloqueios = [(x, y, raio), ...]: o centro do robo nao passa a menos de raio deles."""
    w, h, res = mapa.w, mapa.h, mapa.res
    occ = [1 if v >= mapa.limiar else 0 for v in mapa.dados]
    desc = [v < 0 for v in mapa.dados]
    inf = 1e9
    dist = [inf] * (w * h)
    for j in range(1, h - 1):
        for i in range(1, w - 1):
            k = j * w + i
            if occ[k] and sum(occ[k + a * w + b] for a in (-1, 0, 1) for b in (-1, 0, 1)) >= 3:
                dist[k] = 0.0
    d2 = math.sqrt(2.0)
    for j in range(1, h):
        for i in range(1, w - 1):
            k = j * w + i
            dist[k] = min(dist[k], dist[k - 1] + 1.0, dist[k - w] + 1.0, dist[k - w - 1] + d2, dist[k - w + 1] + d2)
    for j in range(h - 2, -1, -1):
        for i in range(w - 2, 0, -1):
            k = j * w + i
            dist[k] = min(dist[k], dist[k + 1] + 1.0, dist[k + w] + 1.0, dist[k + w + 1] + d2, dist[k + w - 1] + d2)

    def celula(x, y):
        dx, dy = x - mapa.ox, y - mapa.oy
        return int(math.floor((mapa.c * dx + mapa.s * dy) / res)), int(math.floor((-mapa.s * dx + mapa.c * dy) / res))

    def mundo(i, j):
        gx, gy = (i + 0.5) * res, (j + 0.5) * res
        return mapa.ox + mapa.c * gx - mapa.s * gy, mapa.oy + mapa.s * gx + mapa.c * gy

    i0, j0 = celula(*ini)
    i1, j1 = celula(*fim)
    if not (0 <= i0 < w and 0 <= j0 < h and 0 <= i1 < w and 0 <= j1 < h):
        return None
    lim, lim_p, r_p = folga / res, folga_ponta / res, raio_ponta / res
    custo = {(i0, j0): 0.0}
    veio = {}
    fila = [(0.0, i0, j0)]
    while fila:
        c, i, j = heapq.heappop(fila)
        if (i, j) == (i1, j1):
            rota = [mundo(i, j)]
            while (i, j) in veio:
                i, j = veio[(i, j)]
                rota.append(mundo(i, j))
            rota.reverse()
            return rota
        if c > custo.get((i, j), inf):
            continue
        for a in (-1, 0, 1):
            for b in (-1, 0, 1):
                ni, nj = i + a, j + b
                if (a == 0 and b == 0) or not (1 <= ni < w - 1 and 1 <= nj < h - 1):
                    continue
                k = nj * w + ni
                ponta = math.hypot(ni - i0, nj - j0) <= r_p or math.hypot(ni - i1, nj - j1) <= r_p
                if desc[k] or dist[k] < (lim_p if ponta else lim):
                    continue
                if bloqueios:   # PASSAGEM_BLOQUEADA
                    cx, cy = mundo(ni, nj)
                    if any(math.hypot(cx - bx, cy - by) < br for bx, by, br in bloqueios):
                        continue
                centro = 1.0 + 1.5 * max(0.0, 1.0 - dist[k] * res / 0.45)   # prefere o meio do corredor
                n = c + math.hypot(a, b) * centro
                if n < custo.get((ni, nj), inf):
                    custo[(ni, nj)] = n
                    veio[(ni, nj)] = (i, j)
                    heapq.heappush(fila, (n, ni, nj))
    return None


def pontos_obstaculo_novo(mapa, pts_base, pose, alcance=0.60, meia_largura=0.45, dist_parede=0.10,
                          para_choque=0.375, max_pts=40):
    """PASSAGEM_BLOQUEADA (11/11): pontos do LiDAR (base_link) a frente do para-choque, ate `alcance`, levados ao
    mapa com `pose` (x, y, yaw). Fica so o que esta LIVRE no mapa e a mais de `dist_parede` de parede (objeto do
    momento, nao a parede do corredor). Devolve [(x, y), ...] (no maximo max_pts, espalhados)."""
    if mapa is None or pose is None or not pts_base:
        return []
    c, s = math.cos(pose[2]), math.sin(pose[2])
    n = int(math.ceil(dist_parede / mapa.res))
    achados = []
    for x, y in pts_base:
        if not (para_choque < x <= para_choque + alcance and abs(y) <= meia_largura):
            continue
        mx, my = pose[0] + c * x - s * y, pose[1] + s * x + c * y
        if mapa.estado(mx, my) != 0:
            continue
        if any(mapa.estado(mx + a * mapa.res, my + b * mapa.res) == 1
               for a in range(-n, n + 1) for b in range(-n, n + 1) if math.hypot(a, b) <= n):
            continue
        achados.append((mx, my))
    if len(achados) > max_pts:
        passo = len(achados) / float(max_pts)
        achados = [achados[int(i * passo)] for i in range(max_pts)]
    return achados


def com_barreiras(no, mapa):
    """MEMORIA_BARREIRA (25/11): o mapa com as barreiras lembradas (/bloqueios_mapa), para as rotas do alinhador."""
    b, m = getattr(no, 'barreiras_msg', None), getattr(no, 'mapa_msg', None)
    if mapa is None or b is None or m is None or not no.p('memoria_barreira') or \
            b.info.width != mapa.w or b.info.height != mapa.h or not any(v > 0 for v in b.data):
        return mapa
    i, q = m.info, m.info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return Mapa(i.resolution, i.width, i.height, i.origin.position.x, i.origin.position.y, yaw,
                [100 if bv > 0 else mv for mv, bv in zip(m.data, b.data)])


def rota_cruza_bloqueio(pontos, bloqueios):
    """PASSAGEM_BLOQUEADA: algum ponto do caminho passa a menos do raio de um bloqueio?"""
    return bool(pontos) and any(math.hypot(px - bx, py - by) < br
                                for px, py in pontos for bx, by, br in bloqueios)


def _comp_rota(p):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))


def _tangentes(pontos):
    n = len(pontos)
    tang = []
    for i in range(n):
        a = pontos[max(0, i - 2)]
        b = pontos[min(n - 1, i + 2)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        d = math.hypot(dx, dy)
        tang.append((dx / d, dy / d) if d > 1e-9 else (1.0, 0.0))
    return tang


# ROTA_MAPA_OBSTACULO (11/11): objeto do momento na rota do mapa (Patch 04)
def obstaculo_novo_na_rota(mapa, obs, rota, raio=0.35, dist_parede=0.10):
    """Celulas ocupadas do `obs` (Mapa do obstacle_layer global, ocupado = 100) que estao livres no `mapa`,
    a mais de `dist_parede` de qualquer parede do mapa e a menos de `raio` de algum ponto da `rota`.
    Devolve [(x, y), ...]. Sem obs ou sem rota: []."""
    if obs is None or not rota:
        return []
    achados = []
    n = int(math.ceil(dist_parede / mapa.res))
    xs = [p[0] for p in rota]; ys = [p[1] for p in rota]
    x0, x1, y0, y1 = min(xs) - raio, max(xs) + raio, min(ys) - raio, max(ys) + raio
    for j in range(obs.h):
        for i in range(obs.w):
            if obs.dados[j * obs.w + i] < obs.limiar:
                continue
            gx, gy = (i + 0.5) * obs.res, (j + 0.5) * obs.res
            x, y = obs.ox + obs.c * gx - obs.s * gy, obs.oy + obs.s * gx + obs.c * gy
            if x < x0 or x > x1 or y < y0 or y > y1 or mapa.estado(x, y) != 0:
                continue
            if any(mapa.estado(x + a * mapa.res, y + b * mapa.res) == 1
                   for a in range(-n, n + 1) for b in range(-n, n + 1) if math.hypot(a, b) <= n):
                continue
            if any(math.hypot(x - p[0], y - p[1]) < raio for p in rota):
                achados.append((x, y))
    return achados


def _raio(mapa, x, y, ux, uy, alcance):
    passo = mapa.res / 2.0
    s = 0.0
    while s <= alcance:
        if mapa.estado(x + ux * s, y + uy * s) == 1:
            return s
        s += passo
    return None


def larguras_no_caminho(mapa, pontos, alcance=0.60, giros=(0, -15, 15, -30, 30)):
    """Para cada ponto do caminho: (dist_esquerda, dist_direita, eixo local).

    v3.20: alem da perpendicular ao caminho, mede tambem com a reta girada +-15/30 graus e
    fica com a menor largura que encontra as duas paredes. Quando o caminho entra no
    corredor na diagonal (missao5 29/10: -114/-132 graus num corredor de -95), a
    perpendicular ao caminho nao achava a parede de um lado e o corredor sumia da analise.
    O "eixo local" e a direcao perpendicular a medida escolhida.
    """
    saida = []
    for (x, y), (tx, ty) in zip(pontos, _tangentes(pontos)):
        melhor = None
        primeira = None
        for g in giros:
            c, s_ = math.cos(math.radians(g)), math.sin(math.radians(g))
            ux, uy = tx * c - ty * s_, tx * s_ + ty * c
            nx, ny = -uy, ux  # normal para a esquerda
            de = _raio(mapa, x, y, nx, ny, alcance)
            dd = _raio(mapa, x, y, -nx, -ny, alcance)
            if primeira is None:
                primeira = (de, dd, (ux, uy))
            if de is not None and dd is not None and (melhor is None or de + dd < melhor[0] + melhor[1] - 1e-9):
                melhor = (de, dd, (ux, uy))
        saida.append(melhor if melhor is not None else primeira)
    return saida


def _comprimento(pontos, i0, i1):
    return sum(math.hypot(pontos[k + 1][0] - pontos[k][0], pontos[k + 1][1] - pontos[k][1])
               for k in range(i0, i1))


def achar_passagens(pontos, larg, largura_estreita, comprimento_min=0.15, buraco_max=2,
                    giro_max=math.radians(45)):
    """Trechos contiguos onde as duas paredes existem e a soma < largura_estreita.

    v3.15: o trecho termina onde o caminho vira mais de giro_max em relacao ao comeco dele
    (canto de um L cortado na diagonal: no curva5 os dois bracos viraram uma passagem so,
    com eixo de -147 graus, e o preparo foi parar 1 m fora, na area aberta).
    """
    estreito = [de is not None and dd is not None and de + dd < largura_estreita
                for de, dd, _ in larg]
    cos_max = math.cos(giro_max)
    tang = _tangentes(pontos)  # v3.20: virada medida no caminho, nao no eixo local
    trechos = []
    i, n = 0, len(pontos)
    while i < n:
        if not estreito[i]:
            i += 1
            continue
        t0 = tang[i]
        j, ultimo, buraco = i, i, 0
        while j + 1 < n:
            j += 1
            tj = tang[j]
            if tj[0] * t0[0] + tj[1] * t0[1] < cos_max:
                j -= 1
                break
            if estreito[j]:
                ultimo, buraco = j, 0
            else:
                buraco += 1
                if buraco > buraco_max:
                    break
        if _comprimento(pontos, i, ultimo) >= comprimento_min:
            trechos.append((i, ultimo))
        i = ultimo + 1
    return trechos


def geometria_passagem(pontos, larg, i0, i1):
    """Centro de entrada, centro de saida, eixo unitario e largura mediana (no mapa)."""
    centros = []
    larguras = []
    for k in range(i0, i1 + 1):
        de, dd, (tx, ty) = larg[k]
        if de is None or dd is None:
            continue
        nx, ny = -ty, tx
        desloc = (de - dd) / 2.0
        centros.append((pontos[k][0] + nx * desloc, pontos[k][1] + ny * desloc))
        larguras.append(de + dd)
    mx = sum(c[0] for c in centros) / len(centros)
    my = sum(c[1] for c in centros) / len(centros)
    if len(centros) >= 3 and _comprimento(pontos, i0, i1) >= 0.25:
        sxx = sum((c[0] - mx) ** 2 for c in centros)
        syy = sum((c[1] - my) ** 2 for c in centros)
        sxy = sum((c[0] - mx) * (c[1] - my) for c in centros)
        ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
        ax, ay = math.cos(ang), math.sin(ang)
    else:
        tx = sum(larg[k][2][0] for k in range(i0, i1 + 1))
        ty = sum(larg[k][2][1] for k in range(i0, i1 + 1))
        d = math.hypot(tx, ty)
        ax, ay = tx / d, ty / d
    sentido_x = pontos[i1][0] - pontos[i0][0]
    sentido_y = pontos[i1][1] - pontos[i0][1]
    if ax * sentido_x + ay * sentido_y < 0:
        ax, ay = -ax, -ay

    def projetar(c):
        t = (c[0] - mx) * ax + (c[1] - my) * ay
        return (mx + t * ax, my + t * ay)

    larguras.sort()
    return dict(entrada=projetar(centros[0]), saida=projetar(centros[-1]),
                eixo=(ax, ay), largura=larguras[len(larguras) // 2])


def yaw_chegada(caminho, goal_xy, robo=None, recuo=0.30):
    """v3.10: angulo de quem chega ao goal pelo caminho (direcao do ultimo trecho de ~recuo m).

    Sem caminho usa a direcao robo->goal; com o robo ja em cima do ponto (< 15 cm) devolve o
    angulo atual dele (nenhum giro). robo = (x, y, yaw) ou None. Devolve None se nao der.
    """
    gx, gy = goal_xy
    if caminho and len(caminho) >= 2:
        acum = 0.0
        ant = (gx, gy)
        for px, py in reversed(caminho[:-1]):
            acum += math.hypot(px - ant[0], py - ant[1])
            ant = (px, py)
            if acum >= recuo:
                break
        if math.hypot(gx - ant[0], gy - ant[1]) >= 0.05:
            return math.atan2(gy - ant[1], gx - ant[0])
    if robo is not None:
        d = math.hypot(gx - robo[0], gy - robo[1])
        if d < 0.15:
            return robo[2]
        return math.atan2(gy - robo[1], gx - robo[0])
    return None


def ponto_de_curva(caminho, mapa, ang_min=math.radians(60), janela=0.4, d_min=0.6, d_fim=0.5):
    """GIRO_NA_CURVA (25/11): 1a curva fechada do caminho -> (x, y, rumo_entrada, rumo_saida) ou None."""
    if not caminho or len(caminho) < 5 or mapa is None:
        return None
    s = [0.0]
    for a, b in zip(caminho, caminho[1:]):
        s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = s[-1]
    for i in range(1, len(caminho) - 1):
        if s[i] < d_min or s[i] > total - d_fim:
            continue
        j0 = max(k for k in range(i + 1) if s[i] - s[k] >= janela) if s[i] >= janela else 0
        j1 = min((k for k in range(i, len(caminho)) if s[k] - s[i] >= janela), default=len(caminho) - 1)
        x, y = caminho[i]
        h_in = math.atan2(y - caminho[j0][1], x - caminho[j0][0])
        h_out = math.atan2(caminho[j1][1] - y, caminho[j1][0] - x)
        dif = abs(_ang(h_out - h_in))
        if dif >= ang_min:
            melhor = (dif, i, h_in, h_out)
            for m in range(i + 1, len(caminho) - 1):   # no mesmo trecho de curva, o vertice (maior mudanca)
                if s[m] - s[i] > janela:
                    break
                m0 = max(k for k in range(m + 1) if s[m] - s[k] >= janela) if s[m] >= janela else 0
                m1 = min((k for k in range(m, len(caminho)) if s[k] - s[m] >= janela), default=len(caminho) - 1)
                hi = math.atan2(caminho[m][1] - caminho[m0][1], caminho[m][0] - caminho[m0][0])
                ho = math.atan2(caminho[m1][1] - caminho[m][1], caminho[m1][0] - caminho[m][0])
                if abs(_ang(ho - hi)) > melhor[0]:
                    melhor = (abs(_ang(ho - hi)), m, hi, ho)
            x, y = caminho[melhor[1]]
            if giro_livre(mapa, x, y):
                return (x, y, melhor[2], melhor[3])
            return None
    return None


def rumo_do_caminho(caminho, x, y, dist=0.5):
    """v3.30: rumo (rad, frame do caminho) do robo ate o ponto do caminho 'dist' m adiante do ponto
    mais perto dele (como o RotationShim faz com forward_sampling_distance). None se nao der."""
    if not caminho or len(caminho) < 2:
        return None
    k = min(range(len(caminho)), key=lambda i: (caminho[i][0] - x) ** 2 + (caminho[i][1] - y) ** 2)
    acc = 0.0
    while k < len(caminho) - 1 and acc < dist:
        acc += math.hypot(caminho[k + 1][0] - caminho[k][0], caminho[k + 1][1] - caminho[k][1])
        k += 1
    dx, dy = caminho[k][0] - x, caminho[k][1] - y
    if math.hypot(dx, dy) < 0.15:
        return None
    return math.atan2(dy, dx)


def ciscando(cmds, odoms, agora, janela=4.0, n_inv=4, desloc_max=0.05, giro_max_graus=15.0):
    """v3.31 ANTI_CISCADO: (sim/nao, inversoes, deslocamento m, giro graus) na janela.

    cmds = [(t, vx, vy, wz)] do /cmd_vel_nav; odoms = [(t, x, y, yaw)]. Ciscado = o comando troca
    de sentido (|v| > 0,02) n_inv vezes ou mais somando giro e lado, e o robo nao se afastou mais de
    desloc_max do ponto do comeco da janela nem girou giro_max_graus."""
    c = [m for m in cmds if agora - janela < m[0] <= agora]
    o = [m for m in odoms if agora - janela < m[0] <= agora]
    if len(o) < 5:
        return False, 0, 0.0, 0.0

    def inversoes(k):
        s = [math.copysign(1.0, m[k]) for m in c if abs(m[k]) > 0.02]
        return sum(1 for i in range(1, len(s)) if s[i] != s[i - 1])
    inv = inversoes(3) + inversoes(2)
    d = max(math.hypot(m[1] - o[0][1], m[2] - o[0][2]) for m in o)
    g = abs(math.degrees(_ang(o[-1][3] - o[0][3])))
    return (inv >= n_inv and d < desloc_max and g < giro_max_graus), inv, d, g


def pose_livre(mapa, x, y, yaw, footprint=FOOTPRINT, margem=0.03, exigir_conhecido=False):
    """Contorno (com margem) e seu interior sem celulas ocupadas."""
    c, s = math.cos(yaw), math.sin(yaw)
    xs = [p[0] for p in footprint]
    ys = [p[1] for p in footprint]
    x0, x1 = min(xs) - margem, max(xs) + margem
    y0, y1 = min(ys) - margem, max(ys) + margem
    passo = mapa.res / 2.0
    u = x0
    while u <= x1 + 1e-9:
        v = y0
        while v <= y1 + 1e-9:
            wx, wy = x + c * u - s * v, y + s * u + c * v
            if exigir_conhecido and mapa.estado(wx, wy) < 0:
                return False
            if mapa.ocupado_firme(wx, wy):
                return False
            v += passo
        u += passo
    return True


def pose_pela_face(face, dist, ang_face, centro, pose_amcl=None):   # CORRIGE_POSE_MESA (24/11)
    """Pose (x, y, yaw) do base_link no mapa a partir da face cadastrada e da medicao do LiDAR.
    face: dict x, y, yaw (direcao robo->mesa). dist/ang_face/centro: como RastreioFaceWS (base_link).
    centro None: componente ao longo da face vem de pose_amcl (None se nao houver)."""
    fy = face['yaw']
    ux, uy = math.cos(fy), math.sin(fy)
    d = dist * math.cos(ang_face)
    if centro is None:
        if pose_amcl is None:
            return None
        t = (face['x'] - pose_amcl[0]) * -uy + (face['y'] - pose_amcl[1]) * ux
    else:
        t = centro
    x = face['x'] - d * ux + t * uy
    y = face['y'] - d * uy - t * ux
    yaw = math.atan2(math.sin(fy - ang_face), math.cos(fy - ang_face))
    return x, y, yaw


def giro_livre(mapa, x, y, margem=0.03):   # ENTRADA_TRANSLADA (24/11)
    return all(pose_livre(mapa, x, y, math.radians(a), margem=margem) for a in range(0, 360, 15))


def ponto_de_giro(caminho, gx, gy, gyaw, mapa, d_min=0.15, d_max=2.5, passo=0.05, eixos=False):   # CAIXA_LATERAL: eixos
    """ENTRADA_TRANSLADA: ultimo ponto do caminho (vindo do fim) onde o giro cabe e de onde o corpo
    vai ao goal em linha reta com o yaw final. None se o giro ja cabe no goal ou se nao achou."""
    if giro_livre(mapa, gx, gy):
        return None
    pts = list(reversed(caminho))
    acum = 0.0
    for k in range(1, len(pts)):
        acum += math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1])
        if acum > d_max:
            break
        x, y = pts[k]
        d = math.hypot(x - gx, y - gy)
        if d < d_min or k % 2:
            continue
        if not giro_livre(mapa, x, y):
            continue
        n = max(2, int(d / passo))
        if all(pose_livre(mapa, x + (gx - x) * i / n, y + (gy - y) * i / n, gyaw, margem=0.02)
               for i in range(n + 1)):
            return (x, y)
    return ponto_de_giro_eixos(gx, gy, gyaw, mapa, passo=passo) if eixos else None


def ponto_de_giro_eixos(gx, gy, gyaw, mapa, passo=0.05):
    """CAIXA_LATERAL (25/11): giro fora + 2 retas: P (gira) -> A (anda no eixo do yaw) -> goal (de lado).
    O caminho do planner chega na diagonal (a reta bate no pilar) e ao lado da caixa o giro nao cabe
    (parede a 0,41 m). Devolve P, com o menor percurso, ou None."""
    def reta(x0, y0, x1, y1):
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / passo))
        return all(pose_livre(mapa, x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, gyaw, margem=0.02)
                   for i in range(n + 1))
    c, s_ = math.cos(gyaw), math.sin(gyaw)
    melhor = None
    for lado in (1, -1):
        lx, ly = -s_ * lado, c * lado
        for kd in range(6, 31):                       # A a 0,30-1,50 m de lado do goal
            d = kd * passo
            ax, ay = gx + lx * d, gy + ly * d
            if not pose_livre(mapa, ax, ay, gyaw, margem=0.02):
                break
            if not reta(ax, ay, gx, gy):
                continue
            for sinal in (1, -1):
                for kf in range(0, 25):               # P ate 1,20 m de A no eixo do yaw
                    f = kf * passo
                    px, py = ax + c * f * sinal, ay + s_ * f * sinal
                    if not pose_livre(mapa, px, py, gyaw, margem=0.02):
                        break
                    if giro_livre(mapa, px, py):
                        if reta(px, py, ax, ay) and (melhor is None or d + f < melhor[2]):
                            melhor = (px, py, d + f)
                        break
    return None if melhor is None else (melhor[0], melhor[1])


def analisar_juncao(pontos, larg, trechos, k, gA, gap_max=1.30):
    """v3.11: o que vem depois da passagem trechos[k] (geometria gA)?

    - None: nada perto (ou um angulo que nao sei tratar);
    - dict(tipo='reto', ...): outra passagem na mesma linha logo depois (cruzamento em + ou
      T atravessado reto): a travessia continua ate a saida dela;
    - dict(tipo='curva', ...): passagem perpendicular logo depois (L, T ou +, virando): o robo
      para no cruzamento com o corpo centrado no corredor novo e sai DE LADO, sem girar.
    """
    if k + 1 >= len(trechos):
        return None
    ax, ay = gA['eixo']
    # v3.19: o caminho que corta o canto do L na diagonal gera um trecho curto e torto
    # entre os dois bracos (missao4 29/10: -134 graus entre -91 e 180); ele e pulado e a
    # juncao e procurada no trecho seguinte
    kb = k + 1
    while True:
        i0b, i1b = trechos[kb]
        if _comprimento(pontos, trechos[k][1], i0b) > gap_max:
            return None
        gB = geometria_passagem(pontos, larg, i0b, i1b)
        bx, by = gB['eixo']
        dot = ax * bx + ay * by
        diagonal = math.cos(math.radians(60)) <= abs(dot) <= math.cos(math.radians(20))
        if diagonal and _comprimento(pontos, i0b, i1b) < 0.40 and kb + 1 < len(trechos):
            kb += 1
            continue
        break
    cruz = ax * by - ay * bx
    if dot > math.cos(math.radians(20)):
        # mesma linha? (eixo de B nao pode estar deslocado do de A)
        ex, ey = gB['entrada'][0] - gA['saida'][0], gB['entrada'][1] - gA['saida'][1]
        if abs(ex * -ay + ey * ax) > 0.12:
            return None
        return dict(tipo='reto', saida=gB['saida'], largura=min(gA['largura'], gB['largura']),
                    entrada_b=gB['entrada'], indices_b=(i0b, i1b), k_b=kb)
    if abs(dot) < math.cos(math.radians(60)):
        # ponto de encontro dos eixos: A0 + t*a = B0 + u*b
        a0x, a0y = gA['saida']
        b0x, b0y = gB['entrada']
        den = ax * by - ay * bx
        t = ((b0x - a0x) * by - (b0y - a0y) * bx) / den
        jx, jy = a0x + t * ax, a0y + t * ay
        if not (-0.40 <= t <= gap_max):
            return None
        return dict(tipo='curva', juncao=(jx, jy), eixo_b=(bx, by), lado=1.0 if cruz > 0 else -1.0,
                    entrada_b=gB['entrada'], saida_b=gB['saida'], largura_b=gB['largura'],
                    indices_b=(i0b, i1b), k_b=kb)
    return None


def _preparo_no_caminho(mapa, pontos, i0, g, distancia_preparo, desvio_max=0.20):
    """PREPARO_RETORNO_20261116: alternativa na rota existente, com corpo conhecido e livre."""
    ax, ay = g['eixo']
    ex, ey = g['entrada']
    yaw = math.atan2(ay, ax)
    dmin, dmax = distancia_preparo - 0.15, distancia_preparo + 0.40
    opcoes = []
    for (x, y), (tx, ty) in zip(pontos[:i0], _tangentes(pontos)[:i0]):
        d = (ex - x) * ax + (ey - y) * ay
        lateral = abs(-(x - ex) * ay + (y - ey) * ax)
        if not (max(0.15, dmin) <= d <= dmax) or lateral > desvio_max:
            continue
        if tx * ax + ty * ay < math.cos(math.radians(30)):
            continue
        if pose_livre(mapa, x, y, yaw, margem=0.03, exigir_conhecido=True):
            opcoes.append((abs(d - distancia_preparo), lateral, x, y, d))
    if not opcoes:
        return None
    _, _, x, y, d = min(opcoes)
    return (x, y, yaw, d)


def planejar_etapas(mapa, pontos, goal_xy, largura_estreita=0.90, distancia_preparo=0.60,
                    distancia_saida=0.45, ignorar_inicio=0.30, ja_tratadas=(), preparo_lateral_max=0.0):
    """Primeira passagem estreita a frente: devolve (etapas, info) ou ([], motivo).

    ja_tratadas: entradas (x, y) de passagens em que o robo ja se alinhou; sao puladas.
    """
    if len(pontos) < 5:
        return [], 'caminho curto demais'
    larg = larguras_no_caminho(mapa, pontos)
    trechos = achar_passagens(pontos, larg, largura_estreita)
    for k, (i0, i1) in enumerate(trechos):
        if _comprimento(pontos, 0, i0) < ignorar_inicio:
            continue  # o robo ja esta na entrada ou dentro deste trecho
        g = geometria_passagem(pontos, larg, i0, i1)
        juncao = analisar_juncao(pontos, larg, trechos, k, g)
        # cruzamentos retos em sequencia (+ + ...): junta tudo numa passagem so
        kk = k
        while juncao is not None and juncao['tipo'] == 'reto':
            g = dict(g, saida=juncao['saida'], largura=juncao['largura'])
            kk = juncao['k_b']
            juncao = analisar_juncao(pontos, larg, trechos, kk, g)
        if any(math.hypot(g['entrada'][0] - ex, g['entrada'][1] - ey) < 0.40
               for ex, ey in ja_tratadas):
            continue
        ax, ay = g['eixo']
        yaw = math.atan2(ay, ax)
        preparo = None
        # v3.13: as paredes no mapa saem ~6 cm mais grossas que as reais (LiDAR a 4 cm do
        # chao): tenta mais longe e, por ultimo, sem margem (teste curva2, volta pelo corredor)
        distancias = (distancia_preparo, distancia_preparo + 0.10, distancia_preparo - 0.10,
                      distancia_preparo + 0.20, distancia_preparo - 0.15,
                      distancia_preparo + 0.30, distancia_preparo + 0.40)
        for margem_p in (0.03, 0.0):
            for d in distancias:
                px, py = g['entrada'][0] - d * ax, g['entrada'][1] - d * ay
                if pose_livre(mapa, px, py, yaw, margem=margem_p):
                    preparo = (px, py, yaw, d)
                    break
            if preparo is not None:
                break
        preparo_no_caminho = False
        if preparo is None:
            preparo = _preparo_no_caminho(mapa, pontos, i0, g, distancia_preparo)
            preparo_no_caminho = preparo is not None
        # PREPARO_LATERAL (19/11): eixo sem lugar no mapa (paredes engordam): desloca de lado o minimo;
        # a travessia centraliza na boca. Volta do L em 19/11 11:51: 7 cm bastavam.
        preparo_lateral = 0.0
        if preparo is None and preparo_lateral_max > 0.0:
            melhor = None
            for cm in range(1, int(round(preparo_lateral_max * 100)) + 1):
                for sinal in (1.0, -1.0):
                    off = sinal * cm / 100.0
                    for im, margem_p in enumerate((0.03, 0.0)):
                        for idd, d in enumerate(distancias):
                            px = g['entrada'][0] - d * ax - off * ay
                            py = g['entrada'][1] - d * ay + off * ax
                            if pose_livre(mapa, px, py, yaw, margem=margem_p):
                                chave = (cm, im, idd)
                                if melhor is None or chave < melhor[0]:
                                    melhor = (chave, (px, py, yaw, d), off)
                                break
                if melhor is not None:
                    break
            if melhor is not None:
                preparo, preparo_lateral = melhor[1], melhor[2]
        if preparo is None:
            return [], 'passagem em (%.2f, %.2f) sem espaco livre para a pose de preparo' % g['entrada']
        etapas = [dict(tipo='preparo', x=preparo[0], y=preparo[1], yaw=yaw)]
        if preparo_lateral:
            etapas[0]['preparo_lateral'] = preparo_lateral
        if preparo_no_caminho:
            etapas[0]['preparo_no_caminho'] = True
        # Pose de saida no eixo da passagem. Se a distancia padrao nao estiver livre no mapa,
        # tenta mais perto da saida: sem ela, o robo seguiria direto para o goal final e
        # poderia sair do centro da passagem (aconteceu no teste de 21/10 08:43).
        if juncao is not None and juncao['tipo'] == 'curva':
            # v3.11: sem pose de saida pelo Nav2 (o robo sai de lado pelo outro corredor)
            kb = juncao['k_b']
            while kb + 1 < len(trechos):
                trecho_b = dict(eixo=juncao['eixo_b'], saida=juncao['saida_b'], largura=juncao['largura_b'])
                seguinte = analisar_juncao(pontos, larg, trechos, kb, trecho_b)
                if seguinte is None or seguinte['tipo'] != 'reto':
                    juncao['outra_juncao'] = _comprimento(pontos, trechos[kb][1], trechos[kb+1][0]) <= 1.30
                    break
                kb = seguinte['k_b']
                juncao.update(saida_b=seguinte['saida'], largura_b=seguinte['largura'], k_b=kb,
                              indices_b=(juncao['indices_b'][0], seguinte['indices_b'][1]))
            info = dict(entrada=g['entrada'], saida=g['saida'], eixo_graus=math.degrees(yaw),
                        largura_mapa=g['largura'], distancia_preparo=preparo[3], curva=juncao,
                        cruzamentos=kk - k)
            return etapas, info
        restante_goal = (goal_xy[0] - g['saida'][0]) * ax + (goal_xy[1] - g['saida'][1]) * ay
        for ds in (distancia_saida, 0.35, 0.25, 0.15, 0.0):
            if ds > distancia_saida or restante_goal <= ds + 0.10:
                continue
            sx, sy = g['saida'][0] + ds * ax, g['saida'][1] + ds * ay
            if pose_livre(mapa, sx, sy, yaw, margem=0.0 if ds < 0.2 else 0.03):
                etapas.append(dict(tipo='saida', x=sx, y=sy, yaw=yaw))
                break
        info = dict(entrada=g['entrada'], saida=g['saida'], eixo_graus=math.degrees(yaw),
                    largura_mapa=g['largura'], distancia_preparo=preparo[3], cruzamentos=kk - k)
        return etapas, info
    return [], 'nenhuma passagem estreita no caminho'


# ---------------------------------------------------------------------------
# Corredor medido pelo LiDAR + controle da travessia (sem ROS)
# ---------------------------------------------------------------------------
def _reta(pts, tol=0.025):
    """Parede quase paralela ao robo: y = a + b*x.

    Varre angulos de -20 a +20 graus e fica com a faixa de 3 cm que junta mais pontos
    (robusto contra pontos da face da bancada, pes de mesa etc.); depois refina por
    minimos quadrados so com esses pontos.
    """
    if len(pts) < 6:
        return None
    melhor = None
    for k in range(-40, 41):
        ang = math.radians(k * 0.5)
        c, s = math.cos(ang), math.sin(ang)
        offs = sorted(p[1] * c - p[0] * s for p in pts)
        j = 0
        for i in range(len(offs)):
            while offs[i] - offs[j] > 0.03:
                j += 1
            n = i - j + 1
            if melhor is None or n > melhor[0]:
                melhor = (n, ang, (offs[i] + offs[j]) / 2.0)
    n, ang, centro = melhor
    c, s = math.cos(ang), math.sin(ang)
    sel = [p for p in pts if abs(p[1] * c - p[0] * s - centro) <= 0.02]
    for _ in range(2):
        m = len(sel)
        if m < 6:
            return None
        mx = sum(p[0] for p in sel) / m
        my = sum(p[1] for p in sel) / m
        sxx = sum((p[0] - mx) ** 2 for p in sel)
        if sxx < 1e-4:
            return None
        b = sum((p[0] - mx) * (p[1] - my) for p in sel) / sxx
        a = my - b * mx
        sel = [p for p in sel if abs(p[1] - (a + b * p[0])) < tol]
    if len(sel) < 6:
        return None
    xs = [p[0] for p in sel]
    if max(xs) - min(xs) < 0.15 or abs(b) > 0.35:  # parede curta demais ou torta demais
        return None
    return a, b, len(sel)


def medir_corredor(pts, x_min=0.20, x_max=1.40, y_min=0.22, y_max=0.80, x_ref=0.08,
                   largura_conhecida=None):
    """pts: pontos do LiDAR em base_link. Devolve dict ou None.

    angulo: quanto o robo precisa girar (rad, + = anti-horario) para ficar paralelo as paredes.
    desvio: quanto o centro do corredor esta a esquerda do centro do robo (m) em x_ref.
    """
    esq = _reta([p for p in pts if x_min <= p[0] <= x_max and y_min < p[1] < y_max])
    dir_ = _reta([p for p in pts if x_min <= p[0] <= x_max and -y_max < p[1] < -y_min])
    if esq is None and dir_ is None:
        return None
    r = dict(esq=None, dir=None, largura=None, desvio=None)
    if esq:
        r['esq'] = esq[0] + esq[1] * x_ref
    if dir_:
        r['dir'] = dir_[0] + dir_[1] * x_ref
    if esq and dir_:
        # paredes quase paralelas? (senao uma delas nao e parede do corredor)
        if abs(math.atan(esq[1]) - math.atan(dir_[1])) > math.radians(8):
            # fica com a que tem mais pontos
            if esq[2] >= dir_[2]:
                dir_, r['dir'] = None, None
            else:
                esq, r['esq'] = None, None
    if esq and dir_:
        r['angulo'] = math.atan((esq[1] * esq[2] + dir_[1] * dir_[2]) / (esq[2] + dir_[2]))
        r['largura'] = r['esq'] - r['dir']
        r['desvio'] = (r['esq'] + r['dir']) / 2.0
        r['n'] = esq[2] + dir_[2]
        r['lados'] = 2
    else:
        lado = esq or dir_
        r['angulo'] = math.atan(lado[1])
        r['n'] = lado[2]
        r['lados'] = 1
        if largura_conhecida:
            if esq:
                r['desvio'] = r['esq'] - largura_conhecida / 2.0
            else:
                r['desvio'] = r['dir'] + largura_conhecida / 2.0
    return r

def _ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def _sat(v, vmin, vmax):
    """Satura |v| entre vmin e vmax mantendo o sinal (vmin vence a zona morta do smoother)."""
    if v == 0.0:
        return 0.0
    return math.copysign(min(max(abs(v), vmin), vmax), v)


class Travessia:
    """Maquina de estados: aproximar -> medir -> girar/deslocar -> medir ... -> atravessar -> fim.

    Coordenadas: pose do odom (x, y, yaw); a medicao do LiDAR vem em base_link.
    """

    def __init__(self, v_aprox=0.03, v_travessia=0.035, giro_min=0.02, giro_max=0.03,
                 lateral_cmd=0.02, tol_giro=math.radians(2.0), tol_lateral=0.015,
                 antecipacao_giro=math.radians(2.0), aceita_giro=math.radians(3.0),
                 largura_min=0.66, largura_max=1.00, aprox_max=0.60, ciclos_max=8,
                 tempo_max=60.0, largura_padrao=0.82, parada_lidar=False, log=print):
        self.entrada_rapida = False   # ENTRADA_RAPIDA: ligada pelo no
        self.uma_parede = False       # UMA_PAREDE: ligada pelo no
        self.f_corpo = None           # FREIO_FRENTE: (t, folga a frente na largura do corpo)
        # FREIO_CURVA (12/11): -1 automatico, 0 desligado, > 0 tolerancia fixa (m) antes do ponto do cruzamento
        self.freio_curva_tol = -1.0
        self.ult_restante = None
        self.v_aprox = v_aprox
        self.v_trav = v_travessia
        self.giro_min, self.giro_max = giro_min, giro_max
        self.lat_cmd = lateral_cmd
        self.tol_giro, self.tol_lat = tol_giro, tol_lateral
        # o robo ainda gira ~2 graus depois que o comando para (medido no teste de 11:43):
        # para antes, senao fica oscilando +-2-3 graus em volta do alvo
        self.antecipacao = antecipacao_giro
        self.aceita_giro = aceita_giro
        self.larg_min, self.larg_max = largura_min, largura_max
        self.aprox_max = aprox_max
        self.ciclos_max = ciclos_max
        self.tempo_max = tempo_max
        self.log = log
        self.estado = 'aproximar'
        self.motivo = ''
        self.t0 = None
        self.t_estado = None
        self.inicio_estado = None  # pose do odom quando o estado comecou
        self.meds = []             # (t, med, pose_no_scan) com as 2 paredes
        self.meds1 = []            # idem, so uma parede (centro estimado pela largura)
        self.largura_padrao = largura_padrao
        self.ult = None            # ultima medicao boa de 2 paredes
        self.largura = None
        self.ciclos = 0
        self.n_lat = 0
        self.n_bloq = 0
        self.giro_bloqueado = False
        self.frente = 9.0          # folga livre a frente do para-choque (m), do LiDAR
        self.yaw_ref = None
        self.alvo = None
        self.p_aprox_odom = None
        # v3.12: parar no cruzamento pela parede a frente (o mapa erra ~10-15 cm no eixo do
        # corredor: SLAM em corredor nao "ve" bem o avanco; teste curva1 de 27/10)
        self.parada_lidar = parada_lidar
        self.transv = None         # (t, x da parede em base_link, pose do odom no scan)
        self.transv_fora = None    # PAREDE_MAPA: (t, diferenca medida - esperada pelo mapa)
        self.ult_transv = None
        self.parede_parada = None  # folga do para-choque quando parou
        self.largura_b = None      # v3.22: largura (mapa) do corredor novo, onde o corpo fica
        self.folgas = None         # v3.28: (t, folga esq, folga dir) reais da lateral do corpo

    def nova_frente(self, t, folga):   # FREIO_FRENTE
        self.f_corpo = (t, folga)

    def nova_folgas(self, t, esq, dir_):
        self.folgas = (t, esq, dir_)

    def nova_transversal(self, t, x_parede, pose, x_esp=None):
        if x_parede is None:
            return
        # PAREDE_MAPA (09/11, missao26): so vale a parede onde o mapa a espera (+-15 cm). A parede
        # certa bateu em +-5 cm; o bloco entre corredores (-0,9 m) parou o robo na boca do corredor
        # e o obstaculo da prova alem do T aberto (+0,22..0,27 m) o fez passar 24 cm do ponto.
        if x_esp is not None and abs(x_parede - x_esp) > 0.15:
            self.transv_fora = (t, x_parede - x_esp)
            return
        self.transv = (t, x_parede, pose)

    def tol_curva(self):
        """FREIO_CURVA (12/11): quanto antes do ponto do cruzamento o corpo ja cabe no corredor novo (m), ou None."""
        if not self.parada_lidar or self.freio_curva_tol == 0.0:
            return None
        if self.freio_curva_tol > 0.0:
            return min(0.15, self.freio_curva_tol)
        larg = self.largura_b or self.largura or self.largura_padrao
        return min(0.10, max(0.03, (larg + 0.06 - 0.59) / 2.0))

    def gap_alvo(self):
        """Folga para-choque -> parede para o corpo (0,57 m) ficar centrado no corredor novo.

        v3.22 (missao8 03/11): pela largura do corredor NOVO (era a do corredor da travessia:
        0,84 m -> alvo 13 cm, com o novo de ~0,70 m o robo parou 8 cm fora do centro e a
        traseira ficou a 1 cm da quina)."""
        larg = self.largura_b or self.largura or self.largura_padrao
        return max(0.05, (larg - 0.57) / 2.0)

    # ------------------------------------------------------------------
    def _boa(self, med):
        return (med is not None and med['lados'] == 2 and med['n'] >= 14 and
                self.larg_min < med['largura'] < self.larg_max)

    def nova_medicao(self, t, med, pose, frente=None):
        if frente is not None:
            self.frente = frente
        if med is None:
            return
        if med.get('desvio') is not None:
            # v3.28: o centro do corredor nao pode estar mais longe do que o robo cabe la dentro
            larg = self.largura or self.largura_padrao
            if abs(med['desvio']) > max(0.10, larg / 2.0 - 0.315 + 0.05):
                return
        if (med['lados'] == 1 and med['n'] >= 10 and med.get('desvio') is not None and
                abs(med['angulo']) < math.radians(12)):
            self.meds1.append((t, med, pose))
            self.meds1 = self.meds1[-10:]
        if self._boa(med):
            self.meds.append((t, med, pose))
            self.meds = self.meds[-10:]
            self.ult = (t, med, pose)
            self.largura = med['largura'] if self.largura is None else (
                0.8 * self.largura + 0.2 * med['largura'])
        elif (self.estado == 'atravessar' and med['n'] >= 10 and med.get('desvio') is not None
              and self.ult is not None):
            # uma parede so (dentro do corredor): aceita se concordar com o angulo atual
            yaw_eixo = pose[2] + med['angulo']
            if abs(_ang(yaw_eixo - self.yaw_ref)) < math.radians(4):
                self.ult = (t, med, pose)

    def _mudar(self, estado, t, pose):
        self.estado, self.t_estado, self.inicio_estado = estado, t, pose

    def _entrada_centrada(self, vistas, t):
        # Somente duas medidas de ambas as paredes autorizam o primeiro avanço.
        return len(vistas) >= 2 and all(
            0 <= t - instante <= 0.5 and m['lados'] == 2
            and m['largura'] >= self.larg_min + 0.02
            and m['desvio'] is not None and abs(m['desvio']) <= self.tol_lat
            and abs(m['angulo']) <= min(self.tol_giro, self.aceita_giro)
            for instante, m, _ in vistas[-2:])

    def _desloc_robo(self, p0, p1):
        """Deslocamento de p0 ate p1 no referencial do robo em p0."""
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s = math.cos(p0[2]), math.sin(p0[2])
        return c * dx + s * dy, -s * dx + c * dy

    def _erro_lateral(self, pose):
        """Centro do corredor (m, + = a esquerda), corrigido pelo movimento desde o scan."""
        t, med, p_scan = self.ult
        dx, dy = self._desloc_robo(p_scan, pose)
        return med['desvio'] + math.tan(med['angulo']) * dx - dy

    def bloqueio(self, t, pose):
        """O Collision Monitor esta segurando o robo: para e mede de novo (ou desiste)."""
        tol = self.tol_curva() if self.estado == 'atravessar' else None   # FREIO_CURVA (12/11)
        if tol is not None and self.ult_restante is not None and self.ult_restante <= tol:
            self.estado = 'fim'
            self.log('FREIO_CURVA: Collision Monitor segurando %.0f cm %s do ponto do cruzamento (tolerancia %.0f cm): '
                     'paro aqui e sigo a saida' % (abs(self.ult_restante) * 100,
                                                  'antes' if self.ult_restante > 0 else 'alem', tol * 100))
            return
        if self.estado == 'atravessar':
            # dentro da passagem, seguir empurrando so piora (pivota na quina):
            # desiste; o no sai de re e tenta de novo a partir do preparo
            self.abortar('Collision Monitor bloqueando dentro da passagem')
            return
        self.n_bloq += 1
        if self.n_bloq > 4:
            self.abortar('Collision Monitor bloqueou %d vezes' % self.n_bloq)
            return
        self.log('Collision Monitor segurando em "%s": parando para medir de novo' % self.estado)
        self.giro_bloqueado = self.estado == 'girar'
        self._mudar('assentar', t, pose)

    def retomar(self, pausa, t, pose):
        """SENSOR_PARADO: sensores voltaram. O tempo parado nao conta e mede tudo de novo."""
        if self.t0 is None or self.estado in ('fim', 'abortar'):
            return
        self.t0 += pausa
        if self.estado == 'aproximar':
            self.t_estado += pausa
            return
        self._mudar('assentar', t, pose)

    def abortar(self, motivo):
        self.estado, self.motivo = 'abortar', motivo
        self.log('Travessia LiDAR abortada: %s' % motivo)

    # ------------------------------------------------------------------
    def passo(self, t, pose, restante):
        """Devolve (vx, vy, wz). restante: metros ate o centro do robo passar da saida."""
        if self.t0 is None:
            self.t0 = t
            self.p_aprox = pose
            self.p_aprox_odom = pose
            self._mudar('aproximar', t, pose)
        if self.estado in ('fim', 'abortar'):
            return 0.0, 0.0, 0.0
        if t - self.t0 > self.tempo_max:
            self.abortar('tempo maximo')
            return 0.0, 0.0, 0.0
        e = self.estado
        self.ult_restante = restante   # FREIO_CURVA

        if e == 'aproximar':
            vistas = [m for m in self.meds if m[0] >= self.t_estado]
            if self.entrada_rapida and self._entrada_centrada(vistas, t):
                self.yaw_ref = _ang(vistas[-1][2][2] + vistas[-1][1]['angulo'])
                self.log('Paredes vistas: entrando sem parar (%+.1f graus, centro %+.1f cm)' % (
                    math.degrees(vistas[-1][1]['angulo']), vistas[-1][1]['desvio'] * 100))
                self._mudar('atravessar', t, pose)
                return self.v_aprox, 0.0, 0.0
            if len(vistas) >= 2:  # duas leituras boas seguidas: nao e ruido
                self.log('Paredes vistas (largura %.2f m): parando na boca para alinhar' %
                         self.ult[1]['largura'])
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            andou = math.hypot(pose[0] - self.p_aprox[0], pose[1] - self.p_aprox[1])
            if andou > self.aprox_max or self.frente < 0.06:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return self.v_aprox, 0.0, 0.0

        if e == 'assentar':  # espera o robo parar e o LiDAR atualizar
            if t - self.t_estado >= 0.4:
                self._mudar('medir', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'medir':
            novas = [m for m in self.meds if m[0] >= self.t_estado]
            if len(novas) < 4 and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            if len(novas) < 2:
                # Muito fora do centro o LiDAR so ve uma parede: usa ela + a largura conhecida
                novas1 = [m for m in self.meds1 if m[0] >= self.t_estado]
                if len(novas1) >= 2:
                    novas = novas1
                    if self.ult is None or self.ult[0] < novas1[-1][0]:
                        self.ult = novas1[-1]
                    self.log('So vejo uma parede: usando a largura de %.2f m para achar o centro' %
                             (self.largura or self.largura_padrao))
                else:
                    # ainda longe: avanca mais um pouco, se a frente estiver livre
                    andou = math.hypot(pose[0] - self.p_aprox[0], pose[1] - self.p_aprox[1])
                    if andou < self.aprox_max and self.frente >= 0.06:
                        self.estado, self.t_estado, self.inicio_estado = 'aproximar', t, pose
                        return 0.0, 0.0, 0.0
                    self.abortar('nao vejo as paredes da passagem')
                    return 0.0, 0.0, 0.0
            ang = sorted(m[1]['angulo'] for m in novas)[len(novas) // 2]
            if self.uma_parede and novas[-1][1]['lados'] == 1:
                ang = 0.0   # UMA_PAREDE: com uma parede so o angulo oscilou +6/-6 graus e gastou 22 s (missao23)
            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]
            yaw_scan = novas[-1][2][2]
            self.ciclos += 1
            larg = self.largura or self.largura_padrao
            self.log('Medicao %d: girar %+.1f graus, centro a %+.1f cm, largura %.2f m' % (
                self.ciclos, math.degrees(ang), des * 100, larg))
            if larg < self.larg_min + 0.02:
                self.abortar('passagem estreita demais (%.2f m)' % larg)
                return 0.0, 0.0, 0.0
            # Ordem: muito fora do centro -> desliza primeiro (perto da bancada o giro e
            # bloqueado pelo Collision Monitor); depois giro grande; depois deslizes finos
            # (cada deslize pode alterar o yaw); o giro fino fica por ultimo.
            pode = self.ciclos <= self.ciclos_max
            lat_ok = self.n_lat < 5
            bloq, self.giro_bloqueado = self.giro_bloqueado, False
            if pode and lat_ok and (abs(des) > 0.06 or (bloq and abs(des) > self.tol_lat)):
                self.n_lat += 1
                self.alvo = des
                self._mudar('deslocar', t, pose)
            elif pode and abs(ang) > 2 * self.tol_giro:
                self.alvo = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif pode and abs(des) > self.tol_lat and lat_ok:
                self.n_lat += 1
                self.alvo = des
                self._mudar('deslocar', t, pose)
            elif pode and abs(ang) > self.tol_giro:
                self.alvo = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif self._entrada_centrada(novas, t):
                self.yaw_ref = _ang(yaw_scan + ang)
                self.log('Alinhado: atravessando')
                self._mudar('atravessar', t, pose)
            elif pode and abs(des) <= self.tol_lat and abs(ang) <= self.tol_giro:
                # A mediana ou uma parede só pode guiar o ajuste, mas ainda falta confirmar a entrada.
                self._mudar('medir', t, pose)
            else:
                self.abortar('entrada sem centro e angulo confirmados pelas duas paredes')
            return 0.0, 0.0, 0.0

        if e == 'girar':
            err = _ang(self.alvo - pose[2])
            if abs(err) < max(math.radians(0.6), self.antecipacao) or t - self.t_estado > 8.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, _sat(0.5 * err, self.giro_min, self.giro_max)

        if e == 'deslocar':
            _, dy = self._desloc_robo(self.inicio_estado, pose)
            falta = self.alvo - dy
            # Fecha pelo LiDAR (o odom lateral erra 2-3x): para quando a leitura nova das
            # paredes mostra o centro perto/cruzado, ou pelo odom com folga de seguranca.
            novas = ([m for m in self.meds if m[0] >= self.t_estado + 0.25] or
                     [m for m in self.meds1 if m[0] >= self.t_estado + 0.25])
            if novas:
                des_agora = novas[-1][1]['desvio']
                if abs(des_agora) < 0.6 * self.tol_lat or des_agora * self.alvo < 0:
                    self._mudar('assentar', t, pose)
                    return 0.0, 0.0, 0.0
                falta = des_agora
            if abs(dy) > 2.5 * abs(self.alvo) + 0.02 or t - self.t_estado > 6.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            # segura o yaw pelo IMU: deslizar da trancos de yaw no mecanum
            err = _ang(self.inicio_estado[2] - pose[2])
            wz = _sat(0.5 * err, 0.016, 0.03) if abs(err) > math.radians(0.8) else 0.0
            return 0.0, math.copysign(self.lat_cmd, falta), wz

        if e == 'atravessar':
            # FREIO_FRENTE: em qualquer modo. Na missao23 ele seguia para um ponto alem do cruzamento, sem
            # parada pelo LiDAR, e encostou (10:11:03): quem parou foi o CM, ja com a frente a 0 cm.
            fc = self.f_corpo
            if fc is not None and t - fc[0] < 0.5 and fc[1] < 0.10:
                # FREIO_CRUZAMENTO (09/11): missao25 12:11, obstaculo da prova alem de um T aberto.
                # O robo ja estava 23 cm alem do ponto do cruzamento e abortava (re de 1,8 m e
                # nova tentativa). Ja no cruzamento: para aqui e segue para a saida lateral.
                tol = self.tol_curva()   # FREIO_CURVA (12/11)
                if tol is not None and 0.0 < restante <= tol:
                    self.estado = 'fim'
                    self.log('FREIO_CURVA: algo a %.0f cm do para-choque, %.0f cm antes do ponto do cruzamento '
                             '(tolerancia %.0f cm: o corpo ja cabe no corredor novo): paro aqui e sigo a saida' % (
                                 fc[1] * 100, restante * 100, tol * 100))
                    return 0.0, 0.0, 0.0
                if self.parada_lidar and restante <= 0.0:
                    self.estado = 'fim'
                    self.log('FREIO_CRUZAMENTO: algo a %.0f cm do para-choque, ja %.0f cm alem do '
                             'ponto do cruzamento: paro aqui e sigo a saida' % (fc[1] * 100, -restante * 100))
                    return 0.0, 0.0, 0.0
                self.abortar('algo a %.0f cm do para-choque' % (fc[1] * 100))
                return 0.0, 0.0, 0.0
            if self.parada_lidar:
                tv = self.transv
                if tv is not None and t - tv[0] < 0.8 and restante < 1.0:
                    dx, _ = self._desloc_robo(tv[2], pose)
                    gap = tv[1] - dx - 0.365
                    # o robo anda ~0,1 m/s e o LiDAR atrasa ~0,25 s: para 3 cm antes
                    if gap <= self.gap_alvo() + 0.03:
                        self.estado = 'fim'
                        self.parede_parada = gap
                        self.log('Parei no cruzamento pelo LiDAR: parede a %.0f cm do para-choque '
                                 '(alvo %.0f cm; pelo mapa faltavam %+.0f cm)' % (
                                     gap * 100, self.gap_alvo() * 100, restante * 100))
                        return 0.0, 0.0, 0.0
                elif (self.transv_fora is not None and t - self.transv_fora[0] < 0.8
                      and restante <= 0.0):
                    # PAREDE_MAPA: o que esta a frente nao e a parede do mapa: para no ponto do cruzamento
                    self.estado = 'fim'
                    self.log('PAREDE_MAPA: parede/obstaculo a frente fora do mapa (%+.0f cm do esperado): '
                             'parei pelo mapa (%.0f cm alem do ponto)' % (
                                 self.transv_fora[1] * 100, -restante * 100))
                    return 0.0, 0.0, 0.0
                elif restante <= -0.30:
                    self.estado = 'fim'
                    self.log('Sem parede a frente: parei pelo mapa (%.0f cm alem do ponto)' % (-restante * 100))
                    return 0.0, 0.0, 0.0
            elif restante <= 0.0:
                self.estado = 'fim'
                self.log('Travessia LiDAR concluida')
                return 0.0, 0.0, 0.0
            # eixo do corredor: corrige devagar com as medicoes de 2 paredes
            tm, med, p_scan = self.ult
            if med['lados'] == 2 and t - tm < 0.5:
                eixo = _ang(p_scan[2] + med['angulo'])
                self.yaw_ref = _ang(self.yaw_ref + max(-0.01, min(0.01, 0.3 * _ang(eixo - self.yaw_ref))))
            err_yaw = _ang(self.yaw_ref - pose[2])
            err_lat = self._erro_lateral(pose) if t - tm < 1.5 else 0.0
            # v3.26: andando, a ponte ignora giro < 0,02 rad/s (heading hold): minimo 0,022
            wz = _sat(0.4 * err_yaw, 0.022, 0.03) if abs(err_yaw) > math.radians(1.2) else 0.0
            # dentro da passagem so corrige o lado com erro > 2.5 cm: cada deslize da tranco de yaw
            vy = _sat(0.5 * err_lat, 0.016, 0.025) if abs(err_lat) > 0.025 else 0.0
            # v3.28: seguranca pelas folgas reais da lateral do corpo
            f = self.folgas
            if f is not None and t - f[0] < 0.5:
                fe, fd = f[1], f[2]
                if min(fe, fd) < 0.04 and abs(fe - fd) > 0.03:
                    # Perto da quina, afasta só de lado até recuperar folga; reavalia a cada scan.
                    if max(fe, fd) < 0.06:
                        return 0.0, 0.0, 0.0
                    return 0.0, math.copysign(0.016, fe - fd), 0.0
                if (vy > 0 and fe < 0.06) or (vy < 0 and fd < 0.06):
                    vy = 0.0
            vx = self.v_trav
            # VEL_ZONAS: devagar se apertado ou perto do fim: a parada pelo LiDAR foi medida a ~0,11 m/s
            if restante < 0.40 or (f is not None and min(f[1], f[2]) < 0.08) or (
                    fc is not None and fc[1] < 0.30):
                vx = min(vx, 0.035)
            if abs(err_yaw) > math.radians(4) or abs(err_lat) > 0.035:
                vx = 0.0  # corrige parado antes de seguir
            return vx, vy, wz
        return 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------
# v3.11: sair de lado num cruzamento (L, T, +) sem girar (sem ROS)
# ---------------------------------------------------------------------------
def _reta_transversal(pts, tol=0.02):
    """Parede quase perpendicular ao robo (na frente ou atras): x = a + b*y.

    Varre angulos de -15 a +15 graus e fica com a faixa de 3 cm com mais pontos;
    refina por minimos quadrados. Devolve (a, b, n) ou None."""
    if len(pts) < 5:
        return None
    melhor = None
    for k in range(-30, 31):
        th = math.radians(k * 0.5)
        c, s = math.cos(th), math.sin(th)
        offs = sorted(p[0] * c - p[1] * s for p in pts)
        j = 0
        for i in range(len(offs)):
            while offs[i] - offs[j] > 0.03:
                j += 1
            n = i - j + 1
            if melhor is None or n > melhor[0]:
                melhor = (n, th, (offs[i] + offs[j]) / 2.0)
    _, th, centro = melhor
    c, s = math.cos(th), math.sin(th)
    sel = [p for p in pts if abs(p[0] * c - p[1] * s - centro) <= 0.02]
    for _ in range(2):
        m = len(sel)
        if m < 5:
            return None
        mx = sum(p[0] for p in sel) / m
        my = sum(p[1] for p in sel) / m
        syy = sum((p[1] - my) ** 2 for p in sel)
        if syy < 1e-4:
            return None
        b = sum((p[1] - my) * (p[0] - mx) for p in sel) / syy
        a = mx - b * my
        sel = [p for p in sel if abs(p[0] - (a + b * p[1])) < tol]
    if len(sel) < 5:
        return None
    ys = [p[1] for p in sel]
    if max(ys) - min(ys) < 0.12 or abs(b) > 0.27:
        return None
    return a, b, len(sel)


def medir_corredor_lateral(pts, lado, largura_conhecida=0.82, centro_corpo=0.08):
    """Corredor perpendicular ao robo (o robo vai andar de lado, lado=+1 esquerda, -1 direita).

    As paredes do corredor ficam na frente (x > 0) e atras (x < 0) do robo.
    Devolve dict(angulo, desvio, frente, lados, n) ou None:
      angulo = quanto girar (rad, + = anti-horario) para ficar perpendicular as paredes;
      desvio = quanto o centro do corredor esta A FRENTE do centro do corpo (m);
      frente = folga do para-choque ate a parede da frente (m) ou None.
    """
    # so a faixa por onde o robo vai passar (e um pouco para tras): evita a continuacao
    # do corredor antigo e paredes laterais do cruzamento
    # v3.14: so a parede ao lado do corpo (e um pouco a frente): perto da saida, pontos
    # mais longe sao de outras paredes da arena e entortavam a medida (teste curva3 12:07)
    faixa = [p for p in pts if -0.40 <= p[1] * lado <= 0.70]
    fr = _reta_transversal([p for p in faixa if 0.40 <= p[0] <= 0.95])
    # v3.13: a parede de tras so vale perto de onde ela deve estar (frente - largura): no
    # teste curva2 o ajuste pegava pontos soltos a -0,9 m e o centro pulava +-15 cm
    if fr:
        x_tr = fr[0] - largura_conhecida
        tr = _reta_transversal([p for p in faixa if x_tr - 0.10 <= p[0] <= min(-0.22, x_tr + 0.10)])
    else:
        tr = _reta_transversal([p for p in faixa if -0.62 <= p[0] <= -0.22])
    if tr and (tr[2] < 8 or abs(tr[1]) > 0.10):
        tr = None
    if fr and tr and abs(math.atan(fr[1]) - math.atan(tr[1])) > math.radians(6):
        tr = None
    if not fr and not tr:
        return None
    r = dict(frente=(fr[0] - FOOTPRINT[0][0]) if fr else None)
    if fr and tr:
        r['angulo'] = -math.atan((fr[1] * fr[2] + tr[1] * tr[2]) / (fr[2] + tr[2]))
        r['desvio'] = (fr[0] + tr[0]) / 2.0 - centro_corpo
        r['largura'] = fr[0] - tr[0]
        r['lados'], r['n'] = 2, fr[2] + tr[2]
    elif fr:
        r['angulo'] = -math.atan(fr[1])
        r['desvio'] = fr[0] - largura_conhecida / 2.0 - centro_corpo
        r['largura'] = None
        r['lados'], r['n'] = 1, fr[2]
    else:
        r['angulo'] = -math.atan(tr[1])
        r['desvio'] = tr[0] + largura_conhecida / 2.0 - centro_corpo
        r['largura'] = None
        r['lados'], r['n'] = 1, tr[2]
    return r


def medir_parede_a_frente(pts, lado, x_max=1.6):
    """v3.12: parede transversal a frente que marca o fim do cruzamento: a parede da frente
    (L, T vindo da haste) ou a parede do lado de la do corredor novo (+, T pela barra).
    Devolve x (base_link) da parede na linha do robo, ou None."""
    sel = [p for p in pts if 0.30 <= p[0] <= x_max and -0.25 <= p[1] * lado <= 1.40]
    r = _reta_transversal(sel)
    if r is None:
        return None
    return r[0]


def folgas_lateral(pts, lado, alcance=0.40):
    """v3.15: folga real (m) do para-choque e da traseira ate o que estiver na faixa do corpo.

    Devolve (frente, tras, parede_frente, parede_tras): folga = ponto mais perto (None quando
    nao ha nada a menos de `alcance`); parede_* = os pontos ate 4 cm alem do mais perto cobrem
    15 cm ou mais de largura (parede de verdade, nao a quina de um cruzamento vista de lado:
    no curva5 a quina a 18 cm dava "tras 23-26" e o centro brigava com a parede da frente).
    Nao supoe a largura do corredor: no curva4 a largura real variou de 0,80 a 0,87 m e a
    medida por uma parede so errava ate 5 cm.
    """
    x_fr, x_tr = FOOTPRINT[0][0], FOOTPRINT[2][0]
    y_max = FOOTPRINT[0][1]
    fr, tr = [], []
    for x, y in pts:
        yl = y * lado
        if not -y_max <= yl <= y_max + 0.05:
            continue
        if x_fr < x <= x_fr + alcance:
            fr.append((x - x_fr, yl))
        elif x_tr - alcance <= x < x_tr:
            tr.append((x_tr - x, yl))

    def lado_(lst):
        if not lst:
            return None, False
        d0 = min(d for d, _ in lst)
        ys = [yy for d, yy in lst if d <= d0 + 0.04]
        return d0, len(ys) >= 4 and max(ys) - min(ys) >= 0.15

    f, pf = lado_(fr)
    r, pr = lado_(tr)
    return f, r, pf, pr


# v3.20: retangulo que o Collision Monitor confere com o robo girando devagar (|wz| < 0,20 rad/s,
# poligono "stopped" do nav2_params) + 1 cm. Com |wz| >= 0,20 o retangulo e maior
# ([-0,275, 0,435] x +-0,385), por isso o giro aqui e sempre devagar.
RET_GIRO = (-0.255, 0.415, 0.365)
WZ_GIRO = 0.15
RET_GIRO_RAPIDO = (-0.325, 0.485, 0.435)   # CM_RIGIDO (09/11): giro_esq/giro_dir do CM (11 cm) + 1 cm; era (-0.285, 0.445, 0.395)
WZ_GIRO_RAPIDO = 0.30
FOLGA_MOV = 0.07   # o Collision Monitor para o robo a 6 cm do lado para onde ele anda
DIRECOES = {'frente': (1.0, 0.0), 'tras': (-1.0, 0.0), 'esq': (0.0, 1.0), 'dir': (0.0, -1.0)}


def _fora_do_corpo(pts, alcance=0.62):
    x0, x1, y1 = FOOTPRINT[2][0], FOOTPRINT[0][0], FOOTPRINT[0][1]
    return [(x, y) for x, y in pts
            if abs(x) < alcance and abs(y) < alcance and not (x0 <= x <= x1 and -y1 <= y <= y1)]


def giro_bloqueado(pts, ang, ret=RET_GIRO, min_pontos=3):
    """v3.20: pontos que entrariam no retangulo do Collision Monitor girando 'ang' rad no lugar
    (sinal = sentido). Lista vazia = o giro passa. pts no base_link."""
    perto = _fora_do_corpo(pts)
    x0, x1, y1 = ret
    n = max(2, int(abs(ang) / math.radians(5.0)) + 1)
    for k in range(n + 1):
        t = ang * k / n
        c, s_ = math.cos(t), math.sin(t)
        dentro = [(x, y) for x, y in perto
                  if x0 <= c * x + s_ * y <= x1 and abs(-s_ * x + c * y) <= y1]
        if len(dentro) >= min_pontos:
            return dentro
    return []


def folga_direcao(pts, direcao, m=0.02):   # CAIXA_LATERAL (25/11): m = alargamento da faixa do corpo
    """v3.20: distancia do corpo ate o ponto mais perto andando reto nessa direcao (m)."""
    x0, x1, y1 = FOOTPRINT[2][0], FOOTPRINT[0][0], FOOTPRINT[0][1]
    if direcao == 'frente':
        d = [x - x1 for x, y in pts if x > x1 and abs(y) <= y1 + m]
    elif direcao == 'tras':
        d = [x0 - x for x, y in pts if x < x0 and abs(y) <= y1 + m]
    elif direcao == 'esq':
        d = [y - y1 for x, y in pts if y > y1 and x0 - m <= x <= x1 + m]
    else:
        d = [-y1 - y for x, y in pts if y < -y1 and x0 - m <= x <= x1 + m]
    return float(min(d)) if d else 9.0


def plano_desvio(pts, lado_goal=0.0, alcance=0.80, profundidade=0.35, margem=0.08,
                 max_lado=0.80, livre_frente=0.60):
    """v3.27: obstaculo na frente do para-choque: deslizar para que lado e quanto (pts no base_link).

    Devolve None se nada bloqueia a frente (ate 'alcance' m), senao um dict:
      lado: 'esq' | 'dir' | None (nenhum lado serve), desloc (m), frente (m a avancar depois),
      obst: (x_perto, x_longe, y_min, y_max) da peca, opcoes: {lado: (desloc, motivo)}.
    O obstaculo e o grupo de pontos mais perto na faixa do corpo, estendido para os lados por
    pontos vizinhos (vao < 12 cm). Para cada lado: deslocamento ate a lateral do corpo passar a
    peca + margem; serve se cabe na folga desse lado e se a faixa do corpo ja deslocado fica
    livre por 'livre_frente' m a frente. Escolhe o menor deslocamento (empate de 5 cm: o lado do
    destino, sinal de lado_goal)."""
    x0, x1, y1 = FOOTPRINT[2][0], FOOTPRINT[0][0], FOOTPRINT[0][1]
    faixa = [(x, y) for x, y in pts if x1 < x <= x1 + alcance and abs(y) <= y1 + 0.03]
    if len(faixa) < 3:
        return None
    xp = min(x for x, _ in faixa)
    perto = sorted(((x, y) for x, y in pts if xp - 0.02 <= x <= xp + profundidade and abs(y) < 1.5),
                   key=lambda p: p[1])
    nucleo = [p for p in perto if abs(p[1]) <= y1 + 0.03]
    ymin, ymax = min(y for _, y in nucleo), max(y for _, y in nucleo)
    for p in perto:                       # estende para a esquerda por pontos vizinhos
        if ymax < p[1] <= ymax + 0.12:
            ymax = p[1]
    for p in reversed(perto):             # e para a direita
        if ymin - 0.12 <= p[1] < ymin:
            ymin = p[1]
    obst_pts = [p for p in perto if ymin <= p[1] <= ymax]
    xl = max(x for x, _ in obst_pts)
    frente_livre = folga_direcao(pts, 'frente')
    opcoes = {}
    for lado, desloc in (('esq', ymax + y1 + margem), ('dir', y1 - ymin + margem)):
        sinal = 1.0 if lado == 'esq' else -1.0
        if desloc > max_lado:
            opcoes[lado] = (desloc, 'longe demais (a peca encosta numa parede?)')
            continue
        if folga_direcao(pts, lado) < desloc + FOLGA_MOV:
            opcoes[lado] = (desloc, 'sem espaco de lado (%.0f cm)' % (100 * folga_direcao(pts, lado)))
            continue
        yc = sinal * desloc
        bloq = [(x, y) for x, y in pts if x1 < x <= x1 + livre_frente and abs(y - yc) <= y1 + 0.03]
        if len(bloq) >= 3:
            opcoes[lado] = (desloc, 'frente ocupada depois de deslizar (%.0f cm)' % (
                100 * (min(x for x, _ in bloq) - x1)))
            continue
        opcoes[lado] = (desloc, 'ok')
    boas = [(d, l) for l, (d, m) in opcoes.items() if m == 'ok']
    lado = None
    desloc = 0.0
    if boas:
        boas.sort()
        desloc, lado = boas[0]
        if len(boas) == 2 and abs(boas[0][0] - boas[1][0]) < 0.05 and lado_goal:
            lado = 'esq' if lado_goal > 0 else 'dir'
            desloc = opcoes[lado][0]
    return {'lado': lado, 'desloc': desloc, 'frente': max(0.0, xl - x1 + 0.10),
            'obst': (xp, xl, ymin, ymax), 'opcoes': opcoes, 'frente_livre': frente_livre}


def escolher_manobra(pts, ang, passos=(0.04, 0.07, 0.10, 0.14, 0.20, 0.28, 0.36), margem=0.0):
    """v3.20 (missao5 29/10): parar, olhar o LiDAR e decidir como chegar ao angulo 'ang' (rad).

    Devolve um de:
      ('girar', ang)        o giro passa (o Collision Monitor nao vai travar);
      ('mover', dir, dist)  andar dist m para frente/tras/esq/dir antes (a menor distancia que
                            libera o giro; lado vale tanto quanto frente/tras);
      ('girar', ang_longo)  so o giro pelo outro lado passa;
      ('mover', dir, d)     nada libera de uma vez: afasta ate 20 cm dos pontos que travam e
                            olha de novo;
      ('preso',)            sem espaco.
    Antes o Nav2 dava re, o DWB voltava ao mesmo canto e tentava o mesmo giro (3 vezes).
    """
    bloq = giro_bloqueado(pts, ang)
    if not bloq:
        return ('girar', ang)
    perto = _fora_do_corpo(pts)
    folgas = {d: folga_direcao(pts, d) for d in DIRECOES}
    melhor = None
    for nome, (ux, uy) in DIRECOES.items():
        for dist in passos:
            if folgas[nome] < dist + FOLGA_MOV:
                break
            movidos = [(x - dist * ux, y - dist * uy) for x, y in perto]
            if not giro_bloqueado(movidos, ang):
                chave = (dist, -folgas[nome])
                if melhor is None or chave < melhor[0]:
                    melhor = (chave, nome, dist)
                break
    if melhor is not None:
        # MANOBRA_FOLGA (12/11): anda um pouco alem do minimo (o robo anda menos que o pedido), se houver folga
        d = melhor[2]
        if margem > 0.0:
            d = max(d, min(d + margem, folgas[melhor[1]] - FOLGA_MOV))
        return ('mover', melhor[1], d)
    longo = ang - math.copysign(2.0 * math.pi, ang)
    if not giro_bloqueado(pts, longo):
        return ('girar', longo)
    # afasta dos pontos que travam (repulsao) e olha de novo
    rx = ry = 0.0
    for x, y in bloq:
        d = max(math.hypot(x, y), 1e-3)
        rx -= x / d ** 2
        ry -= y / d ** 2
    cand = sorted(((rx * ux + ry * uy, nome) for nome, (ux, uy) in DIRECOES.items()
                   if folgas[nome] >= 0.04 + FOLGA_MOV), reverse=True)
    if cand and cand[0][0] > 0.0:
        return ('mover', cand[0][1], round(min(0.20, folgas[cand[0][1]] - FOLGA_MOV), 2))
    return ('preso',)


class TravessiaLateral:
    """Do cruzamento, atravessa o corredor novo andando DE LADO (mecanum), sem girar.

    Estados: assentar -> medir -> (girar | ajustar) -> ... -> deslizar -> fim.
    Durante o deslize o comando de giro fica em zero: quem segura o rumo e o heading hold
    da ponte (IMU). Se o centro ou o angulo sairem da tolerancia, para e corrige parado.
    """

    def __init__(self, lado, v_lateral=0.08, v_ajuste=0.018, giro_min=0.02, giro_max=0.03,
                 tol_giro=math.radians(3.0), tol_x=0.04, para_x=0.05, para_giro=math.radians(5.0),
                 antecipacao_giro=math.radians(2.0), frente_min=0.035, largura_padrao=0.82,
                 correcoes_max=8, tempo_max=90.0, sem_parede_max=0.60, folga_min=0.045,
                 progresso_reset=0.06, seguranca_max=6, log=print, fundido=False, k_ff=0.2):
        self.fundido, self.k_ff = fundido, k_ff   # ANGULO_FUNDIDO
        self.ref, self.ref_ini, self.gyro = None, None, 0.0
        self.lado = 1.0 if lado > 0 else -1.0
        self.v_lat, self.v_aj = v_lateral, v_ajuste
        self.giro_min, self.giro_max = giro_min, giro_max
        self.tol_giro, self.tol_x = tol_giro, tol_x
        self.para_x, self.para_giro = para_x, para_giro
        self.antecipacao = antecipacao_giro
        self.frente_min = frente_min
        self.largura = largura_padrao
        self.correcoes_max = correcoes_max
        self.tempo_max = tempo_max
        self.sem_parede_max = sem_parede_max
        self.log = log
        self.estado = 'assentar'
        self.motivo = ''
        self.t0 = self.t_estado = None
        self.inicio_estado = None
        self.meds = []          # (t, med, pose_no_scan)
        self.correcoes = 0
        self.alvo = None
        self.p_inicio = None    # pose do odom no comeco (para sair de re/lado se abortar)
        self.ult_med_pose = None
        self.restante = 9.0
        self.n_bloq = 0
        self.pulso = 0.0
        # v3.13: o robo real anda ~0,1 m/s com qualquer comando (PWM minimo): o ajuste para
        # frente/tras e feito em pulsos curtos, medindo pelo LiDAR entre um e outro
        # medido no curva3: um pulso de 0,4 s andou ~8 cm (0,2 m/s com rampa e atraso)
        # v3.26 (PWM60, 04/11): a ponte nova obedece o comando: real ~= 0,078 + 0,75 x comando
        # (frente). Com v_ajuste 0,018 isso da ~0,09 m/s (antes ~0,20 com qualquer comando).
        self.v_real = 0.078 + 0.75 * v_ajuste
        self.pulso_max = 0.06 / self.v_real   # no maximo ~6 cm por pulso (como antes: 0,30 s x 0,20)
        self.w_real = 0.26         # rad/s reais com qualquer comando de giro (teste 26/10)
        self.perto_saida = 0.50    # nos ultimos 0,5 m so sai (as paredes estao acabando)
        self.yaw_ref = None        # rumo (IMU) depois do 1o alinhamento pelas paredes
        self.p_desl = None         # pose ao comecar a deslizar (para contar progresso)
        # v3.15: folga real do para-choque/traseira (folgas_lateral) e parada de seguranca
        self.folga_min = folga_min
        self.progresso_reset = progresso_reset
        self.seguranca_max = seguranca_max
        self.n_seg = 0
        self.restante_fora = 0.25  # v3.18: restante (m) com o corpo inteiro fora (margem 0,60)
        self.folga = None          # (t, frente, tras)
        self._folga_crua = None
        self.paredes = (False, False)
        self.p_correcao = None     # pose na ultima correcao (progresso desde ela)
        self.recuo_curva = False   # RECUO_CURVA (12/11): ligado pelo no
        self.n_recuo_curva = 0
        self.pulso_adapta = False  # PULSO_GIRO_ADAPTA (13/11): ligado pelo no (parametro pulso_giro_adapta)
        # ALINHAR_CRUZAMENTO (19/11): antes do 1o deslize, gira pela parede da frente (corredor novo) mesmo com
        # uma parede so; ligado pelo no. Corredores reais ~9 graus fora do esquadro no teste de 19/11.
        self.alinhar_cruz = False
        self.n_cruz = 0
        # ALINHAR_CRUZAMENTO_ZERO (19/11 noite): o giro parava em +5,6 graus (pulso de 0,40 s cortava antes
        # do IMU chegar; 3 tentativas; faixa so acima de 3 graus) e o deslize diagonal levou o CM a parar.
        # Ligado pelo no: ate 6 tentativas, corrige a partir de 1 grau, pulso longo (o IMU para no alvo).
        self.cruz_zero = False
        self.pulso_ganho = 1.0
        self.pulsos_fracos = 0
        self._giro_pend = None     # (yaw no inicio do pulso, giro pedido) ate medir depois do assentar

    def nova_folga(self, t, frente, tras, parede_fr=True, parede_tr=True):
        # v3.18: um scan que cai mais de 3 cm de uma vez vale so a partir do segundo (curva4
        # ida: 2 cm num scan, 12 cm antes e depois); queda gradual vale na hora (missao2:
        # 6,1 -> 5,4 -> 4,9 -> 3,7 -> 1,0 cm; com "maior das 2" a parada vinha um scan tarde)
        def junta(novo, velho):
            if novo is None or velho is None or velho < novo + 0.03:
                return novo
            return velho
        if any(v is not None and v < 0 for v in (frente, tras)):
            self.abortar('folga negativa medida no corpo')
            return
        ult = self._folga_crua
        self._folga_crua = (frente, tras)
        if ult is not None:
            frente, tras = junta(frente, ult[0]), junta(tras, ult[1])
        self.folga = (t, frente, tras)
        self.paredes = (parede_fr, parede_tr)

    def _folgas(self, t):
        """(frente, tras) recentes ou (None, None)."""
        if self.folga is None or t - self.folga[0] > 0.6:
            return None, None
        return self.folga[1], self.folga[2]

    def _centro_folga(self, t):
        """Quanto o centro do corredor esta a frente do centro do corpo, pelas folgas reais.
        So quando as duas paredes estao na faixa do corpo (dentro do corredor)."""
        ff, ft = self._folgas(t)
        if ff is None or ft is None or ff + ft > 0.40 or not all(self.paredes):
            return None
        return (ff - ft) / 2.0

    def _perigo(self, t):
        """Folga abaixo do minimo: devolve o desvio para se afastar (m) ou None."""
        ff, ft = self._folgas(t)
        # v3.25: afasta pelo menos 3 cm (com 1-2 cm o robo ficava perto da parede e nao
        # conseguia girar para alinhar com o corredor novo)
        if ff is not None and ff < self.folga_min:
            c = self._centro_folga(t)
            return min(c, -0.03) if c is not None and c < 0 else -0.04
        if ft is not None and ft < self.folga_min:
            c = self._centro_folga(t)
            return max(c, 0.03) if c is not None and c > 0 else 0.04
        return None

    def nova_medicao(self, t, med, pose):
        if med is None or med['n'] < 6:
            return
        # o robo chega alinhado pela travessia (+-3 graus): angulo grande e parede errada
        # (cruzamento em + ou T com a frente aberta, pontos de outras paredes).
        # v3.25 (missao11): os corredores reais nao sao a 90 graus (o corredor oeste estava
        # ~10 graus torto em relacao ao norte): ate 16 graus vale se a parede for comprida
        # (>= 15 pontos); antes tudo acima de 8 era jogado fora e o robo deslizava "cego"
        if abs(med['angulo']) > math.radians(16) or \
                (abs(med['angulo']) > math.radians(8) and med['n'] < 15):
            return
        if med['lados'] == 2 and med['largura'] is not None and 0.66 < med['largura'] < 1.0:
            self.largura = 0.8 * self.largura + 0.2 * med['largura']
        self.meds.append((t, med, pose))
        self.meds = self.meds[-12:]
        self.ult_med_pose = pose
        self._funde(med, pose)

    def _funde(self, med, pose):
        """ANGULO_FUNDIDO: ref = rumo do deslize na odom, medido parado antes de sair. A IMU segue o
        giro e a parede so ajusta devagar (constante ~0,7 s), com as 2 paredes a vista e no maximo 3 graus
        da medida inicial. No replay da missao22 o LiDAR indica 7-9 graus por muitos scans seguidos (em 18
        de 19 paradas a IMU nao confirmou): seguir a parede sem limite giraria a quina contra ela."""
        if self.ref is None or self.ref_ini is None or med['n'] < 15 or med['lados'] != 2:
            return
        inov = _ang(_ang(pose[2] + med['angulo']) - self.ref)
        if abs(inov) > math.radians(6.0):
            return
        r = _ang(self.ref + 0.25 * inov)
        dif = _ang(r - self.ref_ini)
        lim = math.radians(3.0)
        self.ref = _ang(self.ref_ini + max(-lim, min(lim, dif)))

    def _deslizar_fundido(self, t, pose, restante):
        """Corrige rumo e centro andando. Devolve (vx, vy, wz) ou None se precisa parar e medir."""
        erro = _ang(self.ref - pose[2]) if self.ref is not None else 0.0
        des = self._centro_folga(t)
        novas = [m for m in self.meds if m[0] >= t - 0.6]
        if des is None and novas and novas[-1][1]['lados'] == 2:
            des = novas[-1][1]['desvio']
        # DESLIZE_SEM_GIRO: na missao23 o giro andando nao mudou o rumo (-0,35 graus/s com qualquer comando) e
        # fez a frente encolher mais rapido (1,4 -> 2,4-4 cm/s). O rumo fica com a ponte; passou de 5 graus, gira parado.
        if abs(erro) > math.radians(5.0) and t - self.t_estado > 0.3:
            pode = self.correcoes < self.correcoes_max
            if pode or abs(erro) > math.radians(7.0):
                if pode:
                    self.correcoes += 1
                else:
                    self.n_seg += 1
                    if self.n_seg > self.seguranca_max:
                        self.abortar('rumo saindo demais (%d vezes)' % self.n_seg)
                        return 0.0, 0.0, 0.0
                self.alvo = self.ref
                self.pulso = min(0.40, max(0.05, abs(erro) / self.w_real - 0.06))
                self.log('Rumo saiu %+.1f graus: corrigindo parado' % math.degrees(-erro))
                self._mudar('girar', t, pose)
                return 0.0, 0.0, 0.0
        if des is not None and abs(des) > 0.15 and self.correcoes < self.correcoes_max:
            self.correcoes += 1
            self.log('Corrigindo parado: centro %+.1f cm' % (des * 100))
            return None
        vy = self.lado * self.v_lat
        vx = -self.k_ff * vy   # deslize torto medido: k 0,05-0,37, para a frente indo para a esquerda
        if des is not None:
            vx += max(-0.04, min(0.04, 0.4 * des))
        else:
            # uma parede so: afasta pela folga real
            ff, ft = self._folgas(t)
            if ff is not None and ff < 0.09:
                vx -= 0.03
            elif ft is not None and ft < 0.09:
                vx += 0.03
        # VX_DESLIZE (19/11 noite): direcao do deslize ate ~22 graus da lateral, dentro do poligono lateral do CM
        # (CM_LATERAL65); com vx +0,023 para a direita caia no diag_dir (11 cm a frente) e o CM parava a 12 cm
        lim = 0.4 * abs(vy)
        vx = max(-lim, min(lim, vx))
        if abs(vx) < 0.004:
            vx = 0.0
        elif abs(vx) < 0.016:
            vx = math.copysign(0.016, vx)   # o velocity_smoother zera abaixo de 0,015
        return vx, vy, 0.0

    def _recuo_para_medir(self, t, pose, novas):
        """RECUO_PARA_MEDIR (19/11 20:4x): parado no cruzamento com a parede da frente a 7-10 cm do para-choque
        (12-15 cm do LiDAR, no ponto cego do Delta-2D) vieram 0-7 pontos dela e o ALINHAR_CRUZAMENTO nao girou
        (ida: n7; volta: nenhuma medida, deslizou ~9 graus torto). A 15 cm ela veio com n69. Antes do 1o deslize,
        sem medida boa da frente, recua ate ~15 cm (se ha espaco atras) e mede de novo. Devolve True se recuou."""
        if not (self.cruz_zero and self.alinhar_cruz and self.fundido) or self.p_desl is not None \
                or self.n_cruz >= 6 or getattr(self, 'n_recuo_medir', 0) >= 2:
            return False
        boas = [m for m in novas if m[1]['lados'] == 1 and m[1]['frente'] is not None and m[1]['n'] >= 15]
        if len(boas) >= 3 or [m for m in novas if m[1]['lados'] == 2]:
            return False
        ff, ft = self._folgas(t)
        if ff is None or ff >= 0.13:
            return False
        recuo = 0.15 - ff
        if ft is not None and ft - recuo < 0.06:
            return False
        self.n_recuo_medir = getattr(self, 'n_recuo_medir', 0) + 1
        self.n_cruz += 1
        self.alvo = -recuo
        self.pulso = min(self.pulso_max, max(0.08, recuo / self.v_real))
        self.p_correcao = pose
        self.log('ALINHAR_CRUZAMENTO: RECUO_PARA_MEDIR: parede da frente a %.0f cm (perto demais para o LiDAR, '
                 '%d medida(s) boas): recuo %.0f cm e meco de novo (tras %s cm)' % (
                     ff * 100, len(boas), recuo * 100, '-' if ft is None else '%.0f' % (ft * 100)))
        self._mudar('ajustar', t, pose)
        return True

    def _angulo_cruzamento(self, novas, duas):
        """ALINHAR_CRUZAMENTO (19/11): angulo (rad) da parede da frente para girar antes do 1o deslize, ou None."""
        if not self.alinhar_cruz or not self.fundido or duas or self.p_desl is not None \
                or self.n_cruz >= (6 if self.cruz_zero else 3):
            return None
        fr = [m[1]['angulo'] for m in novas
              if m[1]['lados'] == 1 and m[1]['frente'] is not None and m[1]['n'] >= 15]
        # ALINHAR_CRUZAMENTO_ZERO v2 (19/11 20:00): volta do L com ~11 graus (folgas 8+3 cm num corredor de
        # 0,85 m) e nenhum giro: no modo zero aceita ate 16 graus (o limite do nova_medicao) e espalhamento 2,5
        espalha, teto = (2.5, 16.0) if self.cruz_zero else (1.5, 12.0)
        motivo = None
        if len(fr) < 3:
            motivo = '%d medida(s) da parede da frente (n>=15) de %d' % (len(fr), len(novas))
        elif max(fr) - min(fr) > math.radians(espalha):
            motivo = 'medidas espalhadas %.1f a %.1f graus' % (math.degrees(min(fr)), math.degrees(max(fr)))
        else:
            a = sorted(fr)[len(fr) // 2]
            if not math.radians(1.0 if self.cruz_zero else 3.0) <= abs(a) <= math.radians(teto):
                motivo = 'parede da frente a %+.1f graus (faixa %.0f-%.0f)' % (
                    math.degrees(a), 1.0 if self.cruz_zero else 3.0, teto)
            else:
                return a
        if self.cruz_zero and motivo != getattr(self, '_cruz_motivo', None):
            self._cruz_motivo = motivo
            todas = ', '.join('%+.1f(n%d,%s)' % (math.degrees(m[1]['angulo']), m[1]['n'],
                                                 '-' if m[1]['frente'] is None else '%.0f' % (m[1]['frente'] * 100))
                              for m in novas[-5:])
            self.log('ALINHAR_CRUZAMENTO: sem giro: %s | medidas: %s' % (motivo, todas))
        return None

    def _avaliar_pulso(self, pose):
        """PULSO_GIRO_ADAPTA (13/11): o pulso de giro girou de verdade (yaw da IMU)? Se nao, alonga o proximo."""
        yaw0, pedido = self._giro_pend
        self._giro_pend = None
        if abs(pedido) < math.radians(1.5):
            return
        feito = _ang(pose[2] - yaw0)
        if feito * pedido >= 0.35 * pedido * pedido:
            self.pulsos_fracos = 0
            return
        self.pulsos_fracos += 1
        if self.pulso_ganho < 3.0:
            self.pulso_ganho = min(3.0, self.pulso_ganho * 1.6)
            self.log('PULSO_GIRO_ADAPTA: o pulso girou %+.1f de %+.1f graus: proximo pulso %.1fx mais longo' % (
                math.degrees(feito), math.degrees(pedido), self.pulso_ganho))
        elif self.pulsos_fracos >= 5 and self.correcoes < self.correcoes_max:
            self.correcoes = self.correcoes_max
            self.log('PULSO_GIRO_ADAPTA: %d pulsos seguidos sem girar: paro as correcoes finas de rumo nesta '
                     'saida (acima de 7 graus a seguranca continua)' % self.pulsos_fracos)

    def _mudar(self, e, t, pose):
        if self.pulso_adapta:   # PULSO_GIRO_ADAPTA (13/11)
            if self.estado == 'girar' and e != 'girar' and self.inicio_estado is not None \
                    and self.alvo is not None:
                self._giro_pend = (self.inicio_estado[2], _ang(self.alvo - self.inicio_estado[2]))
            elif self._giro_pend is not None and self.estado != 'girar':
                self._avaliar_pulso(pose)   # mede depois do assentar (o motor ainda embala no fim do pulso)
            if e == 'girar':
                self.pulso = min(1.2, self.pulso * self.pulso_ganho)
        if self.estado == 'deslizar' and e != 'deslizar' and self.p_desl is not None:
            # deslizou antes de parar para corrigir: zera o contador de correcoes
            # (v3.15: 6 cm; com 15 cm o curva4 abortou a 17 cm do fim depois de 88 cm)
            if abs(self._desloc(self.p_desl, pose)[1]) >= self.progresso_reset:
                self.correcoes = 0
                self.n_seg = 0
        if e == 'deslizar':
            self.p_desl = pose
            self.ref = self.ref_ini = self.yaw_ref if self.yaw_ref is not None else pose[2]   # ANGULO_FUNDIDO
        self.estado, self.t_estado, self.inicio_estado = e, t, pose

    @staticmethod
    def _desloc(p0, p1):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s = math.cos(p0[2]), math.sin(p0[2])
        return c * dx + s * dy, -s * dx + c * dy

    def retomar(self, pausa, t, pose):
        """SENSOR_PARADO: sensores voltaram. O tempo parado nao conta e mede tudo de novo."""
        if self.t0 is None or self.estado in ('fim', 'abortar'):
            return
        self.t0 += pausa
        if self.estado == 'aproximar':
            self.t_estado += pausa
            return
        self._mudar('assentar', t, pose)

    def abortar(self, motivo):
        # Falha nao e prova de saida: sucesso so pelo criterio normal de passo().
        self.estado, self.motivo = 'abortar', motivo
        self.log('Saida lateral abortada: %s' % motivo)

    def bloqueio(self, t, pose):
        """Collision Monitor segurando: para, mede e corrige (afasta da parede) antes de desistir."""
        if self.recuo_curva and self.n_recuo_curva < 2 and self.estado not in ('fim', 'abortar', 'ajustar'):
            # RECUO_CURVA (12/11): o poligono lateral do CM vai 5 cm alem do para-choque e da traseira;
            # parado colado na parede/objeto da frente (ou de tras), anda ~5 cm para longe e desliza de novo
            ff, ft = self._folgas(t)
            sentido = 0.0
            # v2 (12/11 12:07, volta): o /scan_nav do CM lembrava pontos a 4 cm do para-choque com a frente
            # 'medida' a 14 cm no /scan_front: perto do corpo a frente e cega. Traseira com espaco: recua.
            if ft is None or ft >= 0.08:
                sentido = -1.0
            elif ff is not None and ff >= 0.10:
                sentido = 1.0
            if sentido:
                self.n_recuo_curva += 1
                self.alvo = 0.05 * sentido
                self.pulso = min(self.pulso_max, 0.05 / self.v_real)
                self.log('RECUO_CURVA: Collision Monitor segurando em "%s" (folga frente %s / tras %s cm): '
                         '%s 5 cm e deslizo de novo (%d/2)' % (
                             self.estado, '-' if ff is None else '%.0f' % (ff * 100),
                             '-' if ft is None else '%.0f' % (ft * 100),
                             'recuo' if sentido < 0 else 'avanco', self.n_recuo_curva))
                self.p_correcao = pose
                self._mudar('ajustar', t, pose)
                return
        self.n_bloq += 1
        if self.n_bloq > 3:
            self.abortar('Collision Monitor bloqueou %d vezes' % self.n_bloq)
            return
        self.log('Collision Monitor segurando em "%s": parando para medir de novo' % self.estado)
        self._mudar('assentar', t, pose)

    def passo(self, t, pose, restante):
        """Devolve (vx, vy, wz). restante: metros (no eixo do corredor novo) ate sair dele."""
        if self.estado in ('fim', 'abortar'):
            return 0.0, 0.0, 0.0
        if self.t0 is None:
            self.t0 = t
            self.p_inicio = pose
            self.yaw_ref = pose[2]  # rumo de chegada (alinhado pela travessia)
            self._mudar('assentar', t, pose)
            self.log('Cruzamento: saindo de lado para a %s' % ('ESQUERDA' if self.lado > 0 else 'DIREITA'))
        if self.estado in ('fim', 'abortar'):
            return 0.0, 0.0, 0.0
        if t - self.t0 > self.tempo_max:
            self.abortar('tempo maximo')
            return 0.0, 0.0, 0.0
        self.restante = restante
        e = self.estado

        if e == 'assentar':
            if t - self.t_estado >= 0.4:
                self._mudar('medir', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'medir':
            novas = [m for m in self.meds if m[0] >= self.t_estado]
            if len(novas) < 3 and t - self.t_estado < 2.0:
                return 0.0, 0.0, 0.0
            perigo = self._perigo(t)
            if perigo is not None and not novas:
                # v3.21 (missao6 03/11): parada de seguranca sem parede do corredor novo a vista:
                # antes voltava a deslizar com o para-choque a 0 cm (30 s empurrando a parede)
                self.n_seg += 1
                if self.n_seg > self.seguranca_max:
                    self.abortar('encostando na parede (%d vezes)' % self.n_seg)
                    return 0.0, 0.0, 0.0
                self.log('Perto da parede (sem medicao): afastando %+.1f cm' % (perigo * 100))
                self.alvo = perigo
                self.pulso = min(self.pulso_max, max(0.08, abs(perigo) / self.v_real))
                self.p_correcao = pose
                self._mudar('ajustar', t, pose)
                return 0.0, 0.0, 0.0
            if self._recuo_para_medir(t, pose, novas):   # RECUO_PARA_MEDIR (19/11 20:4x)
                return 0.0, 0.0, 0.0
            if not novas and restante < 0.75:
                # quase fora: as paredes ja acabaram (saida para area aberta); termina de sair
                self.log('Paredes acabaram perto da saida: terminando de sair')
                self._mudar('deslizar', t, pose)
                return 0.0, 0.0, 0.0
            if not novas:
                # sem parede agora: segura o rumo pela IMU (rumo do ultimo alinhamento) e segue;
                # se andar sem_parede_max de lado sem ver parede, o deslize aborta
                if self.yaw_ref is not None and abs(_ang(self.yaw_ref - pose[2])) > self.tol_giro \
                        and self.correcoes < self.correcoes_max:
                    self.correcoes += 1
                    self.alvo = self.yaw_ref
                    self.pulso = min(0.40, max(0.05, abs(_ang(self.yaw_ref - pose[2])) / self.w_real - 0.06))
                    self.log('Sem parede agora: voltando ao rumo pela IMU (%+.1f graus)' % math.degrees(
                        _ang(self.yaw_ref - pose[2])))
                    self._mudar('girar', t, pose)
                    return 0.0, 0.0, 0.0
                self.log('Sem parede do corredor novo agora: deslizando com o rumo atual')
                self._mudar('deslizar', t, pose)
                return 0.0, 0.0, 0.0
            ang = sorted(m[1]['angulo'] for m in novas)[len(novas) // 2]
            # UMA_PAREDE: angulo so com as 2 paredes. Na missao22, todo angulo > 5 graus veio de uma parede
            # so (298 de 521); na missao23 esses giros, travados pelo CM, gastaram os primeiros 8 s.
            duas = [m for m in novas if m[1]['lados'] == 2]
            if self.fundido:
                ang = sorted(m[1]['angulo'] for m in duas)[len(duas) // 2] if duas else 0.0
            cruz = self._angulo_cruzamento(novas, duas)   # ALINHAR_CRUZAMENTO (19/11)
            ff_c, ft_c = self._folgas(t)
            if cruz is not None and ff_c is not None and ff_c < 0.10 \
                    and not (ft_c is None or ft_c - (0.12 - ff_c) >= 0.08):
                cruz = None   # sem espaco para recuar antes do giro: segue como antes
            if cruz is not None:
                if ff_c is not None and ff_c < 0.10:
                    self.n_cruz += 1
                    self.alvo = -(0.12 - ff_c)
                    self.pulso = min(self.pulso_max, max(0.08, abs(self.alvo) / self.v_real))
                    self.p_correcao = pose
                    self.log('ALINHAR_CRUZAMENTO: parede da frente a %+.1f graus mas so %.0f cm a frente: '
                             'recuo %.0f cm antes de girar' % (math.degrees(cruz), ff_c * 100, -self.alvo * 100))
                    self._mudar('ajustar', t, pose)
                    return 0.0, 0.0, 0.0
                self.n_cruz += 1
                self.correcoes += 1
                self.alvo = _ang(novas[-1][2][2] + cruz)
                if self.cruz_zero:   # ALINHAR_CRUZAMENTO_ZERO: o IMU (alvo) encerra o giro, o tempo e so teto
                    self.pulso = min(1.0, abs(cruz) / self.w_real + 0.25)
                else:
                    self.pulso = min(0.40, max(0.05, abs(cruz) / self.w_real - 0.06))
                self.log('ALINHAR_CRUZAMENTO: parede da frente (corredor novo) a %+.1f graus com uma parede so: '
                         'giro antes de deslizar (%d/%d)' % (math.degrees(cruz), self.n_cruz,
                                                            6 if self.cruz_zero else 3))
                self._mudar('girar', t, pose)
                return 0.0, 0.0, 0.0
            des = sorted(m[1]['desvio'] for m in novas)[len(novas) // 2]
            yaw_scan = novas[-1][2][2]
            fr = [m[1]['frente'] for m in novas if m[1]['frente'] is not None]
            frente = sorted(fr)[len(fr) // 2] if fr else None
            # v3.15: com as duas paredes na faixa do corpo, o centro vem das folgas reais
            ff, ft = self._folgas(t)
            c_folga = self._centro_folga(t)
            tol_x = self.tol_x
            if c_folga is not None:
                des = c_folga
                # v3.22: corredor apertado (folgas somando pouco): centro mais justo
            elif novas[-1][1]['lados'] == 1:
                tol_x = max(self.tol_x, 0.05)   # uma parede so: a largura e suposta
            self.log('Medicao: girar %+.1f graus, centro %+.1f cm a frente (%s)%s' % (
                math.degrees(ang), des * 100,
                'folgas' if c_folga is not None else '%d parede(s)' % novas[-1][1]['lados'],
                (', folga frente %s / tras %s cm' % ('%.0f' % (ff * 100) if ff is not None else '-',
                                                     '%.0f' % (ft * 100) if ft is not None else '-'))))
            perigo = self._perigo(t)
            if perigo is not None:
                # perto demais de uma parede: afasta sempre (nao gasta correcao)
                self.n_seg += 1
                if self.n_seg > self.seguranca_max:
                    self.abortar('encostando na parede (%d vezes)' % self.n_seg)
                    return 0.0, 0.0, 0.0
                self.log('Perto da parede: afastando %+.1f cm' % (perigo * 100))
                self.alvo = perigo
                self.pulso = min(self.pulso_max, max(0.08, abs(perigo) / self.v_real))
                self.p_correcao = pose
                self._mudar('ajustar', t, pose)
                return 0.0, 0.0, 0.0
            pode = self.correcoes < self.correcoes_max
            # perto da parede da frente (ou de tras) o giro e bloqueado pelo Collision Monitor:
            # primeiro afasta, depois gira
            apertado = abs(des) > self.tol_x and (frente is not None and frente < 0.08 or des > 0.07)
            if pode and abs(ang) > self.tol_giro and not apertado:
                self.correcoes += 1
                self.alvo = _ang(yaw_scan + ang)
                # giro em pulso: o robo gira ~15 graus/s com qualquer comando e ainda embala
                self.pulso = min(0.40, max(0.05, abs(ang) / self.w_real - 0.06))
                self._mudar('girar', t, pose)
            elif pode and abs(des) > tol_x:
                self.correcoes += 1
                self.alvo = des
                self.pulso = min(self.pulso_max, max(0.08, abs(des) / self.v_real))
                self._mudar('ajustar', t, pose)
            elif abs(ang) <= self.para_giro and abs(des) <= self.para_x:
                if duas if self.fundido else novas[-1][1]['n'] >= 12:
                    self.yaw_ref = _ang(yaw_scan + ang)
                elif self.fundido:
                    self.yaw_ref = pose[2]
                self._mudar('deslizar', t, pose)
            elif abs(ang) <= math.radians(8):
                # v3.15: sem correcoes, mas sem perigo (folgas >= 3 cm): segue deslizando;
                # a parada de seguranca continua valendo
                self.log('Sem mais correcoes: sigo deslizando (folgas boas)')
                self._mudar('deslizar', t, pose)
            else:
                self.abortar('nao consegui alinhar em %d correcoes' % self.correcoes)
            return 0.0, 0.0, 0.0

        if e == 'girar':
            err = _ang(self.alvo - pose[2])
            if abs(err) < max(math.radians(0.6), self.antecipacao) or t - self.t_estado >= self.pulso:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, math.copysign(self.giro_min, err)

        if e == 'ajustar':
            # um pulso curto (tempo = erro / velocidade real) e para para medir de novo
            if t - self.t_estado >= self.pulso:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return math.copysign(self.v_aj, self.alvo), 0.0, 0.0

        if e == 'deslizar':
            if restante <= 0.0:
                self.estado = 'fim'
                self.log('Saida lateral concluida')
                return 0.0, 0.0, 0.0
            # v3.15: parada de seguranca pela folga real, com ou sem correcoes sobrando
            if t - self.t_estado > 0.3 and self._perigo(t) is not None:
                ff, ft = self._folgas(t)
                # v3.18: a 4,5 cm (era 3): na missao2 a traseira foi de 6,1 a 1,0 cm em 0,4 s
                self.log('PARADA DE SEGURANCA: folga frente %s / tras %s cm' % (
                    '%.0f' % (ff * 100) if ff is not None else '-', '%.0f' % (ft * 100) if ft is not None else '-'))
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            if self.fundido:
                _, dy = self._desloc(self.inicio_estado, pose)
                if abs(dy) > self.sem_parede_max and not [m for m in self.meds if m[0] >= self.t_estado] \
                        and restante > 0.75:
                    self.abortar('andei %.2f m de lado sem ver parede' % abs(dy))
                    return 0.0, 0.0, 0.0
                cmd = self._deslizar_fundido(t, pose, restante)
                if cmd is None:
                    self._mudar('assentar', t, pose)
                    return 0.0, 0.0, 0.0
                return cmd
            novas = [m for m in self.meds if m[0] >= self.t_estado + 0.3]
            if novas:
                tm, med, p_scan = novas[-1]
                # sem correcao pela odom: de lado ela erra x por causa do yaw e da escala
                c_folga = self._centro_folga(t)
                if c_folga is not None:
                    des, lim_x = c_folga, self.para_x
                elif med['lados'] == 2:
                    des, lim_x = med['desvio'], self.para_x
                elif restante > self.perto_saida:
                    des, lim_x = med['desvio'], 0.07   # uma parede so: a largura e suposta
                else:
                    des, lim_x = 0.0, 1.0   # perto da saida, uma parede so: so seguranca e rumo
                if (abs(med['angulo']) > self.para_giro or abs(des) > lim_x) \
                        and self.correcoes < self.correcoes_max:
                    self.log('Corrigindo parado: %+.1f graus, centro %+.1f cm' % (
                        math.degrees(med['angulo']), des * 100))
                    self._mudar('assentar', t, pose)
                    return 0.0, 0.0, 0.0
            else:
                _, dy = self._desloc(self.inicio_estado, pose)
                if abs(dy) > self.sem_parede_max and not [m for m in self.meds if m[0] >= self.t_estado] \
                        and restante > 0.75:
                    self.abortar('andei %.2f m de lado sem ver parede' % abs(dy))
                    return 0.0, 0.0, 0.0
            # rumo pela IMU entre as medidas (a parede pode sumir perto da saida)
            erro_rumo = abs(_ang(self.yaw_ref - pose[2])) if self.yaw_ref is not None else 0.0
            # v3.15: acima de 7 graus a quina do corpo chega na parede: corrige mesmo sem
            # correcoes sobrando (conta como parada de seguranca)
            rumo_perigo = erro_rumo > math.radians(7.0)
            if erro_rumo > self.para_giro and t - self.t_estado > 0.3 \
                    and (self.correcoes < self.correcoes_max or rumo_perigo):
                if self.correcoes < self.correcoes_max:
                    self.correcoes += 1
                else:
                    self.n_seg += 1
                    if self.n_seg > self.seguranca_max:
                        self.abortar('rumo saindo demais (%d vezes)' % self.n_seg)
                        return 0.0, 0.0, 0.0
                self.alvo = self.yaw_ref
                self.pulso = min(0.40, max(0.05, abs(_ang(self.yaw_ref - pose[2])) / self.w_real - 0.06))
                self.log('Rumo saiu %+.1f graus (IMU): corrigindo parado' % math.degrees(_ang(pose[2] - self.yaw_ref)))
                self._mudar('girar', t, pose)
                return 0.0, 0.0, 0.0
            return 0.0, self.lado * self.v_lat, 0.0
        return 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------
# Encostar de frente numa Service Area pelo LiDAR (sem ROS)
# ---------------------------------------------------------------------------
def medir_face(pts, x_min=0.30, x_max=1.30, y_lim=0.70, tol=0.02, referencia=None):
    """Face segmentada; quando ha WS conhecida seleciona pela referencia, nao por densidade."""
    return selecionar_face(pts,referencia,x_min=x_min,x_max=x_max,y_lim=y_lim,tol=tol)


class Encosto:
    def __init__(self, distancia_alvo=0.08, para_choque=0.365, v_aprox=0.02, v_final=0.016,
                 giro_min=0.02, giro_max=0.03, tol_giro=math.radians(2.0),
                 antecipacao=math.radians(2.0), tol_lat=0.015, lateral_cmd=0.02,
                 tempo_max=60.0, log=print, linear=False):
        self.linear = linear   # ENCOSTO_LINEAR
        self.alvo = distancia_alvo
        self.pc = para_choque
        self.v_aprox, self.v_final = v_aprox, v_final
        self.giro_min, self.giro_max = giro_min, giro_max
        self.tol_giro, self.antecip = tol_giro, antecipacao
        self.tol_lat, self.lat_cmd = tol_lat, lateral_cmd
        self.tempo_max = tempo_max
        self.log = log
        self.estado = 'assentar'
        self.motivo = ''
        self.t0 = self.t_estado = None
        self.ini = None
        self.meds = []
        self.ciclos = 0
        self.n_lat = 0
        self.avancos = 0
        self.alvo_giro = self.alvo_lat = self.yaw_ref = None
        self.resultado = None
        self.obstaculo = None
        self.p_dur = 0.15      # duracao do pulso de avanco final (s), ajustada sozinha
        self.p_ult = None
        self.n_pulsos = 0
        # v3.17: lateral continuo (medido pelo LiDAR) para desvios grandes e correcao do
        # centro tambem depois de encostar
        self.lat_continuo_min = 0.02 if linear else 0.04   # desvio (m) a partir do qual desliza sem parar
        self.v_lat_real = 0.09         # m/s reais de lado com comando 0,02-0,025 (curva4)
        self.l_final = False           # correcao lateral feita ja encostado
        self.n_lat_final = 0
        self.g_final = False           # giro de acerto feito ja encostado
        # ENCOSTO_CEGO (09/11, missao28): a menos de ~15 cm da mesa o meio da face cai na zona
        # cega do LiDAR (< 0,146 m): a face aparece partida, sem centro. Perto, nao corrige de lado
        # nem gira; aceita pelo centro medido de longe (face inteira) e pela ultima medicao valida.
        self.folga_cega = 0.15
        self.tol_centro_final = 0.03
        self.tol_ang_cego = math.radians(4.0)
        self.centro_longe = None       # centro (m) medido ao decidir avancar, face inteira
        self.ult_valida = None         # (t, med) da ultima face valida
        self.n_perda = 0               # PERDA_PARA: vezes que parou para medir de novo
        self.t_parado = None           # FACE_FRAGMENTADA: inicio do ultimo assentar
        # RECUO_LATERAL (09/11, missao32): o deslize de lado vira avanco (k 0,3-1,3); antes de deslizar,
        # recua reto ate sobrar folga para o avanco previsto e a face continuar inteira (>= 15 cm)
        self.folga_min_lat = 0.15      # folga minima no fim do deslize (face inteira fora da zona cega)
        self.folga_max_recuo = 0.45    # nao recua alem disso
        self.recuo_max = 0.20          # por recuo
        self.k_lat = 0.5               # avanco por metro de lado (medido a cada deslize)
        self.n_recuo = 0
        self.recuo_alvo = None
        self.lat_ini = None            # (folga, |centro|) no inicio do ultimo deslize
        self.recuo_cego_max = 2        # RECUO_CEGO (24/11): re de 12 cm quando a 1a medicao nao ve a face
        self.n_recuo_cego = 0

    def nova_medicao(self, t, med, pose):
        if med is None:
            # FALHA_SCAN (09/11, missao29): um scan ruim nao apaga as medicoes; a guarda abaixo
            # (ultima face valida com mais de 0,45 s) para o robo se a falha continuar.
            return
        if not all(math.isfinite(med[k]) for k in ('dist', 'ang')):
            self.meds.clear()
            return
        if med['centro'] is not None and not math.isfinite(med['centro']):
            self.meds.clear()
            return
        if med['dist'] < self.pc:
            self.abortar('folga negativa')
            return
        if med is not None:
            self.meds.append((t, med, pose))
            self.meds = self.meds[-12:]
            self.ult_valida = (t, med)   # ENCOSTO_CEGO

    def retomar(self, pausa, t, pose):
        """SENSOR_PARADO: sensores voltaram. O tempo parado nao conta e mede tudo de novo."""
        if self.t0 is None or self.estado in ('fim', 'abortar'):
            return
        self.t0 += pausa
        if self.estado == 'aproximar':
            self.t_estado += pausa
            return
        self._mudar('assentar', t, pose)

    def abortar(self, motivo):
        self.estado, self.motivo = 'abortar', motivo
        self.log('Encosto abortado: %s' % motivo)

    def _final_cego(self, folga, ang, centro_agora=None):
        """ENCOSTO_CEGO: decide o fim do encosto com a face na zona cega do LiDAR."""
        # sem centro medido de longe (nao passou por "Alinhado"), vale o medido agora
        c = self.centro_longe if self.centro_longe is not None else centro_agora
        if not (0 <= folga and abs(folga - self.alvo) <= 0.025):
            self.abortar('ENCOSTO_CEGO: folga %.1f cm fora do alvo %.0f cm' % (folga * 100, self.alvo * 100))
        elif abs(ang) > self.tol_ang_cego:
            self.abortar('ENCOSTO_CEGO: torto %.1f graus encostado' % math.degrees(ang))
        elif c is None or abs(c) > self.tol_centro_final:
            self.abortar('ENCOSTO_CEGO: centro medido de longe %s fora de +-%.0f cm' % (
                'nao medido' if c is None else '%+.1f cm' % (c * 100), self.tol_centro_final * 100))
        else:
            self.estado = 'fim'
            self.log('ENCOSTO_CEGO: encostado a %.1f cm, %.1f graus; centro medido de longe %+.1f cm '
                     '(perto o LiDAR nao ve o meio da face)' % (folga * 100, math.degrees(ang), c * 100))
        return 0.0, 0.0, 0.0

    def bloqueio(self, t, pose):
        """Collision Monitor segurando: perto da mesa e esperado no avanco final."""
        if self.estado in ('avancar', 'pulso'):
            self.log('Collision Monitor parou o avanco: medindo onde ficou')
            self._mudar('assentar_final', t, pose)
        else:
            self.abortar('Collision Monitor bloqueando em "%s"' % self.estado)

    def _mudar(self, e, t, pose):
        self.estado, self.t_estado, self.ini = e, t, pose
        if e in ('assentar', 'assentar_final'):
            self.t_parado = t   # FACE_FRAGMENTADA: hora em que o robo parou para medir

    def _novas(self, desde):
        return [m for m in self.meds if m[0] >= desde]

    @staticmethod
    def _desloc(p0, p1):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s = math.cos(p0[2]), math.sin(p0[2])
        return c * dx + s * dy, -s * dx + c * dy

    def passo(self, t, pose):
        if self.estado in ('fim', 'abortar'):
            return 0.0, 0.0, 0.0
        if self.t0 is None:
            self.t0 = t
            self._mudar('assentar', t, pose)
        if self.estado in ('fim', 'abortar'):
            return 0.0, 0.0, 0.0
        if t - self.t0 > self.tempo_max:
            self.abortar('tempo maximo')
            return 0.0, 0.0, 0.0
        e = self.estado
        # Nenhum movimento pode continuar usando uma face perdida ou antiga.
        # FACE_FRAGMENTADA (09/11): uma face parcial (so distancia/angulo, medidos agora) mantem o
        # avanco RETO ja autorizado com o centro medido de longe; nunca autoriza girar ou deslizar.
        sem_centro = bool(self.meds) and self.meds[-1][1]['centro'] is None and not (
            e in ('avancar', 'pulso') and self.meds[-1][1].get('parcial', False)
            and self.centro_longe is not None)
        if e in ('avancar', 'pulso', 'girar', 'deslocar', 'lat_mover', 'lat_pulso') and (
                not self.meds or not 0 <= t-self.meds[-1][0] <= 0.45   # FALHA_SCAN: era 0.3
                or sem_centro):
            uv = self.ult_valida
            if e in ('avancar', 'pulso') and uv is not None and 0 <= t - uv[0] <= 1.0 \
                    and uv[1]['dist'] - self.pc < self.folga_cega:
                # ENCOSTO_CEGO: a face entrou na zona cega no fim do avanco: para e mede o final
                self.log('ENCOSTO_CEGO: face na zona cega a %.1f cm: paro e confiro o encosto' % (
                    (uv[1]['dist'] - self.pc) * 100))
                self._mudar('assentar_final', t, pose)
                return 0.0, 0.0, 0.0
            if e in ('girar', 'deslocar', 'lat_mover', 'lat_pulso') and self.n_perda < 3:
                # PERDA_PARA (09/11): a face sumiu numa correcao (giro/deslize): para e mede de novo,
                # parado, em vez de abortar o encosto inteiro (missao de 09/11 13:58: 4 abortos assim)
                self.n_perda += 1
                self.log('PERDA_PARA: perdi a face em "%s": paro e meco de novo (%d/3)' % (e, self.n_perda))
                self._mudar('assentar_final' if self.l_final or self.g_final else 'assentar', t, pose)
                return 0.0, 0.0, 0.0
            self.abortar('face ausente ou desatualizada')
            return 0.0, 0.0, 0.0

        if e in ('avancar', 'pulso') and self.obstaculo is not None:
            self.abortar('obstaculo antes da face WS no corredor de avanco')
            return 0.0, 0.0, 0.0

        if e in ('assentar', 'assentar_final'):
            if t - self.t_estado >= 0.4:
                self._mudar('medir' if e == 'assentar' else 'medir_final', t, pose)
            return 0.0, 0.0, 0.0

        if e in ('medir', 'medir_final'):
            novas = [m for m in self._novas(self.t_estado) if 0 <= t-m[0] <= 0.6]
            if e == 'medir':
                # FACE_FRAGMENTADA (09/11): alinhar (girar/deslizar/decidir avancar) so com centro
                # medido; face so plano (parcial) conta apenas na conferencia final (distancia)
                novas = [m for m in novas if m[1]['centro'] is not None]
            if len(novas) < 4 and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            if len(novas) < 2:
                uv = self.ult_valida
                # FACE_FRAGMENTADA (09/11): so vale a ultima face medida com o robo ja parado
                # (0,3 s depois de parar); medida andando nao autoriza sucesso quando a face some.
                if e == 'medir_final' and uv is not None and 0 <= t - uv[0] <= 5.0 \
                        and self.t_parado is not None and uv[0] >= self.t_parado + 0.3 \
                        and uv[1]['dist'] - self.pc < self.folga_cega:
                    # ENCOSTO_CEGO: encostado, a face sumiu na zona cega: vale a ultima medicao
                    return self._final_cego(uv[1]['dist'] - self.pc, uv[1]['ang'])
                if e == 'medir' and self.n_recuo_cego < self.recuo_cego_max and not self.avancos \
                        and self.centro_longe is None:
                    # RECUO_CEGO (24/11): perto demais o meio da face cai na zona cega: recua e mede
                    self.n_recuo_cego += 1
                    self.log('RECUO_CEGO: sem face inteira na 1a medicao (perto demais?): re de 12 cm '
                             'e meco de novo (%d/%d)' % (self.n_recuo_cego, self.recuo_cego_max))
                    self._mudar('recuar_cego', t, pose)
                    return 0.0, 0.0, 0.0
                if e == 'medir' and any(m[1]['centro'] is None for m in self._novas(self.t_estado)):
                    # FACE_FRAGMENTADA (09/11): so o plano (parcial) nao alinha nem autoriza avancar
                    self.abortar('centro da face nao medido (so face parcial, sem as 2 pontas): '
                                 'nao alinho nem avanco')
                    return 0.0, 0.0, 0.0
                self.abortar('nao vejo a face da mesa a frente')
                return 0.0, 0.0, 0.0
            med = lambda k: sorted(m[1][k] for m in novas)[len(novas) // 2]
            dist, ang = med('dist'), med('ang')
            centros = [m[1]['centro'] for m in novas if m[1]['centro'] is not None]
            centro = sorted(centros)[len(centros) // 2] if len(centros) >= 2 else None
            folga = dist - self.pc
            yaw_scan = novas[-1][2][2]
            if e == 'medir_final':
                self.resultado = dict(folga=folga, ang=ang, centro=centro)
                self.log('Encostado: frente a %.1f cm da mesa, %.1f graus, centro %s' % (
                    folga * 100, math.degrees(ang),
                    ('%+.1f cm' % (centro * 100)) if centro is not None else 'nao medido'))
                if folga > self.alvo + 0.025 and self.avancos < 3:
                    self.avancos += 1
                    self.yaw_ref = _ang(yaw_scan + ang)
                    self._mudar('avancar', t, pose)
                    return 0.0, 0.0, 0.0
                if folga < self.folga_cega:
                    # ENCOSTO_CEGO: perto demais para medir o centro/corrigir de lado
                    return self._final_cego(folga, ang, centro)
                # v3.17: encostado mas fora do centro (missao1 29/10: WS_2 ficou a +16,5 cm,
                # porque de longe uma ponta da face nao aparecia). De lado o robo corre
                # paralelo a face, entao da para corrigir aqui mesmo.
                # o deslize de lado entorta o robo 2-3 graus: acerta o angulo uma vez
                if self.l_final and not self.g_final and abs(ang) > math.radians(2.5) and folga > 0.05:
                    self.g_final = True
                    self.alvo_giro = _ang(yaw_scan + ang)
                    self._mudar('girar', t, pose)
                    return 0.0, 0.0, 0.0
                if centro is not None and abs(centro) > 0.03 and self.n_lat_final < 2 \
                        and folga > 0.035:
                    self.n_lat_final += 1
                    self.l_final = True
                    self.alvo_lat = centro
                    self.log('Encostado fora do centro (%+.1f cm): corrigindo de lado' % (centro * 100))
                    self._mudar('deslocar', t, pose)
                    return 0.0, 0.0, 0.0
                if not (0 <= folga and abs(folga-self.alvo) <= 0.025
                        and abs(ang) <= self.tol_giro
                        and centro is not None and abs(centro) <= self.tol_lat):
                    self.abortar('medicao final fora dos limites de folga/alinhamento')
                    return 0.0, 0.0, 0.0
                self.estado = 'fim'
                return 0.0, 0.0, 0.0
            self.ciclos += 1
            self.log('Medicao %d: face a %.0f cm (para-choque), girar %+.1f graus, centro %s' % (
                self.ciclos, folga * 100, math.degrees(ang),
                ('%+.1f cm' % (centro * 100)) if centro is not None else 'nao visto'))
            if folga < 0.03:
                self.abortar('ja esta perto demais da mesa (%.0f cm)' % (folga * 100))
                return 0.0, 0.0, 0.0
            if self.lat_ini is not None and centro is not None:
                # RECUO_LATERAL: k medido pelo LiDAR no ultimo deslize
                dc = self.lat_ini[1] - abs(centro)
                if dc > 0.03:
                    self.k_lat = min(1.5, max(0.2, (self.lat_ini[0] - folga) / dc))
                    self.log('RECUO_LATERAL: o deslize de %.1f cm avancou %.1f cm (k %.2f)' % (
                        dc * 100, (self.lat_ini[0] - folga) * 100, self.k_lat))
                self.lat_ini = None
            pode = self.ciclos <= 10
            if pode and abs(ang) > self.tol_giro:
                self.alvo_giro = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif (pode and centro is not None and abs(centro) > self.tol_lat and self.n_lat < 5
                  and self.n_recuo < 3 and folga < self.folga_max_recuo - 0.03
                  and folga - self.k_lat * abs(centro) < self.folga_min_lat):
                # RECUO_LATERAL: o deslize ia levar a face para a zona cega: recua reto antes
                self.n_recuo += 1
                precisa = self.folga_min_lat + self.k_lat * abs(centro) + 0.02
                self.recuo_alvo = min(precisa, self.folga_max_recuo, folga + self.recuo_max)
                self.yaw_ref = _ang(yaw_scan + ang)
                self.log('RECUO_LATERAL: centro %+.1f cm com a face a %.0f cm: o deslize avanca ~%.0f cm '
                         '(k %.2f); recuo reto ate %.0f cm (%d/3)' % (
                             centro * 100, folga * 100, self.k_lat * abs(centro) * 100, self.k_lat,
                             self.recuo_alvo * 100, self.n_recuo))
                self._mudar('recuar', t, pose)
            elif pode and centro is not None and abs(centro) > self.tol_lat and self.n_lat < 5:
                self.n_lat += 1
                self.alvo_lat = centro
                self.lat_ini = (folga, abs(centro))   # RECUO_LATERAL
                self._mudar('deslocar', t, pose)
            elif abs(ang) > math.radians(3.0) or (centro is not None and abs(centro) > 0.04):
                self.abortar('nao consegui alinhar com a mesa (%.1f graus, centro %+.1f cm)' % (
                    math.degrees(ang), (centro or 0.0) * 100))
            else:
                self.yaw_ref = _ang(yaw_scan + ang)
                self.centro_longe = centro   # ENCOSTO_CEGO
                self.log('Alinhado com a mesa: avancando ate %.0f cm' % (self.alvo * 100))
                self._mudar('avancar', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'girar':
            err = _ang(self.alvo_giro - pose[2])
            if abs(err) < max(math.radians(0.6), self.antecip) or t - self.t_estado > 8.0:
                self._mudar('assentar_final' if self.g_final and self.estado == 'girar' and self.l_final
                            else 'assentar', t, pose)
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, _sat(0.5 * err, self.giro_min, self.giro_max)

        if e == 'deslocar':
            # Lateral em pulsos medidos pelo LiDAR: com comando pequeno o robo desliza ~15 cm/s
            # e passava do centro (teste de 22/10 09:36: -5 cm virou +5 cm num deslize so).
            self.l_ult = None
            self.l_pulsos = 0
            self.l_dur = 0.15
            self.l_falta = self.alvo_lat
            self.l_yaw = pose[2]
            if abs(self.alvo_lat) >= self.lat_continuo_min:
                # v3.17: desvio grande vai de uma vez, parando pelo LiDAR (missao1: 15 cm
                # levaram 40 s em pulsos de 0,15-0,3 s e o encosto abortou)
                self.l_tmax = min(4.0, abs(self.alvo_lat) / (0.5 * self.v_lat_real) + 0.8)
                self.log('Deslizando %+.1f cm de lado' % (self.alvo_lat * 100))
                self._mudar('lat_mover', t, pose)
            else:
                self._mudar('lat_pulso', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'lat_mover':
            fim_lat = 'assentar_final' if self.l_final else 'assentar'
            novas = [m for m in self._novas(self.t_estado + 0.25) if m[1]['centro'] is not None]
            if novas:
                tl, med_l, _ = novas[-1]
                c = med_l['centro']
                v = self.v_lat_real
                if len(novas) >= 2 and tl - novas[-2][0] > 0.05:
                    v = max(0.0, (abs(novas[-2][1]['centro']) - abs(c)) / (tl - novas[-2][0]))
                previsto = abs(c) - v * (t - tl + 0.30)   # 0,30 s = atraso + parada
                if c * self.l_falta <= 0 or previsto <= 0.5 * self.tol_lat:
                    self._mudar(fim_lat, t, pose)
                    return 0.0, 0.0, 0.0
                if (not self.l_final and self.n_recuo < 3
                        and med_l['dist'] - self.pc < self.folga_min_lat - 0.01):
                    # RECUO_LATERAL: o deslize trouxe a face para perto: para e mede (pode recuar)
                    self.log('RECUO_LATERAL: face a %.1f cm no deslize, centro %+.1f cm: paro e meco' % (
                        (med_l['dist'] - self.pc) * 100, c * 100))
                    self._mudar(fim_lat, t, pose)
                    return 0.0, 0.0, 0.0
            elif t - self.t_estado > abs(self.l_falta) / self.v_lat_real:
                # sem ver as pontas da face: para pelo tempo (velocidade real conhecida)
                self._mudar(fim_lat, t, pose)
                return 0.0, 0.0, 0.0
            if t - self.t_estado > self.l_tmax:
                self._mudar(fim_lat, t, pose)
                return 0.0, 0.0, 0.0
            err = _ang(self.l_yaw - pose[2])
            wz = _sat(0.5 * err, 0.016, 0.03) if abs(err) > math.radians(0.8) else 0.0
            return 0.0, math.copysign(self.lat_cmd, self.l_falta), wz

        if e == 'lat_pulso':
            if t - self.t_estado >= self.l_dur:
                self._mudar('lat_esperar', t, pose)
                return 0.0, 0.0, 0.0
            err = _ang(self.l_yaw - pose[2])
            wz = _sat(0.5 * err, 0.016, 0.03) if abs(err) > math.radians(0.8) else 0.0
            return 0.0, math.copysign(self.lat_cmd, self.l_falta), wz

        if e == 'lat_esperar':
            novas = [m for m in self._novas(self.t_estado + 0.2) if m[1]['centro'] is not None]
            if (len(novas) < 2 or t - self.t_estado < 0.5) and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            self.l_pulsos += 1
            fim_lat = 'assentar_final' if self.l_final else 'assentar'
            if not novas or self.l_pulsos >= 8:
                self._mudar(fim_lat, t, pose)
                return 0.0, 0.0, 0.0
            c = sorted(m[1]['centro'] for m in novas[-3:])[len(novas[-3:]) // 2]
            if abs(c) < self.tol_lat:
                self._mudar(fim_lat, t, pose)
                return 0.0, 0.0, 0.0
            if c * self.l_falta < 0:
                self.l_dur = max(0.10, self.l_dur * 0.5)  # passou do centro: volta com pulso menor
            else:
                andou = abs(self.l_falta) - abs(c)
                if andou > 0.003:
                    novo = 0.5 * abs(c) / (andou / self.l_dur)
                else:
                    novo = self.l_dur * 1.4
                self.l_dur = min(max(novo, 0.10), self.l_dur * 1.4, 0.30)
            self.l_falta = c
            self._mudar('lat_pulso', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'recuar_cego':   # RECUO_CEGO (24/11): re reta pela odom
            dx, _ = self._desloc(self.ini, pose)
            if -dx >= 0.12 or t - self.t_estado > 5.0:
                self._mudar('assentar', t, pose)
                return 0.0, 0.0, 0.0
            return -self.v_aprox, 0.0, 0.0

        if e == 'recuar':
            # RECUO_LATERAL: re reta segurando o rumo da face; para pelo LiDAR (folga prevista)
            novas = self._novas(self.t_estado)
            if novas:
                tl, med_r, _ = novas[-1]
                folga = med_r['dist'] - self.pc
                v = 0.0
                if len(novas) >= 2 and tl - novas[-2][0] > 0.05:
                    v = max(0.0, (folga - (novas[-2][1]['dist'] - self.pc)) / (tl - novas[-2][0]))
                if folga + v * (t - tl + 0.35) >= self.recuo_alvo:
                    self._mudar('assentar', t, pose)
                    return 0.0, 0.0, 0.0
            elif t - self.t_estado > 1.0:
                self._mudar('assentar', t, pose)   # sem face: para e o 'medir' decide
                return 0.0, 0.0, 0.0
            dx, _ = self._desloc(self.ini, pose)
            if t - self.t_estado > 6.0 or -dx > self.recuo_max + 0.10:
                self._mudar('assentar', t, pose)   # guarda pela odom/tempo
                return 0.0, 0.0, 0.0
            err = _ang(self.yaw_ref - pose[2])
            wz = _sat(0.4 * err, 0.016, 0.03) if abs(err) > math.radians(1.0) else 0.0
            return -self.v_aprox, 0.0, wz

        if e == 'avancar':
            # Avanco continuo ate ~6 cm antes do alvo, freando pelo LiDAR (o odom nao serve:
            # com comando pequeno o robo anda ate 2x o que o odom diz - teste de 22/10 08:45).
            novas = self._novas(self.t_estado)
            if novas:
                tl, med, _ = novas[-1]
                folga = med['dist'] - self.pc
                v = 0.0
                if len(novas) >= 2:
                    t0_, m0, _ = novas[-2]
                    if tl - t0_ > 0.05:
                        v = max(0.0, ((m0['dist'] - self.pc) - folga) / (tl - t0_))
                previsto = folga - v * (t - tl + 0.35)  # 0.35 s = atraso + parada
            elif t - self.t_estado < 1.0:
                # ENCOSTO_ESPERA (09/11, missao27): no 1o passo do avanco o scan novo ainda nao
                # chegou (5,9 Hz) e abortava 50 ms depois de alinhar. Espera parado ate 1 s.
                return 0.0, 0.0, 0.0
            else:
                self.abortar('perdi a face da mesa')
                return 0.0, 0.0, 0.0
            # ENCOSTO_LINEAR: vai direto ao alvo (0,10 m/s reais; a previsao para ~1 cm depois do alvo)
            if previsto <= self.alvo + (0.0 if self.linear else 0.08) or t - self.t_estado > 15.0:
                self.p_ult = None
                self.n_pulsos = 0
                self._mudar('esperar', t, pose)
                return 0.0, 0.0, 0.0
            err = _ang(self.yaw_ref - pose[2])
            wz = _sat(0.4 * err, 0.016, 0.03) if abs(err) > math.radians(1.0) else 0.0
            return self.v_aprox, 0.0, wz

        if e == 'esperar':
            # parado: mede a folga com 2 leituras novas e decide o proximo pulso
            novas = self._novas(self.t_estado + 0.2)
            if (len(novas) < 2 or t - self.t_estado < 0.5) and t - self.t_estado < 3.0:
                return 0.0, 0.0, 0.0
            if not novas:
                self.abortar('perdi a face da mesa')
                return 0.0, 0.0, 0.0
            folga = sorted(m[1]['dist'] for m in novas[-3:])[len(novas[-3:]) // 2] - self.pc
            falta = folga - self.alvo
            if falta <= (0.02 if self.linear else 0.012) or self.n_pulsos >= (2 if self.linear else 12):
                self._mudar('assentar_final', t, pose)
                return 0.0, 0.0, 0.0
            if self.p_ult is not None:
                andou = self.p_ult - folga
                # o robo quase nao anda abaixo de ~0.15 s e depois dispara (pulso de 0.40 s
                # andou 6.6 cm no teste de 09:36): cresce no maximo 40% por vez, ate 0.30 s
                if andou > 0.003:
                    taxa = andou / self.p_dur
                    novo = 0.5 * falta / taxa
                else:
                    novo = self.p_dur * 1.4
                self.p_dur = min(max(novo, 0.10), self.p_dur * 1.4, 0.30)
            self.p_ult = folga
            self.n_pulsos += 1
            self.log('Folga %.1f cm: pulso %d de %.2f s' % (folga * 100, self.n_pulsos, self.p_dur))
            self._mudar('pulso', t, pose)
            return 0.0, 0.0, 0.0

        if e == 'pulso':
            if t - self.t_estado >= self.p_dur:
                self._mudar('esperar', t, pose)
                return 0.0, 0.0, 0.0
            return self.v_final, 0.0, 0.0
        return 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------
# No ROS 2
# ---------------------------------------------------------------------------

def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from nav2_msgs.action import ComputePathToPose, DriveOnHeading, NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState
    from sensor_msgs.msg import Imu, LaserScan  # SENSOR_PARADO
    from std_msgs.msg import String, Bool
    from std_srvs.srv import SetBool, Trigger
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import ParameterDescriptor
    from tf2_ros import Buffer, TransformListener

    def yaw_de(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    class SensorParado(Exception):  # SENSOR_PARADO
        pass

    class Cancelado(Exception):
        pass

    class Alinhador(Node):
        def __init__(self):
            super().__init__('alinhador_passagem')
            self.fator_corredor = self.declare_parameter(
                'fator_velocidade_corredor', 0.5, ParameterDescriptor(read_only=True)).value
            if not 0.0 < self.fator_corredor <= 1.0:
                raise ValueError('fator_velocidade_corredor deve estar entre 0 (exclusive) e 1')
            self.em_preparo = False
            self.zona_morta_original = None
            # NAV2_PRINCIPAL: opt-in; legado preservado para A/B.
            self.declare_parameter('nav2_principal', False, ParameterDescriptor(read_only=True))
            self.declare_parameter('supervisor_regioes', os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/config/regioes_passagem.yaml'))
            self.declare_parameter('supervisor_modo', 'observar', ParameterDescriptor(read_only=True))
            from supervisor_passagem import Passagens
            self.passagens_supervisor = Passagens()
            self.supervisor = None
            self.evidencia_passagem = None
            self.imu_supervisor = None
            self.dono_mov = None
            self.nav_pendente = None
            self.nav_resultado = None
            self.declare_parameter('topico_goal', '/goal_pose_alinhado')
            self.declare_parameter('largura_estreita', 0.90)
            self.declare_parameter('distancia_preparo', 0.60)
            # PREPARO_LATERAL (19/11): deslocamento maximo da pose de preparo quando o eixo nao cabe (0 = como antes)
            self.declare_parameter('preparo_lateral_max', 0.15)
            self.declare_parameter('distancia_saida', 0.45)
            self.declare_parameter('planner_id', 'GridBased')
            # VIRAR_REPLANEJA (13/11): pre-giro tambem sem caminho do planner (replaneja / rota do mapa validada)
            self.declare_parameter('virar_replaneja', True)
            self.declare_parameter('max_passagens', 4)
            self.declare_parameter('passagem_largura_minima', 0.40)   # PASSAGEM_IMPOSSIVEL (25/11); 0 = como antes
            self.declare_parameter('passagem_replanejar_max', 2)      # PASSAGEM_IMPOSSIVEL; MEMORIA_BARREIRA: era 5
            self.declare_parameter('periodo_vigia', 1.0)
            # v3: travessia pelo LiDAR ('lidar') ou pelo Nav2 como na v2 ('nav2')
            self.declare_parameter('modo_travessia', 'lidar')
            self.declare_parameter('modo_curva_t', 'lateral')
            self.declare_parameter('limite_vel_angular_real_t', -1.0)
            self.declare_parameter('limite_vel_linear_real_t', -1.0)
            self.declare_parameter('topico_scan_temporal_t', '/scan_temporal_front')
            self.declare_parameter('topico_imu_temporal_t', '/imu/data_temporal')
            if self.get_parameter('modo_curva_t').value != 'lateral':
                raise ValueError('Giro frontal suspenso: use modo_curva_t:=lateral')
            self.declare_parameter('sensor_idade_max', 0.5)     # SENSOR_PARADO
            self.declare_parameter('sensor_espera_max', 60.0)
            self.declare_parameter('sensor_espera_ponte', 6.0)
            self.declare_parameter('angulo_fundido', True)      # ANGULO_FUNDIDO
            self.declare_parameter('fundido_k_ff', 0.2)
            self.declare_parameter('freio_curva_tol', -1.0)   # FREIO_CURVA (12/11): -1 auto, 0 desligado
            self.declare_parameter('entrada_rapida', True)      # ENTRADA_RAPIDA
            self.declare_parameter('recuo_curva', True)   # RECUO_CURVA (12/11): false = como antes
            # ALINHAR_CRUZAMENTO (19/11): gira pela parede da frente antes do 1o deslize da curva (false = como antes)
            self.declare_parameter('alinhar_cruzamento', True)
            # ALINHAR_CRUZAMENTO_ZERO (19/11 noite): leva o angulo a ~0 antes do deslize (false = como antes)
            self.declare_parameter('alinhar_cruzamento_zero', True)
            # PRE_GIRO_REFAZ (19/11 21:0x): o Nav2 FALHOU (abortou, nao cancelou) para o alvo -> a proxima tentativa
            # vira de novo antes (teste 20:58: saindo da WS_2 o 2o pedido pulou o giro de +46 graus pelo
            # PRE_GIRO_UNICO e bateu a frente a 0 cm; o 3o, com giro, foi). false = como antes
            self.declare_parameter('pre_giro_refaz_falha', True)
            # PULSO_GIRO_ADAPTA (13/11): pulso de giro que nao girou fica mais longo (padrao false: 1 mudanca/teste)
            self.declare_parameter('pulso_giro_adapta', False)
            self.declare_parameter('uma_parede', True)          # UMA_PAREDE
            self.declare_parameter('manobra_margem', 0.06)   # MANOBRA_FOLGA (12/11): 0 = como antes
            self.declare_parameter('rota_mapa', True)           # ROTA_MAPA
            self.declare_parameter('planner_livre', True)       # PLANNER_LIVRE (25/11)
            self.declare_parameter('memoria_barreira', True)    # MEMORIA_BARREIRA (25/11)
            self.declare_parameter('caixa_lateral', True)       # CAIXA_LATERAL (25/11)
            self.declare_parameter('rota_retas', True)          # ROTA_RETAS (25/11): retas nos eixos; Nav2 de reserva
            self.declare_parameter('saida_caixa', 0.70)         # SAIDA_CAIXA (25/11): m reto para fora do START; 0 desliga
            # ROTA_MAPA_OBSTACULO (11/11): ignorar | registrar | recusar (padrao ignorar = como antes)
            self.declare_parameter('rota_mapa_obstaculo', 'ignorar')
            # PASSAGEM_BLOQUEADA (11/11): objeto que barrou a travessia vale por N s (0 = como antes)
            self.declare_parameter('passagem_bloqueada_s', 120.0)
            self.declare_parameter('passagem_bloqueada_raio', 0.35)
            self.bloqueios = []          # [(x, y, raio, t)]
            self.trav_motivo, self.trav_obstaculo = '', []
            self.declare_parameter('vel_aproximacao', 0.03)   # comando (PWM60: real ~0,10 m/s)
            self.declare_parameter('vel_travessia', 0.06)   # VEL_ZONAS: era 0.035
            self.declare_parameter('margem_saida', 0.30)      # centro do robo alem da saida
            self.declare_parameter('topico_scan', '/scan_front')
            # se a travessia pelo LiDAR falhar 2 vezes: True = tenta pelo Nav2 (v2),
            # False = para fora da passagem e avisa (o DWB dentro da passagem so piora)
            self.declare_parameter('fallback_nav2', False)
            self.declare_parameter('largura_passagem', 0.82)  # usada quando so uma parede aparece
            self.declare_parameter('curto_max', 1.0)           # v3.23: trajeto curto sem Nav2 (m; 0 desliga)
            # GIRO_PERTO (25/11): goal sem area a <= isto do robo so gira (RViz); 0 = como antes
            self.declare_parameter('so_girar_perto', 0.15)
            # CURTO_SEM_PROGRESSO (13/11): tentativa que nao aproximou 2 cm nao se repete (padrao false: 1 mudanca/teste)
            self.declare_parameter('curto_sem_progresso', False)
            self.declare_parameter('desvio_lidar', False)     # v3.29: DESVIO da v3.27 (desligado: a arvore do Nav2 destrava)
            self.declare_parameter('virar_caminho_graus', 45.0)  # v3.30 VIRAR_CAMINHO: vira antes do Nav2 (0 desliga)
            # PRE_GIRO_UNICO (11/11): nao repete o pre-giro para o mesmo alvo por N s (0 = como antes)
            self.declare_parameter('pre_giro_repetir_s', 20.0)
            self.pre_giro_memo = None   # (x, y, t) do ultimo alvo com pre-giro tentado
            self.declare_parameter('anti_ciscado', True)      # v3.31 ANTI_CISCADO
            self.declare_parameter('giro_final_alinhador', True)  # v3.32 GIRO_FINAL nas mesas
            # GIRO_FINAL_COMUM (11/11): goal sem area tambem com o giro final pelo alinhador (padrao: nao)
            self.declare_parameter('giro_final_comum', False)
            # REINSISTIR (11/11): Nav2 desistiu do GOAL FINAL -> reanalisa, vira e manda de novo (0 desliga)
            self.declare_parameter('reinsistir_goal_final', 2)
            # CURVA_INSISTE (11/11): saida lateral falhou -> repete ate N vezes e depois segue pelo Nav2
            self.declare_parameter('insistir_curva', 2)
            self.declare_parameter('curva_segue_nav2', True)
            self.declare_parameter('ciscado_janela', 4.0)     # s
            self.declare_parameter('ciscado_inversoes', 4)    # trocas de sentido (giro + lado) na janela
            self.declare_parameter('ciscado_max', 3)          # intervencoes por goal
            # CISCADO_PROGRESSO (11/11): da 2a intervencao em diante, so reenvia se o destravar resolveu
            self.declare_parameter('ciscado_exige_destravar', True)
            # CONTROLADOR_ORFAO_ALINHADOR (11/11): FollowPath que sobra depois do NavigateToPose terminar
            self.declare_parameter('cancelar_controlador_orfao', True)
            # v3.5: ir ate uma Service Area e encostar de frente
            self.declare_parameter('arquivo_areas', os.path.expanduser('~/mapas/service_areas.yaml'))
            self.declare_parameter('distancia_encosto', 0.09)   # ENCOSTO_LINEAR: era 0.08; o CM trava em 9,5 cm
            self.declare_parameter('encosto_linear', True)
            self.declare_parameter('encosto_recuo_cego', True)  # RECUO_CEGO (24/11)
            self.declare_parameter('entrada_translada', True)  # ENTRADA_TRANSLADA (24/11)
            self.declare_parameter('finish_chegada', True)     # FINISH_CHEGADA (25/11): sem mesa entra de frente
            self.declare_parameter('finish_perpendicular', True)   # FINISH_PERPENDICULAR (25/11)
            self.declare_parameter('raio_ponto_giro', 0.30)   # ENTRADA_ASSUME (25/11): 0 = como antes
            self.declare_parameter('giro_na_curva', True)     # GIRO_NA_CURVA (25/11)
            self.declare_parameter('corrige_pose_mesa', True)  # CORRIGE_POSE_MESA (24/11)
            # BASELINE_LEGADO (11/11): True = face escolhida pela referencia do mapa (09/11); False = so LiDAR (missao24)
            self.declare_parameter('encosto_referencia_mapa', False)
            self.declare_parameter('para_choque', 0.365)        # frente do robo em base_link (m)
            self.declare_parameter('recuo_saida', 0.25)         # re ao sair da mesa (odom)
            # v3.11: cruzamentos. Na curva o robo sai de lado (mecanum) pelo corredor novo.
            self.declare_parameter('vel_lateral', 0.08)         # v3.26: comando; real ~0,083 m/s de lado (PWM60)
            # v3.17: 0,40 -> 0,60: com 0,40 a quina do robo ainda pegava a ponta da parede ao
            # girar e o Collision Monitor travou o giro 30-60 s (missao1 29/10)
            self.declare_parameter('margem_saida_lateral', 0.60)  # centro alem da saida do corredor
            self.declare_parameter('abertura_saida_curva', 0.70)  # metros livres a partir do para-choque ou traseira
            self.declare_parameter('centro_corpo', 0.08)        # centro do corpo a frente do base_link
            self.area_pendente = None
            self.encostado = False
            self.declare_parameter('arquivo_pontos', os.path.expanduser('~/mapas/pontos.yaml'))
            self.sem_encosto = set()
            # v3.10: chegada em ponto sem mesa: aceita quando fica tempo_chegada s a menos de
            # raio_chegada m do ponto, sem esperar o Nav2 acertar o angulo
            self.declare_parameter('raio_chegada', 0.10)
            # CHEGADA_PERTO (13/11): Nav2 falhou a <= chegada_perto_raio do goal -> o alinhador termina
            self.declare_parameter('chegada_perto', True)
            self.declare_parameter('chegada_perto_raio', 0.15)
            self.declare_parameter('tempo_chegada', 1.5)
            grupo = ReentrantCallbackGroup()
            self.cm_stop = False
            self.odom = None          # (t, x, y, yaw)
            self.scan = None          # (t, pontos em base_link, pose do odom no scan)
            self.scan_seq = 0
            self.laser_tf = None      # (x, y, yaw) do laser em base_link
            self.cmd_final = (0.0, False)  # (t, se o /cmd_vel final esta zerado)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.rota_tent = {}   # ROTA_MAPA
            self.tempos_t = {}
            self.ultimos_stamps_t = {}
            self.odom_t_valido = False
            self.imu_t = None
            self.scan_t_raw = None
            self.controle_t_ativo = False
            self.cancelamento_t = threading.Event()
            self.conflito_t = (0.0, True)
            self.t_imu = None   # SENSOR_PARADO
            self.gyro_z = 0.0
            self.create_subscription(Imu, self.p('topico_imu_temporal_t'),
                                     lambda msg: self.ao_imu(msg, experimental=True),
                                     qos_profile_sensor_data, callback_group=grupo)
            self.create_subscription(LaserScan, self.p('topico_scan_temporal_t'),
                                     lambda msg: self.ao_scan(msg, experimental=True),
                                     qos_profile_sensor_data, callback_group=grupo)
            self.create_subscription(Imu, '/imu/data', self.ao_imu, qos_profile_sensor_data,
                                     callback_group=grupo)
            self.create_subscription(Odometry, '/odom', self.ao_odom, 20, callback_group=grupo)
            self.create_subscription(LaserScan, self.get_parameter('topico_scan').value,
                                     self.ao_scan, qos_profile_sensor_data, callback_group=grupo)
            self.create_subscription(TwistStamped, '/cmd_vel', self.ao_cmd_final, 20,
                                     callback_group=grupo)
            self.ctrl_nz_t = None   # CONTROLADOR_ORFAO_ALINHADOR: ultimo comando nao nulo do controlador
            self.create_subscription(TwistStamped, '/cmd_vel_ctrl', self.ao_cmd_ctrl, 20, callback_group=grupo)
            from action_msgs.srv import CancelGoal
            self.cli_cancel_fp = self.create_client(CancelGoal, '/follow_path/_action/cancel_goal',
                                                    callback_group=grupo)
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                     self.ao_cm, 10, callback_group=grupo)
            self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
            self.mov_cli = self.create_client(SetBool, '/navegacao/especial', callback_group=grupo)
            self.pub_heartbeat = self.create_publisher(TwistStamped, '/navegacao/heartbeat', 1)
            self.create_timer(0.1, self._heartbeat_mov, callback_group=grupo)
            self.pub_t = self.create_publisher(TwistStamped, '/cmd_vel_nav_t', 10)
            self.controle_t = self.create_client(SetBool, '/alinhador/controle_t', callback_group=grupo)
            self.cm_params_t = self.create_client(GetParameters, '/collision_monitor/get_parameters', callback_group=grupo)
            self.ganho_nav2 = self.create_client(SetParametersAtomically,
                '/suavizador_nav/set_parameters_atomically', callback_group=grupo)
            self.consulta_suavizador = self.create_client(GetParameters,
                '/velocity_smoother/get_parameters', callback_group=grupo)
            self.ajuste_suavizador = self.create_client(SetParametersAtomically,
                '/velocity_smoother/set_parameters_atomically', callback_group=grupo)
            self.hw_params_t = self.create_client(GetParameters, '/robo_hardware/get_parameters', callback_group=grupo)
            self.pub_estado_t = self.create_publisher(String, '/alinhador/estado_t', 20)
            self.create_subscription(String, '/hardware/sensor_timing', self.ao_timing_t, 20, callback_group=grupo)
            self.create_service(Trigger, '/navegacao/cancelar', self.cancelar_t, callback_group=grupo)
            self.create_service(Trigger, '/alinhador/cancelar_t', self.cancelar_t, callback_group=grupo)
            self.create_subscription(Bool, '/alinhador/conflito_t', self.ao_conflito_t, 10,
                                     callback_group=grupo)
            # v3.31 ANTI_CISCADO: historico do comando do Nav2 e da odom; cliente do destravar
            self.hist_cmd = collections.deque(maxlen=300)
            self.hist_odom = collections.deque(maxlen=600)
            self.create_subscription(TwistStamped, '/cmd_vel_nav', self.ao_cmd_nav, 20,
                                     callback_group=grupo)
            self.destravar_cli = ActionClient(self, DriveOnHeading, 'destravar', callback_group=grupo)
            # v3.8: resultado de cada /ir_para_area, para missoes nao dependerem dos logs:
            #   "WS_1 SUCESSO folga=8.2cm ang=+0.5 centro=+1.0cm"  ou  "WS_1 FALHOU <motivo>"
            self.pub_resultado = self.create_publisher(String, '/ir_para_area/resultado', 10)
            self.res_area = None
            self.lock = threading.Lock()
            self.mapa_msg = None
            self.plano = None
            self.plano_t = 0.0
            self.goal_pendente = None
            self.evento = threading.Event()
            self.handle_ativo = None
            qos_mapa = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, '/map', self.ao_mapa, qos_mapa,
                                     callback_group=grupo)
            self.create_subscription(Path, '/plan', self.ao_plano, 5, callback_group=grupo)
            self.obs_msg = None   # ROTA_MAPA_OBSTACULO (11/11)
            self.create_subscription(OccupancyGrid, '/global_costmap/obstacle_layer', self.ao_obs_global,
                                     qos_mapa, callback_group=grupo)
            self.barreiras_msg = None   # MEMORIA_BARREIRA (25/11): /bloqueios_mapa do bloqueios_rota.py
            self.create_subscription(OccupancyGrid, '/bloqueios_mapa',
                                     lambda m: setattr(self, 'barreiras_msg', m), qos_mapa, callback_group=grupo)
            self.create_subscription(PoseStamped, self.get_parameter('topico_goal').value,
                                     self.ao_goal, 10, callback_group=grupo)
            self.create_subscription(String, '/ir_para_area', self.ao_ir_area, 10,
                                     callback_group=grupo)
            self.pub_poses = self.create_publisher(PoseArray, '/alinhador/etapas', 1)
            self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=grupo)
            self.plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose',
                                     callback_group=grupo)
            threading.Thread(target=self.trabalhador, daemon=True).start()
            self.get_logger().info('Alinhador pronto. Entrada no corredor exige centro e angulo '
                                   'confirmados pelas duas paredes. Translacao continua no preparo e na travessia a %.0f%%. '
                                   'Envie goals em %s.' % (100 * self.fator_corredor,
                                                           self.get_parameter('topico_goal').value))

        def p(self, nome):
            return self.get_parameter(nome).value

        # ---------------- entradas ----------------
        def ao_mapa(self, msg):
            with self.lock:
                self.mapa_msg = msg

        def ao_obs_global(self, msg):   # ROTA_MAPA_OBSTACULO (11/11)
            with self.lock:
                self.obs_msg = msg

        def _obs_global(self):
            with self.lock:
                m = self.obs_msg
            if m is None:
                return None
            i = m.info
            return Mapa(i.resolution, i.width, i.height, i.origin.position.x, i.origin.position.y,
                        yaw_de(i.origin.orientation), m.data, limiar_ocupado=100)

        def ao_conflito_t(self, msg):
            with self.lock:
                self.conflito_t = (time.monotonic(), msg.data)

        def _stamp_recente(self, stamp):
            return TempoSensor.receber(stamp, self.get_clock().now().nanoseconds,
                                       time.monotonic()) is not None

        def _tempo_sensor_t(self, msg, sensor, atraso=0.0):
            with self.lock:
                ros_ns, mono = self.get_clock().now().nanoseconds, time.monotonic()
                # Janela LiDAR deslizante pode manter o mesmo raio mais antigo.
                # Reutiliza exatamente o prazo original; nunca renova na recepcao.
                anterior = self.tempos_t.get(sensor)
                stamp_ns = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
                if sensor == 'lidar' and anterior is not None and anterior.stamp_ns == stamp_ns:
                    amostra = anterior if anterior.recente(ros_ns, mono) else None
                    self.tempos_t[sensor] = amostra
                    return amostra
                amostra = TempoSensor.receber(msg.header.stamp,
                    ros_ns, mono, atraso,
                    self.ultimos_stamps_t.get(sensor))
                self.tempos_t[sensor] = amostra
                if amostra is not None:
                    self.ultimos_stamps_t[sensor] = amostra.stamp_ns
            return amostra

        def ao_timing_t(self, msg):
            try:
                item = json.loads(msg.data)
                if not isinstance(item, dict):
                    raise ValueError('metadata temporal sem objeto')
                if not item.get('timed_frame') or item.get('valid'):
                    return
                sensors = [item.get('sensor')]
            except (ValueError, TypeError):
                sensors = ['lidar', 'imu']
            with self.lock:
                for sensor in sensors:
                    if sensor in ('lidar', 'imu'):
                        self.tempos_t[sensor] = None
                        if sensor == 'lidar':
                            self.scan_t_raw = None
                        else:
                            self.imu_t = None

        def cancelar_t(self, req, res):
            self.cancelamento_t.set()
            self._cancelar_nav()
            res.success, res.message = True, 'cancelamento solicitado; aguarde estado parado'
            return res

        def _estado_t(self, estado, motivo='', observacao=None):
            m = String()
            m.data = json.dumps(dict(estado=estado, motivo=motivo,
                                    observacao=observacao, stamp_ns=self.get_clock().now().nanoseconds))
            self.pub_estado_t.publish(m)

        def ao_imu(self, msg, experimental=False):   # SENSOR_PARADO
            if not experimental:
                self.t_imu = time.monotonic()
                q = msg.orientation
                self.imu_supervisor = (self.t_imu, yaw_de(q)) if (self._stamp_recente(msg.header.stamp) and msg.orientation_covariance[0] != -1 and all(math.isfinite(v) for v in (q.x,q.y,q.z,q.w)) and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < .05) else None
                self.gyro_z = msg.angular_velocity.z
                return
            q = msg.orientation
            valores = (q.x, q.y, q.z, q.w, msg.angular_velocity.z)
            amostra = self._tempo_sensor_t(msg, 'imu', 0.0)
            valida = (amostra is not None
                      and msg.orientation_covariance[0] != -1
                      and all(math.isfinite(v) for v in valores)
                      and abs(sum(v*v for v in valores[:4])-1.0) < 0.05)
            with self.lock:
                self.imu_t = (amostra.aquisicao_mono, yaw_de(q), msg.angular_velocity.z) if valida else None


        def ao_plano(self, msg):
            with self.lock:
                self.plano = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
                self.plano_t = time.monotonic()

        def ao_goal(self, msg):
            if msg.header.frame_id not in ('map', ''):
                self.get_logger().warn('Goal em "%s" ignorado: use Fixed Frame = map.' %
                                       msg.header.frame_id)
                return
            with self.lock:
                self.goal_pendente = msg
                self.area_pendente = None
            self.evento.set()
            self._cancelar_nav()
            self.get_logger().info('Novo goal: x=%.2f y=%.2f yaw=%.0f graus' % (
                msg.pose.position.x, msg.pose.position.y, math.degrees(yaw_de(msg.pose.orientation))))

        def ao_ir_area(self, msg):
            nome = msg.data.strip().upper()
            # DIAGNOSTICO_ENTRADA_MISSAO_20261109: prova que callback foi executado.
            self.get_logger().info('PEDIDO RECEBIDO /ir_para_area: %s' % nome)
            caminho = self.p('arquivo_areas')
            try:
                import yaml
                with open(caminho) as fa:
                    areas = (yaml.safe_load(fa) or {}).get('service_areas') or {}
            except Exception as exc:
                motivo = 'falha ao ler cadastro: %s' % exc
                self.get_logger().error('Nao consegui ler %s: %s' % (caminho, exc))
                if nome:
                    msg_erro = String()
                    msg_erro.data = '%s FALHOU %s' % (nome, motivo)
                    self.pub_resultado.publish(msg_erro)
                return
            area = areas.get(nome) or areas.get(nome.replace('_', ''))
            # v3.9: pontos sem mesa (START, FINISH...) ficam em ~/mapas/pontos.yaml:
            # o robo so vai ate a pose, sem encostar
            pontos = {}
            try:
                with open(self.p('arquivo_pontos')) as fp:
                    pontos = (yaml.safe_load(fp) or {}).get('pontos') or {}
            except OSError:
                pass
            except Exception as exc:
                self.get_logger().warn('Nao consegui ler %s: %s' % (self.p('arquivo_pontos'), exc))
            if (area is None or 'aproximacao' not in area) and nome in pontos:
                area = {'aproximacao': pontos[nome]}
                self.sem_encosto.add(nome)
            else:
                self.sem_encosto.discard(nome)
            if area is None or 'aproximacao' not in area:
                self.get_logger().error('Area %s nao esta em %s nem em %s (tem: %s)' % (
                    nome, caminho, self.p('arquivo_pontos'),
                    ', '.join(list(areas) + list(pontos)) or 'nenhuma'))
                with self.lock:
                    self.res_area = nome
                m = String()
                m.data = '%s FALHOU area desconhecida' % nome
                self.pub_resultado.publish(m)
                return
            ap = area['aproximacao']
            goal = self._pose(ap['x'], ap['y'], ap['yaw'])
            with self.lock:
                self.goal_pendente = goal
                self.area_pendente = nome
            self.evento.set()
            self._cancelar_nav()
            self.get_logger().info('Indo para %s: aproximacao em x=%.2f y=%.2f yaw=%.0f graus' % (
                nome, ap['x'], ap['y'], math.degrees(ap['yaw'])))

        def ao_odom(self, msg):
            self.odom_t_valido = self._tempo_sensor_t(msg, 'odom') is not None
            p = msg.pose.pose
            with self.lock:
                self.odom = (time.monotonic(), p.position.x, p.position.y, yaw_de(p.orientation))
                self.hist_odom.append(self.odom)

        def ao_cmd_nav(self, msg):
            tw = msg.twist
            with self.lock:
                self.hist_cmd.append((time.monotonic(), tw.linear.x, tw.linear.y, tw.angular.z))

        def ao_cmd_final(self, msg):
            self._tempo_sensor_t(msg, 'saida')
            tw = msg.twist
            zerado = abs(tw.linear.x) < 1e-4 and abs(tw.linear.y) < 1e-4 and abs(tw.angular.z) < 1e-4
            with self.lock:
                self.cmd_final = (time.monotonic(), zerado)

        def ao_cm(self, msg):
            self.cm_stop = msg.action_type == 1  # STOP

        def ao_scan(self, msg, experimental=False):
            # Fluxo temporal ja carrega limite inferior de aquisicao na ponte.
            # Nao somar scan_time novamente; bins agregados tem time_increment=0.
            amostra = self._tempo_sensor_t(msg, 'lidar', 0.0) if experimental else None
            if not self._stamp_recente(msg.header.stamp):
                if experimental:
                    with self.lock:
                        self.scan_t_raw = None
                return
            if self.laser_tf is None:
                try:
                    tr = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
                    self.laser_tf = (tr.transform.translation.x, tr.transform.translation.y,
                                     yaw_de(tr.transform.rotation))
                    self.get_logger().info('Laser em base_link: x=%.3f y=%.3f yaw=%.1f graus' % (
                        self.laser_tf[0], self.laser_tf[1], math.degrees(self.laser_tf[2])))
                except Exception:
                    return
            lx, ly, lth = self.laser_tf
            pts = []
            a = msg.angle_min
            for r in msg.ranges:
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    pts.append((lx + r * math.cos(a + lth), ly + r * math.sin(a + lth)))
                a += msg.angle_increment
            # folga livre na frente do para-choque (x=0.365), na largura do robo
            frente = min([p[0] - 0.365 for p in pts if p[0] > 0.30 and abs(p[1]) < 0.33] or [9.0])
            with self.lock:
                od = self.odom
                if experimental:
                    self.scan_t_raw = ((amostra.aquisicao_mono, tuple(msg.ranges), msg.angle_min,
                                       msg.angle_increment, msg.range_min, msg.range_max, self.laser_tf)
                                       if amostra is not None else None)
                else:
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    self.scan = (time.monotonic(), pts, od[1:] if od else None, frente, stamp)
                    self.scan_seq += 1

        def ao_cmd_ctrl(self, msg):   # CONTROLADOR_ORFAO_ALINHADOR
            t = msg.twist
            if abs(t.linear.x) > 1e-3 or abs(t.linear.y) > 1e-3 or abs(t.angular.z) > 1e-3:
                with self.lock:
                    self.ctrl_nz_t = time.monotonic()

        def _verificar_controlador_orfao(self, nome, espera=0.35, calar=1.5):
            """CONTROLADOR_ORFAO_ALINHADOR (11/11): NavigateToPose terminal e o controlador ainda comanda ->
            cancela todos os FollowPath e espera calar. Devolve 'ok', 'cancelado' ou 'persistente'."""
            if self.p('nav2_principal') or not self.p('cancelar_controlador_orfao'):
                return 'ok'
            t_fim = time.monotonic()
            time.sleep(espera)
            with self.lock:
                c = self.ctrl_nz_t
            if c is None or c < t_fim + 0.2:
                return 'ok'
            from action_msgs.srv import CancelGoal
            self.get_logger().warn('CONTROLADOR_ORFAO: %s terminou mas o controlador segue comandando ha %.1f s: '
                                   'cancelo o FollowPath' % (nome, c - t_fim))
            if self.cli_cancel_fp.wait_for_service(timeout_sec=0.5):
                f = self.cli_cancel_fp.call_async(CancelGoal.Request())
                t1 = time.monotonic() + 1.0
                while not f.done() and time.monotonic() < t1:
                    time.sleep(0.02)
            t_cancel = time.monotonic()
            while time.monotonic() < t_cancel + calar:
                time.sleep(0.05)
                with self.lock:
                    c = self.ctrl_nz_t
                if c is not None and time.monotonic() - c >= 0.3:
                    self.get_logger().info('CONTROLADOR_ORFAO: controlador parado %.1f s depois do cancelamento' % (
                        c - t_cancel if c > t_cancel else 0.0))
                    return 'cancelado'
            self.get_logger().error('CONTROLADOR_ORFAO: o controlador continua comandando depois de cancelar')
            return 'persistente'

        def _enviar_goal(self, cli, req, timeout):
            """ENVIO_GOAL_SEGURO (11/11): envia e espera a aceitacao. Saiu sem handle (goal novo/cancelamento ->
            Cancelado, ou timeout -> None)? O goal aceito depois e cancelado na hora, para nao ficar orfao."""
            envio = cli.send_goal_async(req)
            log = self.get_logger()

            def cancelar_tardio(f):
                try:
                    h = f.result()
                except Exception:
                    return
                if h is not None and h.accepted:
                    log.warn('ENVIO_GOAL_SEGURO: goal aceito depois de abandonado: cancelo')
                    h.cancel_goal_async()
            try:
                h = self._esperar(envio, timeout)
            except Cancelado:
                envio.add_done_callback(cancelar_tardio)
                raise
            if h is None:
                envio.add_done_callback(cancelar_tardio)
            return h

        def _novo_goal_chegou(self):
            with self.lock:
                return self.goal_pendente is not None or self.cancelamento_t.is_set()

        # ---------------- utilidades ----------------
        def _esperar(self, futuro, timeout):
            ev = threading.Event()
            futuro.add_done_callback(lambda _: ev.set())
            limite = time.monotonic() + timeout
            while not ev.is_set():
                if self._novo_goal_chegou():
                    raise Cancelado()
                if time.monotonic() > limite:
                    return None
                ev.wait(0.05)
            return futuro.result()

        def _cancelar_nav(self):
            h = self.handle_ativo
            if h is not None:
                try:
                    h.cancel_goal_async()
                except Exception:
                    pass

        def _pose(self, x, y, yaw):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            return ps

        def _mapa(self):
            with self.lock:
                m = self.mapa_msg
            if m is None:
                return None
            i = m.info
            return Mapa(i.resolution, i.width, i.height, i.origin.position.x, i.origin.position.y,
                        yaw_de(i.origin.orientation), m.data)

        def _caminho(self, goal):
            if not self.plan.wait_for_server(timeout_sec=3.0):
                self.get_logger().warn('compute_path_to_pose indisponivel')
                return None
            req = ComputePathToPose.Goal()
            req.goal = goal
            req.planner_id = self.p('planner_id')
            req.use_start = False
            h = self._esperar(self.plan.send_goal_async(req), 5.0)
            if h is None or not h.accepted:
                return None
            r = self._esperar(h.get_result_async(), 10.0)
            if r is None or r.status != GoalStatus.STATUS_SUCCEEDED:
                return None
            return [(ps.pose.position.x, ps.pose.position.y) for ps in r.result.path.poses]

        def _virar_para_caminho(self, alvo, nome):
            """v3.30 VIRAR_CAMINHO (missao19): o Nav2 so anda para a frente (DWB sem re) e o
            RotationShim desiste do giro quando preve colisao; com o caminho atras o DWB cisca.
            Aqui: se o caminho sai a mais de virar_caminho_graus da frente, vira para ele com o
            _girar_seguro (anda o minimo que libera o giro e gira). Se nao conseguir, segue."""
            lim = float(self.p('virar_caminho_graus'))
            pm = self._pose_mapa_yaw()
            if lim <= 0.0 or pm is None:
                return
            caminho = self._caminho(alvo)
            if (not caminho or len(caminho) < 3) and self.p('virar_replaneja'):   # VIRAR_REPLANEJA (13/11)
                caminho = self._caminho_validado(alvo, nome)
            if not caminho or len(caminho) < 3:
                return
            comp = sum(math.hypot(caminho[k + 1][0] - caminho[k][0], caminho[k + 1][1] - caminho[k][1])
                       for k in range(len(caminho) - 1))
            if comp < 0.40:
                return
            rumo = rumo_do_caminho(caminho, pm[0], pm[1], 0.5)
            if rumo is None:
                return
            ang = _ang(rumo - pm[2])
            if abs(ang) <= math.radians(lim):
                return
            # PRE_GIRO_UNICO (11/11): mesmo alvo com pre-giro tentado ha pouco -> nao repete a correcao
            ax_, ay_ = alvo.pose.position.x, alvo.pose.position.y
            memo, rep = self.pre_giro_memo, float(self.p('pre_giro_repetir_s'))
            if (rep > 0.0 and memo is not None and math.hypot(memo[0] - ax_, memo[1] - ay_) <= 0.15
                    and time.monotonic() - memo[2] < rep):
                self.get_logger().info('%s: caminho a %+.0f graus, mas ja tentei o pre-giro para este alvo ha %.0f s: '
                                       'sigo pelo Nav2 (PRE_GIRO_UNICO)' % (nome, math.degrees(ang),
                                                                            time.monotonic() - memo[2]))
                return
            self.pre_giro_memo = (ax_, ay_, time.monotonic())
            n = nome + ' (virar para o caminho)'
            # v3.32 GIRO_DECIDIDO: decidiu girar, o alinhador gira (o shim desiste quando preve colisao)
            self.get_logger().info('%s: o caminho sai a %+.0f graus da frente: viro antes do Nav2' % (
                n, math.degrees(ang)))
            if self._girar_seguro(rumo, n, tempo_max=30.0, movs_max=4):
                self.get_logger().info('%s: SUCESSO' % n)
            else:
                self.get_logger().warn('%s: nao consegui virar; sigo pelo Nav2' % n)

        def _caminho_validado(self, alvo, nome):
            """VIRAR_REPLANEJA (13/11): planner sem caminho -> limpa costmaps e replaneja 1 vez; ainda sem caminho ->
            rota do mapa SO se o 1o metro estiver livre no costmap de obstaculos atual. None = nao vira."""
            self._limpar_costmaps()
            time.sleep(1.5)
            c = self._caminho(alvo)
            if c and len(c) >= 3:
                self.get_logger().info('%s: VIRAR_REPLANEJA: planner estava sem caminho; limpei os costmaps e ele '
                                       'achou (%d pontos)' % (nome, len(c)))
                return c
            mapa, pm, obs = self._mapa(), self._pose_mapa(), self._obs_global()
            if mapa is None or pm is None or obs is None:
                self.get_logger().warn('%s: VIRAR_REPLANEJA: planner sem caminho e sem mapa/costmap para validar a '
                                       'rota: nao viro' % nome)
                return None
            rota = rota_no_mapa(com_barreiras(self, mapa), pm, (alvo.pose.position.x, alvo.pose.position.y))   # MEMORIA_BARREIRA
            if not rota or len(rota) < 3:
                self.get_logger().warn('%s: VIRAR_REPLANEJA: planner e mapa sem rota: nao viro' % nome)
                return None
            ini, comp = [rota[0]], 0.0
            for a, b in zip(rota, rota[1:]):
                comp += math.hypot(b[0] - a[0], b[1] - a[1])
                ini.append(b)
                if comp >= 1.0:
                    break
            novos = obstaculo_novo_na_rota(mapa, obs, ini)
            if novos:
                self.get_logger().warn('%s: VIRAR_REPLANEJA: rota do mapa com objeto do momento a < 35 cm no 1o metro '
                                       '(%.2f, %.2f): nao viro por ela' % (nome, novos[0][0], novos[0][1]))
                return None
            self.get_logger().info('%s: VIRAR_REPLANEJA: planner continua sem caminho; rota do mapa com o 1o metro '
                                   'livre no costmap atual: viro por ela' % nome)
            return rota

        def _chamar_destravar(self, nome):
            """v3.31: chama o servidor 'destravar' (o mesmo da arvore do Nav2) e espera."""
            if not self.destravar_cli.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn('%s: destravar indisponivel' % nome)
                return False   # CISCADO_PROGRESSO (11/11): devolve se resolveu
            g = DriveOnHeading.Goal()
            g.target.x = 0.25
            g.speed = 0.03
            g.time_allowance.sec = 15
            h = self._enviar_goal(self.destravar_cli, g, 3.0)   # ENVIO_GOAL_SEGURO
            if h is None or not h.accepted:
                self.get_logger().warn('%s: destravar recusou' % nome)
                return False
            r = self._esperar(h.get_result_async(), 35.0)
            ok = r is not None and r.status == GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('%s: destravar %s' % (nome, 'terminou' if ok else 'falhou'))
            return ok

        def _navegar(self, alvo, nome, vigia=None, aceitar_perto=False, virar=True, _cisc=0):
            if self.p('nav2_principal'):
                return self._navegar_principal(alvo, nome, vigia)
            return self._navegar_legado(alvo, nome, vigia, aceitar_perto, virar, _cisc)

        def _giro_na_curva(self, alvo, nome, vigia):
            """GIRO_NA_CURVA (25/11): ate 3 curvas fechadas: vai ate a curva, gira la. 'ok'/'nada'/'interrompido'."""
            feito = 'nada'
            for k in range(3):
                pm, mapa = self._pose_mapa_yaw(), self._mapa()
                caminho = self._caminho(alvo)
                c = ponto_de_curva(caminho, mapa) if pm is not None else None
                if c is None:
                    return feito
                n = '%s (curva %d)' % (nome, k + 1)
                self.get_logger().info('GIRO_NA_CURVA: %s: curva de %+.0f graus em (%.2f, %.2f): vou ate la, giro e sigo' % (
                    n, math.degrees(_ang(c[3] - c[2])), c[0], c[1]))
                self._em_curva, self._raio_tmp = True, 0.25
                try:
                    r = self._navegar(self._pose(c[0], c[1], c[2]), n, vigia, aceitar_perto=True, virar=False)
                finally:
                    self._em_curva, self._raio_tmp = False, None
                if r == 'interrompido':
                    return r
                if r != 'ok':
                    self.get_logger().warn('GIRO_NA_CURVA: %s: nao cheguei na curva; sigo pelo Nav2' % n)
                    return feito
                self._girar_seguro(c[3], n, tempo_max=30.0, movs_max=4)
                feito = 'ok'
            return feito

        def _navegar_legado(self, alvo, nome, vigia=None, aceitar_perto=False, virar=True, _cisc=0):
            """Devolve 'ok', 'falha' ou 'interrompido' (vigia pediu para parar).

            aceitar_perto (v3.10): 'ok' quando o robo fica tempo_chegada s a menos de
            raio_chegada m do alvo, cancelando o Nav2 (o angulo final nao importa).
            virar (v3.30): antes, vira para o rumo do caminho (_virar_para_caminho)."""
            if not self.nav.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('navigate_to_pose indisponivel')
                return 'falha'
            if virar:
                self._virar_para_caminho(alvo, nome)
                if self.p('giro_na_curva') and _cisc == 0 and not getattr(self, '_em_curva', False):   # GIRO_NA_CURVA
                    r_c = self._giro_na_curva(alvo, nome, vigia)
                    if r_c == 'interrompido':
                        return r_c
            self._esperar_sensores(nome)   # SENSOR_PARADO
            req = NavigateToPose.Goal()
            req.pose = alvo
            self.get_logger().info('%s: x=%.2f y=%.2f yaw=%.0f graus' % (
                nome, alvo.pose.position.x, alvo.pose.position.y,
                math.degrees(yaw_de(alvo.pose.orientation))))
            h = self._enviar_goal(self.nav, req, 5.0)   # ENVIO_GOAL_SEGURO (11/11)
            if h is None or not h.accepted:
                self.get_logger().warn('%s: goal %s' % (nome, 'recusado' if h is not None else 'sem aceitacao em 5 s'))
                return 'falha'
            self.handle_ativo = h
            futuro = h.get_result_async()
            t_ini = time.monotonic()
            janela = float(self.p('ciscado_janela'))
            try:
                proxima = time.monotonic() + self.p('periodo_vigia')
                perto_desde = None
                while True:
                    r = self._esperar(futuro, 0.2)
                    if r is not None:
                        break
                    # SENSOR_PARADO: o CM ja segura com 0,5 s; com 1 s tira o goal do Nav2, senao ele
                    # gasta a recuperacao sem dados (missao22 08:36:16,94 a 08:36:17,21)
                    if self._sensor_parado(1.0):
                        h.cancel_goal_async()
                        self._esperar(futuro, 5.0)
                        self.handle_ativo = None
                        self._esperar_sensores(nome)
                        return self._navegar(alvo, nome, vigia, aceitar_perto, False, _cisc)
                    # v3.31 ANTI_CISCADO
                    if self.p('anti_ciscado') and _cisc < int(self.p('ciscado_max')) and \
                            time.monotonic() - t_ini > janela:
                        with self.lock:
                            cm = [c for c in self.hist_cmd if c[0] >= t_ini]
                            om = list(self.hist_odom)
                        sim, inv, d, g = ciscando(cm, om, time.monotonic(), janela,
                                                  int(self.p('ciscado_inversoes')))
                        if sim:
                            h.cancel_goal_async()
                            self._esperar(futuro, 5.0)
                            self._parar()
                            self.get_logger().warn(
                                'ANTI_CISCADO %d: %s: %d inversoes em %.0f s sem sair do lugar (%.0f cm, '
                                '%.0f graus): paro, destravo e mando o goal de novo' % (
                                    _cisc + 1, nome, inv, janela, d * 100, g))
                            self.handle_ativo = None
                            destravou = self._chamar_destravar('ANTI_CISCADO %d' % (_cisc + 1))
                            if not destravou and _cisc >= 1 and self.p('ciscado_exige_destravar'):
                                self.get_logger().warn('ANTI_CISCADO %d: %s: o destravar nao resolveu de novo; nao '
                                                       'repito o goal (CISCADO_PROGRESSO)' % (_cisc + 1, nome))
                                return 'falha'
                            return self._navegar(alvo, nome, vigia, aceitar_perto, virar, _cisc + 1)
                    if aceitar_perto:
                        pm = self._pose_mapa()
                        if pm is not None and math.hypot(pm[0] - alvo.pose.position.x,
                                                         pm[1] - alvo.pose.position.y) <= \
                                (getattr(self, '_raio_tmp', None) or self.p('raio_chegada')):   # ENTRADA_ASSUME
                            perto_desde = perto_desde or time.monotonic()
                            if time.monotonic() - perto_desde >= self.p('tempo_chegada'):
                                h.cancel_goal_async()
                                self._esperar(futuro, 5.0)
                                self._parar()
                                self.get_logger().info(
                                    '%s: SUCESSO (aceito a %.0f cm do ponto, sem exigir angulo%s)' % (
                                        nome, 100 * math.hypot(pm[0] - alvo.pose.position.x,
                                                               pm[1] - alvo.pose.position.y),
                                        ', Collision Monitor em STOP' if self.cm_stop else ''))
                                return 'ok'
                        else:
                            perto_desde = None
                    if vigia is not None and time.monotonic() >= proxima:
                        proxima = time.monotonic() + self.p('periodo_vigia')
                        if vigia():
                            h.cancel_goal_async()
                            self._esperar(futuro, 5.0)
                            self.get_logger().info('%s: interrompido para alinhar' % nome)
                            return 'interrompido'
            finally:
                self.handle_ativo = None
                self._verificar_controlador_orfao(nome)   # CONTROLADOR_ORFAO_ALINHADOR (11/11)
            ok = r.status == GoalStatus.STATUS_SUCCEEDED
            # CHEGADA_PERTO (13/11): o Nav2 falhou ja no goal (DWB sem trajetoria perto de obstaculo)
            if not ok and r.status != GoalStatus.STATUS_CANCELED and self.p('chegada_perto') \
                    and self._chegada_perto(alvo, nome):
                return 'ok'
            if not ok and r.status != GoalStatus.STATUS_CANCELED and self.pre_giro_memo is not None \
                    and self.p('pre_giro_refaz_falha'):   # PRE_GIRO_REFAZ (19/11)
                self.pre_giro_memo = None
                self.get_logger().info('%s: PRE_GIRO_REFAZ: o Nav2 falhou; a proxima tentativa vira para o caminho '
                                       'de novo antes' % nome)
            self.get_logger().info('%s: %s' % (nome, 'SUCESSO' if ok else 'FALHOU'))
            return 'ok' if ok else 'falha'

        def _chegada_perto(self, alvo, nome):
            """CHEGADA_PERTO (13/11): termina os ultimos cm com manobras guardadas pelo LiDAR. True = chegou."""
            pm = self._pose_mapa_yaw()
            if pm is None:
                return False
            gx, gy = alvo.pose.position.x, alvo.pose.position.y
            gyaw = yaw_de(alvo.pose.orientation)
            d = math.hypot(gx - pm[0], gy - pm[1])
            if d > float(self.p('chegada_perto_raio')):
                return False
            n = nome + ' (CHEGADA_PERTO)'
            self.get_logger().warn('%s: o Nav2 falhou a %.0f cm do ponto (%.0f graus do angulo): termino eu' % (
                n, d * 100, math.degrees(abs(_ang(gyaw - pm[2])))))
            self._esperar_sensores(n)
            if d > 0.05:
                self._deslocar_ate(gx, gy, n, tol=0.03, passo_max=0.20)
            pm = self._pose_mapa_yaw()
            if pm is None:
                return False
            if abs(_ang(gyaw - pm[2])) > math.radians(10.0):
                self._girar_seguro(gyaw, n, tempo_max=20.0, movs_max=1)
                pm = self._pose_mapa_yaw()
                if pm is None:
                    return False
            d, e = math.hypot(gx - pm[0], gy - pm[1]), abs(_ang(gyaw - pm[2]))
            if d <= 0.10 and e <= math.radians(10.0):
                self.get_logger().info('%s: SUCESSO (%.0f cm e %.0f graus do goal)' % (n, d * 100, math.degrees(e)))
                return True
            self.get_logger().warn('%s: nao cheguei (%.0f cm e %.0f graus do goal)' % (n, d * 100, math.degrees(e)))
            return False

        def _publicar_vel(self, vx, vy, wz, *, corredor=False):
            if self.p('nav2_principal'):
                if self._novo_goal_chegou() and any(abs(v) > 1e-6 for v in (vx, vy, wz)):
                    raise Cancelado()
                if any(abs(v) > 1e-6 for v in (vx, vy, wz)):
                    self._autoridade_mov('especial')
            if corredor or self.em_preparo:
                vx, vy, wz = comando_corredor(vx, vy, wz, self.fator_corredor)
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = float(vx), float(vy), float(wz)
            (self.pub_t if self.controle_t_ativo else self.pub_cmd).publish(m)

        def _parar(self):
            for _ in range(5):
                self._publicar_vel(0.0, 0.0, 0.0)
                time.sleep(0.05)

        def _pose_mapa(self):
            try:
                tr = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            except Exception:
                return None
            return tr.transform.translation.x, tr.transform.translation.y

        def _pose_mapa_yaw(self, idade_max=None):
            try:
                tr = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            except Exception:
                return None
            if idade_max is not None and not self._stamp_recente(tr.header.stamp):
                return None
            return (tr.transform.translation.x, tr.transform.translation.y,
                    yaw_de(tr.transform.rotation))

        def _sensor_parado(self, idade=None):
            """SENSOR_PARADO: lista de (sensor, idade) acima do limite. O limite padrao e 0,5 s: um scan
            perdido da 0,34 s (5,9 Hz). A /odom nao basta: sem IMU ela segue a 50 Hz com o yaw congelado."""
            lim = self.p('sensor_idade_max') if idade is None else idade
            agora = time.monotonic()
            with self.lock:
                sc, od = self.scan, self.odom
            fontes = (('LiDAR', sc[0] if sc is not None else None),
                      ('odom', od[0] if od is not None else None), ('IMU', self.t_imu))
            # SENSOR_IMU2: a IMU falha ~0,6 s com os motores girando; o yaw congela pouco tempo
            return [(n, 99.0 if t is None else agora - t) for n, t in fontes
                    if t is None or agora - t > (max(lim, 1.5) if n == 'IMU' else lim)]

        def _esperar_sensores(self, nome):
            """Se algum sensor parou: para o robo e espera voltar. Devolve os segundos parados (0 = tudo
            bem). Goal novo levanta Cancelado; sem voltar em sensor_espera_max, levanta SensorParado."""
            parados = self._sensor_parado()
            if not parados:
                return 0.0
            t0 = time.monotonic()
            ult, imu_caiu = 0.0, False  # SENSOR_IMU2: "imu_caiu" = IMU e LiDAR juntos: serial caiu
            self._parar()
            while parados:
                if self._novo_goal_chegou():
                    raise Cancelado()
                agora = time.monotonic()
                nomes = [n for n, _ in parados]
                imu_caiu = imu_caiu or ('IMU' in nomes and 'LiDAR' in nomes)
                if agora - ult >= 2.0:
                    ult = agora
                    self.get_logger().warn('%s: %s parado ha %.1f s: parei' % (nome, parados[0][0], parados[0][1]))
                if agora - t0 > self.p('sensor_espera_max'):
                    raise SensorParado('%s parado ha %.0f s: desisti' % (parados[0][0], agora - t0))
                self._publicar_vel(0.0, 0.0, 0.0)
                time.sleep(0.1)
                parados = self._sensor_parado()
            if imu_caiu and time.monotonic() - t0 >= 1.0:
                # a ponte reconectou: ignora comandos por 5 s (serial_startup_delay_s)
                fim = time.monotonic() + self.p('sensor_espera_ponte')
                self.get_logger().info('%s: IMU voltou: espero a ponte aceitar comandos' % nome)
                while time.monotonic() < fim:
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    self._publicar_vel(0.0, 0.0, 0.0)
                    time.sleep(0.1)
            pausa = time.monotonic() - t0
            self.get_logger().info('%s: sensores voltaram depois de %.1f s: medindo de novo' % (nome, pausa))
            return pausa

        def _sensores(self, idade_scan=0.6):
            agora = time.monotonic()
            with self.lock:
                od, sc = self.odom, self.scan
            if od is None or agora - od[0] > 0.5 or sc is None or agora - sc[0] > idade_scan:
                return None, None
            return od, sc

        def _andar_reto(self, direcao, dist, nome, margem=0.02):   # CAIXA_LATERAL: margem da faixa
            """v3.20: anda 'dist' m numa direcao do robo (frente/tras/esq/dir), medido pela odom,
            parando antes se a folga nessa direcao cair abaixo de 7 cm (o Collision Monitor para
            a 6). Devolve o quanto andou (m)."""
            ux, uy = DIRECOES[direcao]
            self._esperar_sensores(nome)   # SENSOR_PARADO
            od0, _ = self._sensores()
            if od0 is None:
                return 0.0
            c0, s0 = math.cos(od0[3]), math.sin(od0[3])
            v_real = 0.15 if ux else 0.09
            t_max = 2.5 * dist / v_real + 1.0
            if self.em_preparo:
                t_max /= self.fator_corredor
            freio = 0.015 if ux else 0.01   # o robo anda ~1-2 cm depois do comando zero
            feito = 0.0
            t0 = time.monotonic()
            try:
                while rclpy.ok() and time.monotonic() - t0 < t_max:
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO
                    if pausa > 0.0:
                        t0 += pausa
                        continue
                    od, sc = self._sensores()
                    if od is None:
                        break
                    dx, dy = od[1] - od0[1], od[2] - od0[2]
                    feito = (c0 * dx + s0 * dy) * ux + (-s0 * dx + c0 * dy) * uy
                    if feito >= dist - freio or folga_direcao(sc[1], direcao, margem) < FOLGA_MOV:
                        break
                    self._publicar_vel(0.025 * ux, 0.025 * uy, 0.0)
                    time.sleep(0.05)
            finally:
                self._parar()
            time.sleep(0.3)
            od, _ = self._sensores(idade_scan=9.0)
            if od is not None:
                dx, dy = od[1] - od0[1], od[2] - od0[2]
                feito = (c0 * dx + s0 * dy) * ux + (-s0 * dx + c0 * dy) * uy
            self.get_logger().info('%s: andei %.0f cm para %s' % (nome, feito * 100, direcao))
            return feito

        def _rota_retas(self, goal, area):
            """ROTA_RETAS (25/11): retas nos eixos do mapa + giro parado onde cabe. 'ok' | 'falha' | None."""
            if not self.p('rota_retas'):
                return None
            import rota_retas as RR
            mapa, pm = self._mapa(), self._pose_mapa_yaw()
            if mapa is None or pm is None:
                return None
            gx, gy, gyaw = goal.pose.position.x, goal.pose.position.y, yaw_de(goal.pose.orientation)
            if math.hypot(gx - pm[0], gy - pm[1]) < 0.10:
                return None
            n = 'ROTA_RETAS %s' % (area or 'goal')
            try:
                t0 = time.monotonic()
                plano = RR.plano_retas(mapa, pm, (gx, gy, gyaw), pose_livre=pose_livre)
            except Exception as exc:
                self.get_logger().warn('ROTA_RETAS: planejador falhou (%s): sigo como antes' % exc)
                return None
            if not plano:
                self.get_logger().info('ROTA_RETAS: sem rota de retas ate %s: sigo como antes (Nav2)' % area)
                return None
            self.get_logger().info('ROTA_RETAS: %s (%.1f s): %s' % (area, time.monotonic() - t0, RR.descrever(plano)))
            for k, passo in enumerate(plano):
                if passo[0] == 'girar':
                    self._girar_seguro(passo[1], '%s passo %d' % (n, k + 1), tempo_max=30.0, movs_max=4)
                    continue
                x, y = passo[1], passo[2]
                for _ in range(3):
                    pm = self._pose_mapa_yaw()
                    if pm is None:
                        return 'falha'
                    dx, dy = x - pm[0], y - pm[1]
                    c, s_ = math.cos(pm[2]), math.sin(pm[2])
                    fx, ly = c * dx + s_ * dy, -s_ * dx + c * dy
                    if math.hypot(fx, ly) <= 0.04:
                        break
                    # o eixo maior primeiro (a reta), depois o resto
                    for eixo in sorted(('f', 'l'), key=lambda e: -abs(fx if e == 'f' else ly)):
                        v = fx if eixo == 'f' else ly
                        if abs(v) > 0.03:
                            d = ('frente' if v > 0 else 'tras') if eixo == 'f' else ('esq' if v > 0 else 'dir')
                            self._andar_reto(d, abs(v), '%s passo %d' % (n, k + 1), margem=0.0)
                pm = self._pose_mapa_yaw()
                erro = None if pm is None else math.hypot(x - pm[0], y - pm[1])
                if erro is None or erro > 0.10:
                    self.get_logger().warn('ROTA_RETAS: passo %d parou a %s de (%.2f, %.2f): sigo como antes (Nav2)' % (
                        k + 1, '?' if erro is None else '%.0f cm' % (erro * 100), x, y))
                    return 'falha'
            pm = self._pose_mapa_yaw()
            erro = None if pm is None else math.hypot(gx - pm[0], gy - pm[1])
            self.get_logger().info('ROTA_RETAS: %s: cheguei a %s do ponto' % (area, '?' if erro is None else '%.0f cm' % (erro * 100)))
            return 'ok' if erro is not None and erro <= 0.10 else 'falha'

        def _boca_da_caixa(self, area, info, gx, gy):
            """CAIXA_LATERAL (25/11): passagem cuja entrada e a boca da caixa do START/FINISH."""
            if not (self.p('caixa_lateral') and self.p('entrada_translada') and area in self.sem_encosto
                    and str(area).startswith(('START', 'FINISH')) and isinstance(info, dict)):
                return False
            ex, ey = info['entrada'][0], info['entrada'][1]
            if math.hypot(ex - gx, ey - gy) > 1.0:
                return False
            self.get_logger().info('CAIXA_LATERAL: passagem em (%.2f, %.2f) e a boca da caixa do %s: '
                                   'entro pela ENTRADA_TRANSLADA' % (ex, ey, area))
            return True

        def _saida_caixa(self, area):
            """SAIDA_CAIXA (25/11): robo encaixado num START/FINISH anda reto para fora antes de navegar."""
            dist = float(self.p('saida_caixa'))
            self.get_logger().info('SAIDA_CAIXA: confiro (dist %.2f, area %s)' % (dist, area))
            if dist <= 0.0 or str(area or '').startswith(('START', 'FINISH')):
                return
            pm = self._pose_mapa_yaw()
            if pm is None:
                self.get_logger().warn('SAIDA_CAIXA: sem pose no mapa (TF map->base_link): nao confiro a caixa')
                return
            import yaml   # SAIDA_CAIXA: o modulo so importa yaml dentro das funcoes
            try:
                with open(self.p('arquivo_pontos')) as fp:
                    pontos = (yaml.safe_load(fp) or {}).get('pontos') or {}
            except Exception as exc:
                self.get_logger().warn('SAIDA_CAIXA: nao li os pontos: %s' % exc)
                return
            for n, pt in pontos.items():
                if not str(n).startswith(('START', 'FINISH')):
                    continue
                dyaw = abs(math.atan2(math.sin(pm[2] - pt['yaw']), math.cos(pm[2] - pt['yaw'])))
                if math.hypot(pm[0] - pt['x'], pm[1] - pt['y']) <= 0.15 and dyaw <= math.radians(25):
                    self.get_logger().info('SAIDA_CAIXA: robo no %s: ando %.0f cm reto para fora' % (n, dist * 100))
                    self._andar_reto('frente', dist, 'SAIDA_CAIXA', margem=0.0)
                    return
            self.get_logger().info('SAIDA_CAIXA: robo em (%.2f, %.2f, %.0f graus) fora de START/FINISH' % (
                pm[0], pm[1], math.degrees(pm[2])))

        def _marcar_barreiras(self):
            """MEMORIA_BARREIRA (25/11): grava o que o LiDAR ve fora do mapa no /bloqueios_mapa (sobrevive a limpeza)."""
            if not self.p('memoria_barreira'):
                return
            from std_srvs.srv import Trigger
            if getattr(self, 'cli_marcar', None) is None:
                self.cli_marcar = self.create_client(Trigger, '/bloqueios/marcar')
            if self.cli_marcar.wait_for_service(timeout_sec=0.5):
                r = self._esperar(self.cli_marcar.call_async(Trigger.Request()), 2.0)
                if r is not None:
                    self.get_logger().info('MEMORIA_BARREIRA: %s' % r.message)

        def _limpar_costmaps(self, so_local=False):
            """v3.27: limpa os costmaps local e global (os obstaculos que ainda existem voltam no scan).
            MEMORIA_BARREIRA (25/11): marca antes o que o LiDAR ve; so_local=True nao toca o global."""
            self._marcar_barreiras()
            try:
                from nav2_msgs.srv import ClearEntireCostmap
            except Exception:
                return
            if not hasattr(self, 'cli_limpar'):
                self.cli_limpar = {}
            for srv in ('/local_costmap/clear_entirely_local_costmap',) + \
                    (() if so_local else ('/global_costmap/clear_entirely_global_costmap',)):
                cli = self.cli_limpar.get(srv)
                if cli is None:
                    cli = self.cli_limpar[srv] = self.create_client(ClearEntireCostmap, srv)
                if cli.wait_for_service(timeout_sec=1.0):
                    self._esperar(cli.call_async(ClearEntireCostmap.Request()), 2.0)

        def _travado(self, goal, janela=6.0, desloc_min=0.05, giro_min=math.radians(15)):
            """v3.27: o Nav2 nao tira o robo do lugar ha 'janela' s, longe do destino, com algo a frente."""
            pm = self._pose_mapa_yaw()
            if pm is None:
                return False
            agora = time.monotonic()
            hist = getattr(self, 'hist_nav', None)
            if hist is None or getattr(self, 'hist_goal', None) is not goal:
                hist = self.hist_nav = []
                self.hist_goal = goal
            hist.append((agora, pm))
            while hist and agora - hist[0][0] > janela + 1.0:
                hist.pop(0)
            if agora - hist[0][0] < janela:
                return False
            p0 = hist[0][1]
            if math.hypot(pm[0] - p0[0], pm[1] - p0[1]) >= desloc_min or abs(_ang(pm[2] - p0[2])) >= giro_min:
                return False
            if math.hypot(goal.pose.position.x - pm[0], goal.pose.position.y - pm[1]) < 0.35:
                return False
            _, sc = self._sensores()
            return sc is not None and plano_desvio(sc[1]) is not None

        def _desviar(self, goal, nome='DESVIO'):
            """v3.27: re -> mede a peca -> desliza para o lado livre -> passa a peca -> limpa os costmaps."""
            self._parar()
            time.sleep(0.4)
            _, sc = self._sensores()
            if sc is None:
                return False
            fr = folga_direcao(sc[1], 'frente')
            if fr < 0.15:
                self._andar_reto('tras', min(0.20, 0.17 - fr), nome)
                time.sleep(0.5)
                _, sc = self._sensores()
                if sc is None:
                    return False
            lado_goal = 0.0
            pm = self._pose_mapa_yaw()
            if pm is not None:
                dx, dy = goal.pose.position.x - pm[0], goal.pose.position.y - pm[1]
                lado_goal = -math.sin(pm[2]) * dx + math.cos(pm[2]) * dy
            p = plano_desvio(sc[1], lado_goal=lado_goal)
            if p is None:
                self.get_logger().info('%s: nada na frente agora: limpo os costmaps e sigo' % nome)
                self._limpar_costmaps()
                time.sleep(1.0)
                return True
            xp, xl, ymin, ymax = p['obst']
            self.get_logger().info(
                '%s: peca a %.0f cm do para-choque, de %+.0f a %+.0f cm do centro | esq: %.0f cm %s | dir: %.0f cm %s' % (
                    nome, 100 * (xp - FOOTPRINT[0][0]), 100 * ymin, 100 * ymax,
                    100 * p['opcoes']['esq'][0], p['opcoes']['esq'][1],
                    100 * p['opcoes']['dir'][0], p['opcoes']['dir'][1]))
            if p['lado'] is None:
                self.get_logger().warn('%s: nenhum lado livre: limpo os costmaps e deixo o Nav2 tentar' % nome)
                self._limpar_costmaps()
                time.sleep(1.0)
                return False
            self.get_logger().info('%s: deslizo %.0f cm para %s e avanco %.0f cm' % (
                nome, 100 * p['desloc'], p['lado'], 100 * p['frente']))
            feito = self._andar_reto(p['lado'], p['desloc'], nome)
            if feito >= p['desloc'] - 0.04 and p['frente'] > 0.03:
                self._andar_reto('frente', min(p['frente'], 0.60), nome)
            self._limpar_costmaps()
            time.sleep(1.0)   # o global volta a marcar a peca antes de replanejar
            return True

        def _girar_seguro(self, yaw_alvo, nome, tempo_max=40.0, movs_max=4):
            """v3.20: chega ao angulo yaw_alvo (frame map) parando para olhar antes de girar.

            Antes de cada giro confere pelo LiDAR se a quina passa (escolher_manobra, com o
            retangulo do Collision Monitor). Se nao passa, anda o minimo que libera o giro: para
            tras, para frente OU DE LADO, e olha de novo. Gira devagar (|wz| < 0,20: o Collision
            Monitor usa o retangulo menor). True se terminou a +-8 graus.
            """
            self._esperar_sensores(nome)   # SENSOR_PARADO
            pm = self._pose_mapa_yaw()
            od, sc = self._sensores()
            if pm is None or od is None:
                self.get_logger().warn('%s: sem pose/odom/LiDAR para girar' % nome)
                return False
            alvo = _ang(od[3] + _ang(yaw_alvo - pm[2]))   # alvo na odom (IMU): sem pulos do SLAM
            if self.em_preparo:
                tempo_max /= self.fator_corredor
            t0 = time.monotonic()
            movs = 0
            travou = False   # v3.30: depois de um travamento do CM, sempre olha o LiDAR antes de girar
            try:
                while rclpy.ok():
                    if time.monotonic() - t0 > tempo_max:
                        self.get_logger().warn('%s: tempo maximo para girar' % nome)
                        return False
                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO
                    if pausa > 0.0:
                        t0 += pausa
                        continue
                    od, sc = self._sensores()
                    if od is None:
                        self._parar()
                        self.get_logger().warn('%s: sem odom/LiDAR recente' % nome)
                        return False
                    err = _ang(alvo - od[3])
                    if abs(err) <= math.radians(6.0):
                        self._parar()
                        time.sleep(0.4)
                        od, _ = self._sensores(idade_scan=9.0)
                        err = _ang(alvo - od[3]) if od is not None else err
                        if abs(err) <= math.radians(8.0):
                            self.get_logger().info('%s: angulo %.0f graus (erro %.1f)' % (
                                nome, math.degrees(yaw_alvo), math.degrees(err)))
                            return True
                    ang = err
                    try:   # MANOBRA_FOLGA (12/11)
                        margem_m = max(0.0, min(0.15, float(self.p('manobra_margem'))))
                    except Exception:
                        margem_m = 0.0
                    m = escolher_manobra(sc[1], ang, margem=margem_m) if (abs(ang) > math.radians(10.0) or travou) \
                        else ('girar', ang)
                    if m[0] == 'girar':
                        if abs(m[1]) > math.pi:
                            self.get_logger().info('%s: pelo lado curto a quina pega; girando pelo outro '
                                                   'lado (%.0f graus)' % (nome, math.degrees(m[1])))
                        rapido = abs(m[1]) > math.radians(25.0) and \
                            not giro_bloqueado(sc[1], m[1], ret=RET_GIRO_RAPIDO)
                        if not self._girar_um_pouco(m[1], nome, rapido=rapido):
                            # o Collision Monitor travou o giro: olha de novo (conta como manobra)
                            travou = True
                            movs += 1
                            if movs > movs_max:
                                self.get_logger().warn('%s: o giro trava sempre' % nome)
                                return False
                        continue
                    self._parar()
                    if m[0] == 'preso' or movs >= movs_max:
                        self.get_logger().warn('%s: sem espaco para girar nem para me afastar' % nome)
                        return False
                    movs += 1
                    self.get_logger().info('%s: girando %.0f graus a quina pegaria: primeiro %.0f cm '
                                           'para %s' % (nome, math.degrees(ang), m[2] * 100, m[1]))
                    self._andar_reto(m[1], m[2], nome)
            finally:
                self._parar()
            return False

        def _girar_um_pouco(self, ang, nome, rapido=False):
            """Gira 'ang' rad (sinal = sentido; pode passar de 180 no giro pelo outro lado), medido
            pela odom, parando 6 graus antes (inercia; o laco de fora acerta o resto). False se o
            Collision Monitor travar o giro por 0,8 s."""
            sentido = math.copysign(1.0, ang)
            t_cm = None
            od, _ = self._sensores()
            if od is None:
                return True
            ant, girado = od[3], 0.0
            t0 = time.monotonic()
            prazo = (3.0 + abs(ang) / 0.12) / (self.fator_corredor if self.em_preparo else 1.0)
            while rclpy.ok():
                if self._novo_goal_chegou():
                    raise Cancelado()
                od, sc = self._sensores()
                if od is None:
                    return True   # o laco de fora trata
                girado += _ang(od[3] - ant)
                ant = od[3]
                if girado * sentido >= abs(ang) - math.radians(14.0 if rapido else 6.0) or \
                        time.monotonic() - t0 > prazo:
                    self._parar()
                    return True
                if self.cm_stop:
                    t_cm = t_cm or time.monotonic()
                    if time.monotonic() - t_cm > 0.8:
                        self._parar()
                        self.get_logger().info('%s: Collision Monitor travou o giro' % nome)
                        return False
                else:
                    t_cm = None
                self._publicar_vel(0.0, 0.0, sentido * (WZ_GIRO_RAPIDO if rapido else WZ_GIRO))
                time.sleep(0.05)
            return True

        def _giro_final(self, x, y, yaw, nome):
            """v3.32 GIRO_FINAL: ja na posicao de aproximacao, gira para o angulo da mesa pelo
            _girar_seguro e acerta os ultimos cm. 'ok' se ficou a menos de 25 graus (o encosto mede
            a face e corrige o resto), 'falha' se nao."""
            n = nome + ' (giro final)'
            self._girar_seguro(yaw, n, tempo_max=30.0, movs_max=4)
            pm = self._pose_mapa_yaw()
            for _ in range(2):
                self._deslocar_ate(x, y, n, tol=0.04)
                pm = self._pose_mapa_yaw()
                if pm is None or math.hypot(x - pm[0], y - pm[1]) <= 0.06:
                    break
            if pm is not None and abs(_ang(yaw - pm[2])) > math.radians(8.0):
                self._girar_seguro(yaw, n, tempo_max=15.0, movs_max=1)
                pm = self._pose_mapa_yaw()
            if pm is None:
                return 'falha'
            err = abs(_ang(yaw - pm[2]))
            self.get_logger().info('%s: %s (%.0f cm e %.0f graus do ponto)' % (
                n, 'SUCESSO' if err < math.radians(25.0) else 'FALHOU',
                100 * math.hypot(x - pm[0], y - pm[1]), math.degrees(err)))
            return 'ok' if err < math.radians(25.0) else 'falha'

        def _entrada_translada(self, area, gx, gy, gyaw, vigia):   # ENTRADA_TRANSLADA (24/11)
            """Ponto sem mesa onde o giro nao cabe: gira no ultimo ponto que cabe e entra transladando.
            None = nao se aplica (segue o fluxo de antes); 'ok' / 'falha' / 'interrompido'."""
            mapa = self._mapa()
            caminho = self._caminho(self._pose(gx, gy, gyaw)) if mapa is not None else None
            if not caminho or len(caminho) < 2:
                return None
            p = ponto_de_giro(caminho, gx, gy, gyaw, mapa, eixos=bool(self.p('caixa_lateral')))   # CAIXA_LATERAL
            if p is None:
                self.get_logger().info('ENTRADA_TRANSLADA: %s: o giro cabe no ponto ou nao achei ponto de giro: '
                                       'como antes' % area)
                return None
            n = 'ENTRADA %s' % area
            self.get_logger().info('ENTRADA_TRANSLADA: %s: giro nao cabe no ponto; giro em (%.2f, %.2f) para %.0f '
                                   'graus e entro transladando %.0f cm' % (
                                       area, p[0], p[1], math.degrees(gyaw), 100 * math.hypot(gx - p[0], gy - p[1])))
            pm = self._pose_mapa_yaw()
            if pm is None or math.hypot(p[0] - pm[0], p[1] - pm[1]) > 0.10:
                yaw_c = yaw_chegada(self._caminho(self._pose(p[0], p[1], gyaw)), p, pm)
                alvo = self._pose(p[0], p[1], yaw_c if yaw_c is not None else gyaw)
                self._raio_tmp = (min(float(self.p('raio_ponto_giro')),   # ENTRADA_ASSUME; RAIO_FORA (25/11)
                                      max(0.10, math.hypot(gx - p[0], gy - p[1]) - 0.15))
                                  if float(self.p('raio_ponto_giro')) > 0 else None)
                try:
                    r = self._navegar(alvo, n + ' (ponto de giro)', vigia, aceitar_perto=True)
                finally:
                    self._raio_tmp = None
                if r != 'ok':
                    return r
            self._girar_seguro(gyaw, n, tempo_max=30.0, movs_max=4)
            if self.p('caixa_lateral'):   # CAIXA_LATERAL (25/11): eixo cruzado primeiro, ainda fora da caixa
                pm = self._pose_mapa_yaw()
                if pm is not None:
                    dx, dy = gx - pm[0], gy - pm[1]
                    c, s_ = math.cos(pm[2]), math.sin(pm[2])
                    fx, ly = c * dx + s_ * dy, -s_ * dx + c * dy
                    if abs(ly) > abs(fx) and abs(fx) > 0.03:
                        self.get_logger().info('CAIXA_LATERAL: %s: acerto %+.0f cm de frente antes de entrar de lado '
                                               '(%.0f cm)' % (area, fx * 100, abs(ly) * 100))
                        self._andar_reto('frente' if fx > 0 else 'tras', min(abs(fx), 0.5), n, margem=0.0)
            d = None
            for _ in range(4):
                self._deslocar_ate(gx, gy, n, tol=0.04, passo_max=1.5,
                                   margem=0.0 if self.p('caixa_lateral') else 0.02)   # CAIXA_LATERAL
                pm = self._pose_mapa_yaw()
                d = None if pm is None else math.hypot(gx - pm[0], gy - pm[1])
                if d is not None and d <= 0.10:
                    break
            ok = d is not None and d <= 0.15
            self.get_logger().info('ENTRADA_TRANSLADA: %s: %s (%s do ponto)' % (
                area, 'SUCESSO' if ok else 'FALHOU', '?' if d is None else '%.0f cm' % (d * 100)))
            return 'ok' if ok else 'falha'

        def _deslocar_ate(self, x, y, nome, tol=0.03, passo_max=0.30, margem=0.02):
            """v3.20: ultimos cm ate (x, y) no mapa sem girar: de lado e depois frente/tras."""
            pm = self._pose_mapa_yaw()
            if pm is None:
                return
            dx, dy = x - pm[0], y - pm[1]
            c, s_ = math.cos(pm[2]), math.sin(pm[2])
            fx, ly = c * dx + s_ * dy, -s_ * dx + c * dy
            if abs(ly) > tol:
                self._andar_reto('esq' if ly > 0 else 'dir', min(abs(ly), passo_max), nome, margem)
            if abs(fx) > tol:
                self._andar_reto('frente' if fx > 0 else 'tras', min(abs(fx), passo_max), nome, margem)

        def _trajeto_curto(self, goal, livre, nome='TRAJETO CURTO'):
            """v3.23 (missao9): destino perto (<= curto_max), caminho reto e livre: sem Nav2.

            O RotationShim sempre vira o robo para o rumo do caminho (destino atras: gira, anda
            e gira de novo). Aqui: mesa -> gira para o angulo final (giro seguro) e anda de lado e
            de frente/re ate o ponto; ponto sem mesa -> nem gira. 'ok' ou None (usa o Nav2)."""
            dmax = float(self.p('curto_max'))
            pm = self._pose_mapa_yaw()
            if dmax <= 0.0 or pm is None:
                return None
            gx, gy = goal.pose.position.x, goal.pose.position.y
            gyaw = yaw_de(goal.pose.orientation)
            d = math.hypot(gx - pm[0], gy - pm[1])
            if d > dmax:
                return None
            caminho = self._caminho(goal)
            if not caminho or len(caminho) < 2:
                return None
            comp = sum(math.hypot(caminho[k + 1][0] - caminho[k][0], caminho[k + 1][1] - caminho[k][1])
                       for k in range(len(caminho) - 1))
            if comp > 1.25 * d + 0.05:
                return None   # o caminho desvia de alguma coisa: o Nav2 faz
            mapa = self._mapa()
            if mapa is not None:
                n = max(2, int(d / 0.05))
                for k in range(1, n + 1):
                    x = pm[0] + (gx - pm[0]) * k / n
                    y = pm[1] + (gy - pm[1]) * k / n
                    if not pose_livre(mapa, x, y, pm[2] if livre else gyaw, margem=0.02):
                        return None
            self.get_logger().info('%s: %.0f cm em linha reta, sem Nav2 (%s)' % (
                nome, d * 100, 'sem angulo' if livre else 'angulo %.0f graus' % math.degrees(gyaw)))
            if not livre and abs(_ang(gyaw - pm[2])) > math.radians(5.0):
                if not self._girar_seguro(gyaw, nome):
                    return None
            err = 9.0
            for _ in range(4):
                ant = err   # CURTO_SEM_PROGRESSO (13/11)
                self._deslocar_ate(gx, gy, nome, tol=0.03, passo_max=1.0)
                pm = self._pose_mapa_yaw()
                if pm is None:
                    return None
                err = math.hypot(gx - pm[0], gy - pm[1])
                if err <= 0.05:
                    break
                if ant < 9.0 and ant - err < 0.02 and self.p('curto_sem_progresso'):
                    self.get_logger().warn('%s: CURTO_SEM_PROGRESSO: a tentativa nao aproximou (%.0f -> %.0f cm): '
                                           'paro de repetir' % (nome, ant * 100, err * 100))
                    break
            if err > 0.10:   # v3.24: mesa tambem 10 cm (o encosto acerta o centro pelo LiDAR)
                self.get_logger().warn('%s: fiquei a %.0f cm do ponto: sigo pelo Nav2' % (nome, err * 100))
                return None
            if not livre and abs(_ang(gyaw - pm[2])) > math.radians(5.0):
                self._girar_seguro(gyaw, nome, tempo_max=15.0, movs_max=1)
            self.get_logger().info('%s: SUCESSO (%.0f cm do ponto)' % (nome, err * 100))
            return 'ok'

        def _chegar_preparo(self, e, nome):
            self.em_preparo = True
            try:
                return self._movimento_continuo(self._ajustar_pose_preparo, e, nome)
            finally:
                self.em_preparo = False

        def _definir_ganho_nav2(self, ganho):
            parametro = Parameter('fator_comando_corredor', value=ganho).to_parameter_msg()
            req = SetParametersAtomically.Request(parameters=[parametro])
            resposta = self.ganho_nav2.call(req, timeout_sec=2.0)
            if resposta is None or not resposta.result.successful:
                raise RuntimeError('Suavizador Nav2 não confirmou o fator %.2f' % ganho)

        def _definir_zona_morta(self, valores):
            p = Parameter('deadband_velocity', value=valores).to_parameter_msg()
            resposta = self.ajuste_suavizador.call(SetParametersAtomically.Request(parameters=[p]), timeout_sec=2.0)
            if resposta is None or not resposta.result.successful:
                raise RuntimeError('velocity_smoother não confirmou a zona morta do comando')

        def _restaurar_zona_morta(self):
            if self.zona_morta_original is not None:
                self._definir_zona_morta(self.zona_morta_original)
                self.zona_morta_original = None

        def _movimento_continuo(self, manobra, *args, **kwargs):
            # Manobras são sequenciais no trabalhador. O preparo pode chamar
            # o Nav2 por dentro; apenas a manobra externa muda e restaura o filtro.
            if self.zona_morta_original is not None or self.fator_corredor == 1.0:
                return manobra(*args, **kwargs)
            req = GetParameters.Request(names=['deadband_velocity'])
            resposta = self.consulta_suavizador.call(req, timeout_sec=2.0)
            if resposta is None or len(resposta.values) != 1 or len(resposta.values[0].double_array_value) != 3:
                raise RuntimeError('Zona morta do velocity_smoother indisponível')
            self.zona_morta_original = list(resposta.values[0].double_array_value)
            x, y, giro = self.zona_morta_original
            try:
                self._definir_zona_morta([x * self.fator_corredor, y * self.fator_corredor, giro])
                return manobra(*args, **kwargs)
            finally:
                self._restaurar_zona_morta()

        def _navegar_corredor(self, alvo, nome, **opcoes):
            # O DWB continua amostrando frente, ambos os lados e giro puro com
            # seus limites cinemáticos. Reduzimos o vetor já escolhido na saída.
            try:
                self._definir_ganho_nav2(self.fator_corredor)
                return self._movimento_continuo(self._navegar, alvo, nome, **opcoes)
            finally:
                self._definir_ganho_nav2(1.0)

        def _ajustar_pose_preparo(self, e, nome):
            """v3.20: chegar ao preparo sem o Nav2 girar encostado em parede.

            Longe (> 0,35 m): Nav2 ate a posicao SEM o giro final (chega com o rumo do caminho e
            aceita a 10 cm). Depois, aqui: girar pelo LiDAR (_girar_seguro: para, olha, se a
            quina pegaria anda de re, para frente ou de lado o minimo e gira) e corrigir os
            ultimos cm de lado/frente. Perto (refino): so o giro e o deslocamento, sem Nav2.
            Antes (missao5) o Nav2 girava com a quina na parede, o Collision Monitor travava, a
            recuperacao dava re e o DWB voltava ao mesmo canto (1 min 15 s, 2 tentativas).
            """
            if e.get('preparo_no_caminho'):
                self.get_logger().info('PREPARO_RETORNO: preparo na rota existente; eixo prolongado sem pose livre')
            pm = self._pose_mapa_yaw()
            if pm is None:
                return 'falha'
            self.get_logger().info('%s: x=%.2f y=%.2f yaw=%.0f graus' % (
                nome, e['x'], e['y'], math.degrees(e['yaw'])))
            if math.hypot(e['x'] - pm[0], e['y'] - pm[1]) > 0.35:
                alvo = self._pose(e['x'], e['y'], e['yaw'])
                yaw_c = yaw_chegada(self._caminho(alvo), (e['x'], e['y']), pm)
                r = self._navegar_corredor(self._pose(e['x'], e['y'], yaw_c if yaw_c is not None else e['yaw']),
                                           nome + ' (posicao)', aceitar_perto=True)
                if r != 'ok':
                    return r
            rapida = self.p('entrada_rapida')
            pm = self._pose_mapa_yaw() or pm
            if not rapida or abs(_ang(e['yaw'] - pm[2])) > math.radians(8.0):
                if not self._girar_seguro(e['yaw'], nome):
                    self.get_logger().warn('%s: FALHOU (giro)' % nome)
                    return 'falha'
            # v3.24 (missao10): ate 3 passes, conferindo pelo mapa (na volta da missao10 o SLAM
            # corrigiu o map->odom em 0,95 m / 11 graus durante o ajuste e o robo entrou na
            # travessia 20 cm fora do preparo)
            for _ in range(3):
                pm = self._pose_mapa_yaw()
                if pm is None:
                    break
                if rapida and math.hypot(e['x'] - pm[0], e['y'] - pm[1]) <= 0.06:
                    break
                self._deslocar_ate(e['x'], e['y'], nome)
                pm = self._pose_mapa_yaw()
                if pm is None:
                    break
                if abs(_ang(e['yaw'] - pm[2])) > math.radians(5.0):
                    self._girar_seguro(e['yaw'], nome, tempo_max=15.0, movs_max=1)
                    pm = self._pose_mapa_yaw()
                if pm is None or math.hypot(e['x'] - pm[0], e['y'] - pm[1]) <= 0.06:
                    break
            pm = self._pose_mapa_yaw()
            if pm is None:
                self.get_logger().warn('%s: sem pose final para confirmar o preparo' % nome)
                return 'falha'
            distancia = math.hypot(e['x'] - pm[0], e['y'] - pm[1])
            angulo = math.degrees(abs(_ang(e['yaw'] - pm[2])))
            if distancia > 0.06 or angulo > 8.0:
                self.get_logger().warn('%s: preparo fora da tolerancia (%.1f cm e %.1f graus)' %
                                       (nome, distancia * 100, angulo))
                return 'falha'
            self.get_logger().info('%s: SUCESSO (%.1f cm e %.1f graus do preparo no mapa)' %
                                   (nome, distancia * 100, angulo))
            return 'ok'

        def _travessia_lidar(self, info, nome, parada=None, lado=None, largura_b=None):
            return self._movimento_continuo(self._percorrer_com_lidar, info, nome, parada, lado, largura_b)

        def _percorrer_com_lidar(self, info, nome, parada=None, lado=None, largura_b=None):
            """Devolve 'ok' ou 'falha' (o chamador cai para a travessia pelo Nav2).

            parada (v3.11): (x, y) no mapa onde o CENTRO do robo deve parar (cruzamento com
            curva), em vez de passar margem_saida alem da saida."""
            sx, sy = info['saida']
            ax, ay = math.cos(math.radians(info['eixo_graus'])), math.sin(math.radians(info['eixo_graus']))
            margem = self.p('margem_saida')
            if parada is not None:
                sx, sy = parada
                margem = 0.0
            trav = Travessia(v_aprox=self.p('vel_aproximacao'), v_travessia=self.p('vel_travessia'),
                             tempo_max=60.0 / self.fator_corredor,
                             largura_padrao=self.p('largura_passagem'), parada_lidar=lado is not None,
                             log=lambda m: self.get_logger().info('%s: %s' % (nome, m)))
            if largura_b:
                trav.largura_b = largura_b
            try:   # FREIO_CURVA (12/11)
                trav.freio_curva_tol = float(self.p('freio_curva_tol'))
            except Exception:
                pass
            self.get_logger().info('%s: travessia pelo LiDAR ate (%.2f, %.2f) + %.2f m' % (
                nome, sx, sy, margem))
            seq = self.scan_seq
            bloqueado_desde = None
            sem_tf_desde = None
            restante = 9.0
            try:
                while rclpy.ok():
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    agora = time.monotonic()
                    with self.lock:
                        od, sc, sq, cf = self.odom, self.scan, self.scan_seq, self.cmd_final
                    trav.entrada_rapida = self.p('entrada_rapida')
                    trav.uma_parede = self.p('uma_parede')
                    if self.controle_t_ativo:
                        if (not self._precondicoes_t() or self.cm_stop or self.conflito_t[1]
                                or agora-self.conflito_t[0] > 0.3 or agora-cf[0] > 0.3):
                            trav.abortar('aproximacao experimental sem sensores/protecao/conflito')
                            break
                        pausa = 0.0
                    else:
                        self._conferir_evidencia_passagem()
                        pausa = self._esperar_sensores(nome)   # SENSOR_PARADO
                    if pausa > 0.0:
                        with self.lock:
                            od = self.odom
                        trav.retomar(pausa, time.monotonic(), od[1:])
                        bloqueado_desde = None
                        sem_tf_desde = None
                        continue
                    if od is None or agora - od[0] > 0.5:
                        trav.abortar('sem /odom recente')
                        break
                    if sq != seq and sc is not None and sc[2] is not None:
                        seq = sq
                        med = medir_corredor(sc[1], largura_conhecida=trav.largura or
                                             self.p('largura_passagem'))
                        trav.nova_medicao(sc[0], med, sc[2], sc[3])
                        # v3.28: folga real das laterais do corpo (pontos ao lado, da traseira ao para-choque)
                        y1 = FOOTPRINT[0][1]
                        lad = [(x, y) for x, y in sc[1] if FOOTPRINT[2][0] <= x <= FOOTPRINT[0][0] + 0.08]
                        fe = min([y - y1 for x, y in lad if y > y1] or [9.0])
                        fd = min([-y1 - y for x, y in lad if y < -y1] or [9.0])
                        trav.nova_folgas(sc[0], fe, fd)
                        # FREIO_FRENTE: pontos do corpo ficam em x ~0,31 e y ~0,33: so alem do para-choque
                        trav.nova_frente(sc[0], min([x - 0.365 for x, y in sc[1] if x > 0.385 and abs(y) < 0.30] or [9.0]))
                        if lado is not None and trav.estado == 'atravessar' and restante < 1.0:
                            # parede esperada (pelo mapa) a restante + centro_corpo + meia largura
                            x_esp = restante + self.p('centro_corpo') + (trav.largura_b or trav.largura or 0.82) / 2.0
                            trav.nova_transversal(sc[0], medir_parede_a_frente(
                                sc[1], lado, x_max=max(0.6, x_esp + 0.35)), sc[2], x_esp)
                    pm = (self._pose_mapa_yaw(idade_max=0.3) if self.controle_t_ativo else self._pose_mapa())
                    if pm is None and self.controle_t_ativo:
                        trav.abortar('TF ausente/desatualizada na aproximacao experimental')
                        break
                    if pm is None:
                        sem_tf_desde = sem_tf_desde or agora
                        if agora - sem_tf_desde > 1.0:
                            trav.abortar('sem TF map->base_link')
                            break
                    else:
                        sem_tf_desde = None
                        restante = margem - ((pm[0] - sx) * ax + (pm[1] - sy) * ay)
                    vx, vy, wz = trav.passo(agora, od[1:], restante)
                    if trav.estado in ('fim', 'abortar'):
                        break
                    self._publicar_vel(vx, vy, wz, corredor=True)
                    # Collision Monitor segurando o robo: STOP no estado do CM, /cmd_vel final
                    # zerado, ou /cmd_vel final parado de publicar (o CM em STOP para de publicar)
                    mandando = abs(vx) >= 0.015 or abs(vy) >= 0.015 or abs(wz) >= 0.015
                    preso = self.cm_stop or agora - cf[0] > 0.5 or cf[1]
                    if mandando and preso:
                        bloqueado_desde = bloqueado_desde or agora
                        if agora - bloqueado_desde > 1.5:
                            bloqueado_desde = None
                            trav.bloqueio(agora, od[1:])
                            if trav.estado == 'abortar':
                                break
                    else:
                        bloqueado_desde = None
                    time.sleep(0.05)
            finally:
                self._parar()
            ok = trav.estado == 'fim'
            # PASSAGEM_BLOQUEADA (11/11): guarda o que barrou a frente, com o scan do aborto (antes da re)
            self.trav_motivo, self.trav_obstaculo = trav.motivo, []
            if not ok and trav.motivo.startswith('algo a') and float(self.p('passagem_bloqueada_s')) > 0.0:
                with self.lock:
                    sc_ = self.scan
                if sc_ is not None:
                    self.trav_obstaculo = pontos_obstaculo_novo(self._mapa(), sc_[1], self._pose_mapa_yaw())
            if not ok and not self.p('nav2_principal') and not self.controle_t_ativo and trav.p_aprox_odom is not None:
                # se entrou na passagem, sai de re reto (mesmo eixo) antes de qualquer outra coisa
                with self.lock:
                    od = self.odom
                if od is not None:
                    p0 = trav.p_aprox_odom
                    avancou = (od[1] - p0[0]) * math.cos(p0[2]) + (od[2] - p0[1]) * math.sin(p0[2])
                    if avancou > 0.05:
                        self._recuar(avancou, od[3], nome, corredor=True)
            self.get_logger().info('%s (LiDAR): %s' % (nome, 'SUCESSO' if ok else 'FALHOU - ' + trav.motivo))
            return 'ok' if ok else 'falha'

        def _precondicoes_t(self):
            # Recusa antes de iniciar o preparo de uma curva experimental.
            with self.lock:
                tempos, raw = dict(self.tempos_t), self.scan_t_raw
            agora, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
            return (math.isfinite(self.p('limite_vel_angular_real_t'))
                    and self.p('limite_vel_angular_real_t') >= 0.10
                    and math.isfinite(self.p('limite_vel_linear_real_t'))
                    and self.p('limite_vel_linear_real_t') >= 0.03
                    and all(tempos.get(sensor) is not None
                        and tempos[sensor].recente(ros_ns, agora)
                        for sensor in ('lidar', 'imu', 'odom'))
                    and raw is not None and len(raw[1]) >= 3
                    and all(math.isfinite(r) and raw[4] <= r <= raw[5] for r in raw[1])
                    and 0 < abs(raw[3]) < math.pi/2
                    and (math.hypot(0.365, 0.315)+0.05+math.hypot(*raw[6][:2]))
                        *math.sin(abs(raw[3])) <= 0.05
                    and abs(abs(raw[3])*len(raw[1])-2*math.pi) <= 1e-5)

        def _verificar_cm_t(self):
            # Nao permitir correcao angular adicional DEPOIS do Collision Monitor.
            if not self.hw_params_t.wait_for_service(timeout_sec=1.0):
                return False
            hw_req = GetParameters.Request()
            hw_req.names = ['heading_hold_enabled', 'command_authority']
            hw = self._esperar(self.hw_params_t.call_async(hw_req), 1.0)
            if (hw is None or len(hw.values) != 2 or hw.values[0].type != 1 or hw.values[0].bool_value
                    or hw.values[1].string_value != 'collision_monitor'):
                return False
            if not self.cm_params_t.wait_for_service(timeout_sec=1.0):
                return False
            req = GetParameters.Request()
            req.names = ['polygons', 'SweptTurnStop.enabled', 'SweptTurnStop.min_points',
                         'cmd_vel_in_topic', 'cmd_vel_out_topic']
            for name in ('left_turn', 'right_turn'):
                req.names += ['SweptTurnStop.'+name+'.'+key for key in
                    ('points', 'linear_min', 'linear_max', 'theta_min', 'theta_max',
                     'direction_start_angle', 'direction_end_angle')]
            r = self._esperar(self.cm_params_t.call_async(req), 1.0)
            if r is None or len(r.values) != len(req.names):
                return False
            try:
                for i, direction in ((5, 1), (12, -1)):
                    v = r.values[i:i+7]
                    if (not envolve_disco(json.loads(v[0].string_value), math.hypot(.365,.315)+.05)
                            or v[1].double_value > 0 or v[2].double_value < math.hypot(.2,.16)
                            or v[3].double_value > (0.015 if direction > 0 else -0.6)
                            or v[4].double_value < (0.6 if direction > 0 else -0.015)
                            or v[5].double_value > -math.pi or v[6].double_value < math.pi):
                        return False
            except (ValueError, TypeError):
                return False
            return ('SweptTurnStop' in r.values[0].string_array_value
                    and r.values[1].bool_value and r.values[2].integer_value == 1
                    and r.values[3].string_value.lstrip('/') == 'cmd_vel_smoothed'
                    and r.values[4].string_value.lstrip('/') == 'cmd_vel')

        def _arbitro_exclusivo(self):
            saidas = self.get_publishers_info_by_topic('/cmd_vel_nav_seguro')
            entradas = self.get_publishers_info_by_topic('/cmd_vel_nav_t')
            return (len(saidas) == 1 and saidas[0].node_name == 'arbitro_t'
                    and len(entradas) == 1 and entradas[0].node_name == self.get_name()
                    and all(len(pubs) == 1 and pubs[0].node_name == esperado
                        for pubs, esperado in ((self.get_publishers_info_by_topic('/cmd_vel_smoothed'), 'velocity_smoother'),
                                               (self.get_publishers_info_by_topic('/cmd_vel'), 'collision_monitor'))))

        def _adquirir_t(self, gf):
            if not self.controle_t.wait_for_service(timeout_sec=1.0):
                gf.abortar('arbitro_t indisponivel; use navigation.launch.py atualizado')
                return False
            if not self._arbitro_exclusivo():
                gf.abortar('topologia de comandos sem exclusividade')
                return False
            if not self._verificar_cm_t():
                gf.abortar('protecao CM/autoridade do hardware/heading_hold nao confirmada')
                return False
            # Antes do lease: cancelar e confirmar o resultado terminal do Nav2.
            h = self.handle_ativo
            if h is not None:
                self._esperar(h.cancel_goal_async(), 2.0)
                r = self._esperar(h.get_result_async(), 3.0)
                if r is None or r.status not in (4, 5, 6):
                    gf.abortar('Nav2 nao confirmou parada')
                    return False
            req = SetBool.Request()
            req.data = True
            # Nao abandonar um futuro de aquisicao pendente por cancelamento:
            # a resposta deve ser consumida; falhas mantem o bloqueio seguro.
            f = self.controle_t.call_async(req)
            limite = time.monotonic()+2.0
            while not f.done() and time.monotonic() < limite:
                time.sleep(0.02)
            if not f.done():
                gf.abortar('timeout do arbitro')
                # Aquisicao tardia fica bloqueada pelo watchdog ate reset manual.
                return False
            adquirido = f.result().success
            if not adquirido:
                gf.abortar('outro controlador ativo')
                return False
            self.controle_t_ativo = True
            return True

        def _aproximar_t(self, info, curva, nome, parada, largura_b):
            guarda = GiroFrontal(curva['juncao'], curva['eixo_b'], curva['saida_b'])
            ok = False
            try:
                if not self._adquirir_t(guarda):
                    return 'falha'
                self._estado_t('aproximar_centro')
                r = self._travessia_lidar(info, nome, parada, None, largura_b)
                ok = r == 'ok'
                return r
            finally:
                if not ok:
                    self._parar()
                    self.controle_t_ativo = False
                    self._estado_t('abortar', guarda.motivo or 'aproximacao interrompida; watchdog estaciona')

        def _travessia_giro_frontal(self, curva, nome):
            """Lease exclusivo; todas as falhas terminam em zero, sem fallback lateral."""
            gf = GiroFrontal(curva['juncao'], curva['eixo_b'], curva['saida_b'],
                             margem=self.p('margem_saida_lateral'))
            adquirido = False
            try:
                adquirido = getattr(self, 'controle_t_ativo', False) or self._adquirir_t(gf)
                if not adquirido:
                    return 'falha'
                self._estado_t('parar')
                self._parar()
                while rclpy.ok():
                    with self.lock:
                        od, raw, imu, cf, conflito = (self.odom, self.scan_t_raw, self.imu_t,
                                                     self.cmd_final, self.conflito_t)
                        tempos = dict(self.tempos_t)
                    agora = time.monotonic()
                    pm = self._pose_mapa_yaw(idade_max=0.3)
                    if not self.odom_t_valido or od is None or not 0 <= agora-od[0] <= 0.3:
                        pm = None
                    # Giro: disco dos cantos + 5 cm. Avanco: footprint dilatado
                    # em 5 cm e mais 12 cm a frente para latencia/frenagem.
                    raio = math.hypot(0.365, 0.315)+0.05
                    caixa = (-0.255, 0.535, -0.365, 0.365) if gf.estado == 'avancar' else None
                    omega_max = self.p('limite_vel_angular_real_t')
                    v_max = self.p('limite_vel_linear_real_t')
                    idade = max(0.0, time.monotonic()-raw[0]) if raw else math.inf
                    dentro_limites = (math.isfinite(omega_max) and omega_max >= 0.10
                                      and math.isfinite(v_max) and v_max >= 0.03
                                      and imu is not None and abs(imu[2]) <= omega_max)
                    observacao = (avaliar_varredura(*raw[1:], raio, caixa=caixa,
                                      movimento_angular=omega_max*idade,
                                      movimento_linear=v_max*idade)
                                  if raw is not None and dentro_limites else Observacao.DESCONHECIDA)
                    agora = time.monotonic()
                    ros_ns = self.get_clock().now().nanoseconds
                    recentes = all(tempos.get(nome_sensor) is not None
                                   and tempos[nome_sensor].recente(ros_ns, agora)
                                   for nome_sensor in ('lidar', 'imu', 'odom', 'saida'))
                    if not recentes:
                        gf.abortar('idade total invalida/atraso de aquisicao desconhecido')
                    elif observacao is not Observacao.LIVRE:
                        gf.abortar('area varrida: '+observacao.value)
                    cmd = gf.passo(agora, pm, imu, raw[0] if raw else None,
                                   observacao is Observacao.LIVRE,
                                   agora-cf[0] <= 0.3 and cf[1],
                                   cancelado=self._novo_goal_chegou(),
                                   bloqueado=(self.cm_stop or conflito[1] or agora-conflito[0] > 0.3
                                              or agora-cf[0] > 0.3 or not self._arbitro_exclusivo()))
                    self._estado_t(gf.estado, gf.motivo, observacao.value)
                    self._publicar_vel(*cmd)
                    if gf.estado in ('fim', 'abortar'):
                        break
                    time.sleep(0.05)
            finally:
                self._parar()
                if adquirido and gf.estado == 'fim':
                    req = SetBool.Request()
                    req.data = False
                    self._estado_t('devolvendo')
                    confirmado, motivo = devolver_controle(self.controle_t, req,
                        lambda: self._publicar_vel(0.0, 0.0, 0.0))
                    if not confirmado:
                        gf.abortar(motivo)
                # Em falha, watchdog estaciona o arbitro sem liberar fontes normais.
                self.controle_t_ativo = False
                self._estado_t(gf.estado, gf.motivo)
                self.get_logger().info('%s giro frontal: %s %s' % (nome, gf.estado, gf.motivo))
            return 'ok' if gf.estado == 'fim' else 'falha'

        def _travessia_lateral(self, curva, nome):
            return self._movimento_continuo(self._percorrer_de_lado, curva, nome)

        def _percorrer_de_lado(self, curva, nome):
            """v3.11: do cruzamento, sai de lado pelo corredor novo. Devolve 'ok' ou 'falha'."""
            bx, by = curva['eixo_b']
            sx, sy = curva['saida_b']
            margem = self.p('margem_saida_lateral')
            with self.lock:
                od0 = self.odom
            pm0 = self._pose_mapa()
            dist = (margem - ((pm0[0] - sx) * bx + (pm0[1] - sy) * by)) if pm0 else 2.5
            tl = TravessiaLateral(curva['lado'], fundido=self.p('angulo_fundido'), k_ff=self.p('fundido_k_ff'),
                                  v_lateral=self.p('vel_lateral'),
                                  largura_padrao=self.p('largura_passagem'),
                                  tempo_max=(40.0 + 30.0 * max(0.5, dist)) / self.fator_corredor,
                                  log=lambda m: self.get_logger().info('%s: %s' % (nome, m)))
            saida = SaidaCurva(curva, FOOTPRINT, self.p('abertura_saida_curva'))
            abertura, folga_oposta, oposto_aberto = 0, None, False
            tl.restante_fora = margem - 0.35   # v3.18: corpo inteiro alem da saida do corredor
            try:   # RECUO_CURVA (12/11)
                tl.recuo_curva = bool(self.p('recuo_curva'))
            except Exception:
                pass
            try:   # PULSO_GIRO_ADAPTA (13/11)
                tl.pulso_adapta = bool(self.p('pulso_giro_adapta'))
            except Exception:
                pass
            try:   # ALINHAR_CRUZAMENTO (19/11)
                tl.alinhar_cruz = bool(self.p('alinhar_cruzamento'))
            except Exception:
                pass
            try:   # ALINHAR_CRUZAMENTO_ZERO (19/11 noite)
                tl.cruz_zero = bool(self.p('alinhar_cruzamento_zero'))
            except Exception:
                pass
            self.get_logger().info('%s: saindo de lado para a %s ate (%.2f, %.2f) + %.2f m' % (
                nome, 'ESQUERDA' if curva['lado'] > 0 else 'DIREITA', sx, sy, margem))
            seq = self.scan_seq
            bloqueado_desde = None
            sem_tf_desde = None
            restante = 9.0
            try:
                while rclpy.ok():
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    agora = time.monotonic()
                    with self.lock:
                        od, sc, sq, cf = self.odom, self.scan, self.scan_seq, self.cmd_final
                    self._conferir_evidencia_passagem()
                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO
                    if pausa > 0.0:
                        with self.lock:
                            od = self.odom
                        tl.retomar(pausa, time.monotonic(), od[1:])
                        saida.interromper()
                        abertura = 0
                        bloqueado_desde = None
                        sem_tf_desde = None
                        continue
                    if od is None or agora - od[0] > 0.5:
                        tl.abortar('sem /odom recente')
                        break
                    novo_scan = sq != seq and sc is not None and sc[2] is not None
                    if novo_scan:
                        seq = sq
                        med = medir_corredor_lateral(sc[1], tl.lado, tl.largura, self.p('centro_corpo'))
                        folgas = folgas_lateral(sc[1], tl.lado)
                        tl.nova_medicao(sc[0], med, sc[2])
                        tl.nova_folga(sc[0], *folgas)
                    pm = self._pose_mapa_yaw(idade_max=0.3)
                    if pm is None:
                        # TF_TOLERA (09/11, missao27): abortava na 1a leitura falha (em 06/11 tolerava
                        # 1 s). Agora para e espera ate 1 s; se voltar, retoma medindo de novo.
                        saida.interromper()
                        abertura = 0
                        sem_tf_desde = sem_tf_desde or agora
                        if agora - sem_tf_desde > 1.0:
                            tl.abortar('TF map->base_link ausente/desatualizada; restante desconhecido')
                            break
                        self._publicar_vel(0.0, 0.0, 0.0)
                        time.sleep(0.05)
                        continue
                    else:
                        if sem_tf_desde is not None:
                            tl.retomar(agora - sem_tf_desde, agora, od[1:])
                            self.get_logger().info('%s: TF_TOLERA: TF voltou apos %.2f s parado' % (
                                nome, agora - sem_tf_desde))
                        sem_tf_desde = None
                        restante = margem - ((pm[0] - sx) * bx + (pm[1] - sy) * by)
                    tl.gyro = self.gyro_z
                    if tl.estado == 'abortar':
                        break
                    if sc is None or not 0 <= self.get_clock().now().nanoseconds * 1e-9 - sc[4] <= 0.45:
                        saida.interromper()
                        abertura = 0
                    elif novo_scan:
                        abertura = saida.observar(sc[4], sc[1], self.laser_tf, self._mapa(), pm, sc[2],
                                                 med is not None and med['lados'] == 2 and all(folgas[2:]))
                        if abertura:
                            folga_oposta = folgas[1 if abertura > 0 else 0]
                            oposto_aberto = folga_oposta is None and abertura_observada(
                                sc[1], self.laser_tf, -abertura, 0.07, FOOTPRINT)
                    if abertura or saida.finalizando:
                        if not saida.finalizando:
                            self.get_logger().info('%s: corpo fora da curva, abertura de %.2f m confirmada %s; '
                                                   'verificando folga e parada' % (nome, saida.abertura,
                                                   'à frente' if abertura > 0 else 'atrás'))
                        acao = saida.finalizar(agora, abertura, od[1:], folga_oposta, oposto_aberto,
                                              not self.cm_stop and agora - cf[0] < 0.3,
                                              cf[1] and abs(tl.gyro) < 0.03)
                        if acao == 'afastar' and saida.primeiro_afastamento == agora:
                            self.get_logger().info('%s: folga oposta %.1f cm; afastando para o lado aberto até 7 cm'
                                                   % (nome, 100 * folga_oposta))
                        if acao == 'fim':
                            tl.estado = 'fim'
                            self.get_logger().info('%s: saída da curva confirmada pelo LiDAR e mapa; robô parado' % nome)
                        elif acao == 'bloqueada':
                            tl.abortar('abertura identificada, mas sem confirmar folga e parada para sair')
                        vx, vy, wz = (abertura * tl.v_aj if acao == 'afastar' else 0.0), 0.0, 0.0
                    else:
                        vx, vy, wz = tl.passo(agora, od[1:], restante)
                    if tl.estado in ('fim', 'abortar'):
                        break
                    self._publicar_vel(vx, vy, wz, corredor=True)
                    mandando = abs(vx) >= 0.015 or abs(vy) >= 0.015 or abs(wz) >= 0.015
                    if mandando and (self.cm_stop or agora - cf[0] > 0.5 or cf[1]):
                        bloqueado_desde = bloqueado_desde or agora
                        if agora - bloqueado_desde > 1.5:
                            bloqueado_desde = None
                            tl.bloqueio(agora, od[1:])
                            if tl.estado == 'abortar':
                                break
                    else:
                        bloqueado_desde = None
                    time.sleep(0.05)
            finally:
                self._parar()
            ok = tl.estado == 'fim'
            self.get_logger().info('%s: %s' % (nome, 'SUCESSO' if ok else 'FALHOU - ' + tl.motivo))
            # CURVA_INSISTE_SEGURO: so timeout permite nova tentativa; CM, TF e sensores encerram.
            return 'ok' if ok else ('repetir' if tl.motivo == 'tempo maximo' else 'falha')

        def _encostar(self, nome):
            """Da pose de aproximacao: alinha com a face pelo LiDAR e avanca ate a distancia."""
            try:
                import yaml
                with open(self.p('arquivo_areas')) as fp:
                    face = (yaml.safe_load(fp) or {})['service_areas'][nome]['face']
                rastreio = RastreioFaceWS(face, usar_mapa=bool(self.p('encosto_referencia_mapa')))  # BASELINE_LEGADO
            except (OSError, KeyError, TypeError, ValueError) as exc:
                self._parar()
                self._publicar_resultado(nome, False, 'referencia WS indisponivel: '+str(exc))
                return
            enc = Encosto(linear=self.p('encosto_linear'),
                          distancia_alvo=self.p('distancia_encosto'), para_choque=self.p('para_choque'),
                          log=lambda m: self.get_logger().info('ENCOSTAR %s: %s' % (nome, m)))
            if not self.p('encosto_recuo_cego'):   # RECUO_CEGO
                enc.recuo_cego_max = 0
            self.get_logger().info('ENCOSTAR %s: medindo a face da mesa' % nome)
            seq = self.scan_seq
            bloqueado_desde = None
            motivo_ant = ''
            try:
                while rclpy.ok():
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    agora = time.monotonic()
                    with self.lock:
                        od, sc, sq, cf = self.odom, self.scan, self.scan_seq, self.cmd_final
                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO
                    if pausa > 0.0:
                        with self.lock:
                            od = self.odom
                        enc.retomar(pausa, time.monotonic(), od[1:])
                        bloqueado_desde = None
                        continue
                    if od is None or agora - od[0] > 0.5:
                        enc.abortar('sem /odom recente')
                        break
                    if sq != seq and sc is not None and sc[2] is not None:
                        seq = sq
                        pm = self._pose_mapa_yaw(idade_max=0.3)
                        pose_scan = pose_mapa_no_scan(pm,od[1:],sc[2]) if pm is not None else None
                        med = rastreio.medir(sc[1],sc[0],sc[2],pose_scan)
                        enc.nova_medicao(sc[0], med, sc[2])
                        enc.obstaculo = obstaculo_antes_da_face(sc[1],med,enc.pc)
                        if med is None:
                            self.get_logger().info('ENCOSTAR %s: %s' % (nome,rastreio.motivo))
                        elif rastreio.motivo != motivo_ant:   # FACE_FRAGMENTADA: loga a troca de modo
                            self.get_logger().info('ENCOSTAR %s: %s (d%.3f c%s a%+.1fg)' % (
                                nome, rastreio.motivo, med['dist'],
                                '%+.3f' % med['centro'] if med['centro'] is not None else ' nd',
                                math.degrees(med['ang'])))
                        motivo_ant = rastreio.motivo
                    vx, vy, wz = enc.passo(agora, od[1:])
                    if enc.estado in ('fim', 'abortar'):
                        break
                    self._publicar_vel(vx, vy, wz)
                    mandando = abs(vx) >= 0.015 or abs(vy) >= 0.015 or abs(wz) >= 0.015
                    if mandando and (self.cm_stop or agora - cf[0] > 0.5 or cf[1]):
                        bloqueado_desde = bloqueado_desde or agora
                        if agora - bloqueado_desde > 1.5:
                            bloqueado_desde = None
                            enc.bloqueio(agora, od[1:])
                            if enc.estado == 'abortar':
                                break
                    else:
                        bloqueado_desde = None
                    time.sleep(0.05)
            finally:
                self._parar()
            if enc.estado == 'fim':
                self.encostado = True
                self.get_logger().info('ENCOSTAR %s: SUCESSO' % nome)
                r = enc.resultado or {}
                det = 'folga=%.1fcm ang=%+.1f centro=%s' % (
                    r.get('folga', 0.0) * 100, math.degrees(r.get('ang', 0.0)),
                    ('%+.1fcm' % (r['centro'] * 100)) if r.get('centro') is not None else 'nd')
                try:   # CORRIGE_POSE_MESA (24/11): nunca derruba o encosto
                    self._corrigir_pose_mesa(nome, face, r, enc.pc)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error('CORRIGE_POSE_MESA: erro %s' % exc)
                self._publicar_resultado(nome, True, det)
            else:
                self.get_logger().warn('ENCOSTAR %s: FALHOU - %s' % (nome, enc.motivo))
                # se ficou perto da mesa, o proximo goal tambem sai de re antes de girar
                if enc.meds and enc.meds[-1][1]['dist'] - enc.pc < 0.25:
                    self.encostado = True
                self._publicar_resultado(nome, False, 'encosto: %s' % enc.motivo)

        def _corrigir_pose_mesa(self, nome, face, r, pc):   # CORRIGE_POSE_MESA (24/11)
            if not self.p('corrige_pose_mesa') or r.get('folga') is None or r.get('ang') is None:
                return
            if 'amcl' not in self.get_node_names():
                self.get_logger().info('CORRIGE_POSE_MESA: %s: sem AMCL (SLAM): nao corrijo' % nome)
                return
            pm = self._pose_mapa_yaw()
            nova = pose_pela_face(face, r['folga'] + pc, r['ang'], r.get('centro'), pm)
            if nova is None or pm is None:
                self.get_logger().warn('CORRIGE_POSE_MESA: %s: sem pose atual no mapa' % nome)
                return
            dd = math.hypot(nova[0] - pm[0], nova[1] - pm[1])
            da = abs(math.atan2(math.sin(nova[2] - pm[2]), math.cos(nova[2] - pm[2])))
            txt = '%s: AMCL (%.3f, %.3f, %.1f) -> mesa (%.3f, %.3f, %.1f): %.1f cm, %.1f graus%s' % (
                nome, pm[0], pm[1], math.degrees(pm[2]), nova[0], nova[1], math.degrees(nova[2]),
                dd * 100, math.degrees(da), '' if r.get('centro') is not None else ' (centro nao medido: so profundidade/angulo)')
            if dd > 0.40 or da > math.radians(15):
                self.get_logger().warn('CORRIGE_POSE_MESA: NAO corrijo (grande demais, face/mesa duvidosa) ' + txt)
                return
            if dd < 0.02 and da < math.radians(1.0):
                self.get_logger().info('CORRIGE_POSE_MESA: ja certo ' + txt)
                return
            from geometry_msgs.msg import PoseWithCovarianceStamped
            if getattr(self, 'pub_init_mesa', None) is None:
                self.pub_init_mesa = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            m = PoseWithCovarianceStamped()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = nova[0], nova[1]
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(nova[2] / 2), math.cos(nova[2] / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.03 ** 2
            m.pose.covariance[35] = math.radians(2.0) ** 2
            self.pub_init_mesa.publish(m)
            self.get_logger().warn('CORRIGE_POSE_MESA: corrigido ' + txt)
            time.sleep(0.5)

        def _publicar_resultado(self, nome, ok, texto):
            if ok:
                self.rota_tent = {}   # ROTA_MAPA
            self.res_area = nome
            m = String()
            m.data = '%s %s %s' % (nome, 'SUCESSO' if ok else 'FALHOU', texto)
            self.pub_resultado.publish(m)
            self.get_logger().info('RESULTADO: %s' % m.data)

        def _recuar(self, distancia, yaw_alvo, nome, *, corredor=False):
            """Re reta segurando o yaw pelo IMU (distancia em unidades do odom)."""
            self.get_logger().info('%s: saindo de re %.2f m (odom) para tentar de novo' % (
                nome, distancia))
            with self.lock:
                od = self.odom
            if od is None:
                return
            x0, y0 = od[1], od[2]
            t0 = time.monotonic()
            bloqueado_desde = None
            try:
                prazo = 12.0 / self.fator_corredor if corredor else 12.0
                while rclpy.ok() and time.monotonic() - t0 < prazo:
                    if self._novo_goal_chegou():
                        raise Cancelado()
                    pausa = self._esperar_sensores(nome)   # SENSOR_PARADO
                    if pausa > 0.0:
                        t0 += pausa
                        bloqueado_desde = None
                        continue
                    with self.lock:
                        od, cf = self.odom, self.cmd_final
                    if math.hypot(od[1] - x0, od[2] - y0) >= distancia:
                        break
                    agora = time.monotonic()
                    if self.cm_stop or agora - cf[0] > 0.5 or cf[1]:
                        bloqueado_desde = bloqueado_desde or agora
                        if agora - bloqueado_desde > 2.0:
                            self.get_logger().warn('%s: re bloqueada pelo Collision Monitor' % nome)
                            break
                    else:
                        bloqueado_desde = None
                    err = _ang(yaw_alvo - od[3])
                    wz = _sat(0.5 * err, 0.016, 0.03) if abs(err) > math.radians(1.0) else 0.0
                    self._publicar_vel(-0.03, 0.0, wz, corredor=corredor)
                    time.sleep(0.05)
            finally:
                self._parar()

        def _publicar_etapas(self, etapas):
            pa = PoseArray()
            pa.header.frame_id = 'map'
            pa.header.stamp = self.get_clock().now().to_msg()
            for e in etapas:
                pose = Pose()
                pose.position.x, pose.position.y = e['x'], e['y']
                pose.orientation.z = math.sin(e['yaw'] / 2.0)
                pose.orientation.w = math.cos(e['yaw'] / 2.0)
                pa.poses.append(pose)
            self.pub_poses.publish(pa)

        # ---------------- logica principal ----------------
        def trabalhador(self):
            while rclpy.ok():
                self.evento.wait(0.5)
                with self.lock:
                    goal = self.goal_pendente
                    area = self.area_pendente
                    self.goal_pendente = None
                    self.area_pendente = None
                    self.evento.clear()
                if goal is None:
                    continue
                self.get_logger().info('TRABALHADOR RECEBEU: %s' % (area or 'goal sem area'))
                self.cancelamento_t.clear()
                try:
                    self._restaurar_zona_morta()  # repete se o serviço falhou ao terminar a manobra anterior
                    if self.p('nav2_principal'):
                        self._drenar_nav()
                    if self.encostado:
                        # de frente para a mesa a 8 cm, girar bate a quina: sai de re reto antes
                        self.encostado = False
                        with self.lock:
                            od = self.odom
                        if od is not None:
                            self._recuar(self.p('recuo_saida'), od[3], 'SAIR DA MESA')
                    self.res_area = None
                    self._saida_caixa(area)   # SAIDA_CAIXA (25/11)
                    if (self._rota_retas(goal, area) == 'ok' and area is not None
                            and area in self.sem_encosto):   # ROTA_RETAS (25/11): ponto sem mesa: chegou
                        self._publicar_resultado(area, True, 'chegou (ROTA_RETAS)')
                    else:
                        self.executar(goal, area)
                    if area is not None and self.res_area is None:
                        self._publicar_resultado(area, False, 'nao chegou na pose de aproximacao '
                                                 '(navegacao, preparo ou passagem falhou)')
                except Cancelado:
                    self.get_logger().info('Goal substituido por um novo.')
                    if area is not None and self.res_area is None:
                        self._publicar_resultado(area, False, 'cancelado por outro goal')
                except Exception as exc:
                    self.get_logger().error('Erro: %s' % exc)
                    if area is not None and self.res_area is None:
                        self._publicar_resultado(area, False, 'erro: %s' % exc)
                finally:
                    if self.p('nav2_principal'):
                        try:
                            self._drenar_nav()
                            if self.dono_mov == 'especial':
                                self._parar()
                        except Exception as exc:
                            self.get_logger().error('Autoridade bloqueada: %s' % exc)
                        finally:
                            self.dono_mov = None


        def _heartbeat_mov(self):
            if self.p('nav2_principal') and self.dono_mov:
                m = TwistStamped(); m.header.frame_id = self.dono_mov
                m.header.stamp = self.get_clock().now().to_msg()
                self.pub_heartbeat.publish(m)

        def _drenar_nav(self):
            """Espera terminal, nao apenas resposta de cancelamento; nao ignora preempcao."""
            limite = time.monotonic() + 5.0
            if self.nav_pendente is not None:
                while not self.nav_pendente.done() and time.monotonic() < limite:
                    time.sleep(.02)
                if not self.nav_pendente.done():
                    self.dono_mov = None
                    raise RuntimeError('aceitacao Nav2 pendente; autoridade bloqueada')
                h = self.nav_pendente.result()
                self.nav_pendente = None
                if h is not None and h.accepted:
                    self.handle_ativo = h
                    self.nav_resultado = h.get_result_async()
            if self.handle_ativo is not None and self.nav_resultado is not None:
                if not self.nav_resultado.done():
                    self.handle_ativo.cancel_goal_async()
                while not self.nav_resultado.done() and time.monotonic() < limite:
                    time.sleep(.02)
                if not self.nav_resultado.done():
                    self.dono_mov = None
                    raise RuntimeError('Nav2 sem resultado terminal; autoridade bloqueada')
            self.handle_ativo = None
            self.nav_resultado = None

        def _autoridade_mov(self, dono):
            if self.dono_mov == dono:
                return
            self._drenar_nav()
            if self._novo_goal_chegou():
                raise Cancelado()
            if not self.mov_cli.wait_for_service(timeout_sec=2.0):
                raise RuntimeError('arbitro_nav_principal indisponivel')
            req = SetBool.Request(); req.data = dono == 'especial'
            self.dono_mov = None
            futuro = self.mov_cli.call_async(req)
            # Uma troca atrasada nao autoriza enviar movimento sem resposta.
            resposta = self._esperar(futuro, 3.0)
            if resposta is None or not resposta.success:
                self.dono_mov = None
                raise RuntimeError('troca de autoridade nao confirmada')
            self.dono_mov = dono
            self._heartbeat_mov()

        def _log_nav_principal(self, nivel, mensagem):
            # LOGGER_SEVERIDADE_20261111: rclpy fixa severidade por callsite.
            texto = 'NAV2_PRINCIPAL: '+mensagem
            if nivel == 'info':
                self.get_logger().info(texto)
            elif nivel in ('warn', 'warning'):
                self.get_logger().warning(texto)
            elif nivel == 'error':
                self.get_logger().error(texto)
            else:
                raise ValueError('nivel de log desconhecido: '+nivel)

        def _navegar_principal(self, alvo, nome, vigia=None):
            # Sem pre-giro, atalho, anti-ciscado paralelo ou reenviar falha autonoma.
            self._autoridade_mov('nav2')
            if not self.nav.wait_for_server(timeout_sec=5.0):
                self._log_nav_principal('error', 'navigate_to_pose indisponivel; verifique configuracao/lifecycle do Nav2')
                return 'falha'
            if self._novo_goal_chegou():
                raise Cancelado()
            self._log_nav_principal('info', 'enviando goal ao Nav2')
            req = NavigateToPose.Goal(); req.pose = alvo
            self.nav_pendente = self.nav.send_goal_async(req)
            def cancelar_tardio(f):
                h = f.result()
                if h is not None and h.accepted and (self.dono_mov != 'nav2' or self._novo_goal_chegou()):
                    h.cancel_goal_async()
            self.nav_pendente.add_done_callback(cancelar_tardio)
            try:
                h = self._esperar(self.nav_pendente, 5.0)
                if h is None:
                    self._log_nav_principal('error', 'Nav2 nao confirmou recebimento do goal em 5 s')
                    return 'falha'
                self.nav_pendente = None
                if not h.accepted:
                    self._log_nav_principal('error', 'goal RECUSADO pelo Nav2; confira bt_navigator ativo')
                    return 'falha'
                self._log_nav_principal('info', 'goal aceito pelo Nav2')
                self.handle_ativo = h
                self.nav_resultado = h.get_result_async()
                proxima = time.monotonic() + self.p('periodo_vigia')
                while True:
                    r = self._esperar(self.nav_resultado, .1)
                    if r is not None:
                        if r.status == GoalStatus.STATUS_SUCCEEDED:
                            return 'ok'
                        # Se abortou antes do periodo de vigia, ainda pode haver passagem confirmada
                        # no plano DESTE goal; cancelado/preemptado nunca aciona manobra.
                        if r.status == GoalStatus.STATUS_ABORTED and not self._novo_goal_chegou() and vigia and vigia():
                            return 'interrompido'
                        payload = getattr(r, 'result', None)
                        self._log_nav_principal('warn', 'goal terminou: status=%s codigo=%s motivo=%s' % (
                            r.status, getattr(payload, 'error_code', '?'), getattr(payload, 'error_msg', '')))
                        return 'falha'
                    if vigia and time.monotonic() >= proxima:
                        proxima = time.monotonic() + self.p('periodo_vigia')
                        if vigia():
                            return 'interrompido'
            finally:
                self._drenar_nav()

        def _esperar_parada_supervisor(self):
            # Retira heartbeat; lease expira e arbitro zera pela cadeia existente.
            self.dono_mov = None
            limite = time.monotonic()+3.0
            estavel = None
            self.icp_parado_desde = None
            while time.monotonic() < limite:
                if self._novo_goal_chegou():
                    raise Cancelado()
                self._conferir_evidencia_passagem()
                agora = time.monotonic()
                with self.lock:
                    cf = self.cmd_final
                if agora-cf[0] < .3 and cf[1]:
                    estavel = estavel or agora
                    fisico = (self.evidencia_passagem is None or (self.icp_parado_desde is not None and agora-self.icp_parado_desde >= .4 and abs(self.gyro_z) < .03))
                    if agora-estavel >= .4 and fisico:
                        return
                else:
                    estavel = None
                time.sleep(.02)
            raise RuntimeError('parada final nao confirmada')

        def _iniciar_evidencia_passagem(self):
            from supervisor_passagem import Evidencia
            with self.lock:
                sc, imu = self.scan, self.imu_supervisor
            if sc is None or imu is None or time.monotonic()-min(sc[0],imu[0]) > .5:
                raise RuntimeError('sensores ausentes antes da passagem')
            self.evidencia_passagem = Evidencia(self._pose_mapa_yaw(idade_max=.3), imu[1], sc[0])
            self.scan_evidencia = sc
            self.icp_parado_desde = None

        def _conferir_evidencia_passagem(self):
            if self.evidencia_passagem is None:
                return
            from recuperacao_avancada import medir_deslocamento
            with self.lock:
                sc, imu = self.scan, self.imu_supervisor
            agora = time.monotonic()
            if sc is None or imu is None or agora-min(sc[0],imu[0]) > .5:
                raise RuntimeError('sensor obsoleto durante passagem')
            if sc[0]-self.scan_evidencia[0] < .25:
                return
            m = medir_deslocamento(self.scan_evidencia[1], sc[1])
            self.evidencia_passagem.atualizar(self._pose_mapa_yaw(idade_max=.3), imu[1], sc[0], m)
            if math.hypot(m.dx,m.dy) < .01 and abs(m.dth) < .01 and abs(self.gyro_z) < .03:
                self.icp_parado_desde = self.icp_parado_desde or sc[0]
            else:
                self.icp_parado_desde = None
            self.scan_evidencia = sc
            if time.monotonic()-sc[0] > .5:
                raise RuntimeError('ICP excedeu prazo do sensor')

        def _confirmar_saida_supervisor(self, saida, eixo):
            from supervisor_passagem import saida_livre
            self._parar()
            self._esperar_parada_supervisor()
            self._conferir_evidencia_passagem()
            ev = self.evidencia_passagem
            with self.lock:
                sc = self.scan
            a = math.atan2(eixo[1],eixo[0])-ev.pose[2]
            livre = saida_livre(sc[1], (math.cos(a),math.sin(a)))
            if not ev.completa(saida,eixo,FOOTPRINT,livre,True):
                raise RuntimeError('saida incompleta/incerta/obstruida; retomada proibida')
            self.evidencia_passagem = None

        def _executar_principal(self, goal, area=None):
            from controle_nav_principal import passagem_confirmada
            import yaml
            from supervisor_passagem import Supervisor, regiao_conhecida
            with open(self.p('supervisor_regioes')) as arquivo_regioes:
                regioes = yaml.safe_load(arquivo_regioes)['regioes']
            self.supervisor = Supervisor(goal, self.get_logger().info, self.p('supervisor_modo'))
            goal = self.supervisor.goal
            tratadas = self.supervisor.tratadas
            # Todo plano deve pertencer ao envio atual, nao ao goal anterior.
            limite_passagens = max(0, int(self.p('max_passagens')))
            try:
                while True:
                    alerta = {}
                    inicio = time.monotonic()
                    def vigia():
                        if len(tratadas) >= limite_passagens:
                            return False
                        with self.lock:
                            pontos, plano_t, sc = self.plano, self.plano_t, self.scan
                        if plano_t < inicio or time.monotonic()-plano_t > 16.0 or sc is None or time.monotonic()-sc[0] > .5:
                            return False
                        pm = self._pose_mapa_yaw(idade_max=.3)
                        if pm is None or not pontos:
                            return False
                        # Plano precisa terminar no goal atual (inclui goals RViz concorrentes).
                        gx, gy = goal.pose.position.x, goal.pose.position.y
                        if math.hypot(pontos[-1][0]-gx, pontos[-1][1]-gy) > .15:
                            return False
                        # Nav2 mantem o plano por ate 15 s; analisa somente o trecho restante.
                        k = min(range(len(pontos)), key=lambda i:(pontos[i][0]-pm[0])**2+(pontos[i][1]-pm[1])**2)
                        pontos = pontos[k:]
                        etapas, info = planejar_etapas(self._mapa(), pontos, (gx,gy),
                            self.p('largura_estreita'), self.p('distancia_preparo'),
                            self.p('distancia_saida'), ignorar_inicio=0.0, ja_tratadas=tratadas+self.passagens_supervisor.bloqueadas(pm)) if self._mapa() else ([],None)
                        if not etapas:
                            return False
                        a = math.radians(info['eixo_graus'])-pm[2]
                        c, sn = math.cos(a), math.sin(a)
                        pts = [(c*x+sn*y,-sn*x+c*y) for x,y in sc[1]]
                        med = medir_corredor(pts)
                        if not passagem_confirmada(info, pm, med, self.p('largura_estreita')):
                            return False
                        # PASSAGEM_INICIADA_M35: ja paralelo dentro da passagem, nao recuar ao preparo.
                        entrada = info['entrada']
                        dentro = math.hypot(pm[0]-entrada[0],pm[1]-entrada[1]) < .30
                        alinhado = abs(_ang(math.radians(info['eixo_graus'])-pm[2])) <= math.radians(8)
                        alerta.update(etapas=etapas, info=info, dentro_alinhado=dentro and alinhado)
                        if self.supervisor.modo == 'observar':
                            self.supervisor.mudar('NAV2', 'observacao: passagem confirmada; sem transferencia')
                            return False
                        if not regiao_conhecida(info, regioes):
                            self.supervisor.mudar('NAV2', 'candidato fora das regioes cadastradas; transferencia vetada')
                            return False
                        return True
                    resultado = self._navegar_principal(goal, 'NAV2 PRINCIPAL', vigia)
                    if resultado != 'interrompido':
                        if resultado != 'ok':
                            self.supervisor.mudar('FALHA', 'Nav2 terminou sem sucesso')
                        if resultado == 'ok' and area is not None:
                            if area in self.sem_encosto:
                                self._publicar_resultado(area, True, 'Nav2 chegou')
                            else:
                                self._autoridade_mov('especial')
                                self._encostar(area)
                        return
                    # Resultado terminal do Nav2 confirmado antes de assumir a manobra.
                    etapas, info = alerta['etapas'], alerta['info']
                    self.supervisor.mudar('TRANSFERENCIA', 'Nav2 terminal; aguardando parada')
                    self._esperar_parada_supervisor()
                    self.supervisor.mudar('APROXIMACAO', 'preparo existente fora da entrada')
                    self._autoridade_mov('especial')
                    e = etapas[0]
                    if not alerta.get('dentro_alinhado') and self._chegar_preparo(e, 'PREPARO CONFIRMADO') != 'ok':
                        return
                    self._autoridade_mov('especial')
                    self.supervisor.mudar('ALINHADOR', 'controle exclusivo; rotina existente')
                    self._iniciar_evidencia_passagem()
                    curva = info.get('curva')
                    ax = math.cos(math.radians(info['eixo_graus']))
                    ay = math.sin(math.radians(info['eixo_graus']))
                    parada = ((curva['juncao'][0]-self.p('centro_corpo')*ax,
                               curva['juncao'][1]-self.p('centro_corpo')*ay) if curva else None)
                    r = self._travessia_lidar(info,'CORREDOR CONFIRMADO',parada,
                        curva['lado'] if curva else None, curva.get('largura_b') if curva else None)
                    if r != 'ok':
                        return
                    tratadas.append(info['entrada'])
                    if curva:
                        if self._travessia_lateral(curva,'T CONFIRMADO') != 'ok':
                            return  # nao ativa CURVA_INSISTE no novo modo
                        tratadas.append(curva['entrada_b'])
                    saida = curva['saida_b'] if curva else info['saida']
                    eixo = curva['eixo_b'] if curva else (ax,ay)
                    self._confirmar_saida_supervisor(saida,eixo)
                    self.passagens_supervisor.concluir(info['entrada'], (ax,ay))
                    if curva:
                        self.passagens_supervisor.concluir(curva['entrada_b'], eixo)
                    self.supervisor.mudar('RETOMADA', 'footprint/ICP/IMU/saida/parada confirmados')
                    self._autoridade_mov('nav2')
                    self.supervisor.mudar('NAV2', 'nova action com goal original; replanejamento')
                    # Nova NavigateToPose: BT reinicia caminho_do_goal e replaneja com costmap atual.
            except Exception as exc:
                self.supervisor.mudar('FALHA', str(exc))
                raise
            finally:
                try:
                    self._drenar_nav()
                    if self.dono_mov == 'especial':
                        self._parar()
                finally:
                    self.evidencia_passagem = None
                    if self.supervisor.estado not in ('NAV2', 'RETOMADA'):
                        self.supervisor.mudar('FALHA', 'interrupcao/cancelamento ou manobra sem confirmacao')
                    self.dono_mov = None  # lease expira; proximo goal solicita nova autoridade

        def _bloqueios_ativos(self):
            """PASSAGEM_BLOQUEADA: [(x, y, raio)] ainda validos."""
            val = float(self.p('passagem_bloqueada_s'))
            if val <= 0.0:
                return []
            agora = time.monotonic()
            self.bloqueios = [b for b in self.bloqueios if agora - b[3] <= val]
            return [b[:3] for b in self.bloqueios]

        def _desviar_bloqueio(self, gx, gy, nome):
            """PASSAGEM_BLOQUEADA: a travessia abortou por objeto a frente. Marca o objeto e devolve True se o mapa tem
            outro caminho ate o goal sem ele (o chamador reanalisa e nao repete a travessia)."""
            if float(self.p('passagem_bloqueada_s')) <= 0.0 or not self.trav_motivo.startswith('algo a'):
                return False
            if getattr(self, 'desvios_bloq', 0) >= 2:
                self.get_logger().warn('%s: PASSAGEM_BLOQUEADA: ja desviei 2 vezes neste goal: sigo como antes' % nome)
                return False
            pts = self.trav_obstaculo
            if not pts:
                self.get_logger().info('%s: PASSAGEM_BLOQUEADA: %s, mas nada livre no mapa a frente (parede?): '
                                       'sigo como antes' % (nome, self.trav_motivo))
                return False
            agora, raio = time.monotonic(), float(self.p('passagem_bloqueada_raio'))
            self.bloqueios += [(x, y, raio, agora) for x, y in pts]
            mapa, pm = self._mapa(), self._pose_mapa()
            rota = rota_no_mapa(com_barreiras(self, mapa), pm, (gx, gy), bloqueios=self._bloqueios_ativos()) \
                if mapa is not None and pm is not None else None
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            onde = '(%.2f..%.2f, %.2f..%.2f)' % (min(xs), max(xs), min(ys), max(ys))
            if rota is None:
                self.get_logger().warn('%s: PASSAGEM_BLOQUEADA: objeto em %s (%s) e o mapa nao tem outro caminho: '
                                       'tento de novo por aqui' % (nome, onde, self.trav_motivo))
                return False
            self.get_logger().warn('%s: PASSAGEM_BLOQUEADA: objeto em %s (%s); o mapa tem outro caminho (%.1f m): '
                                   'nao insisto, replanejo por ele' % (nome, onde, self.trav_motivo, _comp_rota(rota)))
            self.desvios_bloq = getattr(self, 'desvios_bloq', 0) + 1
            return True

        def _analisar(self, gx, gy, tratadas, pontos=None, goal=None):
            mapa = self._mapa()
            if pontos is None:
                pontos = self._caminho(goal)
                # v3.27 (missao14 08:17): com algo marcado no corredor, o planner mandou o robo dar a
                # volta na arena (34 m para um destino a 3 m). Caminho > 2,5x a reta + 2 m: limpa os
                # costmaps e planeja de novo uma vez.
                pm = self._pose_mapa()
                if pontos and pm is not None:
                    comp = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pontos, pontos[1:]))
                    reta = math.hypot(gx - pm[0], gy - pm[1])
                    if comp > 2.5 * reta + 2.0:
                        self.get_logger().warn('Caminho de %.1f m para um destino a %.1f m: limpo os costmaps '
                                               'e planejo de novo' % (comp, reta))
                        self._limpar_costmaps()
                        time.sleep(1.5)
                        novo = self._caminho(goal)
                        if novo:
                            pontos = novo
            # ROTA_MAPA (missao23): com um objeto no corredor norte, o planner mandou seguir reto e dar a volta
            # pelo leste; o robo encostou. Obstaculo nao muda a rota: ele vai pelo caminho mais curto do mapa e
            # contorna la. So na 3a tentativa para o mesmo destino aceita o desvio do planner.
            pm = self._pose_mapa()
            bloq = self._bloqueios_ativos()   # PASSAGEM_BLOQUEADA (11/11)
            if bloq and goal is None and rota_cruza_bloqueio(pontos, bloq):
                return [], 'caminho do planner pelo bloqueio marcado (o goal decide)'   # vigia nao interrompe
            if goal is not None and mapa is not None and pm is not None and self.p('rota_mapa') \
                    and (self.rota_tent.get((round(gx, 1), round(gy, 1)), 0) <= 2 or bloq):
                rota = rota_no_mapa(com_barreiras(self, mapa), pm, (gx, gy), bloqueios=bloq)   # MEMORIA_BARREIRA
                if bloq and rota is None:
                    self.get_logger().warn('PASSAGEM_BLOQUEADA: sem caminho no mapa sem o objeto marcado: uso o mapa '
                                           'inteiro')
                    rota = rota_no_mapa(com_barreiras(self, mapa), pm, (gx, gy))   # MEMORIA_BARREIRA
                elif bloq and rota_cruza_bloqueio(pontos, bloq):
                    self.get_logger().warn('PASSAGEM_BLOQUEADA: o caminho do planner passa pelo objeto marcado: '
                                           'uso a rota do mapa sem ele (%.1f m)' % _comp_rota(rota))
                    pontos = None
                if rota is not None and pontos is not None and self.p('planner_livre') and \
                        _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5:   # PLANNER_LIVRE (25/11)
                    self.get_logger().info('PLANNER_LIVRE: fico com o caminho do planner (%.1f m; o mais curto do mapa '
                                           'tem %.1f m): ele ve o que barra o curto' % (_comp_rota(pontos), _comp_rota(rota)))
                if rota is not None and (pontos is None or (not self.p('planner_livre') and   # PLANNER_LIVRE
                                                            _comp_rota(pontos) > 1.3 * _comp_rota(rota) + 0.5)):
                    modo_obs = str(self.p('rota_mapa_obstaculo'))   # ROTA_MAPA_OBSTACULO (11/11)
                    novos = obstaculo_novo_na_rota(mapa, self._obs_global(), rota) if modo_obs in (
                        'registrar', 'recusar') else []
                    if novos:
                        self.get_logger().warn('ROTA_MAPA_OBSTACULO (%s): %d celula(s) de objeto do momento a '
                                               '< 35 cm da rota do mapa, a 1a em (%.2f, %.2f)' % (
                                                   modo_obs, len(novos), novos[0][0], novos[0][1]))
                    alt = None   # PASSAGEM_BLOQUEADA: planner sem caminho e objeto na rota do mapa
                    if novos and modo_obs == 'recusar' and pontos is None and float(self.p('passagem_bloqueada_s')) > 0:
                        r_ = float(self.p('passagem_bloqueada_raio'))
                        passo = max(1, len(novos) // 40)
                        alt = rota_no_mapa(mapa, pm, (gx, gy),
                                           bloqueios=list(bloq) + [(x, y, r_) for x, y in novos[::passo]])
                    if novos and modo_obs == 'recusar' and pontos is not None:
                        self.get_logger().warn('ROTA_MAPA_OBSTACULO: fico com o planner (%.1f m)' % _comp_rota(pontos))
                    elif alt is not None:
                        self.get_logger().warn('PASSAGEM_BLOQUEADA: planner sem caminho e objeto na rota do mapa: vou '
                                               'pela rota do mapa sem ele (%.1f m; a direta tinha %.1f m)' % (
                                                   _comp_rota(alt), _comp_rota(rota)))
                        pontos = alt
                    else:
                        self.get_logger().warn('Planner: %s; mapa: %.1f m: sigo a rota do mapa' % (
                            'sem caminho' if pontos is None else '%.1f m' % _comp_rota(pontos), _comp_rota(rota)))
                        pontos = rota
            if mapa is None or pontos is None:
                return [], 'sem mapa ou sem caminho'
            etapas, info = planejar_etapas(mapa, pontos, (gx, gy), self.p('largura_estreita'),
                                           self.p('distancia_preparo'), self.p('distancia_saida'),
                                           ja_tratadas=tratadas,
                                           preparo_lateral_max=float(self.p('preparo_lateral_max')))  # PREPARO_LATERAL
            lmin = float(self.p('passagem_largura_minima'))   # PASSAGEM_IMPOSSIVEL (25/11)
            if not etapas or lmin <= 0 or info['largura_mapa'] >= lmin:
                return etapas, info
            if goal is None:
                return [], 'PASSAGEM_IMPOSSIVEL: largura %.2f m no mapa (vigia ignora)' % info['largura_mapa']
            for k in range(int(self.p('passagem_replanejar_max'))):
                self.get_logger().warn('PASSAGEM_IMPOSSIVEL %d: passagem de %.2f m no mapa em (%.2f, %.2f) (o robo tem '
                                       '0,65 m): mapa e costmap discordam; limpo os costmaps e replanejo' % (
                                           k + 1, info['largura_mapa'], info['entrada'][0], info['entrada'][1]))
                if not (self.p('memoria_barreira') and info['largura_mapa'] < 0.05):   # MEMORIA_BARREIRA
                    self._limpar_costmaps(so_local=bool(self.p('memoria_barreira')))
                time.sleep(1.5)
                if self._novo_goal_chegou():
                    return [], 'PASSAGEM_IMPOSSIVEL: goal novo'
                novo = self._caminho(goal)
                if not novo:
                    continue
                etapas, info = planejar_etapas(mapa, novo, (gx, gy), self.p('largura_estreita'),
                                               self.p('distancia_preparo'), self.p('distancia_saida'),
                                               ja_tratadas=tratadas,
                                               preparo_lateral_max=float(self.p('preparo_lateral_max')))
                if not etapas or info['largura_mapa'] >= lmin:
                    self.get_logger().info('PASSAGEM_IMPOSSIVEL: resolvido no replanejamento %d' % (k + 1))
                    return etapas, info
            self.get_logger().warn('PASSAGEM_IMPOSSIVEL: continua depois de %d replanejamentos: sem preparo/travessia, '
                                   'sigo com o Nav2' % int(self.p('passagem_replanejar_max')))
            return [], 'PASSAGEM_IMPOSSIVEL: sigo com o Nav2'

        def executar(self, goal, area=None):
            if self.p('nav2_principal'):
                return self._executar_principal(goal, area)
            return self._executar_legado(goal, area)

        def _executar_legado(self, goal, area=None):
            gx, gy = goal.pose.position.x, goal.pose.position.y
            ch = (round(gx, 1), round(gy, 1))   # ROTA_MAPA: tentativas para o mesmo destino
            self.rota_tent = {ch: self.rota_tent.get(ch, 0) + 1}
            self.desvios_bloq = 0   # PASSAGEM_BLOQUEADA: desvios neste goal (max 2)
            tratadas = []
            alerta = {}

            def vigia():
                with self.lock:
                    pontos, idade = self.plano, time.monotonic() - self.plano_t
                if pontos is None or idade > 3.0 or len(tratadas) >= int(self.p('max_passagens')):
                    return False
                etapas, info = self._analisar(gx, gy, tratadas, pontos=pontos)
                if etapas and self._boca_da_caixa(area, info, gx, gy):   # CAIXA_LATERAL (25/11)
                    return False
                if etapas:
                    alerta['etapas'], alerta['info'] = etapas, info
                    return True
                return False

            while True:
                if len(tratadas) >= int(self.p('max_passagens')):
                    etapas, info = [], 'limite de passagens atingido'
                elif 'etapas' in alerta:
                    etapas, info = alerta.pop('etapas'), alerta.pop('info')
                else:
                    etapas, info = self._analisar(gx, gy, tratadas, goal=goal)
                if etapas and self._boca_da_caixa(area, info, gx, gy):   # CAIXA_LATERAL (25/11)
                    etapas, info = [], 'CAIXA_LATERAL: a passagem e a boca da caixa do %s' % area
                if etapas:
                    tratadas.append(info['entrada'])
                    self.get_logger().info(
                        'Passagem %d: entrada (%.2f, %.2f), eixo %.0f graus, largura no mapa '
                        '%.2f m, preparo a %.2f m da entrada' % (
                            len(tratadas), info['entrada'][0], info['entrada'][1],
                            info['eixo_graus'], info['largura_mapa'], info['distancia_preparo']))
                    if etapas[0].get('preparo_lateral'):   # PREPARO_LATERAL (19/11)
                        self.get_logger().warn(
                            'PREPARO_LATERAL: eixo sem lugar no mapa: preparo deslocado %+.0f cm para o lado '
                            '(a travessia centraliza na boca)' % (etapas[0]['preparo_lateral'] * 100))
                    self._publicar_etapas(etapas)
                    e = etapas[0]
                    if (info.get('curva') is not None and self.p('modo_curva_t') == 'giro_frontal'
                            and not self._precondicoes_t()):
                        self._parar()
                        self.get_logger().error('Curva experimental recusada antes do preparo: '
                            'temporalidade, limites reais de movimento ou cobertura angular insuficientes')
                        return
                    ok = self._chegar_preparo(e, 'PREPARO %d (alinhar fora da passagem)' % len(tratadas)) == 'ok'
                    if ok:
                        # De perto o mapa da passagem fica melhor: refina uma vez.
                        novas, ninfo = self._analisar(gx, gy, tratadas[:-1], goal=goal)
                        if novas and math.hypot(ninfo['entrada'][0] - info['entrada'][0],
                                                ninfo['entrada'][1] - info['entrada'][1]) < 0.40:
                            n0 = novas[0]
                            dyaw = abs(math.atan2(math.sin(n0['yaw'] - e['yaw']),
                                                  math.cos(n0['yaw'] - e['yaw'])))
                            dpos = math.hypot(n0['x'] - e['x'], n0['y'] - e['y'])
                            rapida = self.p('entrada_rapida')   # ENTRADA_RAPIDA: era 4 graus e 4 cm
                            if dyaw > math.radians(8.0 if rapida else 4.0) or dpos > (0.06 if rapida else 0.04):
                                self.get_logger().info(
                                    'Refinando preparo: %.0f graus e %.0f cm de diferenca' % (
                                        math.degrees(dyaw), dpos * 100))
                                etapas, info = novas, ninfo
                                tratadas[-1] = info['entrada']
                                self._publicar_etapas(etapas)
                                e = etapas[0]
                                ok = self._chegar_preparo(e, 'PREPARO %d refinado' % len(tratadas)) == 'ok'
                    if not ok and self.p('modo_travessia') == 'lidar' and not self.p('fallback_nav2'):
                        self.get_logger().error(
                            'Preparo %d falhou. Parei fora da passagem; mande o goal de novo.' % len(tratadas))
                        return
                    if not ok:
                        self.get_logger().warn('Preparo falhou: seguindo para o goal final.')
                    elif self.p('modo_travessia') == 'lidar':
                        n = len(tratadas)
                        curva = info.get('curva')
                        parada = None
                        if curva is not None:
                            # v3.11: para no cruzamento com o corpo centrado no corredor novo
                            ax_ = math.cos(math.radians(info['eixo_graus']))
                            ay_ = math.sin(math.radians(info['eixo_graus']))
                            jx, jy = curva['juncao']
                            parada = ((jx, jy) if self.p('modo_curva_t') == 'giro_frontal' else
                                      (jx - self.p('centro_corpo') * ax_, jy - self.p('centro_corpo') * ay_))
                            self.get_logger().info(
                                'Passagem %d termina num cruzamento em (%.2f, %.2f) com curva para a %s: '
                                'modo de curva: %s' % (
                                    n, jx, jy, 'ESQUERDA' if curva['lado'] > 0 else 'DIREITA', self.p('modo_curva_t')))
                        elif info.get('cruzamentos'):
                            self.get_logger().info('Passagem %d atravessa %d cruzamento(s) reto(s): '
                                                   'uma travessia so ate (%.2f, %.2f)' % (
                                                       n, info['cruzamentos'], info['saida'][0],
                                                       info['saida'][1]))
                        lado_c = curva['lado'] if curva is not None else None
                        larg_b = curva.get('largura_b') if curva is not None else None
                        lado_parada = None if self.p('modo_curva_t') == 'giro_frontal' else lado_c
                        experimental_t = curva is not None and self.p('modo_curva_t') == 'giro_frontal'
                        if experimental_t:
                            r = self._aproximar_t(info, curva, 'TRAVESSIA %d' % n, parada, larg_b)
                            if r != 'ok':
                                self.get_logger().error('Aproximacao experimental interrompida, sem retentativa automatica')
                                return
                        else:
                            r = self._travessia_lidar(info, 'TRAVESSIA %d' % n, parada, lado_parada, larg_b)
                        if r != 'ok' and self._desviar_bloqueio(gx, gy, 'TRAVESSIA %d' % n):
                            tratadas.pop()   # PASSAGEM_BLOQUEADA (11/11): reanalisa pelo outro caminho
                            continue
                        if r != 'ok':
                            self.get_logger().warn('Travessia pelo LiDAR falhou: voltando ao preparo '
                                                   'para tentar de novo.')
                            if self._chegar_preparo(e, 'PREPARO %d (2a tentativa)' % n) == 'ok':
                                r = self._travessia_lidar(info, 'TRAVESSIA %d (2a tentativa)' % n,
                                                          parada, lado_parada, larg_b)
                        if r == 'ok' and curva is not None:
                            if self.p('modo_curva_t') == 'giro_frontal':
                                r = self._travessia_giro_frontal(curva, 'CURVA %d' % n)
                            else:
                                r = self._travessia_lateral(curva, 'CURVA %d' % n)
                                # CURVA_INSISTE (11/11): na prova nao pode parar; repete a saida lateral daqui
                                k_ins = 0
                                while r == 'repetir' and k_ins < min(2, max(0, int(self.p('insistir_curva')))) and \
                                        not self._novo_goal_chegou():
                                    k_ins += 1
                                    self.get_logger().warn('CURVA_INSISTE %d/%d: a saida lateral falhou; tento de '
                                                           'novo daqui' % (k_ins, int(self.p('insistir_curva'))))
                                    r = self._travessia_lateral(curva, 'CURVA %d (insiste %d)' % (n, k_ins))
                            if r == 'ok':
                                if self.p('modo_curva_t') == 'giro_frontal':
                                    self.get_logger().info('Curva concluida e devolucao confirmada: '
                                        'arbitro parado. Retome explicitamente apos conferir a area.')
                                    if area is not None:
                                        self._publicar_resultado(area, False,
                                            'curva concluida; missao interrompida para retomada supervisionada')
                                    return
                                tratadas.append(curva['entrada_b'])
                                continue  # reanalisa o caminho a partir da saida do corredor novo
                            if (r == 'repetir' and self.p('modo_curva_t') != 'giro_frontal'
                                    and self.p('curva_segue_nav2') and not self._novo_goal_chegou()):  # CURVA_INSISTE_SEGURO
                                self.get_logger().warn('CURVA_INSISTE: saida lateral da curva %d falhou de novo; '
                                                       'nao paro: sigo para o goal pelo Nav2' % n)
                                tratadas.append(curva['entrada_b'])
                                continue
                            self.get_logger().error(
                                'Nao consegui sair de lado pelo corredor novo (curva %d). Parei; ajude o '
                                'robo e mande o goal de novo.' % n)
                            return
                        if r == 'ok':
                            continue  # reanalisa o caminho a partir da saida
                        if self._desviar_bloqueio(gx, gy, 'TRAVESSIA %d (2a tentativa)' % n):
                            tratadas.pop()   # PASSAGEM_BLOQUEADA (11/11)
                            continue
                        if not (self.p('fallback_nav2') and len(etapas) > 1):
                            self.get_logger().error(
                                'Nao consegui atravessar a passagem %d. Parei fora dela; mande o '
                                'goal de novo (ou ajude o robo).' % n)
                            return
                        e = etapas[1]
                        if self._navegar_corredor(self._pose(e['x'], e['y'], e['yaw']),
                                                  'TRAVESSIA %d (Nav2)' % n, virar=False) == 'ok':
                            continue
                        self.get_logger().warn('Travessia falhou: seguindo para o goal final.')
                    elif len(etapas) > 1:
                        if self.p('modo_travessia') == 'lidar':
                            self.get_logger().warn('Travessia pelo LiDAR falhou: tentando pelo Nav2.')
                        e = etapas[1]
                        if self._navegar_corredor(self._pose(e['x'], e['y'], e['yaw']),
                                                  'TRAVESSIA %d' % len(tratadas), virar=False) == 'ok':
                            continue  # reanalisa o caminho a partir da saida
                        self.get_logger().warn('Travessia falhou: seguindo para o goal final.')
                else:
                    self.get_logger().info('Analise: %s.' % info)
                # goal final, vigiando o caminho para passagens que aparecerem no mapa
                livre = area is not None and area in self.sem_encosto
                fin_reto = livre and str(area).startswith('FINISH') and self.p('finish_perpendicular')   # FINISH_ENTRA_RETO
                if livre and self.p('entrada_translada') and (fin_reto or not self.p('finish_chegada')):   # ENTRADA_TRANSLADA; FINISH_CHEGADA
                    yaw_ent = yaw_de(goal.pose.orientation) + (math.pi if fin_reto else 0.0)   # FINISH_ENTRA_RETO
                    r_ent = self._entrada_translada(area, gx, gy, yaw_ent, vigia)
                    if r_ent == 'ok':
                        self._publicar_resultado(area, True, 'chegou (ponto sem mesa, entrada transladando)')
                        return
                    if r_ent == 'falha':
                        return
                    if r_ent == 'interrompido':
                        continue
                if livre:
                    # v3.10: ponto sem mesa: angulo de chegada pelo caminho, nada de giro final
                    yaw = yaw_chegada(self._caminho(goal), (gx, gy), self._pose_mapa_yaw())
                    if str(area).startswith('FINISH') and self.p('finish_perpendicular'):   # FINISH_PERPENDICULAR
                        yaw = yaw_de(goal.pose.orientation) + math.pi
                        self.get_logger().info('FINISH_PERPENDICULAR: %s: entro de frente para a caixa (%.0f graus)' % (
                            area, math.degrees(math.atan2(math.sin(yaw), math.cos(yaw)))))
                    if yaw is not None:
                        self.get_logger().info('%s: angulo livre, usando o de chegada (%.0f graus)' % (
                            area, math.degrees(yaw)))
                        goal = self._pose(gx, gy, yaw)
                # GIRO_PERTO (25/11): goal sem area colado no robo = intencao de girar no lugar
                pm_gp = self._pose_mapa_yaw()
                lim_gp = float(self.p('so_girar_perto'))
                if area is None and lim_gp > 0.0 and pm_gp is not None and \
                        math.hypot(gx - pm_gp[0], gy - pm_gp[1]) <= lim_gp:
                    yaw_gp = yaw_de(goal.pose.orientation)
                    self.get_logger().info('GIRO_PERTO: goal a %.0f cm (<= %.0f): so giro para %.0f graus, sem andar'
                                           % (100 * math.hypot(gx - pm_gp[0], gy - pm_gp[1]), 100 * lim_gp,
                                              math.degrees(yaw_gp)))
                    ok_gp = self._girar_seguro(yaw_gp, 'GIRO_PERTO')
                    self.get_logger().info('GIRO_PERTO: %s' % ('SUCESSO' if ok_gp else 'nao terminou o giro'))
                    return
                resultado = self._trajeto_curto(goal, livre)
                if resultado is None:
                    def vigia_final():
                        # v3.27: passagem nova no caminho OU robo travado na frente de uma peca
                        if vigia():
                            return True
                        if self.p('desvio_lidar') and alerta.get('desvios', 0) < 3 and self._travado(goal):
                            alerta['desviar'] = True
                            return True
                        return False
                    # GIRO_FINAL_COMUM (11/11): com giro_final_comum, goal sem area segue o mesmo caminho
                    if (area is not None and not livre and self.p('giro_final_alinhador')) or \
                            (area is None and self.p('giro_final_comum')):
                        # v3.32 GIRO_FINAL: Nav2 so ate a posicao, chegando com o rumo do caminho;
                        # o giro para a mesa e do alinhador
                        gyaw = yaw_de(goal.pose.orientation)
                        yaw_c = yaw_chegada(self._caminho(goal), (gx, gy), self._pose_mapa_yaw())
                        alvo = self._pose(gx, gy, yaw_c) if yaw_c is not None else goal
                        resultado = self._navegar(alvo, 'GOAL FINAL', vigia_final, aceitar_perto=True)
                        nav_falhou = resultado == 'falha'   # REINSISTIR (11/11)
                        if resultado == 'ok':
                            resultado = self._giro_final(gx, gy, gyaw, 'GOAL FINAL')
                    else:
                        resultado = self._navegar(goal, 'GOAL FINAL', vigia_final, aceitar_perto=livre)
                        nav_falhou = resultado == 'falha'   # REINSISTIR (11/11)
                    # REINSISTIR (11/11, missao33): o Nav2 desistiu (destravar sem alternativa); antes de dar FALHOU,
                    # faz o que o operador fazia: reanalisa, vira para o caminho e manda o goal de novo
                    if nav_falhou and alerta.get('reinsistir', 0) < int(self.p('reinsistir_goal_final')) \
                            and not self._novo_goal_chegou():
                        alerta['reinsistir'] = alerta.get('reinsistir', 0) + 1
                        self.get_logger().warn(
                            'REINSISTIR %d/%d: o Nav2 desistiu do GOAL FINAL; reanaliso, viro para o caminho e '
                            'mando de novo' % (alerta['reinsistir'], int(self.p('reinsistir_goal_final'))))
                        self._limpar_costmaps(so_local=bool(self.p('memoria_barreira')))   # REINSISTIR_LIMPA (25/11); MEMORIA_BARREIRA
                        fim_espera = time.monotonic() + 1.5
                        while time.monotonic() < fim_espera and not self._novo_goal_chegou():
                            time.sleep(0.05)
                        self.hist_nav = None
                        continue
                    if resultado == 'interrompido' and alerta.pop('desviar', False):
                        alerta['desvios'] = alerta.get('desvios', 0) + 1
                        self.get_logger().warn('DESVIO %d: o Nav2 nao sai do lugar ha 6 s com algo na frente' % alerta['desvios'])
                        self._desviar(goal, 'DESVIO %d' % alerta['desvios'])
                        self.hist_nav = None
                        continue
                if resultado != 'interrompido':
                    if resultado == 'ok' and area is not None:
                        if area in self.sem_encosto:
                            self._publicar_resultado(area, True, 'chegou (ponto sem mesa)')
                        else:
                            self._encostar(area)
                    return

    rclpy.init()
    no = Alinhador()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(no)
    try:
        executor.spin()
    except KeyboardInterrupt:
        no._cancelar_nav()
        no._parar()
    finally:
        executor.shutdown()
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
