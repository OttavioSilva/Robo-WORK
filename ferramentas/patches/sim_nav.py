#!/usr/bin/env python3
"""Compara duas versoes do alinhador num simulador simples, sem ROS e sem robo: encosto e saida lateral.
Fisica medida: 0,10 m/s no PWM minimo, LiDAR 5,9 Hz, deriva do deslize k, ~0,06 m/s de lado.
E simulador: serve para achar erro de logica e comparar versoes, nao garante o tempo no robo.

Uso: python3 -I sim_nav.py <alinhador_antigo.py> <alinhador_novo.py>
"""
import importlib.util
import math
import random
import sys


def carrega(arq, nome):
    sp = importlib.util.spec_from_file_location(nome, arq)
    m = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(m)
    return m


def sim_encosto(mod, linear, v_real, seed):
    random.seed(seed)
    kw = dict(linear=True, distancia_alvo=0.09) if linear else dict(distancia_alvo=0.08)
    e = mod.Encosto(para_choque=0.365, log=lambda s: None, **kw)
    folga, y, yaw, v, t = 0.31, 0.04, 0.0, 0.0, 0.0
    prox, pulsos, ant, minf, dt = 0.0, 0, False, 9.0, 0.05
    while t < 60.0 and e.estado not in ('fim', 'abortar'):
        pose = (0.31 - folga, y, yaw)
        if t >= prox:
            prox = t + 0.17
            e.nova_medicao(t, dict(dist=folga + 0.365 + random.gauss(0, 0.001), ang=-yaw + random.gauss(0, 0.002),
                                   centro=-y + random.gauss(0, 0.002), span=0.8, n=30), pose)
        vx, vy, wz = e.passo(t, pose)
        if folga <= 0.095:      # o Collision Monitor trava em 9,5 cm
            vx = min(vx, 0.0)
        alvo = v_real if vx > 0.005 else 0.0
        v += (alvo - v) * min(1.0, dt / 0.12)
        folga -= v * dt
        y += (0.07 if vy > 0.005 else -0.07 if vy < -0.005 else 0.0) * dt
        yaw += wz * dt
        andando = vx > 0.005 or abs(vy) > 0.005
        pulsos += andando and not ant
        ant = andando
        minf = min(minf, folga)
        t += dt
    return t, pulsos, folga, y, e.estado


def sim_lateral(mod, novo, lado, k, seed):
    random.seed(seed)
    kw = dict(fundido=True) if novo else {}
    tl = mod.TravessiaLateral(lado, log=lambda s: None, tempo_max=120.0, **kw)
    X_F, X_T, LARG = 0.375, -0.215, 0.84
    x, yaw, s, t, prox, dt = -0.04, math.radians(1.0), 0.0, 0.0, 0.0, 0.05
    falso = 0.0
    paradas, ant, minf, yaw_max, alvo_fase = 0, True, 9.0, 0.0, 0.0
    tr_yaw = 0.0
    fim = 1.89   # missao23: 1,29 m + margem de 0,60 m
    while t < 120.0 and tl.estado not in ('fim', 'abortar'):
        restante = fim - s
        pose = (x, lado * s, yaw)
        f_fr = LARG / 2 - (x + X_F)
        f_tr = (x + X_T) + LARG / 2
        minf = min(minf, f_fr, f_tr)
        if t >= prox:
            prox = t + 0.17
            ang = -yaw + random.gauss(0, math.radians(1.2))
            duas = 0.45 < s < 0.75 or s > 1.3     # missao23: as 2 paredes so em parte do caminho
            if not duas:
                if random.random() < 0.05:
                    falso = random.choice((-1, 1)) * math.radians(random.uniform(4.0, 9.0))
                ang += falso                      # uma parede so: angulo falso e persistente
            if restante > 0.25:
                des = (-0.08 - x) + random.gauss(0, 0.01)   # centro do corredor a frente do centro do corpo
                tl.nova_medicao(t, dict(angulo=ang, desvio=des, frente=f_fr, lados=2 if duas else 1, n=25, largura=LARG), pose)
            tl.nova_folga(t, f_fr if f_fr < 0.40 else None, f_tr if f_tr < 0.40 else None, True, True)
        tl.gyro = tr_yaw
        vx, vy, wz = tl.passo(t, pose, restante)
        m_lat = abs(vy) > 0.005
        if m_lat:
            vr = 0.05 * math.copysign(1.0, vy)    # missao23: ~0,05 m/s
            # missao23: o giro andando nao muda o rumo e aumenta a deriva (1,4 -> 2,4-4 cm/s)
            x += ((k + min(0.6, 8.0 * abs(wz))) * vr + 0.5 * (vx if abs(vx) > 0.015 else 0.0)) * dt
            s += abs(vr) * dt * (1 if vy * lado > 0 else -1)
            tr_yaw = random.gauss(0, 0.03) - 0.006 * math.copysign(1.0, vy)
        else:
            if abs(vx) > 0.005:
                x += math.copysign(0.10, vx) * dt
            tr_yaw = math.copysign(0.26, wz) if abs(wz) > 0.012 else 0.0
        yaw += tr_yaw * dt
        yaw_max = max(yaw_max, abs(yaw))
        if ant and not m_lat:
            paradas += 1
        ant = m_lat
        t += dt
    return t, paradas, minf, math.degrees(yaw_max), tl.estado


def main():
    velho, novo = carrega(sys.argv[1], 'velho'), carrega(sys.argv[2], 'novo')
    print('== ENCOSTO: de 31 cm, 4 cm fora do centro; 5 sementes por velocidade real')
    print('versao  v_real  tempo   pulsos  folga final  centro   ')
    for nome, m, lin in (('antiga', velho, False), ('nova', novo, True)):
        for v in (0.08, 0.10, 0.13):
            r = [sim_encosto(m, lin, v, s) for s in range(5)]
            est = {x[4] for x in r}
            print('%-7s %.2f   %4.1f s  %4.1f    %4.1f-%4.1f cm  %+.1f cm  %s' % (
                nome, v, sum(x[0] for x in r) / 5, sum(x[1] for x in r) / 5, 100 * min(x[2] for x in r),
                100 * max(x[2] for x in r), 100 * sum(x[3] for x in r) / 5, est))
    print('== SAIDA LATERAL: 1,89 m, corredor 84 cm, 0,05 m/s, angulo falso com uma parede; 8 sementes')
    print('versao  lado  k     tempo   paradas  menor folga  rumo max')
    for nome, m, n in (('antiga', velho, True), ('nova', novo, True)):
        for lado, k in ((1, 0.28), (-1, 0.28), (1, 0.10), (1, 0.60)):
            r = [sim_lateral(m, n, lado, k, s) for s in range(8)]
            print('%-7s %+d   %.2f  %5.1f s  %5.1f    %5.1f cm    %5.1f gr  %s' % (
                nome, lado, k, sum(x[0] for x in r) / 8, sum(x[1] for x in r) / 8, 100 * min(x[2] for x in r),
                max(x[3] for x in r), sorted({x[4] for x in r})))


if __name__ == '__main__':
    main()
