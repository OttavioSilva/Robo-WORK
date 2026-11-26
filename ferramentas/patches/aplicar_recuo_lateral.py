#!/usr/bin/env python3
"""RECUO_LATERAL (09/11): no encosto, o deslize de lado vira avanco (missao32: T3 andou 17 cm de lado e
avancou 13 cm, k ~0,8; T4 9 cm de lado e 12 cm de avanco, k ~1,3) e a face entra na zona cega do LiDAR
com o robo ainda fora do centro. Compensar com re durante o deslize nao da: a re minima do motor e
~0,10 m/s (PWM 70) contra 0,02-0,05 m/s de avanco a compensar.

O que muda (so a classe Encosto do alinhador_passagem.py):
  - antes de deslizar, preve o avanco (k_lat * |centro|); se a folga prevista ficar abaixo de 15 cm
    (onde a face ainda aparece inteira), recua reto pelo LiDAR ate a folga necessaria e so entao desliza;
  - no deslize, se a folga cair abaixo de 14 cm com o centro ainda fora, para e mede (pode recuar de novo);
  - k_lat comeca em 0,5 e e medido pelo LiDAR a cada deslize (folga perdida / centro corrigido);
  - limites: 3 recuos, 20 cm por recuo, folga ate 45 cm (recua so se ganhar >= 3 cm); Collision Monitor atras -> aborta (como hoje).
Nao muda: deteccao da face, giro, avanco final, ENCOSTO_CEGO, correcao lateral ja encostado.

Uso:
  python3 aplicar_recuo_lateral.py                       # aplica no src (backup .bak_recuo_lateral_<data>)
  python3 aplicar_recuo_lateral.py --arquivo <copia.py>  # aplica numa copia
  python3 aplicar_recuo_lateral.py --desfazer [--arquivo ...]
"""
import argparse
import datetime
import glob
import os
import py_compile
import shutil
import sys
import tempfile

MARCA = 'RECUO_LATERAL'
PADRAO = os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts/alinhador_passagem.py')

TROCAS = [
    # 1) parametros novos no __init__ do Encosto
    ("""        self.t_parado = None           # FACE_FRAGMENTADA: inicio do ultimo assentar
""",
     """        self.t_parado = None           # FACE_FRAGMENTADA: inicio do ultimo assentar
        # RECUO_LATERAL (09/11, missao32): o deslize de lado vira avanco (k 0,3-1,3); antes de deslizar,
        # recua reto ate sobrar folga para o avanco previsto e a face continuar inteira (>= 15 cm)
        self.folga_min_lat = 0.15      # folga minima no fim do deslize (face inteira fora da zona cega)
        self.folga_max_recuo = 0.45    # nao recua alem disso
        self.recuo_max = 0.20          # por recuo
        self.k_lat = 0.5               # avanco por metro de lado (medido a cada deslize)
        self.n_recuo = 0
        self.recuo_alvo = None
        self.lat_ini = None            # (folga, |centro|) no inicio do ultimo deslize
"""),
    # 2) no 'medir': atualiza k_lat e decide recuar antes de deslizar
    ("""            if folga < 0.03:
                self.abortar('ja esta perto demais da mesa (%.0f cm)' % (folga * 100))
                return 0.0, 0.0, 0.0
            pode = self.ciclos <= 10
            if pode and abs(ang) > self.tol_giro:
                self.alvo_giro = _ang(yaw_scan + ang)
                self._mudar('girar', t, pose)
            elif pode and centro is not None and abs(centro) > self.tol_lat and self.n_lat < 5:
                self.n_lat += 1
                self.alvo_lat = centro
                self._mudar('deslocar', t, pose)
""",
     """            if folga < 0.03:
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
"""),
    # 3) no deslize continuo: para se a face chegar perto demais com o centro ainda fora
    ("""                previsto = abs(c) - v * (t - tl + 0.30)   # 0,30 s = atraso + parada
                if c * self.l_falta <= 0 or previsto <= 0.5 * self.tol_lat:
                    self._mudar(fim_lat, t, pose)
                    return 0.0, 0.0, 0.0
""",
     """                previsto = abs(c) - v * (t - tl + 0.30)   # 0,30 s = atraso + parada
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
"""),
    # 4) estado novo 'recuar' (antes do 'avancar')
    ("""        if e == 'avancar':
            # Avanco continuo ate ~6 cm antes do alvo, freando pelo LiDAR""",
     """        if e == 'recuar':
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
            # Avanco continuo ate ~6 cm antes do alvo, freando pelo LiDAR"""),
]


def aplicar(caminho):
    texto = open(caminho).read()
    if MARCA in texto:
        print('%s ja tem %s: nada a fazer' % (caminho, MARCA))
        return 0
    novo = texto
    for i, (velho, subst) in enumerate(TROCAS, 1):
        n = novo.count(velho)
        if n != 1:
            print('ERRO: trecho %d encontrado %d vezes (esperado 1) em %s; nada gravado' % (i, n, caminho))
            return 1
        novo = novo.replace(velho, subst)
    # conferencia: compila e so a classe Encosto mudou
    with tempfile.NamedTemporaryFile('w', suffix='.py', delete=False) as tmp:
        tmp.write(novo)
    try:
        py_compile.compile(tmp.name, doraise=True)
    finally:
        os.unlink(tmp.name)
    ini, fim = novo.index('class Encosto:'), novo.index('def main():')
    ini0, fim0 = texto.index('class Encosto:'), texto.index('def main():')
    if novo[:ini] != texto[:ini0] or novo[fim:] != texto[fim0:]:
        print('ERRO: a troca saiu da classe Encosto; nada gravado')
        return 1
    bak = '%s.bak_recuo_lateral_%s' % (caminho, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(caminho, bak)
    with open(caminho, 'w') as fp:
        fp.write(novo)
    print('%s aplicado em %s (backup %s)' % (MARCA, caminho, bak))
    return 0


def desfazer(caminho):
    baks = sorted(glob.glob(caminho + '.bak_recuo_lateral_*'))
    if not baks:
        print('nenhum backup .bak_recuo_lateral_* para %s' % caminho)
        return 1
    shutil.copy2(baks[-1], caminho)
    print('%s voltou para %s' % (caminho, baks[-1]))
    return 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--arquivo', default=PADRAO)
    ap.add_argument('--desfazer', action='store_true')
    a = ap.parse_args()
    sys.exit(desfazer(a.arquivo) if a.desfazer else aplicar(a.arquivo))
