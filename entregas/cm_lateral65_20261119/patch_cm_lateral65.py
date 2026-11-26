#!/usr/bin/env python3
"""CM_LATERAL65 (19/11 noite): no perfil operacional gerado, o poligono lateral do CM vale de 65 a 135 graus
(era 80, CM_PROTECAO 11/11). O deslize da curva para a direita com centragem (vx +0,023, vy -0,08 = -74 graus)
caia no diag_dir, que vai 11 cm a frente do para-choque, e o CM parava com a parede a 12 cm.
Nao mexe no nav2_params.yaml de base (os testes de perfil e o --desfazer do CM_PROTECAO dependem dele)."""
import argparse
from pathlib import Path
import yaml

LIM = 1.134  # 65 graus
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--arquivo', required=True)
a = p.parse_args()
f = Path(a.arquivo)
cfg = yaml.safe_load(f.read_text())
v = cfg['collision_monitor']['ros__parameters']['VelocityPolygonStop']
esperado = {('diag_esq', 'direction_end_angle'): 1.396, ('left', 'direction_start_angle'): 1.396,
            ('right', 'direction_end_angle'): -1.396, ('diag_dir', 'direction_start_angle'): -1.396}
novo = {('diag_esq', 'direction_end_angle'): LIM, ('left', 'direction_start_angle'): LIM,
        ('right', 'direction_end_angle'): -LIM, ('diag_dir', 'direction_start_angle'): -LIM}
atual = {k: v[k[0]][k[1]] for k in esperado}
if atual == novo:
    print('CM_LATERAL65: ja aplicado'); raise SystemExit(0)
if atual != esperado:
    raise SystemExit('CM_LATERAL65: setores inesperados %s; nada alterado' % atual)
for k, val in novo.items():
    v[k[0]][k[1]] = val
f.write_text(yaml.safe_dump(cfg, sort_keys=False))
print('CM_LATERAL65: lateral 65-135 graus (diag 45-65) em %s' % f)
