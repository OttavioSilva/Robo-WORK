#!/usr/bin/env python3
"""PORTA_BASE (25/11/2026): base pela porta fisica (by-path) em vez do by-id.
Base e garra usam CH340 sem numero de serie: o by-id aponta para qualquer um dos dois.
Hub: garra em usb-0:3.3, base em usb-0:3.4 (medido tirando o cabo da garra).
Uso: python3 aplicar_porta_base.py [--arquivo YAML] [--desfazer]
"""
import argparse, datetime, glob, os, shutil, sys
import yaml

MARCA = 'PORTA_BASE'
ANTIGA = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
NOVA = '/dev/serial/by-path/pci-0000:00:14.0-usb-0:3.4:1.0-port0'

ap = argparse.ArgumentParser()
ap.add_argument('--arquivo', default=os.path.expanduser('~/robo_jazzy_ws/src/robo_hardware/config/hardware.yaml'))
ap.add_argument('--desfazer', action='store_true')
a = ap.parse_args()
p = a.arquivo

if a.desfazer:
    baks = sorted(glob.glob(p + '.bak_porta_base_*'))
    if not baks:
        sys.exit('sem backup')
    shutil.copy2(baks[-1], p); print(f'voltou {p} <- {baks[-1]}'); sys.exit()

txt = open(p).read()
if MARCA in txt:
    sys.exit(f'ja aplicado ({MARCA}). Nada feito.')
linha = f'    serial_port: {ANTIGA}\n'
if txt.count(linha) != 1:
    sys.exit('!!! linha serial_port esperada nao encontrada. Nada gravado.')
novo = txt.replace(linha, f'    # {MARCA} (25/11): era {ANTIGA} (by-id pega base OU garra)\n'
                          f'    serial_port: {NOVA}\n')
a0, a1 = yaml.safe_load(txt), yaml.safe_load(novo)
a1p = a1['robo_hardware']['ros__parameters']
assert a1p['serial_port'] == NOVA
a1p['serial_port'] = ANTIGA
assert a0 == a1, 'mudou mais que serial_port'
bak = f'{p}.bak_porta_base_{datetime.datetime.now():%Y%m%d_%H%M%S}'
shutil.copy2(p, bak)
open(p, 'w').write(novo)
print(f'gravado {p} (backup {bak})')
if not os.path.exists(NOVA):
    print(f'AVISO: {NOVA} nao existe agora (cabo da base ligado no hub?)')
