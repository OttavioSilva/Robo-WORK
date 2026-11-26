#!/usr/bin/env python3
"""NAV_INTEGRADA: comparativo offline (arvores anterior/reduzida/integrada; Shim x DWB direto).

Arvores: interpretador bt_mini com servidores simulados (mesmos cenarios para as tres).
Controle: sonda native_navigation (DWB/Navfn/costmaps reais, laco ideal, sem CM) com o YAML anterior
(backup .bak_navintegrada_*) e o integrado. Dominio 166, sem descoberta. Grava em
entregas/nav_integrada_20261110/resultados/comparativo.{json,md}.
"""
import glob
import json
import os
import subprocess
import sys
import tempfile

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
NAV = os.path.join(WS, 'src', 'robo_navigation')
OUT = os.path.join(WS, 'entregas', 'nav_integrada_20261110', 'resultados')
SONDA = os.path.join(WS, 'build', 'sondas_nav_integrada', 'native_navigation', 'native_navigation')
sys.path.insert(0, os.path.join(NAV, 'test'))
import test_arvore_integrada as T  # noqa: E402


def arvores():
    seg_uma = lambda m, t: T.NO_VALID_CONTROL if 1.0 <= t < 1.15 else None   # noqa: E731
    seg_sempre = lambda m, t: T.NO_VALID_CONTROL if t >= 1.0 else None       # noqa: E731
    casos = {
        'caminho livre': dict(),
        'obstaculo novo no restante (t=2,3 s)': dict(duracao=8.0, invalido=T._obstaculo_em(2.3)),
        'obstaculo so no trecho percorrido': dict(invalido=lambda m, c, t: t >= 2.0 and c[0] == 'smooth'),
        'suavizacao colide, original valido': dict(suaviza=lambda m, c, k, t: not k),
        'DWB sem trajetoria 1x': dict(seguir=seg_uma),
        'DWB sem trajetoria, recuperacao impossivel': dict(seguir=seg_sempre, destravar=lambda m, t: False),
    }
    res = []
    for caso, kw in casos.items():
        for nome, xml in T.ARVORES.items():
            st, m = T.rodar(xml, T.Mundo(**kw))
            planos = [e[0] for e in m.log if e[1] == 'ComputePathToPose']
            res.append({'caso': caso, 'arvore': nome, 'resultado': st, 'planejamentos': len(planos),
                        'atraso_replan_s': round(planos[1] - 2.3, 2) if caso.startswith('obstaculo novo') and len(planos) > 1 else None,
                        'destravar': m.conta('DriveOnHeading'), 'spin': m.conta('Spin'), 'limpezas': m.conta('Clear'),
                        'tempo_s': round(m.arv.t, 1)})
    return res


def nativo():
    if not os.path.exists(SONDA):
        return {'erro': 'sonda nao compilada (rode validar_nav_integrada.sh)'}
    baks = sorted(glob.glob(os.path.join(NAV, 'config', 'nav2_params.yaml.bak_navintegrada_*')))
    perfis = {'anterior (Shim)': baks[0] if baks else None, 'integrada (DWB direto)': os.path.join(NAV, 'config', 'nav2_params.yaml')}
    env = dict(os.environ, ROS_DOMAIN_ID='166', ROS_AUTOMATIC_DISCOVERY_RANGE='OFF')
    res = []
    for perfil, y in perfis.items():
        if not y:
            continue
        for cen, rep in (('open', False), ('lateral', False), ('curve', False), ('final_yaw', False),
                         ('obstacle', True), ('dynamic', True), ('corner', True), ('near', True)):
            with tempfile.TemporaryDirectory() as d:
                out = os.path.join(d, 'r.json')
                subprocess.run([SONDA, '--ros-args', '--params-file', y, '-p', 'experiment.scenario:=' + cen,
                                '-p', 'experiment.output:=' + out, '-p',
                                'experiment.replanning:=' + ('true' if rep else 'false')],
                               env=env, capture_output=True, timeout=300)
                r = json.load(open(out)) if os.path.exists(out) else {'error': 'sem saida'}
            r['perfil'] = perfil
            res.append(r)
    return res


def main():
    os.makedirs(OUT, exist_ok=True)
    a, n = arvores(), nativo()
    json.dump({'arvores': a, 'nativo': n}, open(os.path.join(OUT, 'comparativo.json'), 'w'), indent=1)
    L = ['# Comparativo offline NAV_INTEGRADA', '',
         'Gerado por `ferramentas/testes/comparar_nav_integrada.py`. Nada disto e medicao no robo.', '',
         '## Arvores (servidores simulados, mesmos cenarios)', '',
         '| Caso | Arvore | Resultado | Planejamentos | Atraso replan s | destravar | spin | limpezas |',
         '|---|---|---|---:|---:|---:|---:|---:|']
    for r in a:
        L.append('| %s | %s | %s | %d | %s | %d | %d | %d |' % (
            r['caso'], r['arvore'], r['resultado'], r['planejamentos'],
            '-' if r['atraso_replan_s'] is None else r['atraso_replan_s'], r['destravar'], r['spin'], r['limpezas']))
    L += ['', '## Controle (DWB/Navfn/costmaps reais, laco ideal sem CM nem fisica)', '']
    if isinstance(n, dict):
        L.append(n['erro'])
    else:
        L += ['| Perfil | Cenario | Chegou | Duracao s | Sem translacao s | Giro intermediario rad | Intersecoes | max vy |',
              '|---|---|---|---:|---:|---:|---:|---:|']
        for r in n:
            L.append('| %s | %s | %s | %s | %s | %s | %s | %s |' % (
                r['perfil'], r.get('scenario'), r.get('complete'), r.get('duration_s'), r.get('no_translation_s'),
                round(r.get('intermediate_rotation_rad', 0), 3), r.get('collisions'), r.get('max_vy')))
    open(os.path.join(OUT, 'comparativo.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))


if __name__ == '__main__':
    main()
