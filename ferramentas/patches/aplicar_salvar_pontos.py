#!/usr/bin/env python3
"""SALVAR_PONTOS (24/11/2026): salvar_mapa.sh grava os pontos MARCADOS (teleop), nao mais (0,0,0) fixo.

Antes o <mapa>_pontos.yaml era sempre START = FINISH = (0,0,0) e ignorava o que o teleop marcou
(D-pad: START/FINISH e, com TELEOP_CAIXA2, START_2/FINISH_2). Agora: se ~/mapas/pontos.yaml tem START,
copia todos os pontos dele; senao, igual a antes (0,0,0).
Uso: aplicar_salvar_pontos.py | --desfazer | --arquivo <copia>
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_texto import rodar  # noqa: E402

PADRAO = os.path.expanduser('~/robo_jazzy_ws/ferramentas/subida_unica/salvar_mapa.sh')

TROCAS = [
    ("""cat > "$BASE"_pontos.yaml <<EOF
""",
     """# SALVAR_PONTOS (24/11): usa os pontos marcados pelo teleop (START/FINISH e caixa 2), se houver
if python3 - "$BASE" <<'PY'
import sys, os, time, yaml
src = os.path.expanduser('~/mapas/pontos.yaml')
p = (yaml.safe_load(open(src)) or {}).get('pontos') or {}
if 'START' not in p:
    sys.exit(1)
p.setdefault('FINISH', dict(p['START']))
with open(sys.argv[1] + '_pontos.yaml', 'w') as f:
    f.write('# Pontos marcados (teleop) copiados de ~/mapas/pontos.yaml pelo salvar_mapa.sh em %s\\n'
            % time.strftime('%d/%m %H:%M'))
    yaml.safe_dump({'pontos': p}, f, sort_keys=True)
print('>>> pontos marcados: ' + ', '.join(sorted(p)))
PY
then :; else
echo ">>> sem START marcado em ~/mapas/pontos.yaml: START = FINISH = (0,0,0)"
cat > "$BASE"_pontos.yaml <<EOF
"""),
    ("""    yaw: 0.0
EOF
FALTA=0
""",
     """    yaw: 0.0
EOF
fi  # SALVAR_PONTOS
FALTA=0
"""),
]


def conferir(velho, novo):
    assert novo.count('SALVAR_PONTOS') == 2
    import subprocess
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.sh', delete=False) as f:
        f.write(novo)
    assert subprocess.run(['bash', '-n', f.name]).returncode == 0, 'bash -n falhou'
    os.unlink(f.name)


if __name__ == '__main__':
    rodar('salvar_pontos', 'SALVAR_PONTOS', PADRAO, TROCAS, conferir, doc=__doc__)
