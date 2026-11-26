"""NAV_INTEGRADA: destravar v4 REAL (processo ROS) num dominio isolado (166, so localhost).

O robo usa o dominio 0; nada aqui chega a ele. Sem ROS no ambiente, pula.
"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

AQUI = Path(__file__).resolve().parent
NO = AQUI.parent / 'scripts' / 'destravar_node.py'
CEN = AQUI / 'ros_cenario_destravar.py'


def rodar(modo, dominio):
    env = dict(os.environ, ROS_DOMAIN_ID=str(dominio), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    try:
        import rclpy  # noqa: F401
    except ImportError:
        pytest.skip('sem ROS')
    no = subprocess.Popen([sys.executable, str(NO)], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True)
    try:
        r = subprocess.run([sys.executable, str(CEN), modo], env=env, capture_output=True, text=True, timeout=120)
    finally:
        no.send_signal(2)
        try:
            log, _ = no.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            no.kill()
            log, _ = no.communicate()
    linha = [ln for ln in r.stdout.splitlines() if ln.startswith('RESULTADO_JSON=')]
    assert linha, r.stdout[-3000:] + r.stderr[-3000:] + log[-3000:]
    resultado = json.loads(linha[0].split('=', 1)[1])
    assert r.returncode == 0 and 'erro' not in resultado, (resultado, r.stderr[-3000:], log[-3000:])
    return resultado, log


def test_cm_inativo_aborta_sem_mover():
    r, log = rodar('cm_inativo', 166)
    assert r['aceito'] and r['status'] == 6, r          # ABORTED
    assert 'Collision Monitor inativo' in r['error_msg'] and r['cmds_nao_nulos'] == 0


def test_contorno_normal_devolve_ao_nav2_sem_mover():
    r, log = rodar('cm_ativo', 167)
    assert r['status'] == 4, (r, log[-2000:])            # SUCCEEDED
    assert 'contorno normal' in r['error_msg'] and r['cmds_nao_nulos'] == 0
    assert 'DIAGNOSTICO' in log and 'DECISAO: retomar' in log


def test_comando_estrangeiro_impede_manobra():
    r, log = rodar('estrangeiro', 168)
    assert r['status'] == 6 and 'exclusivo' in r['error_msg'], r
    assert r['cmds_nao_nulos'] == 0


def test_cancelamento_durante_manobra_para():
    r, log = rodar('cancelar', 169)
    assert r['status'] in (5, 6), (r, log[-3000:])       # CANCELED/ABORTED, nunca SUCCEEDED
    assert r['cmds_nao_nulos'] > 0, (r, log[-3000:])       # estava andando quando cancelou
    assert r['ultimo_cmd'] == [0.0, 0.0, 0.0]
    assert 'oscilacao' in log and 'DECISAO: lateral' in log
