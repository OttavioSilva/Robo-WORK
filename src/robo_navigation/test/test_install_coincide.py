"""INSTALL_COINCIDE (11/11/2026): programa instalado com RENAME vira COPIA no --symlink-install (config e launch
viram link). Em 11/11 o lidar.yaml (link) perdeu os setores bloqueados as 12:36, mas o scan_filter_node instalado
continuou o de 01/10, sem a mascara do corpo: o /scan_front do robo saiu com ~6.300 autorretornos em 185 scans (raios
-156..-114 e +104..+166 graus) para o SLAM e o alinhador. Este teste falha se a copia instalada ficar diferente do src.
"""
from pathlib import Path

import pytest

WS = Path(__file__).resolve().parents[3]
PARES = [
    ('install/robo_lidar/lib/robo_lidar/scan_filter_node', 'src/robo_lidar/scripts/scan_filter_node.py'),
    ('install/robo_localization/lib/robo_localization/command_imu_odometry_node',
     'src/robo_localization/scripts/command_imu_odometry_node.py'),
]


@pytest.mark.parametrize('inst, src', PARES)
def test_programa_instalado_igual_ao_src(inst, src):
    pi, ps = WS / inst, WS / src
    if not pi.exists():
        pytest.skip('sem install (workspace nao compilado)')
    assert pi.read_bytes() == ps.read_bytes(), (
        '%s difere de %s: rode  cd ~/robo_jazzy_ws && colcon build --symlink-install --packages-select %s' % (
            inst, src, inst.split('/')[1]))


def test_lidar_yaml_sem_setor_exige_mascara_do_corpo():
    """Sem setores bloqueados, o filtro instalado TEM de ter a mascara do corpo ligada."""
    import yaml
    p = yaml.safe_load((WS / 'src/robo_lidar/config/lidar.yaml').read_text())['scan_filter']['ros__parameters']
    if not str(p.get('blocked_intervals_deg', '')).strip():
        assert p.get('body_filter_enabled') is True
        inst = WS / PARES[0][0]
        if inst.exists():
            assert 'ponto_do_corpo' in inst.read_text()
