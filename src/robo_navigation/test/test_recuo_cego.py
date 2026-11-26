"""RECUO_CEGO (24/11): encosto que comeca perto demais recua 12 cm e mede de novo (2x), depois aborta."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from alinhador_passagem import Encosto  # noqa: E402

PARCIAL = dict(dist=.42, ang=0., centro=None, span=.28, parcial=True)


def medir_sem_face(e, t, pose):
    e._mudar('medir', t, pose)
    for k in range(40):
        e.nova_medicao(t + .1 * k, PARCIAL, pose)
        cmd = e.passo(t + .05 + .1 * k, pose)
        if e.estado != 'medir':
            return cmd, t + .1 * k
    return cmd, t + 4


def test_recua_duas_vezes_e_depois_aborta():
    e = Encosto(distancia_alvo=.08, log=lambda _: None)
    e.t0 = 0.
    pose, t = (0., 0., 0.), 10.
    for n in (1, 2):
        _, t = medir_sem_face(e, t, pose)
        assert e.estado == 'recuar_cego' and e.n_recuo_cego == n
        vx, vy, wz = e.passo(t + .1, pose)
        assert vx < 0 and vy == 0 and wz == 0
        pose = (pose[0] - .12, 0., 0.)          # andou 12 cm para tras
        e.passo(t + 1.5, pose)
        assert e.estado == 'assentar'
        t += 2
    medir_sem_face(e, t, pose)
    assert e.estado == 'abortar'


def test_recuo_cego_respeita_guarda_de_tempo():
    e = Encosto(distancia_alvo=.08, log=lambda _: None)
    e.t0 = 0.
    _, t = medir_sem_face(e, 10., (0., 0., 0.))
    assert e.estado == 'recuar_cego'
    e.passo(t + 5.5, (0., 0., 0.))              # nao andou (preso): 5 s e para
    assert e.estado == 'assentar'


def test_desligado_aborta_como_antes():
    e = Encosto(distancia_alvo=.08, log=lambda _: None)
    e.recuo_cego_max = 0
    e.t0 = 0.
    medir_sem_face(e, 10., (0., 0., 0.))
    assert e.estado == 'abortar'
