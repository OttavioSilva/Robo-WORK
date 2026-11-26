"""PARTIDA_LATERAL (12/11): modelos offline validados na missao34 (CURVA 1).
- Smoother: nav2 velocity_smoother OPEN_LOOP, scale_velocities, deadband 0,015, estado interno ANTES do deadband
  (nav2_params.yaml atual). Contra /cmd_vel_smoothed: 99,5% das 1264 amostras com erro < 0,003.
- Ponte: calculate_motor_pwm/wheel_speed_to_pwm (hardware_bridge_node.cpp + hardware.yaml). Contra
  /hardware/tx_frames a partir de /cmd_vel_hardware: 1335/1335 quadros identicos (diferenca maxima 0 PWM).
Sem ROS: importavel nos testes."""
import math

HZ = 20.0
ACC, DEC = (0.2, 0.2, 0.8), (-0.5, -0.5, -1.2)
DB = (0.015, 0.015, 0.015)
VMAX, VMIN = (0.20, 0.16, 0.40), (-0.16, -0.16, -0.40)


def _lim(vc, vt, a, d):
    if abs(vt) >= abs(vc) and vc * vt >= 0:
        return -a / HZ, a / HZ
    return d / HZ, -d / HZ


class Smoother:
    def __init__(self):
        self.cur = [0.0, 0.0, 0.0]

    def step(self, cmd):
        cmd = [max(VMIN[i], min(VMAX[i], cmd[i])) for i in range(3)]
        eta = 1.0
        for i in range(3):
            lo, hi = _lim(self.cur[i], cmd[i], ACC[i], DEC[i])
            dv = cmd[i] - self.cur[i]
            e = hi / dv if dv > hi else (lo / dv if dv < lo else -1.0)
            if e > 0 and abs(1 - e) > abs(1 - eta):
                eta = e
        out = []
        for i in range(3):
            lo, hi = _lim(self.cur[i], cmd[i], ACC[i], DEC[i])
            out.append(self.cur[i] + max(lo, min(hi, eta * (cmd[i] - self.cur[i]))))
        self.cur = out
        return [0.0 if abs(out[i]) < DB[i] else out[i] for i in range(3)]


L, W, VMAXW, DBV, PMIN, PMINROT, PMAX = 0.340, 0.615, 0.310, 0.005, 70, 50, 215
K = 0.5 * (L + W)


def rodas(vx, vy, wz):
    """FL, FR, RL, RR (m/s) como a ponte."""
    return [vx - vy - K * wz, vx + vy + K * wz, vx + vy - K * wz, vx - vy + K * wz]


def pwm(vx, vy, wz=0.0):
    ws = rodas(vx, vy, wz)
    g = max(abs(s) for s in ws)
    if g > VMAXW:
        ws = [s * VMAXW / g for s in ws]
    pm = PMINROT if (math.hypot(vx, vy) < DBV and abs(wz) > 0) else PMIN
    out = []
    for s in ws:
        if abs(s) < DBV:
            out.append(0)
            continue
        p = int(math.floor(pm + min(1.0, abs(s) / VMAXW) * (PMAX - pm) + 0.5))
        out.append(-p if s < 0 else p)
    return out


def replay(comandos):
    """comandos: lista de (vx, vy, wz) do alinhador a 20 Hz -> [(smoothed, pwm)]."""
    s = Smoother()
    res = []
    for c in comandos:
        o = s.step(list(c))
        res.append((o, pwm(*o)))
    return res
