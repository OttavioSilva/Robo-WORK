import json, sys, bisect, math
D = json.load(open('m34_curva1.json')); S = json.load(open('m34_segmentos.json'))
nav, cv, hw, tx, imu, sc = (D[k] for k in ['/cmd_vel_nav', '/cmd_vel', '/cmd_vel_hardware', '/hardware/tx_frames', '/imu/data', '/scan_front'])
def em(L, t):
    i = bisect.bisect_right([x[0] for x in L], t) - 1
    return L[i] if i >= 0 else None
for k in map(int, sys.argv[1:]):
    t0 = S[k]['t']; print('--- seg', k, S[k]['hora'])
    eventos = sorted([(x[0], 'tx', x) for x in tx if t0 - 0.3 <= x[0] <= t0 + 1.6] + [(x[0], 'sc', x) for x in sc if t0 - 0.6 <= x[0] <= t0 + 1.6])
    for t, k2, x in eventos:
        n, c, h, i = em(nav, t), em(cv, t), em(hw, t), em(imu, t)
        if k2 == 'tx':
            print('%+5.2f nav %+.3f %+.3f %+.3f | cv %+.3f %+.3f | hw %+.3f %+.3f %+.3f | PWM %4d %4d %4d %4d | gz %+.3f' % (
                t - t0, n[1], n[2], n[3], c[1], c[2], h[1], h[2], h[3], *x[2:6], i[2]))
        else:
            print('%+5.2f        SCAN frente %s  tras %s' % (t - t0, '%.3f' % x[1] if x[1] else '-', '%.3f' % x[2] if x[2] else '-'))
