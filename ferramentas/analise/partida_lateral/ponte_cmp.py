import json, bisect
from modelo import pwm
D = json.load(open('m34_curva1.json'))
hw, tx = D['/cmd_vel_hardware'], D['/hardware/tx_frames']
th = [x[0] for x in hw]
ok = n = 0; dif = []
for x in tx:
    i = bisect.bisect_right(th, x[0]) - 1
    if i < 0: continue
    p = pwm(*hw[i][1:])
    real = list(x[2:6])
    d = max(abs(a - b) for a, b in zip(p, real)); dif.append(d)
    n += 1; ok += d <= 1
dif.sort()
print('quadros %d | |PWM modelo - real| <= 1 em %.1f%% | mediana %d p95 %d max %d' % (n, 100*ok/n, dif[n//2], dif[int(.95*n)], dif[-1]))
