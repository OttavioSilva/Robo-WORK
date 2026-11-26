import json, sys, bisect, math, statistics as st, datetime
D = json.load(open(sys.argv[1]))
nav, cv, hw, tx, imu, sc = (D[k] for k in ['/cmd_vel_nav', '/cmd_vel', '/cmd_vel_hardware', '/hardware/tx_frames', '/imu/data', '/scan_front'])
def hora(t): return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4]
def em(L, t):
    i = bisect.bisect_right([x[0] for x in L], t) - 1
    return L[i] if i >= 0 else None
# segmentos: cmd_vel_nav com |vy|>=0.03
segs, ini = [], None
for i, (t, vx, vy, wz) in enumerate(nav):
    ativo = abs(vy) >= 0.03
    if ativo and ini is None: ini = i
    if not ativo and ini is not None:
        segs.append((ini, i - 1)); ini = None
tsc = [s[0] for s in sc]
def parede(t, lado):
    i = bisect.bisect_left(tsc, t)
    for j in range(i, min(i + 3, len(sc))):
        if sc[j][lado] is not None: return sc[j][0], sc[j][lado]
    return None
print('seg | inicio | dur s | vx_nav med (min..max) | vx /cmd_vel med | vx_hw med | PWM 1o quadro FL FR RL RR | avanco LiDAR cm (frente,tras) @0,5 @1,0 @fim | dyaw graus')
res = []
for a, b in segs:
    t0, t1 = nav[a][0], nav[b][0]
    if t1 - t0 < 0.3: continue
    vxn = [x[1] for x in nav[a:b+1]]
    vxc = [x[1] for x in cv if t0 <= x[0] <= t1]
    vxh = [x[1] for x in hw if t0 <= x[0] <= t1 and (abs(x[2]) > 0.01)]
    pw = [x for x in tx if t0 - 0.05 <= x[0] <= t1 and any(x[2:6])]
    pw0 = pw[0][2:6] if pw else None
    pwm_med = [round(st.median(abs(p[2+k]) for p in pw)) for k in range(4)] if pw else None
    av = []
    for dt in (0.5, 1.0, t1 - t0):
        v = []
        for lado in (1, 2):
            p0, p1 = parede(t0 - 0.1, lado), parede(t0 + dt, lado)
            if p0 and p1 and abs(p1[1] - p0[1]) < 0.25:
                v.append(-(p1[1] - p0[1]) * 100)
        av.append(tuple(round(x, 1) for x in v))
    y0, y1 = em(imu, t0), em(imu, t1 + 0.2)
    dyaw = math.degrees(math.atan2(math.sin(y1[1] - y0[1]), math.cos(y1[1] - y0[1])))
    r = dict(t=t0, hora=hora(t0), dur=round(t1 - t0, 2), vx_nav=round(st.median(vxn), 3), vx_nav_max=round(max(vxn), 3),
             vx_cv=round(st.median(vxc), 3) if vxc else None, vx_cv_max=round(max(vxc), 3) if vxc else None,
             vx_hw=round(st.median(vxh), 3) if vxh else None, vy=round(st.median(x[2] for x in nav[a:b+1]), 3),
             pwm0=pw0, pwm_med=pwm_med, av=av, dyaw=round(dyaw, 2))
    res.append(r)
    print(r)
json.dump(res, open('m34_segmentos.json', 'w'), indent=1)
