import json, bisect, statistics as st
D = json.load(open('m34_curva1.json')); S = json.load(open('m34_segmentos.json'))
tx, sc, hw, cv = D['/hardware/tx_frames'], D['/scan_front'], D['/cmd_vel_hardware'], D['/cmd_vel']
def pos(t):
    """avanco (m, + frente) pela media das 2 paredes, scan mais proximo depois de t"""
    for s in sc:
        if s[0] >= t: return s
def adv(a, b):
    v = []
    for k in (1, 2):
        if a[k] is not None and b[k] is not None and abs(b[k]-a[k]) < .25: v.append(-(b[k]-a[k]))
    return st.mean(v) if v else None
linhas = []
for s in S:
    t0 = s['t']
    on = [x for x in tx if x[0] >= t0 - .05 and any(x[2:6])]
    if not on: continue
    ton = on[0][0]
    toff = next((x[0] for x in tx if x[0] > ton and not any(x[2:6])), None)
    # vx chegou ao hardware quando?
    tvx = next((h[0] for h in hw if h[0] >= ton and abs(h[1]) >= .01), None)
    a0, a1, a2, a3 = pos(ton - .12), pos(ton + .35), pos(toff), pos(toff + .45)
    r = dict(hora=s['hora'], dur_motor=round(toff - ton, 2), atraso_vx=round(tvx - ton, 2) if tvx else None,
             rampa_cm=adv(a0, a1), regime_cm=adv(a1, a2), inercia_cm=adv(a2, a3),
             rampa_dt=a1[0]-a0[0], regime_dt=a2[0]-a1[0])
    for k in ('rampa_cm', 'regime_cm', 'inercia_cm'):
        r[k] = round(r[k]*100, 1) if r[k] is not None else None
    r['regime_cm_s'] = round(r['regime_cm']/r['regime_dt'], 1) if r['regime_cm'] is not None and r['regime_dt'] > .1 else None
    pw = [x for x in tx if ton + .4 <= x[0] < toff]
    if pw:
        r['pwm_tras(FL,RR)'] = round(st.mean((abs(p[2]) + abs(p[5]))/2 for p in pw))
        r['pwm_frente(FR,RL)'] = round(st.mean((abs(p[3]) + abs(p[4]))/2 for p in pw))
        r['pwm_esq(FL,RL)'] = round(st.mean((abs(p[2]) + abs(p[4]))/2 for p in pw))
        r['pwm_dir(FR,RR)'] = round(st.mean((abs(p[3]) + abs(p[5]))/2 for p in pw))
    r['vx_hw_med'] = s['vx_hw']
    linhas.append(r); print(r)
def med(k):
    x = [r[k] for r in linhas if r.get(k) is not None]; return round(st.median(x), 2), round(min(x), 2), round(max(x), 2), len(x)
print('MEDIANA (min,max,n): rampa', med('rampa_cm'), 'regime', med('regime_cm'), 'cm/s', med('regime_cm_s'), 'inercia', med('inercia_cm'), 'atraso vx', med('atraso_vx'))
json.dump(linhas, open('m34_fases.json', 'w'), indent=1)
