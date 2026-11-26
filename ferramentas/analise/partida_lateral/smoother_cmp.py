import json, bisect
from modelo import Smoother
D = json.load(open('m34_curva1.json')); S = json.load(open('m34_segmentos.json'))
nav, sm = D['/cmd_vel_nav'], D['/cmd_vel_smoothed']; tn = [x[0] for x in nav]
for lag in (0.0, 0.02, 0.04):
  M = Smoother(); ok = 0; outs = []
  for t, vx, vy, wz in sm:
    i = bisect.bisect_right(tn, t - lag) - 1
    c = nav[i][1:] if i >= 0 and t - nav[i][0] < 0.35 else (0, 0, 0)
    o = M.step(list(c)); outs.append((t, o, vx, vy))
    ok += max(abs(o[0]-vx), abs(o[1]-vy)) < 0.003
  print('lag', lag, 'concordancia %.1f%%' % (100*ok/len(sm)))
t0 = S[5]['t']
for t, o, vx, vy in outs:
  if t0 - .1 < t < t0 + .6: print('%+.2f modelo %+.3f %+.3f | bag %+.3f %+.3f' % (t-t0, o[0], o[1], vx, vy))
