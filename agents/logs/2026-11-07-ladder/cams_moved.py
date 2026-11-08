import numpy as np, itertools
from scipy.optimize import least_squares
from joint_moved import photos, LINES, proj, corners, W, H
from solve import rot
def cam_solve(n, f_prior=1100, f_sig=400, inits=None):
    ph = photos[n]
    def res(c):
        r = []
        for k, (obs, s) in ph['lines'].items():
            A, B = (np.array(p) for p in LINES[k]); u = proj(c, np.array([A, B])); d = u[1] - u[0]
            nn = np.array([-d[1], d[0]]) / np.linalg.norm(d); r += [((np.array(o) - u[0]) @ nn) / s for o in obs]
        for P, (uv, s) in ph['pts'].items(): r += list((proj(c, np.array(P))[0] - uv) / s)
        r.append((c[6] - f_prior) / f_sig); return np.array(r)
    best = None
    for init in inits or [ph['init']]:
        s = least_squares(res, np.array(init, float))
        rr = res(s.x); rms = float(np.sqrt(np.mean(rr ** 2)))
        if best is None or rms < best[1]: best = (s.x, rms, rr)
    return best
def floor(c, uv, z=0.02):
    R = rot(*c[3:6]); d = R.T @ np.array([(uv[0] - W / 2) / c[6], (uv[1] - H / 2) / c[6], 1.]); t = (z - c[2]) / d[2]
    return (c[:3] + t * d)[:2]
if __name__ == '__main__':
    import sys
    inits = {'n2': [[x, y, 1.5, h, 15, 0, 1100] for x in (-2.8, -2.2, -1.6) for y in (-3.6, -4.2) for h in (100, 115, 130)],
             'n1': [[x, y, 1.5, h, 15, 0, 1000] for x in (-3.5, -4.0) for y in (-2.6, -2.0, -1.4) for h in (-20, -40, -60)]}
    for n in ('n2', 'n1'):
        for fp, fs in ((1100, 400), (1100, 60), (900, 60), (1300, 60)):
            c, rms, rr = cam_solve(n, fp, fs, inits[n])
            ft = np.array([floor(c, f) for f in photos[n]['feet']])
            print(n, 'f prior', fp, fs, 'cam', np.round(c, 2), 'rms %.2f' % rms, 'feet', np.round(ft, 2).tolist())
