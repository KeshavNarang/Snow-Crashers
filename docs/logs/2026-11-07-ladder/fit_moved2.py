"""Joint fit of the moved ladder with its top cap observed too (the same ladder: top 1.71 m)
and the 5 ft rails' height free (prior 1.524 +- 0.05)."""
import sys, copy, numpy as np
from scipy.optimize import least_squares
import joint_moved as jm
from joint_moved import proj, LINES, corners
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[3] / 'goalnav'))
from localsim import messy_ladder as ml
P = copy.deepcopy(jm.photos)
P['n2']['feet'] = P['n2']['feet'][:3]
P['n2']['init'] = [-2.13, -4.18, 1.54, 107.5, 24.5, 5.8, 1170]
P['n1']['init'] = [-4.1, -1.8, 1.66, -23., 20., -5.2, 1158]
ASSIGN = {'n2': (0, 1, 2), 'n1': (2, 1, 0)}
CAP_OBS = {'n2': {'SW': (343, 182), 'NE': (583, 188)}, 'n1': {'NE': (735, 328), 'SW': (1000, 335)}}
def cap_corner(L, which):
    cx, cy, yaw, w, s = L; a = np.radians(yaw); u = np.array([np.cos(a), np.sin(a)]); v = np.array([-u[1], u[0]])
    c = np.array([cx, cy]) + u * (ml.TOP_FRONT_U + ml.TOP_REAR_U) / 2
    su, sv = {'SW': (-1, 1), 'NE': (1, -1), 'SE': (-1, -1), 'NW': (1, 1)}[which]   # rear/front, +v (west at yaw 90)
    return np.array([*(c + u * su * ml.CAP[0] / 2 + v * sv * ml.CAP[1] / 2), ml.TOP_Z])
def solve(photos=P, use_cap=True, free_rail=True, yaw0=90., cap_sig=8, names=('n2', 'n1'), f_prior=(1100, 400)):
    x0 = sum((photos[n]['init'] for n in names), []) + [-2.75, -2.47, yaw0, 0.50, 0.97, 1.524]
    def res(x):
        r = []; L = x[-6:-1]; rz = x[-1] if free_rail else 1.524; C = corners(L)
        for i, n in enumerate(names):
            c = x[7 * i:7 * i + 7]; ph = photos[n]
            for k, (obs, s) in ph['lines'].items():
                A, B = (np.array(p, float) for p in LINES[k])
                if k.endswith('5ft'): A[2] = B[2] = rz
                u = proj(c, np.array([A, B])); d = u[1] - u[0]; nn = np.array([-d[1], d[0]]) / np.linalg.norm(d)
                r += [((np.array(o) - u[0]) @ nn) / s for o in obs]
            for Pw, (uv, s) in ph['pts'].items(): r += list((proj(c, np.array(Pw))[0] - uv) / s)
            for f, k in zip(ph['feet'], ASSIGN[n]): r += list((proj(c, np.array([*C[k], 0.02]))[0] - f) / 10)
            if use_cap:
                for which, uv in CAP_OBS[n].items(): r += list((proj(c, cap_corner(L, which))[0] - uv) / cap_sig)
            r.append((c[6] - f_prior[0]) / f_prior[1])
        r += [(L[3] - 0.50) / 0.03, (L[4] - 0.97) / 0.04, (x[-1] - 1.524) / 0.05]
        return np.array(r)
    s = least_squares(res, np.array(x0, float)); r = res(s.x)
    cov = np.linalg.pinv(s.jac.T @ s.jac) * max(1., np.mean(r ** 2))
    return s.x, float(np.sqrt(np.mean(r ** 2))), np.sqrt(np.diag(cov))
def show(tag, out):
    x, rms, sd = out; L = x[-6:-1]
    print(f'{tag:34s} centre {np.round(L[:2], 3)} yaw {L[2]:.1f} w {L[3]:.2f} s {L[4]:.2f} rail_z {x[-1]:.3f} sd {np.round(sd[-6:-3], 3)} rms {rms:.2f}'
          f' f {x[6]:.0f}/{x[13]:.0f} camz {x[2]:.2f}/{x[9]:.2f}')
    return x
if __name__ == '__main__':
    best = show('feet + cap, rail z free', solve())
    show('feet + cap, rail z 1.524', solve(free_rail=False))
    show('feet only, rail z free', solve(use_cap=False))
    for y0 in (60., 120., 270.): show(f'feet + cap, yaw start {y0:.0f}', solve(yaw0=y0))
    q = copy.deepcopy(P); q['n2']['pts'] = {}; show('no barrier panel', solve(q))
    q = copy.deepcopy(P); q['n2']['lines'].pop('wfloor'); show('no tarp edge', solve(q))
    q = copy.deepcopy(P); [q[n]['lines'].pop('post') for n in q]; show('no corner post', solve(q))
    show('f prior 1170 +- 60', solve(f_prior=(1170, 60)))
    np.save('moved_best.npy', best)
