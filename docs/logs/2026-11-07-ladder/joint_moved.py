"""The moved ladder (photos/New_Ladder_1/2.jpeg, 1200x1600): cameras solved against
the net frame (SW corner post, both floor rails, both 5 ft rails) and, in photo 2,
barrier panel 1; the feet fitted as one rectangle, trying every foot-to-corner assignment."""
import itertools, numpy as np
from scipy.optimize import least_squares
from solve import rot
W, H = 1200, 1600
N = 3.048 + 0.024                       # pipe centrelines just outside the net
def proj(c, P):
    R = rot(*c[3:6]); q = (np.atleast_2d(P) - c[:3]) @ R.T
    return np.column_stack([c[6] * q[:, 0] / q[:, 2] + W / 2, c[6] * q[:, 1] / q[:, 2] + H / 2])
LINES = {   # world line (A, B) -> observed pixels, sigma
 'post':   ((-N, -N, 0.3), (-N, -N, 2.5)),
 'sfloor': ((-N, -N, 0.024), (0., -N, 0.024)),
 'wfloor': ((-N, -N, 0.024), (-N, 0., 0.024)),
 's5ft':   ((-N, -N, 1.524), (0., -N, 1.524)),
 'w5ft':   ((-N, -N, 1.524), (-N, 0., 1.524)),
}
photos = {
 'n2': dict(init=[-2.4, -4.0, 1.5, 115, 15, 0, 1100],
            lines={'post': ([(160, 350), (165, 800), (172, 1200)], 10),
                   'sfloor': ([(200, 1275), (500, 1410), (800, 1550)], 8),
                   'wfloor': ([(246, 1200), (336, 1100), (442, 1000), (631, 800)], 12),   # the tarp's edge
                   's5ft': ([(620, 318), (900, 320), (1180, 322)], 8)},
            pts={(-3.07, 0.29, 1.75): ((755, 225), 12), (-3.07, 0.29, 0.94): ((752, 472), 15), (-3.07, 0.29, 0.28): ((752, 625), 12),
                 (-2.12, 0.36, 0.94): ((1000, 470), 15), (-2.12, 0.36, 0.28): ((1000, 620), 12)},
            feet=[(265, 1238), (522, 1325), (703, 1010), (535, 1045)]),
 'n1': dict(init=[-3.8, -2.0, 1.5, -40, 15, 0, 1000],
            lines={'post': ([(1140, 700), (1145, 1100), (1150, 1350)], 15),
                   'wfloor': ([(1080, 1385), (900, 1500), (750, 1600)], 10),
                   'sfloor': ([(800, 1090), (950, 1240), (1070, 1370)], 10),
                   'w5ft': ([(300, 550), (600, 535), (1000, 497)], 12),
                   's5ft': ([(491, 430), (700, 434), (983, 440)], 12)},
            pts={}, feet=[(385, 1370), (815, 1175), (1010, 1350)]),
}
def corners(L):
    cx, cy, yaw, w, s = L; a = np.radians(yaw)
    u = np.array([np.cos(a), np.sin(a)]); v = np.array([-u[1], u[0]]); c = np.array([cx, cy])
    return [c - u * s / 2 + v * w / 2, c - u * s / 2 - v * w / 2, c + u * s / 2 - v * w / 2, c + u * s / 2 + v * w / 2]   # rear L, rear R, front R, front L
def solve(assign, yaw0, names=('n2', 'n1'), extra=None):
    x0 = sum((photos[n]['init'] for n in names), []) + [-2.6, -2.6, yaw0, 0.50, 0.97]
    def res(x):
        r = []; L = x[-5:]; C = corners(L)
        for i, n in enumerate(names):
            c = x[7 * i:7 * i + 7]; ph = photos[n]
            for k, (obs, s) in ph['lines'].items():
                A, B = (np.array(p) for p in LINES[k]); u = proj(c, np.array([A, B])); d = u[1] - u[0]
                nn = np.array([-d[1], d[0]]) / np.linalg.norm(d)
                r += [((np.array(o) - u[0]) @ nn) / s for o in obs]
            for P, (uv, s) in ph['pts'].items(): r += list((proj(c, np.array(P))[0] - uv) / s)
            for f, k in zip(ph['feet'], assign[n]): r += list((proj(c, np.array([*C[k], 0.02]))[0] - f) / 10)
            r.append((c[6] - 1100) / 400)             # weak prior: a phone's main camera, ~1100 px at 1200 wide
        r += [(L[3] - 0.50) / 0.03, (L[4] - 0.97) / 0.04]
        return np.array(r)
    s = least_squares(res, np.array(x0, float))
    r = res(s.x); cov = np.linalg.pinv(s.jac.T @ s.jac) * max(1., np.mean(r ** 2))
    return s.x, float(np.sqrt(np.mean(r ** 2))), np.sqrt(np.diag(cov))[-5:]
if __name__ == '__main__':
    results = []
    for a2 in itertools.permutations(range(4), 4):
        for a1 in itertools.permutations(range(4), 3):
            for yaw0 in (0., 90., 180., 270.):
                try: x, rms, sd = solve({'n2': a2, 'n1': a1}, yaw0)
                except Exception: continue
                results.append((rms, a2, a1, x, sd))
    results.sort(key=lambda t: t[0])
    seen = set()
    for rms, a2, a1, x, sd in results[:40]:
        L = x[-5:]; key = (round(L[0], 1), round(L[1], 1), round(L[2] % 360 / 10))
        if key in seen: continue
        seen.add(key)
        print(f'rms {rms:.2f} a2 {a2} a1 {a1} L {np.round(L[:2], 3)} yaw {L[2] % 360:.1f} w {L[3]:.2f} s {L[4]:.2f} sd {np.round(sd[:3], 3)}'
              f' cams n2 {np.round(x[:7], 2)} n1 {np.round(x[7:14], 2)}')
