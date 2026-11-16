import sys, copy, numpy as np, cv2
import fit_moved2 as f2
from joint_moved import proj
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[3] / 'goalnav'))
from localsim import messy_ladder as ml
q = copy.deepcopy(f2.P); [q[n]['lines'].pop('post') for n in q]
rows = []
for tag, (x, rms, sd) in (('feet+cap', f2.solve()), ('no post', f2.solve(q))):
    L = x[-6:-1]; g = ml.ladder(tuple(L[:2]), L[2]); outs = []
    for i, (n, f) in enumerate((('n2', 'New_Ladder_2.jpeg'), ('n1', 'New_Ladder_1.jpeg'))):
        c = x[7 * i:7 * i + 7]; im = cv2.imread(str(__import__('pathlib').Path(__file__).resolve().parents[3] / 'photos' / f))
        for name, (v, t) in g.items():
            col = {'Ladder rails': (0, 0, 255), 'Ladder metal': (255, 0, 255), 'Ladder top': (0, 255, 0)}[name]
            uv = proj(c, v)
            for tri in t: cv2.polylines(im, [uv[tri].astype(int).reshape(-1, 1, 2)], True, col, 1)
        cv2.putText(im, tag, (20, 60), 0, 2, (0, 255, 255), 3); outs.append(im)
    rows.append(np.hstack(outs))
cv2.imwrite('overlay_moved.jpg', cv2.resize(np.vstack(rows), (1200, 1600)))
