"""Build a self-contained 3D replay (``viewer.html``) of a goal-navigation run.

    python3 -m localsim.viewer3d localsim/out/goalnav/seed3

Shows the true irregular course (walls, tarps, tunnel and its tarp, boulders,
the near-invisible net and the room behind it), the drone and what it
believed: its live map growing, the raw depth returns, each goal's estimate
and the path it planned. three.js is inlined (``localsim/vendor/three``),
so the file opens offline in any browser.
"""
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drone_ref.course import read_obj      # noqa: E402

VENDOR = ROOT / 'localsim/vendor/three'
TEMPLATE = ROOT / 'localsim/viewer3d.html'
KINDS = (('Tunnel tarp', 'rocktarp'), ('Tunnel drape', 'rocktarp'), ('Tarp', 'tarp'), ('Panel', 'plywood'),
         ('Frame', 'wood'), ('Paver', 'paver'), ('Tube oval', 'wiremesh'), ('Tube', 'metal'),
         ('Boulder', 'rock'), ('Floor', 'floor'))


def _kind(name):
    for prefix, kind in KINDS:
        if name.startswith(prefix):
            return kind
    return 'other'


def _uv(kind, points, meta):
    """Texture coordinates in metres: along the wall and up for panels and
    tarps, around and along the tube, plan view for the floor."""
    if kind in ('plywood', 'tarp'):
        xy = points[:, :2] - points[:, :2].mean(axis=0)
        axis = np.linalg.svd(xy, full_matrices=False)[2][0]
        u = xy @ axis
        return np.column_stack([u - u.min(), points[:, 2]])
    if kind in ('wiremesh', 'rocktarp'):
        yaw = meta['tunnel']['yaw']
        axis = np.array([-np.sin(yaw), np.cos(yaw)])
        centre = np.asarray(meta['tunnel']['mouths']['north']['xy'])
        along = (points[:, :2] - centre) @ axis
        across = (points[:, :2] - centre) @ np.array([axis[1], -axis[0]])
        angle = np.arctan2(points[:, 2] - 0.94, across)
        return np.column_stack([angle * 0.85, along])
    if kind in ('floor', 'rock', 'paver'):
        return points[:, :2] + points[:, 2:3] * 0.7
    return None


def geometry(obj_path, meta):
    """Course meshes by material: {kind: [{name, v, i, uv, print}]}."""
    mesh = read_obj(obj_path)
    vertices = np.asarray(mesh.vertices)
    prints = {w['name']: w.get('print', 'scifi') for w in meta.get('walls', [])}
    out = {}
    for name, tris in mesh.groups.items():
        if not tris:
            continue
        tris = np.asarray(tris)
        used, inverse = np.unique(tris, return_inverse=True)
        kind = _kind(name)
        points = vertices[used]
        if kind == 'tarp' and prints.get(name[len('Tarp '):]) == 'rock':
            kind = 'rocktarp_wall'
        uv = _uv('tarp' if kind == 'rocktarp_wall' else kind, points, meta)
        entry = {'name': name, 'v': np.round(points, 3).reshape(-1).tolist(),
                 'i': inverse.reshape(-1).astype(int).tolist()}
        if uv is not None:
            entry['uv'] = np.round(uv, 3).reshape(-1).tolist()
        out.setdefault(kind, []).append(entry)
    return out


def scan_points(path=ROOT / 'assets/course-scan.npz', voxel=0.03):
    """The course scan, thinned to ``voxel``: base64 int16 xyz (mm) and uint8 rgb."""
    import base64
    if not path.exists():
        return None
    data = np.load(path)
    xyz, rgb = data['xyz_mm'], data['rgb']
    _, keep = np.unique(np.floor(xyz / (voxel * 1000)).astype(np.int64), axis=0, return_index=True)
    xyz, rgb = np.ascontiguousarray(xyz[keep]), np.ascontiguousarray(rgb[keep])
    return {'xyz': base64.b64encode(xyz.astype('<i2').tobytes()).decode(), 'rgb': base64.b64encode(rgb.tobytes()).decode(),
            'source': str(data['source']) if 'source' in data else ''}


def build(run_dir):
    run_dir = Path(run_dir)
    trace = json.loads((run_dir / 'trace.json').read_text())
    meta = json.loads((run_dir / 'course/course.json').read_text())
    keep = ('clutter', 'nets', 'pad', 'launch', 'lines', 'tunnel', 'seed', 'severity', 'walls', 'boulders',
            'guideline', 'hall')
    report = run_dir / 'report.json'
    pitch = json.loads(report.read_text()).get('camera_pitch_deg') if report.exists() else None
    data = {'trace': trace, 'meta': {**{k: meta.get(k) for k in keep}, 'camera_pitch_deg': pitch},
            'geometry': geometry(run_dir / 'course/course.obj', meta), 'scan': scan_points()}
    html = TEMPLATE.read_text()
    html = html.replace('/*THREE*/', (VENDOR / 'three.module.min.js').read_text())
    html = html.replace('/*ORBIT*/', (VENDOR / 'OrbitControls.js').read_text())
    html = html.replace('/*DATA*/', json.dumps(data, separators=(',', ':')).replace('</', '<\\/'))
    path = run_dir / 'viewer.html'
    path.write_text(html)
    return path


if __name__ == '__main__':
    for arg in sys.argv[1:]:
        print(build(arg))
