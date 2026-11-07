"""The labelled course map: the scan with the code's names, the goal chain, one simulated run, the window wall's elevation and the drone's side view, as one HTML page.

    PYTHONPATH=. python3 tools/course_map.py . RUN_DIR OUT.html
"""
import json, math, sys
from pathlib import Path
import numpy as np, yaml
from drone_ref.course import read_obj
from localsim import messy_course

S, H = 92, 3.35                      # px per metre, half extent shown
def xy(p): return (p[0] + H) * S, (H - p[1]) * S
def pts(a): return ' '.join(f'{x:.1f},{y:.1f}' for x, y in map(xy, a))

def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def hull(p):
    p = sorted(map(tuple, np.round(p, 4)))
    def half(seq):
        out = []
        for q in seq:
            while len(out) >= 2 and cross(out[-2], out[-1], q) <= 0: out.pop()
            out.append(q)
        return out
    lo, hi = half(p), half(p[::-1])
    return np.array(lo[:-1] + hi[:-1])

SHORT = {'M1 maze divider': 'M1', 'M2 maze divider': 'M2', 'M3 maze divider': 'M3', 'C1 chicane wall': 'C1',
         'A1 approach wall': 'A1', 'C2a chicane wall (window)': 'C2a', 'C2b chicane wall': 'C2b',
         'Barrier 1': 'Barrier 1', 'Barrier 2': 'Barrier 2', 'Barrier 3 (cut to 55 in)': 'Barrier 3'}
LABEL_AT = {'M1': (0.16, 0.0, -90), 'M2': (0.16, 0.0, -90), 'M3': (0.16, 0.0, -90), 'C1': (0.2, 0, 0), 'A1': (0, 0.18, 0),
            'C2a': (-0.42, -0.2, 0), 'C2b': (0.18, 0, 0), 'Barrier 1': (0, -0.17, 0), 'Barrier 2': (0, -0.17, 0), 'Barrier 3': (0, -0.17, 0)}

def text(p, s, cls='lab', rot=0, anchor='middle'):
    x, y = xy(p)
    r = f' transform="rotate({rot} {x:.1f} {y:.1f})"' if rot else ''
    return f'<text x="{x:.1f}" y="{y:.1f}" class="{cls}" text-anchor="{anchor}"{r}>{s}</text>'

def frame(title):
    out = [f'<rect x="0" y="0" width="{2*H*S}" height="{2*H*S}" class="bg"/>']
    for v in np.arange(-3, 3.01, 1):
        a, b = xy((v, -H)), xy((v, H)); out.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" class="grid"/>')
        a, b = xy((-H, v)), xy((H, v)); out.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" class="grid"/>')
        out.append(text((v, -H + 0.08), f'{v:+.0f}', 'tick')); out.append(text((-H + 0.06, v - 0.03), f'{v:+.0f}', 'tick', anchor='start'))
    return out

def raster_png(points, colours, x0, x1, y0, y1, res, dot=1):
    """Top-down image of points (later ones drawn over earlier), as a base64 PNG."""
    import base64, struct, zlib
    W, H_ = int((x1 - x0) / res), int((y1 - y0) / res)
    img = np.full((H_, W, 3), 251, np.uint8)
    i = ((points[:, 0] - x0) / res).astype(int); j = H_ - 1 - ((points[:, 1] - y0) / res).astype(int)
    ok = (i >= 0) & (i < W) & (j >= 0) & (j < H_)
    for dx in range(dot):
        for dy in range(dot):
            img[np.clip(j[ok] - dy, 0, H_ - 1), np.clip(i[ok] + dx, 0, W - 1)] = colours[ok]
    raw = b''.join(b'\x00' + img[y].tobytes() for y in range(H_))
    def chunk(t, d): return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', W, H_, 8, 2, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b'')
    return base64.b64encode(png).decode()

# Where the scan puts things (m, course frame), measured in it.
SCAN_WALLS = {'M1': ((-1.95, 0.52), (-1.95, 2.24)), 'M2': ((-0.51, 1.46), (-0.51, 3.0)), 'M3': ((1.0, 0.51), (1.0, 2.32)),
              'A1': ((-1.47, 0.16), (-1.47, -0.96)), 'C1': ((1.11, 0.26), (1.55, -1.24)),
              'C2a': ((-0.74, -3.05), (-0.19, -1.38))}
SCAN_LABEL = {'M1': (-0.17, 0.0, -90), 'M2': (0.17, 0.0, -90), 'M3': (-0.17, 0.0, -90), 'A1': (0.17, 0, -90),
              'C1': (0.25, 0.1, 0), 'C2a': (0.62, 0.62, 0)}

def scan_panel(root, spec, nominal):
    d = np.load(root / 'assets/course-scan.npz')
    w, c = d['xyz_mm'] / 1000., d['rgb']
    sel = (w[:, 2] > 0.3) & (w[:, 2] < 1.7)
    floor = (w[:, 2] < 0.03)
    pick = np.concatenate([np.where(floor)[0], np.where(sel)[0][np.argsort(w[sel, 2])]])
    cols = c[pick].astype(float)
    cols[:len(np.where(floor)[0])] = 255 - (255 - cols[:len(np.where(floor)[0])]) * 0.28    # floor faint
    img = raster_png(w[pick], cols.astype(np.uint8), -H, H, -H, H, 0.01, dot=2)
    out = frame('scan')
    out.append(f'<image href="data:image/png;base64,{img}" x="0" y="0" width="{2*H*S}" height="{2*H*S}" preserveAspectRatio="none"/>')
    n = messy_course.NET
    out += [text((-1.6, n + 0.12), 'Net, north (next course 3 ft beyond)', 'net-l'), text((0, -n - 0.19), 'Net, south side', 'net-l'),
            text((n + 0.14, 0), 'Net, east', 'net-l', 90), text((-n - 0.12, 0), 'Net, west (glass windows 8 ft out)', 'net-l', -90)]
    for name, (a, b) in SCAN_WALLS.items():
        mid = (np.asarray(a) + np.asarray(b)) / 2
        dx, dy, rot = SCAN_LABEL[name]
        out.append(text(mid + (dx, dy), name if name != 'C2a' else 'C2a (window wall)', 'wall-l', rot))
    out.append(text((-1.0, 0.62), 'Barrier', 'wall-l'))
    out.append(text((2.62, 1.1), 'TUNNEL', 'tube-l'))
    # The window, as measured in the scan.
    centre, axis = np.array([-0.455, -2.219]), np.array([0.3103, 0.9506])
    out.append(f'<polyline points="{pts([centre - axis * 0.45, centre + axis * 0.45])}" class="window"/>')
    out.append(text(centre + (0.95, 0.27), 'WINDOW (CP4)', 'win-l'))
    out.append(text(centre + (0.95, 0.15), '0.90 wide, 0.87-1.50 m up', 'small'))
    # The mat moves between visits: its centre anywhere in this box (clear of the window wall), three rocks round it.
    (x0, x1), (y0, y1) = messy_course.PAD_AREA
    out.append(f'<polygon points="{pts([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])}" class="mat"/>')
    out.append(text(((x0 + x1) / 2, y1 + 0.16), 'PAD MAT (CP5): centre somewhere here', 'pad-l'))
    out.append(text(((x0 + x1) / 2, y1 + 0.04), 'with three large rocks round it', 'small'))
    pad = next(g for g in spec['goals'] if g['kind'] == 'pad')
    for k, look in enumerate(pad.get('lookouts', [])):
        lx, ly = xy(look); out.append(f'<circle cx="{lx}" cy="{ly}" r="5" class="mat-cad"/>')
        out.append(text(np.asarray(look) + (0, 0.1), f'pad lookout {k + 1}', 'small'))
    sx, sy = xy((-2.25, -2.30)); out.append(f'<rect x="{sx-0.3*S}" y="{sy-0.3*S}" width="{0.6*S}" height="{0.6*S}" class="mat-cad"/>')
    for key, line in nominal['lines'].items():
        if line.get('checkpoint') in (1, 2, 3) and key != 'gate_b':
            out.append(f'<polyline points="{pts([line["start"], line["end"]])}" class="cp"/>')
            e = np.asarray(line['end']) if key != 'gate_a' else np.asarray(line['start']) + (0.1, 0.15)
            out.append(text(e + {1: (0.32, 0.03), 2: (0.12, 0.0), 3: (0.42, 0.0)}[line['checkpoint']], f'CP{line["checkpoint"]}', 'cp-l'))
    lx = spec['launch_cad']
    x, y = xy(lx); out.append(f'<rect x="{x-9}" y="{y-9}" width="18" height="18" class="launch"/>')
    out.append(f'<path d="M{x} {y-16} l-6 9 h12 z" class="launch-arrow"/>')
    out.append(text(np.asarray(lx) + (0, -0.25), 'LAUNCH', 'launch-l')); out.append(text(np.asarray(lx) + (0, -0.37), 'facing north', 'small'))
    goals = spec['goals']
    chain = [lx] + [gl['at'] for gl in goals]
    out.append(f'<polyline points="{pts(chain)}" class="chain"/>')
    for k, goal in enumerate(goals, 1):
        x, y = xy(goal['at'])
        out.append(f'<circle cx="{x}" cy="{y}" r="8" class="goal goal-{goal["kind"]}"/>')
        out.append(f'<text x="{x}" y="{y+3.5}" class="goal-n" text-anchor="middle">{k}</text>')
    return out, goals

def window_panel(root):
    d = np.load(root / 'assets/course-scan.npz')
    w, c = d['xyz_mm'] / 1000., d['rgb']
    a, axis = np.array([-0.743, -3.075]), np.array([0.3103, 0.9506])
    nrm = np.array([-axis[1], axis[0]])
    rel = w[:, :2] - a; u, v = rel @ axis, rel @ nrm
    sel = (np.abs(v) < 0.15) & (u > -0.2) & (u < 2.0)
    pts2 = np.column_stack([u[sel], w[sel, 2]])
    order = np.argsort(-np.abs(v[sel]))
    img = raster_png(pts2[order], c[sel][order], -0.2, 2.0, 0.0, 2.0, 0.01, dot=2)
    s2 = 250
    W2, H2 = 2.2 * s2, 2.0 * s2
    X = lambda uu: (uu + 0.2) * s2; Y = lambda z: (2.0 - z) * s2
    o = [f'<style>.wp text{{font-size:15px}} .wp .tick{{font-size:13px}}</style><g class="wp">', f'<rect width="{W2+190}" height="{H2+34}" class="bg"/>', f'<image href="data:image/png;base64,{img}" x="0" y="0" width="{W2}" height="{H2}" preserveAspectRatio="none"/>']
    for z in (0.5, 1.0, 1.5):
        o.append(f'<line x1="0" y1="{Y(z)}" x2="{W2}" y2="{Y(z)}" class="grid"/>'); o.append(f'<text x="4" y="{Y(z)-3}" class="tick">{z:.1f} m</text>')
    o.append(f'<rect x="{X(0.33)}" y="{Y(1.50)}" width="{0.895*s2}" height="{0.63*s2}" class="win-box"/>')
    for z, label in ((0.87, 'sill 0.87 m'), (1.50, 'top 1.50 m'), (1.80, 'wall top ~1.8 m'), (0.27, 'panel bottom ~0.27 m')):
        o.append(f'<line x1="{X(1.6)}" y1="{Y(z)}" x2="{W2+10}" y2="{Y(z)}" class="dim"/><text x="{W2+14}" y="{Y(z)+4}" class="small" text-anchor="start">{label}</text>')
    o.append(f'<text x="{X(0.78)}" y="{Y(1.2)}" class="win-l">0.90 m</text>')
    o.append(f'<text x="{X(0.78)}" y="{H2+24}" class="small" text-anchor="middle">along the wall from its south end (m): wall 1.78 m, window centred (0.33-1.23)</text>')
    o.append(f'<text x="4" y="22" class="small2" text-anchor="start">seen from the tunnel side</text>')
    o.append('</g>')
    return o, W2 + 190, H2 + 34

def run_panel(root, run_dir):
    meta = json.loads((run_dir / 'course/course.json').read_text())
    mesh = read_obj(run_dir / 'course/course.obj'); V = np.asarray(mesh.vertices)
    out = frame('run')
    nets = meta['nets']
    box = [(-nets['west'], -nets['south']), (nets['east'], -nets['south']), (nets['east'], nets['north']), (-nets['west'], nets['north']), (-nets['west'], -nets['south'])]
    out.append(f'<polyline points="{pts(box)}" class="net"/>')
    for name, idx in mesh.groups.items():
        p = V[np.unique(np.asarray(idx))]
        if name.startswith(('Paver', 'Frame')):
            tri = np.asarray(idx).reshape(-1, 3)
            low = tri[(V[tri][:, :, 2] < 0.35).all(axis=1)]
            out += [f'<polygon points="{pts(V[t][:, :2])}" class="stand"/>' for t in low]
        elif name.startswith('Tube oval'):
            out.append(f'<polygon points="{pts(hull(p[:, :2]))}" class="tube"/>')
        elif name.startswith('Boulder'):
            out.append(f'<polygon points="{pts(hull(p[:, :2]))}" class="boulder"/>')
    for w in meta['walls']:
        for a, b in w['pieces']:
            out.append(f'<polyline points="{pts([a, b])}" class="wall-line"/>')
        a, b = np.asarray(w['pieces'][0][0]), np.asarray(w['pieces'][-1][1])
        name = w['name']
        if not name.startswith('Barrier'):
            d = b - a; d = d / np.linalg.norm(d); nrm = np.array([-d[1], d[0]])
            mid = (a + b) / 2 + nrm * 0.17
            rot = -math.degrees(math.atan2(d[1], d[0]))
            rot = rot + 180 if rot > 90 else rot - 180 if rot < -90 else rot
            out.append(text(mid, name, 'wall-l', rot))
    p = meta['pad']; x, y = xy((p['x'], p['y'])); out.append(f'<circle cx="{x}" cy="{y}" r="{p["radius_m"]*S}" class="pad"/>')
    out.append(f'<polyline points="{pts(meta["guideline"]["points"])}" class="tape"/>')
    for key, line in meta['lines'].items():
        if line.get('checkpoint') and key != 'gate_b':
            out.append(f'<polyline points="{pts([line["start"], line["end"]])}" class="cp"/>')
            out.append(text(np.asarray(line['end']) + (0.05, 0.08), f'CP{line["checkpoint"]}', 'cp-l'))
    trace = json.loads((run_dir / 'trace.json').read_text())['frames']
    flown = [f['p'][:2] for f in trace[::5] if f['state'] not in ('IDLE',)]
    est = [f['est'][:2] for f in trace[::5] if 'est' in f and f['state'] not in ('IDLE',)]
    seg, cur, last = [], [], None
    states = [f['state'] for f in trace[::5] if f['state'] != 'IDLE']
    ret = [i for i, s in enumerate(states) if s.startswith('RETURN')]
    split = ret[0] if ret else len(flown)
    out.append(f'<polyline points="{pts(flown[:split])}" class="flown"/>')
    if ret: out.append(f'<polyline points="{pts(flown[split:])}" class="flown-ret"/>')
    rep = json.loads((run_dir / 'report.json').read_text()) if (run_dir / 'report.json').exists() else {}
    return out, meta, rep, trace

def side_view(cfg):
    W, Hh, s = 900, 400, 900           # px per metre
    gy, cx = 260, 330                  # the feet's level, the centre
    def Y(z): return gy - z * s
    def X(u): return cx + u * s
    o = [f'<rect width="{W}" height="{Hh}" class="bg"/>']
    o.append(f'<rect x="{X(-0.33)}" y="{Y(0.17)}" width="{0.66*s}" height="{0.27*s}" class="band"/>')
    o.append(f'<line x1="{X(-0.36)}" y1="{Y(0)}" x2="{X(0.36)}" y2="{Y(0)}" class="dim"/>')
    parts = [(-0.071, 0.071, 0.0, 0.037, 'core'), (-0.178, 0.178, 0.038, 0.042, 'arm'),
             (-0.176, -0.143, 0.039, 0.074, 'motor'), (0.143, 0.176, 0.039, 0.074, 'motor'),
             (-0.0235, 0.0235, 0.047, 0.086, 'px4'), (-0.023, 0.023, 0.080, 0.116, 'batt')]
    for u0, u1, z0, z1, cls in parts:
        o.append(f'<rect x="{X(u0)}" y="{Y(z1)}" width="{(u1-u0)*s}" height="{(z1-z0)*s}" class="{cls}"/>')
    for u in (-0.160, 0.160):
        o.append(f'<rect x="{X(u-0.089)}" y="{Y(0.092)}" width="{0.178*s}" height="4" class="prop"/>')
    for z, label in ((0.0, 'feet'), (0.065, 'Pixhawk IMU ~65 mm = PX4 origin (code: 70)'), (0.092, 'rotor tops 92 mm (your 3 5/8 in)'),
                     (0.116, 'battery top 116 mm (CAD) = top the code uses'), (-0.10, 'band bottom: 10 cm under the feet (boulders)'), (0.17, 'band top: 5 cm over the battery')):
        o.append(f'<line x1="{X(0.19)}" y1="{Y(z)}" x2="{X(0.35)}" y2="{Y(z)}" class="dim"/>')
        o.append(f'<text x="{X(0.355)}" y="{Y(z)+4}" class="small" text-anchor="start">{label}</text>')
    o.append(f'<text x="{X(-0.30)}" y="{Y(0.0)-4}" class="small2" text-anchor="start">bottom plate</text>')
    o.append(f'<text x="{X(0.0)}" y="{Y(0.0)+18}" class="small" text-anchor="middle">rotor tips 0.29 m from the Pixhawk each side (7 in props): 0.58 m span</text>')
    o.append(f'<text x="{X(0.0)}" y="{Y(-0.10)+34}" class="small2" text-anchor="middle">blue band: the heights the planner keeps clear around the body (old code: 0.16 m above and below the origin)</text>')
    legend = [('core', 'plates, Jetson'), ('arm', 'arms'), ('motor', 'motors'), ('px4', 'Pixhawk'), ('batt', 'battery'), ('prop', 'props')]
    for k, (cls, name) in enumerate(legend):
        o.append(f'<rect x="{20 + k*95}" y="14" width="12" height="12" class="{cls}"/><text x="{36 + k*95}" y="24" class="small" text-anchor="start">{name}</text>')
    return o, W, Hh

INLINE = '''body{margin:0;background:none;color:var(--foreground);font-family:var(--font-sans)}
.row{flex-direction:column} .card{background:var(--card);color:var(--card-foreground);border-color:var(--border);border-radius:var(--radius);padding:16px}
p.note{color:var(--muted-foreground)} td{border-color:var(--border)} h1{display:none} .card svg{border-radius:6px}'''

def main(root, run_dir, out_path, inline=''):
    root, run_dir = Path(root), Path(run_dir)
    spec = yaml.safe_load((root / 'config/goals.yaml').read_text())
    nominal_c = messy_course.generate(1, 0.3)      # near nominal (at 0 the rocks cannot move off the mat)
    nominal = {'lines': nominal_c.lines, 'guideline': nominal_c.guideline}
    cad, goals = scan_panel(root, spec, nominal)
    win, ww, wh = window_panel(root)
    run, meta, rep, trace = run_panel(root, run_dir)
    side, sw, sh = side_view(None)
    size = 2 * H * S
    kinds = {'waypoint': 'waypoint', 'reach': 'reach (dead end, as far as safe)', 'gap': 'gap (round a wall end)',
             'tube': 'tube (tunnel)', 'opening': 'opening (window)', 'pad': 'pad (land)'}
    rows = ''.join(f'<tr><td><span class="dot goal-{g["kind"]}">{k}</span></td><td><code>{g["id"]}</code></td><td>{g["kind"]}</td>'
                   f'<td>{g.get("checkpoint","")}</td><td>{g.get("height","cruise")}</td></tr>' for k, g in enumerate(goals, 1))
    t_end = trace[-1]['t']
    html = f'''<!doctype html><html><head><meta charset="utf-8"><title>Goalnav course map</title><style>
body{{font:13px/1.4 system-ui,sans-serif;margin:16px;color:#1d2433;background:#f4f5f7}}
@@INLINE@@
h1{{font-size:18px;margin:0 0 4px}} h2{{font-size:14px;margin:6px 0}} .row{{display:flex;gap:16px;flex-wrap:wrap;align-items:flex-start}}
.card{{background:#fff;border:1px solid #d9dce3;border-radius:8px;padding:10px}} p.note{{color:#555;max-width:640px;margin:4px 0}}
svg text{{font-family:system-ui,sans-serif}} .bg{{fill:#fbfbfa}} .grid{{stroke:#e8e8e4;stroke-width:1}} .tick{{font-size:9px;fill:#999}}
.net{{fill:none;stroke:#7aa2c8;stroke-width:2;stroke-dasharray:6 4}} .net-l{{font-size:10px;fill:#5a82a8;text-anchor:middle}}
.wall{{fill:#4b5263;stroke:#2c313c;stroke-width:1}} .wall-l{{font-size:12px;font-weight:700;fill:#1d2433;paint-order:stroke;stroke:#fff;stroke-width:3px;text-anchor:middle}}
.wall-line{{stroke:#4b5263;stroke-width:{0.10*S};stroke-linecap:butt;fill:none}} .stand{{fill:#c9ccd3;stroke:none}}
.tube{{fill:#e9dcc4;stroke:#a8865a;stroke-width:1.5}} .tube-l{{font-size:12px;font-weight:700;fill:#7a5a30;text-anchor:middle}}
.boulder{{fill:#b9a99a;stroke:#857463}} .small{{font-size:10px;fill:#444}} .small2{{font-size:10px;fill:#777}}
.pad{{fill:#dff0e0;stroke:#4a9a55;stroke-width:1.5}} .pad-l{{font-size:11px;font-weight:700;fill:#2e7a3a;text-anchor:middle}}
.tape{{fill:none;stroke:#111;stroke-width:2.5;opacity:.55}} .window{{stroke:#d33;stroke-width:6;fill:none}} .win-l{{font-size:11px;font-weight:700;fill:#c22;text-anchor:middle}}
.cp{{stroke:#e8890c;stroke-width:3;fill:none;stroke-dasharray:2 3}} .cp-l{{font-size:12px;font-weight:800;fill:#c66f00;text-anchor:middle}}
.launch{{fill:#fff;stroke:#7b3fb5;stroke-width:2}} .launch-arrow{{fill:#7b3fb5}} .launch-l{{font-size:11px;font-weight:700;fill:#7b3fb5;text-anchor:middle}}
.chain{{fill:none;stroke:#2f6fd6;stroke-width:1.5;stroke-dasharray:5 3;opacity:.8}}
.goal{{stroke:#fff;stroke-width:1.5}} .goal-n{{font-size:9px;font-weight:700;fill:#fff}}
.goal-waypoint{{fill:#2f6fd6;background:#2f6fd6}} .goal-reach{{fill:#7b3fb5;background:#7b3fb5}} .goal-gap{{fill:#0f8f8f;background:#0f8f8f}}
.goal-tube{{fill:#a8652a;background:#a8652a}} .goal-opening{{fill:#d33;background:#d33}} .goal-pad{{fill:#2e7a3a;background:#2e7a3a}}
.dot{{display:inline-block;width:18px;height:18px;border-radius:9px;color:#fff;font-size:10px;font-weight:700;text-align:center;line-height:18px}}
.flown{{fill:none;stroke:#2f6fd6;stroke-width:2}} .flown-ret{{fill:none;stroke:#7b3fb5;stroke-width:1.5;stroke-dasharray:4 3}}
table{{border-collapse:collapse;font-size:12px}} td{{padding:2px 6px;border-bottom:1px solid #eee}} code{{font-size:11px}}
.floor{{stroke:#333;stroke-width:2}} .core{{fill:#8a8f99}} .arm{{fill:#2c313c}} .motor{{fill:#555}} .px4{{fill:#d33}} .batt{{fill:#e8890c}}
.prop{{fill:#2f6fd6;opacity:.6}} .mat{{fill:#5aa0d0;fill-opacity:.35;stroke:#2e7a3a;stroke-width:2}} .mat-cad{{fill:none;stroke:#2e7a3a;stroke-width:1.5;stroke-dasharray:4 3}}
.win-box{{fill:none;stroke:#d33;stroke-width:2;stroke-dasharray:5 3}} .dim{{stroke:#999;stroke-width:1;stroke-dasharray:3 2}} .band{{fill:#2f6fd6;opacity:.07}}
</style></head><body>
<h1>Goal navigation: course map with the names the code uses</h1>
<p class="note">Metres: x east, y north, origin at the centre of the nets' square (the scan was fitted to the nets and the barrier).
Walls are named as in the organizers' CAD (M = maze divider, C = chicane, A = approach). The numbered dots are the goal chain in
<code>config/goals.yaml</code>; in flight each goal is looked for in the live map near where the previous one was actually found.</p>
<div class="row">
<div class="card"><h2>The real course: phone scan of 2026-11-06, wall slice 0.3-1.7 m, with the code's names</h2>
<svg width="100%" style="max-width:{size}px;display:block" viewBox="0 0 {size} {size}">{''.join(cad)}</svg></div>
<div class="card"><h2>Goal chain</h2><table><tr><td></td><td>id</td><td>kind</td><td>CP</td><td>height (m)</td></tr>{rows}</table>
<p class="note" style="max-width:330px">Kinds: <b>reach</b> = go as far as is safe (the first corridor's dead end, CP1 seen on the floor tape);
<b>gap</b> = round a wall's end; <b>tube</b> = tunnel (flown lower, under the tarp); <b>opening</b> = window (measured from depth, then the
line-up manoeuvre); <b>pad</b> = found by camera, land. Cruise height is 1.15 m in the safe profile.</p></div>
</div>
<div class="row" style="margin-top:16px">
<div class="card"><h2>One simulated run: seed {meta["seed"]}, 70% severity, safe profile</h2>
<p class="note" style="max-width:{size}px">The as-built course the simulator made for this seed (walls moved and turned, stands' feet in light grey,
pad and window moved, nets a few cm off). Blue: the path flown, from launch to the pad ({rep.get("checkpoints", "")}); dashed purple: the
return to launch. Orange dotted: checkpoint lines. Run length {t_end:.0f} s.</p>
<svg width="100%" style="max-width:{size}px;display:block" viewBox="0 0 {size} {size}">{''.join(run)}</svg></div>
<div class="card"><h2>Window wall C2a, from the scan</h2>
<svg width="100%" style="max-width:{ww}px;display:block" viewBox="0 0 {ww} {wh}">{''.join(win)}</svg></div>
<div class="card"><h2>Sky Beta side view (from the CAD and your measurement)</h2>
<svg width="100%" style="max-width:{sw}px;display:block" viewBox="0 0 {sw} {sh}">{''.join(side)}</svg>
<p class="note" style="max-width:{sw}px">Part heights are from <code>sky-beta.step</code> (mm over the bottom plate). Props aren't in the CAD; the
92 mm rotor-top height is yours. The PX4 origin is the Pixhawk's IMU, about 65 mm up; the code assumes 70 mm, so the drone's top is believed
about 5 mm higher than it is (a safe error).</p></div>
</div></body></html>'''
    html = html.replace('@@INLINE@@', INLINE if inline else '')
    Path(out_path).write_text(html)
    print(out_path, len(html))

if __name__ == '__main__':
    main(*sys.argv[1:5])
