"""Top-down SVG of the inflated map, planned path, flown path and checkpoints."""
from collections import defaultdict

SCALE = 100      # pixels per metre
HALF = 3.1       # metres shown either side of the origin


def _xy(x, y):
    return (x + HALF) * SCALE, (HALF - y) * SCALE


def _speed_colour(speed, top=1.5):
    f = max(0., min(1., speed / top))
    return f'rgb({int(40 + 200 * f)},{int(90 + 60 * (1 - abs(f - .5) * 2))},{int(220 - 200 * f)})'


def write_svg(path, course, planned, trace, referee, start_offset=0.):
    grid = course['grid']
    res, (ox, oy) = grid['resolution'], grid['origin']
    rows = defaultdict(list)
    for cx, cy in grid['occupied']:
        rows[cy].append(cx)
    parts = []
    for cy, xs in rows.items():
        xs.sort()
        start = prev = xs[0]
        for cx in xs[1:] + [None]:
            if cx is not None and cx == prev + 1:
                prev = cx
                continue
            x0, y0 = _xy(ox + start * res, oy + (cy + 1) * res)
            parts.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{(prev - start + 1) * res * SCALE:.1f}" '
                         f'height="{res * SCALE:.1f}" fill="#d9d9d9"/>')
            if cx is not None:
                start = prev = cx
    for line in referee.all_lines():
        (x1, y1), (x2, y2) = _xy(*line.start_xy), _xy(*line.end_xy)
        parts.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#8C1515" '
                     f'stroke-width="3" stroke-dasharray="8 5"/>')
    pad = course['landing']
    px, py = _xy(pad['x'], pad['y'])
    parts.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="{pad["radius_m"] * SCALE:.1f}" fill="none" '
                 f'stroke="#2a7" stroke-width="2"/>')
    if planned:
        points = ' '.join('%.1f,%.1f' % _xy(x, y) for x, y in planned)
        parts.append(f'<polyline points="{points}" fill="none" stroke="#333" stroke-width="1.5" '
                     f'stroke-dasharray="4 3"/>')
    for a, b in zip(trace, trace[1:]):
        (x1, y1), (x2, y2) = _xy(a[1], a[2]), _xy(b[1], b[2])
        parts.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
                     f'stroke="{_speed_colour(b[4])}" stroke-width="4" stroke-linecap="round"/>')
    for _, _, absolute in referee.touches:
        sample = min(trace, key=lambda row: abs(row[0] + start_offset - absolute)) if trace else None
        if sample:
            x, y = _xy(sample[1], sample[2])
            parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="7" fill="none" stroke="red" stroke-width="2"/>')
    size = 2 * HALF * SCALE
    legend = ('<text x="8" y="18" font-family="sans-serif" font-size="13">grey: inflated map · dashed: plan · '
              'colour: flown speed (blue slow, red 1.5 m/s) · red ring: wall touch</text>')
    Path = type(path)
    Path(path).write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="{size:.0f}" height="{size:.0f}" '
                          f'viewBox="0 0 {size:.0f} {size:.0f}"><rect width="100%" height="100%" fill="white"/>'
                          + ''.join(parts) + legend + '</svg>')
