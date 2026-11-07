"""A page that always shows the newest run's 3D replay.

    python3 -m http.server 8765 --bind 127.0.0.1 --directory localsim/out/goalnav
    # then open http://127.0.0.1:8765/

Each finished ``localsim.goalrun`` calls ``update()``, which rewrites
``runs.json`` (every run with a viewer, newest first, and its outcome) and
``index.html``. The page polls ``runs.json`` and lists new runs as they land
(marked new) without leaving the one being watched; click one to open it. The
3D view keeps its camera from run to run.
"""
import json
from pathlib import Path

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>goalnav runs</title>
<style>
 body{margin:0;display:flex;height:100vh;background:#0d1117;color:#e6edf3;font:13px system-ui,sans-serif}
 #list{width:260px;overflow:auto;border-right:1px solid #30363d}
 #list div{padding:6px 10px;border-bottom:1px solid #21262d;cursor:pointer}
 #list div.on{background:#1f6feb33} .ok{color:#3fb950} .bad{color:#f85149} .dim{color:#8b949e}
 #head{padding:8px 10px;border-bottom:1px solid #30363d}
 iframe{flex:1;border:0;height:100vh}
</style></head><body>
<div id="list"><div id="head">Click a run to view it</div></div>
<iframe id="view"></iframe>
<script>
let shown = null, stamp = null, seen = null;
function show(run) {
  shown = run.dir; document.getElementById('view').src = run.dir + '/viewer.html#t=1';
  document.querySelectorAll('#list .run').forEach(e => e.classList.toggle('on', e.dataset.dir === run.dir));
}
async function poll() {
  try {
    const runs = await (await fetch('runs.json?' + Date.now())).json();
    const key = runs.map(r => r.dir + r.mtime).join();
    if (key !== stamp) {
      stamp = key; window.newest = runs[0];
      const list = document.getElementById('list');
      list.querySelectorAll('.run').forEach(e => e.remove());
      for (const r of runs) {
        const d = document.createElement('div'); d.className = 'run'; d.dataset.dir = r.dir;
        const fresh = seen && !seen.has(r.dir) ? ' <span class="ok">new</span>' : '';
        d.innerHTML = `<b>${r.dir}</b>${fresh}<br><span class="${r.finished ? 'ok' : 'bad'}">${r.outcome}</span> <span class="dim">${r.when}</span>`;
        d.onclick = () => show(r); list.appendChild(d);
      }
      if (seen === null) seen = new Set(runs.map(r => r.dir));
      if (runs.length && !shown) show(runs[0]);
      else document.querySelectorAll('#list .run').forEach(e => e.classList.toggle('on', e.dataset.dir === shown));
    }
  } catch (e) {}
  setTimeout(poll, 4000);
}
poll();
</script></body></html>
"""


def update(root):
    """Rewrite ``runs.json`` and ``index.html`` under ``root``."""
    import time
    root = Path(root)
    runs = []
    for viewer in root.glob('*/viewer.html'):
        report = viewer.parent / 'report.json'
        result = json.loads(report.read_text()) if report.exists() else {}
        finished = bool(result.get('finish_s'))
        course = result.get('course', {})
        outcome = (f"CP{result.get('checkpoints', 0)} via {result.get('route')}"
                   + (f", pad {result['finish_s']:.0f} s" if finished else f", {(result.get('crashed') or result.get('final') or ['?'])[-1]}")
                   + f" · sev {course.get('severity')} · {(result.get('profile') or {}).get('name', '')}")
        mtime = viewer.stat().st_mtime
        runs.append({'dir': viewer.parent.name, 'mtime': mtime, 'finished': finished, 'outcome': outcome,
                     'when': time.strftime('%H:%M:%S', time.localtime(mtime))})
    runs.sort(key=lambda r: r['mtime'], reverse=True)
    (root / 'runs.json').write_text(json.dumps(runs[:200]))
    (root / 'index.html').write_text(PAGE)


if __name__ == '__main__':
    import sys
    update(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / 'localsim/out/goalnav')
