"""Build a portable, synchronized overview from an existing cheese run."""
import base64
import json
import io

from PIL import Image
from pathlib import Path
import sys


def build(directory):
    directory = Path(directory)
    trace = json.loads((directory / 'trace.json').read_text())
    report = json.loads((directory / 'report.json').read_text())
    meta = json.loads((directory / 'course/course.json').read_text())
    cameras = []
    for entry in trace['cams']['frames'][::3]:
        images = {}
        for kind in ('rgb', 'depth'):
            path = directory / 'cam' / f"{entry['k']:04d}_{kind}.png"
            if path.exists():
                image = Image.open(path)
                image.thumbnail((128, 80))
                buffer = io.BytesIO()
                image.save(buffer, format='JPEG', quality=55)
                images[kind] = 'data:image/jpeg;base64,' + base64.b64encode(buffer.getvalue()).decode()
        cameras.append({**entry, **images})
    data = {'frames': trace['frames'], 'cameras': cameras, 'meta': meta, 'report': report}
    html = Path(__file__).with_name('visual.html').read_text()
    html = html.replace('/*DATA*/', json.dumps(data, separators=(',', ':')).replace('</', '<\\/'))
    target = directory / 'cheese.html'
    target.write_text(html)
    print(target)


if __name__ == '__main__':
    build(sys.argv[1])
