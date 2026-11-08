#!/usr/bin/env python3
"""mutation_check: prove the evals can fail. Applies one deliberate bug at a time to a copy of the code and
reports whether the test suite catches it ("FAILED" is the good outcome). Used when writing the evals.

    python3 mutation_check.py MUTATIONS.json

MUTATIONS.json: [{"file": "driver_params.py", "label": "...", "old": "...", "new": "..."}, ...]
Every module and test file in this folder is copied to a temporary directory; this folder is never modified.
"""
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


def run(mutations):
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for m in mutations:
            for f in glob.glob(os.path.join(HERE, '*.py')):
                shutil.copy(f, tmp)
            path = os.path.join(tmp, m['file'])
            src = open(path).read()
            if m['old'] not in src:
                results.append((m['label'], 'TARGET NOT FOUND'))
                continue
            open(path, 'w').write(src.replace(m['old'], m['new'], 1))
            tests = [os.path.basename(t)[:-3] for t in glob.glob(os.path.join(tmp, 'test_*.py'))]
            r = subprocess.run([sys.executable, '-m', 'unittest'] + sorted(tests), cwd=tmp, capture_output=True, text=True)
            last = [line for line in r.stderr.splitlines() if line.startswith(('OK', 'FAILED'))]
            results.append((m['label'], last[-1] if last else 'no result'))
    return results


if __name__ == '__main__':
    for label, outcome in run(json.load(open(sys.argv[1]))):
        print(f'{label:<44} {outcome}')
