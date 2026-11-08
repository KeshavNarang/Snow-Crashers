#!/usr/bin/env python3
"""Live A/B on the drone host: what each matcher costs the running stack (read-only otherwise).

For each method (and 'none' = idle baseline) it loops stereo_bench.py --soak inside the skybeta
container for S seconds while sampling, once a second:
  tegrastats            total CPU % (6 cores), GPU (GR3D_FREQ) %, board power (VDD_IN)
  ros_summary.json      cuVSLAM odometry Hz, its tracking time, left camera Hz (status sidecar)

    python3 /tmp/gpubench/live_ab.py 25 sgbm vpi-cuda vpi-cuda-full vpi-ofa
"""
import json
import re
import subprocess
import sys
import threading
import time

SUMMARY = '/run/skybeta-status/ros_summary.json'


def sample(stop, rows):
    teg = subprocess.Popen(['tegrastats', '--interval', '1000'], stdout=subprocess.PIPE, text=True)
    try:
        while not stop.is_set():
            line = teg.stdout.readline()
            if not line:
                break
            cpu = [int(c) for c in re.findall(r'(\d+)%@\d+', line.split('CPU [')[1].split(']')[0])] \
                if 'CPU [' in line else []
            gpu = re.search(r'GR3D_FREQ (\d+)%', line)
            watt = re.search(r'VDD_IN (\d+)mW', line)
            try:
                vio = json.load(open(SUMMARY))['vio']
            except Exception:  # noqa: BLE001
                vio = {}
            rows.append({'cpu': sum(cpu) / len(cpu) if cpu else None, 'gpu': int(gpu.group(1)) if gpu else None,
                         'watt': int(watt.group(1)) / 1000.0 if watt else None,
                         'odom_hz': vio.get('odom_rate_hz'), 'track_ms': vio.get('track_execution_time_mean_ms'),
                         'cam_hz': vio.get('cam_left_rate_hz')})
    finally:
        teg.terminate()


ENV = 'source skybeta_env.sh >/dev/null 2>&1; export PYTHONPATH=/tmp/gpubench/overwall_nav_pkg:/tmp/gpubench:$PYTHONPATH; '
# node-<name>: a second stereo_depth (uncapped, rate_hz 30) on a test topic, next to the live one.
NODE_ARGS = {'sgbm': 'matcher:=sgbm', 'vpi-cuda': 'matcher:=vpi-cuda',
             'vpi-ofa': 'matcher:=vpi-ofa -p num_disparities:=128',
             'vpi-cuda-full': 'matcher:=vpi-cuda -p scale:=1.0 -p num_disparities:=192 -p step:=4'}


def live_node(name, seconds, rows, warmup=10.0):
    params = ' -p '.join(['rate_hz:=30.0', 'output:=/overwall/front_test', 'log_period_s:=1000.0', NODE_ARGS[name]])
    node = subprocess.Popen(
        ['docker', 'exec', 'skybeta-stack', 'bash', '-c',
         ENV + 'exec python3 -c "from overwall_nav.stereo_depth_node import main; main()" '
         f'--ros-args -r __node:=stereo_depth_test -p {params}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(warmup)
        rows.clear()                           # sample the stack only once the test node is warm
        out = subprocess.run(['docker', 'exec', 'skybeta-stack', 'bash', '-c',
                              ENV + f'python3 /tmp/gpubench/topic_stats.py /overwall/front_test {seconds}'],
                             capture_output=True, text=True)
        lines = [ln for ln in out.stdout.splitlines() if ln.startswith('{')]
        return json.loads(lines[-1]) if lines else {'error': (out.stdout + out.stderr)[-300:]}
    finally:
        subprocess.run(['docker', 'exec', 'skybeta-stack', 'pkill', '-f', '__node:=stereo_depth_test'])
        node.wait(timeout=10)


def mean(rows, key):
    xs = [r[key] for r in rows if r.get(key) is not None]
    return round(sum(xs) / len(xs), 1) if xs else None


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    methods = ['none'] + (sys.argv[2:] or ['sgbm', 'vpi-cuda'])
    results = []
    for m in methods:
        rows, stop = [], threading.Event()
        th = threading.Thread(target=sample, args=(stop, rows), daemon=True)
        th.start()
        soak = None
        if m == 'none':
            time.sleep(seconds)
        elif m.startswith('node-'):
            soak = live_node(m[5:], seconds, rows)
        else:
            out = subprocess.run(
                ['docker', 'exec', 'skybeta-stack', 'bash', '-c',
                 f'cd /tmp/gpubench && PYTHONPATH=/tmp/gpubench python3 stereo_bench.py --pairs pairs.npz '
                 f'--soak {seconds} --methods {m}'], capture_output=True, text=True)
            lines = [ln for ln in out.stdout.splitlines() if ln.startswith('{')]
            soak = json.loads(lines[-1]) if lines else {'error': (out.stdout + out.stderr)[-300:]}
        stop.set()
        th.join(timeout=3)
        rows = rows[2:] if not m.startswith('node-') else rows   # skip start-up seconds
        res = {'method': m, 'soak': soak, 'cpu_total_pct': mean(rows, 'cpu'), 'gpu_pct': mean(rows, 'gpu'), 'board_w': mean(rows, 'watt'),
               'odom_hz': mean(rows, 'odom_hz'), 'odom_hz_min': min((r['odom_hz'] for r in rows if r['odom_hz']), default=None),
               'track_ms': mean(rows, 'track_ms'), 'cam_hz': mean(rows, 'cam_hz'), 'samples': len(rows)}
        results.append(res)
        print(json.dumps(res), flush=True)
        time.sleep(3)
    json.dump(results, open('/tmp/gpubench/live_ab.json', 'w'), indent=1)


if __name__ == '__main__':
    main()
