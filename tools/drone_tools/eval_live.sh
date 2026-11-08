#!/usr/bin/env bash
# Live end-to-end eval of cpu_watch.py, to run ON THE DRONE HOST (not in the container):
#     bash eval_live.sh [path/to/cpu_watch.py]
# It starts a known load, a busy loop pinned to core 5 that stops itself after 14 s, and checks that
# cpu_watch names that core as saturated, finds the process at ~1 core, flags it as not part of the
# stack, and calls it single-thread-bound. It changes nothing else and kills nothing.
# Exit status 0 = all checks passed.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
tool="${1:-$here/cpu_watch.py}"
CORE=5
echo "cores: $(nproc); burner pinned to core $CORE for 14 s, sampling 8 s"
timeout 14 taskset -c $CORE python3 -c 'while True: pass' &
burner=$!
sleep 2
out="$(python3 "$tool" --once 8 --json)"
wait $burner 2>/dev/null
printf '%s' "$out" | python3 -c '
import json, sys
r = json.load(sys.stdin)
core = int(sys.argv[1])
fails = []
def check(ok, what):
    print(("PASS " if ok else "FAIL ") + what)
    if not ok:
        fails.append(what)
check(r["core_pct"][core] >= 85, f"core {core} reads >= 85% busy (got {r['"'"'core_pct'"'"'][core]})")
others = [c for i, c in enumerate(r["core_pct"]) if i != core]
check(max(r["core_peak"]) >= 90, "a core peaked >= 90%")
burn = [p for p in r["procs"] if "while" in p["label"] or p["label"].startswith("python") and p["cores"] >= 0.8 and not p["container"]]
check(bool(burn), "the burner is listed among the top processes")
if burn:
    b = max(burn, key=lambda p: p["cores"])
    check(0.85 <= b["cores"] <= 1.15, f"burner uses ~1 core (got {b['"'"'cores'"'"']})")
    check(not b["stack"], "burner is marked as not part of the stack")
v = " | ".join(r["verdict"])
check(f"CORE {core} SATURATED" in v, "verdict says the core is saturated")
check("SINGLE-THREAD BOUND" in v, "verdict says single-thread bound")
check("STRAY" in v, "verdict flags the stray process")
print(); print("verdict lines:"); [print("  -", x) for x in r["verdict"]]
sys.exit(1 if fails else 0)
' "$CORE"
