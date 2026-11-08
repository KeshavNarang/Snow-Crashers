# Multi-agent protocol

## Drone lock (one agent on the drone at a time)
The drone is shared. Before running anything beyond a read-only look:
1. `git pull`, then check `tools/drone-lock status` and `agents/STATUS.md`.
2. Take the lock: `tools/drone-lock acquire "<agent/topic>" "<what you will do>"`.
3. Release when done: `tools/drone-lock release`. A lock older than 2 h is stale; ask the user.
The lock is also written on the drone (`/tmp/agent.lock`) so agents on other machines see it.

## Logs and handoff
- Append each drone session to `agents/logs/YYYY-MM-DD-<topic>.md`: what you ran, what you saw,
  what failed. Keep failed evidence.
- Update your row in `agents/STATUS.md` (owner, branch, state: todo/active/blocked/done, next step).
- Save useful artifacts (ros topic dumps, params, bag summaries) under `agents/logs/`, small files only.

## Rules of the road
- Never commit `.secrets/`. Never write the password into any tracked file.
- Don't pass the project's code review hazards: no hard-coded course waypoints in new code.
