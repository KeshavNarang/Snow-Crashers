# Multi-agent protocol

Current operational status and competition guidance: [docs/STATUS.md](../docs/STATUS.md).

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
- The earlier ban on fixed course waypoints was a competition assumption. The user reports flexible organizer rules; fixed-waypoint shortcuts are within the current requested scope. Keep deployment evidence and flight safeguards separate from scoring assumptions.

- Real RGB is required for pad finding and may be enabled under the current competition guidance. Never fabricate RGB-sensor values or substitute simulated/replayed/colourized mono data as live RGB. See [camera requirements](../docs/CAMERAS.md#required-for-the-real-pad-mission).
