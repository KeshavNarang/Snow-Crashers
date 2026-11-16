# Documentation refresh — 2026-11-07

Started approximately 20:10 PST. Documentation only; no drone connection,
controller changes, camera changes, motor commands or PX4 parameter changes.

User confirms several successful simple up-and-move flights, flexible organizer
rules, permission to enable RGB, and a non-negotiable requirement for real RGB
to find the pad. User explicitly prohibits faked RGB-sensor values.

Added central docs index, current status, flight procedure, camera contracts and
simulator guide. Package/agent/organizer entry points link there. Removed stale
universal “never flown”, “simulator-only goalnav”, absent colour hardware and
autostart claims; marked the old race/lab simulator references as historical.
Dated source logs and human-owned root notes are preserved. Removed a plaintext
login credential from the setup guide in favor of the local secret file reference;
this does not remove it from git history or claim a credential rotation.

Source baseline: origin/main 02e983a. Also read the goalnav-oak-depth handoff at
71e6096 and sim-walls branch evidence. Branch implementation/deployment claims
are labelled separately from what this main checkout contains. No new live
hardware state is asserted. The local cheese result at fd96af8 is labelled as
a single simulation using the older scan/CPU-stereo model.

Validation: documentation diff/whitespace check and local Markdown path checks.
No behavioral tests added or run because this changes documentation only.
