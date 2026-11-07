"""Flying through the window: line up, settle, then go straight through.

The window is barely wider than the drone (a few cm a side), so it is not
left to the planner. Once the opening has been measured (its centre between
the jambs, its axis square to the wall, its pass height between sill and
lintel):

1. settle: hold a point ``approach`` in front of the centre, on the axis, at
   the pass height, for ``settle_s``, while the close-range map sharpens the
   measurement (the target moves with it);
2. traverse: slide a target point along the axis through the opening at
   ``speed`` (still measuring while both jambs are in view, so odometry
   drift since the settle does not carry into the opening), the drone held
   to it stiffly across the axis, heading locked
   along it. Short of the wall it backs off whenever the drone is more than
   ``tolerance()`` off the line (a share of the measured slack) or ``height_tol`` off the pass height; once
   the props reach the plane it only slows (``creep``);
3. done ``exit`` past the wall plane.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class WindowConfig:
    approach: float = 0.9        # both jambs well inside the view (the OAK-D's left strip and edges)
    exit: float = 0.55
    settle_s: float = 1.0
    speed: float = 0.15
    lateral_tol: float = 0.04    # until the width is measured; then a share of the slack:
    slack_share: float = 0.4     # ... of the gap a side between rotor tips and jambs
    tol_range: tuple = (0.02, 0.08)
    half_span: float = 0.29      # rotor tips from the body origin (Sky Beta CAD, 7 in props)
    height_tol: float = 0.08     # the window has ~0.3 m of vertical slack
    near: float = 0.40           # within this of the wall plane (either side) the tolerances apply
    gain: float = 2.0            # along the axis
    lateral_gain: float = 6.0    # acceleration feedforward per metre off the line (1/s^2)
    lateral_damping: float = 3.5 # ... per m/s across it (critically damped at ~2.4 rad/s)
    lateral_i: float = 1.0       # integral across it: wall pull toward the nearer jamb is steady
    max_accel: float = 2.0
    stable: float = 0.03         # the last measurements agree to this before going in
    damping: float = 0.8
    limit: float = 0.4
    view_until: float = 0.62     # body this far short of the plane: the left jamb enters the OAK-D's left strip
    commit: float = 0.30         # past this short of the plane (the props reach it) never back off
    creep: float = 0.4           # committed and off the line: this fraction of ``speed``


class WindowManeuver:
    def __init__(self, config=None):
        self.cfg = config or WindowConfig()
        self.goal_id = None

    def active_for(self, goal):
        return self.goal_id == goal.id

    def start(self, now, goal):
        self.goal_id, self.phase, self.since, self.s = goal.id, 'settle', now, -self.cfg.approach
        self.centres = []
        self.sills = []
        self.widths = []
        self.integral = 0.
        self.along = -self.cfg.approach
        self.committed = self.backing = False

    def observe(self, points, goal, z_low, z_high, plane=0.05, reach=0.7, search=0.4):
        """One depth frame's jambs: points on the wall's face, at the pass
        height, either side of the opening. Their gap's midpoint is the window
        centre seen from here and now (no map history, no drift). The face is
        found in the frame too (the densest slab across the axis beside the
        opening): the estimate can sit 0.2 m off it, a fit to a low, short
        window's panel taking in what is seen through the hole."""
        if goal.axis is None or self.phase == 'done' or self.along > -self.cfg.view_until:
            return
        centre, axis = np.asarray(goal.estimate, float), np.asarray(goal.axis, float)
        along_wall = np.array([-axis[1], axis[0]])
        p = np.asarray(points, float)
        rel, z = p[:, :2] - centre, p[:, 2]
        d, u = rel @ axis, rel @ along_wall
        beside = (z > z_low) & (z < z_high) & (np.abs(u) > 0.2) & (np.abs(u) < reach) & (np.abs(d) < search)
        if beside.sum() < 10:
            return
        counts, edges = np.histogram(d[beside], bins=np.arange(-search, search + 1e-9, 0.02))
        counts = np.convolve(counts, np.ones(3), 'same')
        face = float(edges[int(np.argmax(counts))] + 0.01)
        on = np.abs(d - face) < plane
        pick = on & (z > z_low) & (z < z_high) & (np.abs(u) < reach)
        left, right = u[pick][u[pick] < -0.15], u[pick][u[pick] > 0.15]
        if len(left) < 3 or len(right) < 3:
            return
        a, b = np.percentile(left, 90), np.percentile(right, 10)     # the jambs' inner edges
        if 0.5 < b - a < 1.2:
            self.centres.append(centre + axis * face + along_wall * (a + b) / 2)
            self.widths.append(float(b - a))
        sill = on & (np.abs(u) < 0.2) & (z > z_low - 0.5) & (z < z_low)
        if sill.sum() >= 5:
            self.sills.append(float(np.percentile(z[sill], 90)))     # the sill's top edge

    def measured_sill(self):
        return float(np.median(self.sills[-15:])) if len(self.sills) >= 5 else None

    def measured_width(self):
        return float(np.median(self.widths[-15:])) if len(self.widths) >= 5 else None

    def tolerance(self):
        """How far off the line the drone may be going in: a share of the
        measured gap between its rotor tips and the jambs (windows vary)."""
        width = self.measured_width()
        if width is None:
            return self.cfg.lateral_tol
        slack = width / 2 - self.cfg.half_span
        return float(np.clip(self.cfg.slack_share * slack, *self.cfg.tol_range))

    def measured_centre(self):
        if len(self.centres) < 5:
            return None
        return np.median(np.array(self.centres[-15:]), axis=0)

    def step(self, now, dt, position, velocity, height_error, goal):
        """(target XY, velocity feedforward, acceleration feedforward, yaw, phase)."""
        cfg = self.cfg
        centre, axis = np.asarray(goal.estimate, float), np.asarray(goal.axis, float)
        normal = np.array([-axis[1], axis[0]])
        rel = np.asarray(position, float) - centre
        along, lateral = float(rel @ axis), float(rel @ normal)
        self.along = along
        feedforward = np.zeros(2)
        tol = self.tolerance()
        if self.phase == 'settle':
            self.s = -cfg.approach
            recent = np.array(self.centres[-8:]) if len(self.centres) >= 8 else None
            steady = recent is not None and float(np.max(np.linalg.norm(recent - np.median(recent, axis=0), axis=1))) < cfg.stable
            settled = (now - self.since > cfg.settle_s and abs(lateral) < tol / 2 and steady
                       and abs(along - self.s) < 0.05 and abs(height_error) < cfg.height_tol)
            if now - self.since > 6. and abs(lateral) < tol / 2 and abs(height_error) < cfg.height_tol:
                settled = True                             # The measurement will not settle further.
            if settled:
                self.phase = 'traverse'
        if self.phase == 'traverse':
            near = abs(along) < cfg.near
            lined_up = abs(lateral) < tol and abs(height_error) < cfg.height_tol
            # Committed once the front props reach the plane: backing out (or
            # hovering) between the jambs, in their suction, is worse than
            # creeping on through while the lateral loop straightens it.
            self.committed = getattr(self, 'committed', False) or along > -cfg.commit
            speed = cfg.speed
            if not self.committed and near and not lined_up:
                # Off the line short of the wall: back off and line up again.
                self.s = max(self.s - cfg.speed * dt, -cfg.near - 0.1)
                feedforward = -axis * cfg.speed
                self.backing = True
                speed = None
            elif not self.committed and getattr(self, 'backing', False) and abs(lateral) > tol / 2:
                speed = None                               # Hold until well on the line.
            elif self.committed and not lined_up:
                speed = cfg.speed * cfg.creep
            if speed is not None:
                self.backing = False
                self.s = min(self.s + speed * dt, cfg.exit + 0.1)
                feedforward = axis * speed
            if along > cfg.exit:
                self.phase = 'done'
        target = centre + axis * self.s
        error = target - np.asarray(position, float)
        v = np.asarray(velocity, float)
        across, v_across = float(error @ normal), float(v @ normal)
        self.integral = float(np.clip(self.integral + cfg.lateral_i * across * dt, -0.5, 0.5))
        # A full PX4 trajectory setpoint: position, velocity and an acceleration
        # feedforward, so corrections reach thrust without the velocity loop's lag.
        accel = (axis * (cfg.gain * float(error @ axis) - 2.5 * float((v - feedforward) @ axis))
                 + normal * (cfg.lateral_gain * across - cfg.lateral_damping * v_across + self.integral))
        norm = float(np.linalg.norm(accel))
        if norm > cfg.max_accel:
            accel *= cfg.max_accel / norm
        return target, feedforward, accel, math.atan2(axis[1], axis[0]), self.phase
