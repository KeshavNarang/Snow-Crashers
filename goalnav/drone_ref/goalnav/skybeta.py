"""What goalnav sends to skybeta's flight_interface, and reads back from it.

The organizers' stack owns everything between team code and PX4
(skybetav1/docs/competitor_guide.md): teams publish ENU setpoints on
``/skybeta/setpoint/pose`` or ``/skybeta/setpoint/twist`` and read
``/skybeta/state``; nothing else may publish to ``/fmu/in/*``. What reaches
PX4 (skybeta_flight_interface/flight_interface.py), at its own 10 Hz:

- either a position or a velocity setpoint, never both, and no
  acceleration feedforward;
- a pose is floored at 0.3 m above the first odometry sample and persists;
  a twist is capped at 1.5 m/s (its norm) and expires after 1 s, falling back
  to the last pose sent, or to PX4's own position if none was;
- the interface never arms, disarms or lands.

So goalnav flies on twists only: one ENU velocity that carries what PX4's
position loop used to add (the position error and the acceleration
feedforward), and a yaw rate toward the wanted heading. Twists alone also
mean that if goalnav stalls the drone hovers where it is (no stale pose to
fly back to), and that the last metres of a landing can go below the pose
floor. Positions are PX4's local frame in ENU (goalnav's odometry frame),
which flight_interface maps to NED by the plain permutation with its
default ``anchor_setpoints:=false``.
"""
from dataclasses import dataclass
import json
import math

import numpy as np

from drone_ref.goalnav.geometry import wrap, yaw_of

TWIST_TOPIC = '/skybeta/setpoint/twist'
STATE_TOPIC = '/skybeta/state'
MAX_SPEED = 1.5                 # flight_interface max_speed_mps (default): it scales the whole vector


@dataclass
class TwistGains:
    """PX4 1.16 defaults for the loops a twist now has to stand in for."""
    pos_p: float = 0.95         # MPC_XY_P: position error -> velocity
    z_pos_p: float = 1.0        # MPC_Z_P
    vel_p: float = 1.8          # MPC_XY_VEL_P_ACC: an acceleration feedforward a is a velocity a / P
    z_vel_p: float = 4.0        # MPC_Z_VEL_P_ACC
    yaw_p: float = 2.8          # MC_YAW_P: heading error -> yaw rate
    max_yaw_rate: float = 1.4   # rad/s, the follower's turn rate


def twist(setpoint, telemetry, gains=TwistGains()):
    """(ENU velocity, yaw rate) for one setpoint: its velocity, plus the
    position error and the acceleration feedforward as PX4's position and
    velocity loops would have turned them into velocity."""
    v = np.asarray(setpoint.velocity, float).copy()
    if setpoint.position is not None:
        error = np.asarray(setpoint.position, float) - telemetry.position
        v += np.array([gains.pos_p, gains.pos_p, gains.z_pos_p]) * error
    if setpoint.acceleration is not None:
        v += np.asarray(setpoint.acceleration, float) / np.array([gains.vel_p, gains.vel_p, gains.z_vel_p])
    speed = float(np.linalg.norm(v))
    if speed > MAX_SPEED:
        v *= MAX_SPEED / speed          # What flight_interface would do; done here so status shows it.
    rate = gains.yaw_p * wrap(setpoint.yaw - yaw_of(telemetry.rotation))
    return v, float(np.clip(rate, -gains.max_yaw_rate, gains.max_yaw_rate))


def parse_state(text):
    """``/skybeta/state`` JSON -> (armed, offboard_active, landed); landed is
    None until PX4 has reported it."""
    try:
        state = json.loads(text)
    except (TypeError, ValueError):
        return False, False, None
    return bool(state.get('armed')), bool(state.get('offboard_active')), state.get('landed')


def stamp(msg, now_s):
    msg.header.stamp.sec = int(math.floor(now_s))
    msg.header.stamp.nanosec = int((now_s - math.floor(now_s)) * 1e9)
    msg.header.frame_id = 'odom'
    return msg
