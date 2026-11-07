"""PX4 1.16 multicopter takeoff handling, as in mc_pos_control (verified in SITL).

Port of Takeoff.cpp plus the takeoff parts of MulticopterPositionControl.cpp:

- disarmed -> spoolup on arming; -> ready after COM_SPOOLUP_TIME;
- ready -> rampup when Offboard wants takeoff: a fresh setpoint whose z
  position is above the estimate, or whose vertical velocity or acceleration
  points up (any amount);
- before rampup PX4 commands zero thrust and resets the integrators;
- rampup: the upward velocity limit ramps linearly over MPC_TKO_RAMP_T from
  -g / MPC_Z_VEL_P_ACC (zero thrust) to MPC_Z_VEL_MAX_UP, so liftoff waits
  until about 45 % of the ramp; the vertical acceleration feedforward is
  dropped; -> flight when the ramp ends (MPC_TKO_RAMP_T 0: at once);
- flight -> ready if the land detector reports landed;
- the tilt limit is MPC_TILTMAX_LND before flight and MPC_TILTMAX_AIR in
  flight, and always moves toward its target at 0.2 rad/s (hard-coded), so
  with the stock 12 degrees full tilt arrives 2.9 s after the ramp ends.
"""

GRAVITY = 9.81
TILT_SLEW_RAD_S = 0.2


class Takeoff:
    def __init__(self, cfg):
        self.cfg = cfg
        self.state = 'disarmed'
        self.armed_for = 0.
        self.progress = 0.
        # On the pad PX4 sits in Hold with position control running, so the
        # slewed limit has settled at the landed value before arming.
        self.tilt = min(cfg.landed_tilt_rad, cfg.max_tilt_rad)

    @property
    def taken_off(self):
        return self.state in ('rampup', 'flight')

    def update(self, armed, landed, want_takeoff, dt):
        cfg = self.cfg
        self.armed_for = self.armed_for + dt if armed else 0.
        if self.state == 'disarmed' and armed:
            self.state = 'spoolup'
        if self.state == 'spoolup' and self.armed_for >= cfg.spoolup_s:
            self.state = 'ready'
        if self.state == 'ready' and want_takeoff:
            self.state, self.progress = 'rampup', 0.
        if self.state == 'rampup' and self.progress >= 1.:
            self.state = 'flight'
        if self.state == 'flight' and landed:
            self.state = 'ready'
        if not armed:
            self.state = 'disarmed'
        if self.state == 'rampup':
            self.progress = self.progress + dt / cfg.takeoff_ramp_s if cfg.takeoff_ramp_s > dt else 1.
        target = cfg.max_tilt_rad if self.state == 'flight' else min(cfg.landed_tilt_rad, cfg.max_tilt_rad)
        step = TILT_SLEW_RAD_S * dt
        self.tilt += max(-step, min(step, target - self.tilt))

    def up_limit(self):
        """Upward velocity limit, m/s (negative: PX4 demands descent, i.e. no thrust)."""
        cfg = self.cfg
        start = -GRAVITY / max(cfg.z_vel_p, 0.01)
        if not self.taken_off:
            return start
        if self.state == 'rampup' and self.progress < 1.:
            return start + self.progress * (cfg.max_up_speed - start)
        return cfg.max_up_speed
