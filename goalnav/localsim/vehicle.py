"""PX4-like multicopter, estimator and uXRCE-DDS topics, as seen from ROS.

Modelled: PX4's offboard position/velocity/acceleration control law with its
velocity limits, the thrust-vector attitude loop (attitude P gain and body-rate
limit), tilt and thrust limits, PX4's takeoff handling (``localsim.takeoff``), yaw
tracking, offboard-signal loss failsafe, mode/arm/force-disarm commands,
AUTO_LAND descent, land detection, free fall after a disarm in the air, battery
drain, and an external-vision-fused local position with drift, noise and
reported eph/epv. PX4's local frame is the aircraft's body frame at start (as
with cuVSLAM), not the course frame. Rough aerodynamics (ground effect, rotor
drag, wall pull, tunnel, turbulence) come from ``localsim.aero``; after a
disarmed drop the airframe slides to rest on Coulomb friction. Not modelled:
rotor dynamics, mixer saturation, tipping over.
"""
from dataclasses import dataclass
import math

import numpy as np

from . import fakeros as m
from .aero import Aero, AeroConfig
from .takeoff import Takeoff

GRAVITY = 9.81


@dataclass
class VehicleConfig(AeroConfig):
    """Sky Beta with PX4 1.16 default multicopter gains (aero fields: AeroConfig).

    Airframe: about 1.30 kg ready to fly, X2807 1500KV on 4S with 7 inch props,
    thrust-to-weight about 4.5 (conservative end of the vendor claims).
    Control law (PX4 PositionControl, offboard velocity + optional feedforward):
    acc_sp = acc_ff + P (v_sp - v) + integral - D a, with a the measured
    acceleration (velocity derivative, low-passed at MPC_VELD_LP), so D works
    against every acceleration, commanded or not; tilt from the horizontal
    acceleration only (MPC_ACC_DECOUPLE), limited by MPC_TILTMAX_AIR (or
    MPC_TILTMAX_LND before the takeoff ramp ends) and available thrust;
    tracking anti-windup on the horizontal integrator. Attitude follows with
    a short lag.
    """
    vel_p: float = 1.8            # MPC_XY_VEL_P_ACC
    vel_i: float = 0.4            # MPC_XY_VEL_I_ACC
    vel_d: float = 0.2            # MPC_XY_VEL_D_ACC
    z_vel_p: float = 4.0          # MPC_Z_VEL_P_ACC
    z_vel_i: float = 2.0          # MPC_Z_VEL_I_ACC
    max_tilt_rad: float = math.radians(45)   # MPC_TILTMAX_AIR
    thrust_to_weight: float = 4.5
    thrust_min: float = 0.12      # MPC_THR_MIN, fraction of full thrust
    # Attitude loop: rate = MC_ROLL_P/MC_PITCH_P x tilt error, limited to
    # MC_ROLLRATE_MAX/MC_PITCHRATE_MAX; the rate loop follows with a short lag.
    attitude_p: float = 6.5
    max_tilt_rate: float = math.radians(220)
    rate_time_constant: float = 0.04       # Rate loop plus motor spin-up, fitted to PX4 SITL.
    # Roll/pitch acceleration per unit of motor headroom min(c, 1 - c) at
    # collective c: 4 x 0.106 m x 14.4 N / 0.009 kg m^2 (localsim/rigidbody.py).
    # At hover (c 0.22) that is about 150 rad/s^2; PX4 SITL saturates motors
    # at zero in hard tilt reversals, so this, not the rate limit, binds.
    tilt_authority: float = 676.
    max_xy_speed: float = 12.     # MPC_XY_VEL_MAX
    max_up_speed: float = 3.      # MPC_Z_VEL_MAX_UP
    max_down_speed: float = 1.5   # MPC_Z_VEL_MAX_DN
    spoolup_s: float = 1.0        # COM_SPOOLUP_TIME: no takeoff right after arming
    # Takeoff as tuned in config/px4_race.params (keep in sync); PX4's stock
    # values are 3 s and 12 degrees (suite row stock_takeoff).
    takeoff_ramp_s: float = 0.1   # MPC_TKO_RAMP_T: climb-speed limit ramp (localsim.takeoff)
    landed_tilt_rad: float = math.radians(45)   # MPC_TILTMAX_LND, until the ramp ends
    accel_lp_hz: float = 5.       # MPC_VELD_LP: filter on the D term's acceleration
    drag_per_m: float = 0.02      # quadratic drag, 1/m (about 22 m/s terminal at 45 degrees)
    pos_p: float = 0.95           # MPC_XY_P
    z_pos_p: float = 1.0          # MPC_Z_P
    yaw_p: float = 2.8            # MC_YAW_P
    max_yaw_rate: float = math.radians(200)  # MC_YAWRATE_MAX
    land_speed: float = 0.7       # MPC_LAND_SPEED
    offboard_loss_s: float = 1.0
    # Estimator. Measured in the lab at rest: eph 0.035, epv 0.093.
    eph: float = 0.035
    epv_landed: float = 0.093
    epv_flying: float = 0.04
    noise_m: float = 0.004
    drift_m_per_sqrt_s: float = 0.004
    yaw_drift_rad_per_sqrt_s: float = 0.0005
    range_min_m: float = 0.10
    range_bitfield: int = 0      # The lab reported 0; 1 means rangefinder fused.
    battery_drain_per_s: float = 0.0025


class Vehicle:
    def __init__(self, bus, position, yaw, config=None, seed=0, body_height=0.07, world=None, walls=None):
        self.bus, self.cfg = bus, config or VehicleConfig()
        self.rng = np.random.default_rng(seed)
        self.aero = Aero(self.cfg, world, walls, seed)
        self.sliding = False              # Disarmed drop still sliding on the floor.
        self.p = np.array(position, dtype=float)     # Course frame ENU, body origin.
        self.p[2] = body_height
        self.ground = body_height
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.a_cmd = np.zeros(3)          # Commanded acceleration (after tilt/thrust limits).
        self.body_z = np.array([0., 0., 1.])
        self.body_rate = np.zeros(3)
        self.yaw_rate = 0.
        self.takeoff = Takeoff(self.cfg)
        self.accel = np.zeros(3)          # PX4's filtered acceleration estimate (D term).
        self.thrust_fraction = 0.
        self.impacts = []                 # (time, vertical speed, horizontal speed) after free fall.
        self.land_xy = None
        self.integral = np.zeros(3)       # m/s^2, PX4's _vel_int.
        self.last_v = np.zeros(3)
        self.yaw = yaw
        self.armed, self.landed = False, True
        self.nav_state = m.VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        self.failsafe = False
        self.battery = 1.
        self.last_mode_ns = -10**12
        self.setpoint_times = []
        self.setpoint = None
        self.landed_since = 0.
        self.armed_at, self.flew = 0., False
        self.frame_origin, self.frame_yaw = self.p.copy(), yaw
        self.drift = np.zeros(3)
        self.yaw_drift = 0.
        self.events = []
        bus.subscribe('/fmu/in/offboard_control_mode', self._mode)
        bus.subscribe('/fmu/in/trajectory_setpoint', self._setpoint)
        bus.subscribe('/fmu/in/vehicle_command', self._command)

    # -- inputs ----------------------------------------------------------
    def _mode(self, message):
        self.last_mode_ns = self.bus.ns

    def _setpoint(self, message):
        self.setpoint = message
        self.setpoint_times = [t for t in self.setpoint_times if self.bus.ns - t < 1e9] + [self.bus.ns]

    def _command(self, message):
        accepted = False
        if message.command == 176 and message.param2 == 6.:
            accepted = len(self.setpoint_times) >= 10 and self.bus.ns - self.last_mode_ns < 5e8
            if accepted:
                self.nav_state = m.VehicleStatus.NAVIGATION_STATE_OFFBOARD
        elif message.command == 400:
            if message.param1 == 1.:
                accepted = self.landed and not self.failsafe
                if accepted and not self.armed:
                    self.armed_at, self.flew, self.sliding = self.bus.seconds, False, False
                self.armed = self.armed or accepted
            else:
                # 21196 is PX4's force flag: disarm even in the air (motors stop).
                accepted = self.landed or message.param2 == 21196.
                if accepted:
                    self.armed = False
        elif message.command == 21:
            accepted = self.armed
            if accepted:
                self.nav_state = m.VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        self.events.append((self.bus.seconds, 'command', int(message.command), float(message.param1), accepted))
        self.bus.publish('/fmu/out/vehicle_command_ack', m.VehicleCommandAck(
            timestamp=self.bus.ns // 1000, command=message.command, result=0 if accepted else 4,
            target_system=message.source_system, target_component=message.source_component))

    # -- local frame -----------------------------------------------------
    def _to_local(self, vector):
        """Course ENU vector -> PX4 local NED (north = start heading)."""
        c, s = math.cos(self.frame_yaw), math.sin(self.frame_yaw)
        return np.array([c * vector[0] + s * vector[1], s * vector[0] - c * vector[1], -vector[2]])

    def _from_local(self, ned):
        c, s = math.cos(self.frame_yaw), math.sin(self.frame_yaw)
        return np.array([c * ned[0] + s * ned[1], s * ned[0] - c * ned[1], -ned[2]])

    def heading(self):
        return math.atan2(math.sin(self.frame_yaw - self.yaw), math.cos(self.frame_yaw - self.yaw))

    # -- dynamics --------------------------------------------------------
    def step(self, dt):
        cfg, now_s = self.cfg, self.bus.seconds
        offboard = self.nav_state == m.VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if self.armed and offboard and self.bus.ns - self.last_mode_ns > cfg.offboard_loss_s * 1e9:
            self.failsafe = True
            self.nav_state = m.VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            self.events.append((now_s, 'failsafe', 'offboard signal lost'))
        target_v, target_yaw, feedforward, yaw_ff = np.zeros(3), self.yaw, np.zeros(3), 0.
        if self.armed and self.nav_state == m.VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # AUTO_LAND descends while holding the horizontal position where it began.
            if self.land_xy is None:
                self.land_xy = (self.p + self.drift)[:2].copy()
            hold = cfg.pos_p * (self.land_xy - (self.p + self.drift)[:2])
            target_v = np.array([hold[0], hold[1], -cfg.land_speed])
        elif self.armed and offboard and self.setpoint is not None:
            target_v = self._from_local(np.nan_to_num(np.asarray(self.setpoint.velocity, float)))
            position = np.asarray(self.setpoint.position, float)
            if np.all(np.isfinite(position)):
                # PX4 position loop on its own (drifting) estimate.
                estimate = self._to_local(self.p - self.frame_origin + self.drift)
                gains = np.array([cfg.pos_p, cfg.pos_p, cfg.z_pos_p])
                target_v = target_v + self._from_local(gains * (position - estimate))
            feedforward = self._from_local(np.nan_to_num(np.asarray(self.setpoint.acceleration, float)))
            if math.isfinite(self.setpoint.yaw):
                target_yaw = self.frame_yaw - self.setpoint.yaw
            if math.isfinite(self.setpoint.yawspeed):
                yaw_ff = -float(self.setpoint.yawspeed)      # NED clockwise -> course CCW
        self.takeoff.update(self.armed, self.landed, self._want_takeoff(), dt)
        if self.takeoff.state == 'rampup' and self.setpoint is not None \
                and math.isfinite(self.setpoint.velocity[2]):
            feedforward[2] = 0.           # PX4 drops the vertical feedforward during the ramp.
        target_v = self._limit_velocity(target_v)
        if not self.takeoff.taken_off:
            # PX4 commands no thrust (100 m/s^2 down) and no corrections until the ramp.
            target_v, feedforward = np.zeros(3), np.array([0., 0., -100.])
            self.integral[:] = 0.
        thrust = self._velocity_control(target_v, feedforward, dt)
        speed = np.linalg.norm(self.v)
        drag = -cfg.drag_per_m * speed * self.v
        if self.armed and self.takeoff.taken_off:
            scale, extra = self.aero.step(self.p, self.v, self.body_z, thrust, dt)
            self.a = thrust * scale * self.body_z - np.array([0., 0., GRAVITY]) + drag + extra
        elif self.armed or self.p[2] <= self.ground + 1e-6:
            self.a = np.zeros(3)          # Idle on the ground.
        else:
            self.a = np.array([0., 0., -GRAVITY]) + drag      # Disarmed in the air: free fall.
        self.v = self.v + self.a * dt
        if not self.armed:
            self.integral[:] = 0.
            if self.p[2] <= self.ground + 1e-6 and not self.sliding:
                self.v[:] = 0.
        error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
        rate = max(-cfg.max_yaw_rate, min(cfg.max_yaw_rate, yaw_ff + cfg.yaw_p * error))
        self.yaw_rate = rate if self.armed and not self.landed else 0.
        self.yaw = math.atan2(math.sin(self.yaw + self.yaw_rate * dt), math.cos(self.yaw + self.yaw_rate * dt))
        self.p = self.p + self.v * dt
        if self.p[2] <= self.ground:
            self._ground_contact(now_s, dt)
        on_ground = self.p[2] <= self.ground + 0.01 and abs(self.v[2]) < 0.1
        # PX4's land detector clears as soon as the thrust setpoint leaves idle.
        idle = cfg.thrust_min + 0.1 * (1. / cfg.thrust_to_weight - cfg.thrust_min)
        if on_ground and not (self.armed and self.thrust_fraction > idle):
            self.landed_since = self.landed_since or now_s
            self.landed = now_s - self.landed_since > 0.4 or not self.armed
        else:
            self.landed_since, self.landed = 0., False
            self.flew = self.flew or self.p[2] > self.ground + 0.3
        if self.landed and not self.sliding:
            self.v[:2] = 0.
            # PX4 COM_DISARM_LAND (2 s after a flight) and COM_DISARM_PRFLT (10 s).
            waited = now_s - max(self.landed_since, self.armed_at)
            if self.armed and waited > (2.4 if self.flew else 10.):
                self.armed = False
                self.events.append((now_s, 'auto-disarm after landing'))
        if self.armed:
            self.battery -= cfg.battery_drain_per_s * dt
        root = math.sqrt(dt)
        self.drift += self.rng.normal(0., cfg.drift_m_per_sqrt_s * root, 3) * (not self.landed)
        self.yaw_drift += self.rng.normal(0., cfg.yaw_drift_rad_per_sqrt_s * root) * (not self.landed)

    def _ground_contact(self, now_s, dt):
        """Floor contact. A disarmed drop keeps its horizontal speed and slides
        to rest on Coulomb friction (no tipping, no bounce)."""
        if not self.armed and self.v[2] < -0.05:
            horizontal = float(np.hypot(*self.v[:2]))
            self.impacts.append((now_s, float(-self.v[2]), horizontal))
            self.events.append((now_s, 'dropped onto the ground', round(float(-self.v[2]), 2)))
            self.v[2] = 0.
            self.sliding = horizontal > 0.
        self.p[2] = self.ground
        self.v[2] = max(0., self.v[2])
        if self.sliding:
            speed, slowing = float(np.hypot(*self.v[:2])), self.cfg.ground_friction * GRAVITY * dt
            if self.armed or speed <= slowing:
                self.v[:2], self.sliding = 0., False
            else:
                self.v[:2] *= 1. - slowing / speed
        elif self.landed is False and np.hypot(*self.v[:2]) < .3:
            self.v[:2] = 0.

    def _limit_velocity(self, target_v):
        """PX4 velocity-setpoint limits, with the smooth-takeoff climb ramp."""
        cfg = self.cfg
        up = min(self.takeoff.up_limit(), cfg.max_up_speed)
        horizontal = math.hypot(target_v[0], target_v[1])
        out = target_v.copy()
        if horizontal > cfg.max_xy_speed:
            out[:2] *= cfg.max_xy_speed / horizontal
        # PX4's constrain(): the up limit wins even when it is below -max_down (ramp start).
        out[2] = up if out[2] > up else max(-cfg.max_down_speed, out[2])
        return out

    def _want_takeoff(self):
        """PX4 offboard: a fresh setpoint above the estimate or climbing (any amount)."""
        if not self.armed or self.setpoint is None or self.bus.ns - self.setpoint_times[-1] > 1e9:
            return False
        position, velocity, acceleration = (np.asarray(getattr(self.setpoint, k), float)
                                            for k in ('position', 'velocity', 'acceleration'))
        estimate = self._to_local(self.p - self.frame_origin + self.drift)
        return bool((math.isfinite(position[2]) and position[2] < estimate[2])
                    or (math.isfinite(velocity[2]) and velocity[2] < 0.)
                    or (math.isfinite(acceleration[2]) and acceleration[2] < 0.))

    def _velocity_control(self, target_v, feedforward, dt):
        """PX4 PositionControl::_velocityControl; returns the collective thrust (m/s^2)."""
        cfg = self.cfg
        alpha = dt / (dt + 1. / (2. * math.pi * cfg.accel_lp_hz)) if cfg.accel_lp_hz > 0 else 1.
        self.accel += alpha * ((self.v - self.last_v) / dt - self.accel)
        self.last_v = self.v.copy()
        error = target_v - self.v
        gains = np.array([cfg.vel_p, cfg.vel_p, cfg.z_vel_p])
        wanted = feedforward + gains * error + self.integral - cfg.vel_d * self.accel * [1, 1, 0]
        tilt = self.takeoff.tilt if self.armed else cfg.max_tilt_rad
        thrust = self._attitude_step(wanted, dt, tilt, cfg.thrust_min if self.takeoff.state == 'flight' else 0.)
        full = cfg.thrust_to_weight * GRAVITY
        if (thrust <= cfg.thrust_min * full + 1e-6 and error[2] <= 0) or (thrust >= full - 1e-6 and error[2] >= 0):
            error[2] = 0.                 # Vertical anti-windup at the thrust limits.
        produced = self.a_cmd[:2]
        if np.dot(wanted[:2], wanted[:2]) > np.dot(produced, produced):
            error[:2] -= 2. / cfg.vel_p * (wanted[:2] - produced)     # Tracking anti-windup.
        self.integral += error * np.array([cfg.vel_i, cfg.vel_i, cfg.z_vel_i]) * dt
        self.integral[2] = max(-GRAVITY, min(GRAVITY, self.integral[2]))
        self.thrust_fraction = thrust / full
        return thrust

    def _attitude_step(self, wanted, dt, tilt_limit, thrust_min):
        """PX4 thrust vector: tilt-limited direction, attitude P loop, rate limit.

        Returns the collective thrust (m/s^2 along body z). As PX4 with
        MPC_ACC_DECOUPLE, the direction comes from the horizontal acceleration
        and gravity only, and the thrust gives the wanted vertical acceleration
        along the *desired* body axis.
        """
        cfg = self.cfg
        desired = np.array([wanted[0], wanted[1], GRAVITY])
        desired = desired / np.linalg.norm(desired)
        tilt = math.acos(max(-1., min(1., desired[2])))
        if tilt > tilt_limit:
            horizontal = desired[:2] / max(1e-9, np.hypot(*desired[:2]))
            desired = np.array([*(horizontal * math.sin(tilt_limit)), math.cos(tilt_limit)])
        thrust = max(0., wanted[2] + GRAVITY) / desired[2]
        thrust = max(thrust_min * cfg.thrust_to_weight * GRAVITY,
                     min(cfg.thrust_to_weight * GRAVITY, thrust))
        self.a_cmd = thrust * desired - np.array([0., 0., GRAVITY])
        if not self.armed or not self.takeoff.taken_off:
            self.body_z, self.body_rate = np.array([0., 0., 1.]), np.zeros(3)
            return thrust
        axis = np.cross(self.body_z, desired)
        angle = math.atan2(np.linalg.norm(axis), float(np.dot(self.body_z, desired)))
        rate_cmd = np.zeros(3)
        if angle > 1e-9:
            rate_cmd = axis / np.linalg.norm(axis) * min(cfg.attitude_p * angle, cfg.max_tilt_rate)
        change = (rate_cmd - self.body_rate) * min(1., dt / cfg.rate_time_constant)
        # The rate loop lags in the body frame: while yawing, the lagging roll/pitch
        # rates turn with the body, so the thrust vector trails a turning setpoint.
        change += np.cross([0., 0., self.yaw_rate], self.body_rate) * dt
        collective = self.thrust_fraction
        most = cfg.tilt_authority * min(collective, 1. - collective) * dt
        size = np.linalg.norm(change)
        if size > most > 0:
            change *= most / size
        self.body_rate = self.body_rate + change
        rate = np.linalg.norm(self.body_rate)
        if rate > 1e-9:
            k = self.body_rate / rate
            theta = rate * dt
            z = self.body_z
            z = z * math.cos(theta) + np.cross(k, z) * math.sin(theta) + k * np.dot(k, z) * (1 - math.cos(theta))
            self.body_z = z / np.linalg.norm(z)
        return thrust

    def attitude(self):
        """Body FRD -> local NED quaternion (w, x, y, z) from yaw and acceleration."""
        z_local = self._to_local(self.body_z)        # NED; up is -z.
        heading = self.heading() + self.yaw_drift
        c, s = math.cos(heading), math.sin(heading)
        forward = c * z_local[0] + s * z_local[1]
        right = -s * z_local[0] + c * z_local[1]
        pitch, roll = -math.atan2(forward, -z_local[2]), math.atan2(right, -z_local[2])
        cy, sy, cp, sp, cr, sr = (math.cos(heading / 2), math.sin(heading / 2), math.cos(pitch / 2),
                                  math.sin(pitch / 2), math.cos(roll / 2), math.sin(roll / 2))
        return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                         cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy], dtype=np.float32)

    @property
    def on_ground(self):
        """Truth for the referee: the body rests on the floor (PX4's land
        detector, ``landed``, clears earlier, as soon as the thrust leaves idle)."""
        return bool(self.p[2] <= self.ground + 1e-3)

    def rotation(self):
        """Course-frame body rotation (FLU -> ENU) including tilt."""
        roll_pitch = math.atan2(np.hypot(*self.body_z[:2]), self.body_z[2])
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        yaw_matrix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        if roll_pitch < 1e-6:
            return yaw_matrix
        axis = np.array([-self.body_z[1], self.body_z[0], 0.]) / np.hypot(*self.body_z[:2])   # Tilt toward thrust.
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        tilt = np.eye(3) + math.sin(roll_pitch) * k + (1 - math.cos(roll_pitch)) * k @ k
        return tilt @ yaw_matrix

    # -- outputs ---------------------------------------------------------
    def publish_fast(self):
        """Local position and attitude, as PX4 sends them (about 50 Hz)."""
        cfg = self.cfg
        us = self.bus.ns // 1000
        noise = cfg.noise_m if not self.landed else min(cfg.noise_m, 0.004)
        estimate = self._to_local(self.p - self.frame_origin + self.drift) + self.rng.normal(0., noise, 3)
        velocity = self._to_local(self.v) + self.rng.normal(0., 0.003, 3)
        height = self.p[2] - self.ground + 0.0007
        range_valid = height >= cfg.range_min_m
        self.bus.publish('/fmu/out/vehicle_local_position_v1', m.VehicleLocalPosition(
            timestamp=us, timestamp_sample=us, x=float(estimate[0]), y=float(estimate[1]),
            z=float(estimate[2]), vx=float(velocity[0]), vy=float(velocity[1]), vz=float(velocity[2]),
            heading=float(self.heading() + self.yaw_drift), eph=cfg.eph,
            epv=cfg.epv_landed if not range_valid else cfg.epv_flying,
            dist_bottom=float(height + self.ground), dist_bottom_valid=bool(range_valid),
            dist_bottom_var=0.0055, dist_bottom_sensor_bitfield=cfg.range_bitfield if range_valid else 0))
        self.bus.publish('/fmu/out/vehicle_attitude', m.VehicleAttitude(
            timestamp=us, timestamp_sample=us, q=self.attitude()))

    def publish_slow(self):
        us = self.bus.ns // 1000
        self.bus.publish('/fmu/out/vehicle_status_v1', m.VehicleStatus(
            timestamp=us, arming_state=2 if self.armed else 1, nav_state=self.nav_state,
            failsafe=self.failsafe))
        self.bus.publish('/fmu/out/vehicle_land_detected', m.VehicleLandDetected(
            timestamp=us, landed=self.landed))
        self.bus.publish('/fmu/out/battery_status_v1', m.BatteryStatus(
            timestamp=us, remaining=float(self.battery), connected=True))
