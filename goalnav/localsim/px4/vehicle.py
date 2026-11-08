"""Real PX4 1.16 SITL flying localsim's rigid body, seen from ROS as the lab's topics.

Drop-in for ``localsim.vehicle.Vehicle`` (same attributes and publish calls).
PX4 runs in lockstep: every 4 ms we send the IMU (plus baro and mag) for the
body's state, PX4 runs EKF2, the position/attitude/rate controllers and the
allocator, and its motor outputs drive the next 4 ms of physics. cuVSLAM is
stood in for by ODOMETRY from truth with noise, drift and latency.

Toward the nodes it plays the uXRCE-DDS bridge: TrajectorySetpoint becomes
SET_POSITION_TARGET_LOCAL_NED, VehicleCommand becomes COMMAND_LONG, and
PX4's MAVLink telemetry is republished as the ``_v1`` px4_msgs topics. Three
differences from DDS: PX4 derives the offboard control mode from which
setpoint fields are set (our nodes send all of them), the downward range
(dist_bottom) is synthesized from truth as in the Python model, and the
per-state reset counters are inferred from ODOMETRY's sum (ResetCounters).
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .. import fakeros as m
from ..rigidbody import QuadConfig, RigidBody
from .links import CompanionLink, SimLink, mavlink
from .process import PX4Process

EARTH_FIELD_NED = np.array([0.21, 0.0, 0.42])      # gauss; not fused (EKF2_MAG_TYPE none)
FRD = np.diag([1., -1., -1.])
# PX4 custom (main mode, sub mode) -> vehicle_status nav_state.
NAV_STATES = {(1, 0): 0, (2, 0): 1, (3, 0): 2, (4, 4): 3, (4, 3): 4, (4, 5): 5, (5, 0): 10,
              (6, 0): 14, (7, 0): 15, (4, 2): 17, (4, 6): 18, (4, 8): 19, (4, 9): 20}
EST = mavlink    # ESTIMATOR_STATUS flag constants live in the dialect.


@dataclass
class PX4Config(QuadConfig):
    """Rigid body plus the cuVSLAM stand-in (rates and noise as the Python model)."""
    ev_rate_hz: float = 30.
    ev_latency_s: float = 0.04     # Matches EKF2_EV_DELAY in the airframe.
    noise_m: float = 0.004
    velocity_noise: float = 0.01
    drift_m_per_sqrt_s: float = 0.004
    yaw_drift_rad_per_sqrt_s: float = 0.0005
    range_min_m: float = 0.10
    warmup_max_s: float = 60.


def _quaternion(rotation):
    """Rotation matrix -> (w, x, y, z)."""
    w = math.sqrt(max(0., 1. + rotation[0, 0] + rotation[1, 1] + rotation[2, 2])) / 2.
    x = math.copysign(math.sqrt(max(0., 1. + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) / 2.,
                      rotation[2, 1] - rotation[1, 2])
    y = math.copysign(math.sqrt(max(0., 1. - rotation[0, 0] + rotation[1, 1] - rotation[2, 2])) / 2.,
                      rotation[0, 2] - rotation[2, 0])
    z = math.copysign(math.sqrt(max(0., 1. - rotation[0, 0] - rotation[1, 1] + rotation[2, 2])) / 2.,
                      rotation[1, 0] - rotation[0, 1])
    return np.array([w, x, y, z])


def _yaw_of(q):
    w, x, y, z = q
    return math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))


class ResetCounters:
    """EKF2's per-state reset counters for vehicle_local_position.

    MAVLink ODOMETRY carries only their sum. An increase counts as a position
    (xy, z) or heading reset when the estimate jumps by more than its velocity
    explains (5 cm, 0.05 rad between ODOMETRY messages), else as a velocity
    reset (vxy and vz; after a hard drop PX4 resets only these)."""
    JUMP_M, JUMP_RAD = 0.05, 0.05

    def __init__(self):
        self.total, self.last = None, None
        self.counts = dict.fromkeys(('xy', 'z', 'heading', 'vxy', 'vz'), 0)

    def update(self, odometry, local):
        if odometry is None or local is None:
            return
        state = (odometry.time_usec / 1e6, np.array([odometry.x, odometry.y, odometry.z]),
                 np.array([local.vx, local.vy, local.vz]), _yaw_of(odometry.q))
        if self.total is None:
            self.total = odometry.reset_counter
            self.counts = dict.fromkeys(self.counts, self.total)
        elif odometry.reset_counter != self.total and self.last is not None:
            steps = (odometry.reset_counter - self.total) % 256
            then, position, velocity, yaw = self.last
            jump = state[1] - position - velocity * max(0., state[0] - then)
            turn = abs(math.atan2(math.sin(state[3] - yaw), math.cos(state[3] - yaw)))
            kinds = [kind for kind, hit in (('xy', math.hypot(*jump[:2]) > self.JUMP_M),
                                            ('z', abs(jump[2]) > self.JUMP_M),
                                            ('heading', turn > self.JUMP_RAD)) if hit] or ['vxy', 'vz']
            for kind in kinds:
                self.counts[kind] = (self.counts[kind] + steps) % 256
            self.total = odometry.reset_counter
        self.last = state


class PX4Vehicle:
    dt = 0.004          # PX4 SITL IMU step (IMU_INTEG_RATE 250 Hz).
    substeps = 2

    def __init__(self, bus, position, yaw, config=None, seed=0, world=None, walls=None,
                 instance=0, param_files=(), out_dir=None):
        self.bus, self.cfg = bus, config or PX4Config()
        self.rng = np.random.default_rng(seed)
        self.body = RigidBody(position, yaw, self.cfg, world, walls, seed)
        self.frame_yaw, self.frame_origin = yaw, self.body.p.copy()
        c, s = math.cos(yaw), math.sin(yaw)
        self.to_ned = np.array([[c, s, 0.], [s, -c, 0.], [0., 0., -1.]])   # Course ENU -> PX4 local NED.
        self.drift, self.yaw_drift = np.zeros(3), 0.
        self.events, self.impacts = [], []
        self.history = deque(maxlen=64)
        self.resets, self.odometry = ResetCounters(), None
        self.published = None
        self.sysid = instance + 1
        self.out_dir = out_dir
        self.px4 = PX4Process(instance, param_files, out_dir)
        self.sim = SimLink(self.px4.sim_port)
        self.companion = None
        self.px4_us, self.next_ev_us, self.offset_us = 1_000_000, 0, 0
        try:
            self.px4.start()
            self.sim.accept(60., self.px4.alive)
            self.companion = CompanionLink(*self.px4.companion_ports)
            self._warm_up()
        except BaseException:
            self.close()
            raise
        bus.subscribe('/fmu/in/trajectory_setpoint', self._setpoint)
        bus.subscribe('/fmu/in/vehicle_command', self._command)

    # -- state the harness reads ------------------------------------------
    @property
    def p(self):
        return self.body.p

    @property
    def v(self):
        return self.body.v

    @property
    def yaw(self):
        return self.body.yaw

    @property
    def armed(self):
        return self.sim.armed

    @property
    def landed(self):
        state = self._latest('EXTENDED_SYS_STATE')
        return state is None or state.landed_state == mavlink.MAV_LANDED_STATE_ON_GROUND

    @property
    def on_ground(self):
        """Truth for the referee (``landed`` is PX4's land detector)."""
        return self.body.on_ground

    def rotation(self):
        return self.body.R

    # -- lockstep ----------------------------------------------------------
    def step(self, dt):
        for _ in range(max(1, round(dt / self.dt))):
            self._lockstep()
        self._publish_estimate()
        while self.body.impacts:
            vertical, horizontal = self.body.impacts.pop(0)
            self.impacts.append((self.bus.seconds, vertical, horizontal))
            self.events.append((self.bus.seconds, 'dropped onto the ground', round(vertical, 2)))
        for acknowledgement in self.companion.acks:
            self.events.append((self.bus.seconds, 'ack', int(acknowledgement.command), int(acknowledgement.result)))
            self.bus.publish('/fmu/out/vehicle_command_ack', m.VehicleCommandAck(
                timestamp=self.bus.ns // 1000, command=acknowledgement.command, result=acknowledgement.result,
                target_system=acknowledgement.target_system, target_component=acknowledgement.target_component))
        self.companion.acks.clear()
        for when, text in self.companion.texts:
            self.events.append((round((when - self.offset_us) / 1e6, 3), 'px4', text))
        self.companion.texts.clear()

    def _lockstep(self):
        accel, gyro = self.body.imu()
        body_to_ned = self.to_ned @ self.body.R @ FRD
        altitude = float(self.body.p[2])
        pressure = 1013.25 * (1. - 2.25577e-5 * altitude) ** 5.25588
        self.sim.send_sensors(self.px4_us, accel, gyro, body_to_ned.T @ EARTH_FIELD_NED, pressure, altitude)
        self._vision(body_to_ned)
        self.sim.wait_controls(self.px4_us)
        for _ in range(self.substeps):
            self.body.step(self.sim.controls[:4], self.dt / self.substeps, self.sim.armed)
        self.px4_us += round(self.dt * 1e6)
        self.companion.poll(self.px4_us)
        moving = not self.body.on_ground
        root = math.sqrt(self.dt)
        self.drift += self.rng.normal(0., self.cfg.drift_m_per_sqrt_s * root, 3) * moving
        self.yaw_drift += self.rng.normal(0., self.cfg.yaw_drift_rad_per_sqrt_s * root) * moving

    def _vision(self, body_to_ned):
        """cuVSLAM stand-in: delayed, noisy, drifting pose and velocity in PX4's local NED."""
        cfg = self.cfg
        c, s = math.cos(self.yaw_drift), math.sin(self.yaw_drift)
        drift_yaw = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        position = self.to_ned @ (self.body.p - self.frame_origin) + self.drift
        self.history.append((self.px4_us, position, _quaternion(drift_yaw @ body_to_ned), self.to_ned @ self.body.v))
        if self.px4_us < self.next_ev_us:
            return
        self.next_ev_us = self.px4_us + round(1e6 / cfg.ev_rate_hz)
        wanted = self.px4_us - cfg.ev_latency_s * 1e6
        _, position, q, velocity = min(self.history, key=lambda item: abs(item[0] - wanted))
        position = position + self.rng.normal(0., cfg.noise_m, 3)
        velocity = velocity + self.rng.normal(0., cfg.velocity_noise, 3)
        pose_cov, velocity_cov = [0.] * 21, [0.] * 21
        pose_cov[0] = pose_cov[6] = pose_cov[11] = max(cfg.noise_m, 1e-3) ** 2
        pose_cov[15] = pose_cov[18] = pose_cov[20] = 0.01 ** 2
        velocity_cov[0] = velocity_cov[6] = velocity_cov[11] = max(cfg.velocity_noise, 1e-3) ** 2
        self.sim.send(mavlink.MAVLink_odometry_message(
            self.px4_us, mavlink.MAV_FRAME_LOCAL_NED, mavlink.MAV_FRAME_LOCAL_NED, *map(float, position),
            [float(v) for v in q], *map(float, velocity), math.nan, math.nan, math.nan, pose_cov, velocity_cov,
            0, mavlink.MAV_ESTIMATOR_TYPE_VISION, 100))

    def _warm_up(self):
        """Boot PX4 until EKF2 has a local position and arming checks pass."""
        start, ready_at = self.px4_us, None
        while self.px4_us - start < self.cfg.warmup_max_s * 1e6:
            self._lockstep()
            if ready_at is None and self._ready():
                ready_at = self.px4_us
            if ready_at is not None and self.px4_us - ready_at > 2e6:
                break
        self.offset_us = self.px4_us - self.bus.ns // 1000
        took = (self.px4_us - start) / 1e6
        self.events.append((self.bus.seconds, 'px4 ready' if ready_at else 'px4 not ready', round(took, 2)))
        for when, text in self.companion.texts:
            self.events.append((round((when - self.px4_us) / 1e6, 3), 'px4 boot', text))
        self.companion.texts.clear()

    def _ready(self):
        """EKF2 aligned and fusing vision (SYS_STATUS's pre-arm bit is for Hold, not Offboard)."""
        status = self._latest('ESTIMATOR_STATUS')
        needed = (EST.ESTIMATOR_ATTITUDE | EST.ESTIMATOR_VELOCITY_HORIZ | EST.ESTIMATOR_POS_HORIZ_REL
                  | EST.ESTIMATOR_POS_VERT_ABS | EST.ESTIMATOR_PRED_POS_HORIZ_REL)
        return status is not None and status.flags & needed == needed

    def _latest(self, kind):
        item = self.companion.latest.get(kind) if self.companion else None
        return item[0] if item else None

    # -- ROS inputs -> MAVLink ----------------------------------------------
    def _setpoint(self, message):
        fields, mask = [], 0
        for group, bits in ((message.position, (1, 2, 4)), (message.velocity, (8, 16, 32)),
                            (message.acceleration, (64, 128, 256))):
            for value, bit in zip(np.asarray(group, float), bits):
                finite = math.isfinite(value)
                fields.append(float(value) if finite else 0.)
                mask |= 0 if finite else bit
        yaw, yawspeed = float(message.yaw), float(message.yawspeed)
        mask |= (0 if math.isfinite(yaw) else 1024) | (0 if math.isfinite(yawspeed) else 2048)
        self.companion.send(mavlink.MAVLink_set_position_target_local_ned_message(
            self.px4_us // 1000, self.sysid, 1, mavlink.MAV_FRAME_LOCAL_NED, mask, *fields,
            yaw if math.isfinite(yaw) else 0., yawspeed if math.isfinite(yawspeed) else 0.))

    def _command(self, message):
        self.events.append((self.bus.seconds, 'command', int(message.command), float(message.param1)))
        params = [float(getattr(message, f'param{i}', 0.) or 0.) for i in range(1, 8)]
        self.companion.send(mavlink.MAVLink_command_long_message(
            self.sysid, 1, int(message.command), 0, *params))

    # -- MAVLink telemetry -> ROS -------------------------------------------
    def _sample_us(self, message):
        return min(self.bus.ns // 1000, message.time_boot_ms * 1000 - self.offset_us)

    def publish_fast(self):
        """Nothing on the harness's 50 Hz tick: the estimate goes out as PX4 sends it."""

    def _publish_estimate(self):
        """Each new local position (100 Hz, as uXRCE-DDS forwards EKF2's output), then
        the attitude (the node checks its yaw against the latest heading to 0.05 rad)."""
        us = self.bus.ns // 1000
        local, attitude = self._latest('LOCAL_POSITION_NED'), self._latest('ATTITUDE_QUATERNION')
        if local is None or attitude is None or local is self.published:
            return
        self.published = local
        q = np.array([attitude.q1, attitude.q2, attitude.q3, attitude.q4], dtype=np.float32)
        status, odometry = self._latest('ESTIMATOR_STATUS'), self._latest('ODOMETRY')
        flags = status.flags if status else 0
        if odometry is not self.odometry:
            self.odometry = odometry
            self.resets.update(odometry, local)
        resets = self.resets.counts
        height = self.body.p[2] - self.cfg.body_height + 0.0007
        range_valid = bool(height >= self.cfg.range_min_m)
        self.bus.publish('/fmu/out/vehicle_local_position', m.VehicleLocalPosition(
            timestamp=us, timestamp_sample=self._sample_us(local), x=local.x, y=local.y, z=local.z,
            vx=local.vx, vy=local.vy, vz=local.vz, heading=_yaw_of(q),
            xy_valid=bool(flags & EST.ESTIMATOR_POS_HORIZ_REL), z_valid=bool(flags & EST.ESTIMATOR_POS_VERT_ABS),
            v_xy_valid=bool(flags & EST.ESTIMATOR_VELOCITY_HORIZ), v_z_valid=bool(flags & EST.ESTIMATOR_VELOCITY_VERT),
            heading_good_for_control=bool(flags & EST.ESTIMATOR_ATTITUDE),
            eph=float(status.pos_horiz_accuracy) if status else 1., epv=float(status.pos_vert_accuracy) if status else 1.,
            xy_reset_counter=resets['xy'], z_reset_counter=resets['z'], heading_reset_counter=resets['heading'],
            vxy_reset_counter=resets['vxy'], vz_reset_counter=resets['vz'],
            dist_bottom=float(height + self.cfg.body_height), dist_bottom_valid=range_valid,
            dist_bottom_var=0.0055, dist_bottom_sensor_bitfield=0))
        self.bus.publish('/fmu/out/vehicle_attitude', m.VehicleAttitude(
            timestamp=us, timestamp_sample=self._sample_us(attitude), q=q))

    def publish_slow(self):
        us = self.bus.ns // 1000
        heartbeat, system = self._latest('HEARTBEAT'), self._latest('SYS_STATUS')
        if heartbeat is not None:
            mode = heartbeat.custom_mode
            nav_state = NAV_STATES.get(((mode >> 16) & 0xFF, (mode >> 24) & 0xFF), 4)
            armed = bool(heartbeat.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            prearm = bool(system and system.onboard_control_sensors_health & mavlink.MAV_SYS_STATUS_PREARM_CHECK)
            self.bus.publish('/fmu/out/vehicle_status_v1', m.VehicleStatus(
                timestamp=us, arming_state=2 if armed else 1, nav_state=nav_state,
                failsafe=heartbeat.system_status == mavlink.MAV_STATE_CRITICAL, pre_flight_checks_pass=prearm))
        if self._latest('EXTENDED_SYS_STATE') is not None:
            self.bus.publish('/fmu/out/vehicle_land_detected', m.VehicleLandDetected(timestamp=us, landed=self.landed))
        if system is not None:
            self.bus.publish('/fmu/out/battery_status', m.BatteryStatus(
                timestamp=us, remaining=max(0., system.battery_remaining / 100.), connected=True))

    def close(self):
        """Stop PX4 and keep its ULog next to the run outputs."""
        keep = None
        if self.out_dir:
            keep = str(self.out_dir) + '/px4.ulg'
        self.px4.stop(keep_ulog_as=keep)
        self.sim.close()
        if self.companion:
            self.companion.close()
