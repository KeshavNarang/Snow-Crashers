"""ROS node: listen to every sensor the mission uses for a few seconds, print PASS/WARN/FAIL, exit.

Never commands anything. Drone on the ground, still and level (the tilt checks assume it):

    ros2 launch overwall_nav sensor_check.launch.py     # starts stereo_depth too, ~15 s

Judgement in checks.py.
"""
import json
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from overwall_nav.checks import evaluate, oak_imu_pitch_deg, verdict
from overwall_nav.perception import analyze

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:          # not on a laptop
    VisualSlamStatus = None


class SensorCheck(Node):

    def __init__(self):
        super().__init__('sensor_check')
        self.declare_parameter('duration_s', 12.0)
        self.declare_parameter('expected_pitch_deg', 27.0)
        self.duration = float(self.get_parameter('duration_s').value)
        self.expected = float(self.get_parameter('expected_pitch_deg').value)
        self.c = {'left_images': 0, 'right_images': 0, 'imu': 0, 'odom': 0, 'fronts': 0}
        self.info, self.state, self.vo = {}, None, None
        self.valid, self.proc, self.age, self.fronts_on_ground = [], [], [], []
        self.floor_pitch, self.floor_height, self.tf_pitch = [], [], None
        self.accel, self.imu_frame = [], None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, '/oak/left/image_mono', lambda m: self.count('left_images'), be)
        self.create_subscription(Image, '/oak/right/image_mono', lambda m: self.count('right_images'), be)
        self.create_subscription(CameraInfo, '/oak/left/camera_info', lambda m: self.info.setdefault('left', m), 1)
        self.create_subscription(CameraInfo, '/oak/right/camera_info', lambda m: self.info.setdefault('right', m), 1)
        self.create_subscription(Imu, '/oak/imu/data', self.on_imu, be)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', lambda m: self.count('odom'), be)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.create_subscription(String, '/overwall/front', self.on_front, 10)
        if VisualSlamStatus is not None:
            self.create_subscription(VisualSlamStatus, '/visual_slam/status',
                                     lambda m: setattr(self, 'vo', int(m.vo_state)), be)
        self.started = time.monotonic()
        self.done = False
        self.create_timer(0.5, self.maybe_finish)
        self.get_logger().info(f'sensor check: listening for {self.duration:.0f} s. Keep the drone still and level.')

    def count(self, key):
        self.c[key] += 1

    def on_imu(self, msg):
        self.count('imu')
        self.imu_frame = msg.header.frame_id
        a = msg.linear_acceleration
        self.accel.append((a.x, a.y, a.z))

    def on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
        except ValueError:
            pass

    def on_front(self, msg):
        try:
            f = json.loads(msg.data)
        except ValueError:
            return
        self.count('fronts')
        self.valid.append(f.get('valid', 0.0))
        self.proc.append(f.get('proc_ms', 0.0))
        self.age.append(f.get('age_ms', 0.0))
        self.tf_pitch = f.get('tf_pitch_deg')
        floor = f.get('floor')
        if floor:
            self.floor_pitch.append(floor['pitch_deg'])
            self.floor_height.append(floor['height_m'])
        a = analyze(f.get('cells', []), floor['height_m'] if floor else None, float(f.get('top_edge_deg', 0.0)))
        self.fronts_on_ground.append(a['front'])

    def stats(self):
        med = lambda xs: statistics.median(xs) if xs else None  # noqa: E731
        s = dict(self.c, duration=time.monotonic() - self.started,
                 left_info='left' in self.info, right_info='right' in self.info,
                 valid_median=med(self.valid) or 0.0, proc_ms_median=med(self.proc) or 0.0,
                 age_ms_median=med(self.age) or 0.0, floor_pitch_deg=med(self.floor_pitch),
                 floor_height_m=med(self.floor_height), tf_pitch_deg=self.tf_pitch,
                 state=self.state, vo_state=self.vo)
        seen = [x for x in self.fronts_on_ground if x is not None]
        s['front_on_ground'] = med(seen) if len(seen) > len(self.fronts_on_ground) / 2 else None
        if 'left' in self.info and 'right' in self.info:
            try:
                t = self.tf_buffer.lookup_transform(self.info['left'].header.frame_id,
                                                    self.info['right'].header.frame_id, Time()).transform.translation
                s['baseline_m'] = (t.x ** 2 + t.y ** 2 + t.z ** 2) ** 0.5
            except Exception:  # noqa: BLE001
                s['baseline_m'] = None
        if self.accel:
            mean = [statistics.fmean(a[k] for a in self.accel) for k in range(3)]
            s['imu_g'] = sum(v * v for v in mean) ** 0.5
            s['imu_pitch_deg'] = oak_imu_pitch_deg(mean)    # the IMU sits in the camera housing
        return s

    def maybe_finish(self):
        if self.done or time.monotonic() - self.started < self.duration:
            return
        self.done = True
        results = evaluate(self.stats(), expected_pitch=self.expected)
        width = max(len(n) for _, n, _ in results)
        lines = [f'  {s:4}  {n:<{width}}  {d}' for s, n, d in results]
        self.get_logger().info('sensor check results:\n' + '\n'.join(lines) + f'\n  => {verdict(results)}')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = SensorCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass
