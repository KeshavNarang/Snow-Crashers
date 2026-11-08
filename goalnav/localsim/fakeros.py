"""In-process stand-in for the ROS 2 APIs our nodes import (also the
organizers' skybeta flight_interface, run unmodified in the simulator).

It runs the real ``mission_node.main`` and ``vision_node.main`` against a
deterministic simulated clock and a queued topic bus. Message classes copy the
value types rclpy really uses (``bytes`` diagnostic levels, NumPy CameraInfo
matrices, ``array('B')`` image data) so type mistakes show up here too.
It is not DDS: there is no QoS, discovery or message loss unless injected.
"""
from array import array
from collections import defaultdict, deque
import contextlib
import copy
from pathlib import Path
import sys
import types
from unittest import mock

import numpy as np

# The message version lines flight_interface resolves its /fmu topic names from.
PX4_MSGS_SHARE = str(Path(__file__).with_name('px4_msgs'))


class Msg:
    _defaults = {}

    def __init__(self, **values):
        for key, value in self._defaults.items():
            setattr(self, key, value() if callable(value) else copy.deepcopy(value))
        for key, value in values.items():
            setattr(self, key, value)


def _msg(type_name, constants=None, **defaults):
    return type(type_name, (Msg,), {"_defaults": defaults, **(constants or {})})


Time = _msg('Time', sec=0, nanosec=0)
Header = _msg('Header', stamp=Time, frame_id='')
Point = _msg('Point', x=0., y=0., z=0.)
Quaternion = _msg('Quaternion', x=0., y=0., z=0., w=1.)
Pose = _msg('Pose', position=Point, orientation=Quaternion)
PoseCov = _msg('PoseWithCovariance', pose=Pose, covariance=lambda: np.zeros(36))
Vector3 = _msg('Vector3', x=0., y=0., z=0.)
Twist = _msg('Twist', linear=Vector3, angular=Vector3)
TwistCov = _msg('TwistWithCovariance', twist=Twist, covariance=lambda: np.zeros(36))
Odometry = _msg('Odometry', header=Header, child_frame_id='', pose=PoseCov, twist=TwistCov)
PoseStamped = _msg('PoseStamped', header=Header, pose=Pose)
TwistStamped = _msg('TwistStamped', header=Header, twist=Twist)
Image = _msg('Image', header=Header, height=0, width=0, encoding='', is_bigendian=0, step=0,
             data=lambda: array('B'))
RegionOfInterest = _msg('RegionOfInterest', x_offset=0, y_offset=0, height=0, width=0, do_rectify=False)
CameraInfo = _msg('CameraInfo', header=Header, height=0, width=0, distortion_model='plumb_bob',
                  d=lambda: array('d', [0.] * 5), k=lambda: np.zeros(9), r=lambda: np.eye(3).ravel(),
                  p=lambda: np.zeros(12), binning_x=0, binning_y=0, roi=RegionOfInterest)
PointField = _msg('PointField', name='', offset=0, datatype=7, count=1)
PointCloud2 = _msg('PointCloud2', header=Header, height=1, width=0, fields=list, is_bigendian=False,
                   point_step=12, row_step=0, data=lambda: array('B'), is_dense=False)
KeyValue = _msg('KeyValue', key='', value='')
DiagnosticStatus = _msg('DiagnosticStatus', {'OK': b'\x00', 'WARN': b'\x01', 'ERROR': b'\x02', 'STALE': b'\x03'},
                        level=b'\x00', name='', message='', hardware_id='', values=list)
DiagnosticArray = _msg('DiagnosticArray', header=Header, status=list)
String = _msg('String', data='')
Bool = _msg('Bool', data=False)
Float32 = _msg('Float32', data=0.)
ParameterDescriptor = _msg('ParameterDescriptor', name='', read_only=False)
Trigger = _msg('Trigger')
TriggerResponse = _msg('TriggerResponse', success=False, message='')

# PX4 1.16 messages, fields our nodes read or write.
VehicleStatus = _msg('VehicleStatus', {
    'ARMING_STATE_DISARMED': 1, 'ARMING_STATE_ARMED': 2, 'NAVIGATION_STATE_ALTCTL': 1,
    'NAVIGATION_STATE_POSCTL': 2, 'NAVIGATION_STATE_AUTO_LOITER': 4,
    'NAVIGATION_STATE_OFFBOARD': 14, 'NAVIGATION_STATE_AUTO_LAND': 18},
    timestamp=0, arming_state=1, nav_state=4, failsafe=False, pre_flight_checks_pass=True)
VehicleLocalPosition = _msg(
    'VehicleLocalPosition', timestamp=0, timestamp_sample=0, x=0., y=0., z=0., vx=0., vy=0., vz=0.,
    heading=0., xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True, eph=0., epv=0.,
    heading_good_for_control=True, dead_reckoning=False, xy_reset_counter=0, z_reset_counter=0,
    heading_reset_counter=0, vxy_reset_counter=0, vz_reset_counter=0, dist_bottom=0.,
    dist_bottom_valid=False, dist_bottom_var=0., dist_bottom_sensor_bitfield=0, dist_bottom_reset_counter=0)
VehicleAttitude = _msg('VehicleAttitude', timestamp=0, timestamp_sample=0,
                       q=lambda: np.array([1., 0., 0., 0.], dtype=np.float32), quat_reset_counter=0)
VehicleLandDetected = _msg('VehicleLandDetected', timestamp=0, landed=True)
BatteryStatus = _msg('BatteryStatus', timestamp=0, remaining=1., connected=True)
OffboardControlMode = _msg('OffboardControlMode', timestamp=0, position=False, velocity=False,
                           acceleration=False, attitude=False, body_rate=False)
TrajectorySetpoint = _msg('TrajectorySetpoint', timestamp=0, position=lambda: np.zeros(3, np.float32),
                          velocity=lambda: np.zeros(3, np.float32), acceleration=lambda: np.zeros(3, np.float32),
                          jerk=lambda: np.zeros(3, np.float32), yaw=0., yawspeed=0.)
VehicleCommand = _msg('VehicleCommand', {'VEHICLE_CMD_DO_SET_MODE': 176}, timestamp=0, command=0, param1=0., param2=0., target_system=0,
                      target_component=0, source_system=0, source_component=0, from_external=False)
VehicleCommandAck = _msg('VehicleCommandAck', timestamp=0, command=0, result=0, target_system=0,
                         target_component=0)
VehicleOdometry = _msg('VehicleOdometry', timestamp=0)
DistanceSensor = _msg('DistanceSensor', timestamp=0, current_distance=0., signal_quality=-1, orientation=25)
PointStamped = _msg('PointStamped', header=Header, point=Point)


class Bus:
    """Queued topic delivery plus a simulated clock shared by every node."""

    def __init__(self, params):
        self.params = params
        self.ns = 0
        self.subscribers = defaultdict(list)
        self.queue = deque()
        self.timers = []          # [next_ns, period_ns, callback]
        self.services = {}
        self.nodes = []
        self.log = []
        self.tf = {}

    @property
    def seconds(self):
        return self.ns / 1e9

    def publish(self, topic, message):
        self.queue.append((topic, copy.deepcopy(message)))

    def subscribe(self, topic, callback):
        self.subscribers[topic].append(callback)

    def drain(self, limit=10_000):
        for _ in range(limit):
            if not self.queue:
                return
            topic, message = self.queue.popleft()
            for callback in list(self.subscribers.get(topic, ())):
                callback(message)

    def run_timers(self):
        for timer in self.timers:
            if self.ns >= timer[0]:
                timer[0] += timer[1] * max(1, (self.ns - timer[0]) // timer[1] + 1)
                timer[2]()
                self.drain()

    def call(self, service):
        response = self.services[service](Trigger(), TriggerResponse())
        self.drain()
        return response


def _modules(bus):
    class Logger:
        def __init__(self, name):
            self.name = name

        def __getattr__(self, level):
            return lambda text, **_: bus.log.append((bus.seconds, self.name, level, str(text)))

    class Clock:
        def now(self):
            return types.SimpleNamespace(nanoseconds=bus.ns)

    class Publisher:
        def __init__(self, topic):
            self.topic = topic

        def publish(self, message):
            bus.publish(self.topic, message)

    class Node:
        def __init__(self, name):
            self._name = name
            self._params = {'use_sim_time': bus.params.get(name, {}).get('use_sim_time', False)}
            bus.nodes.append(self)

        def declare_parameter(self, name, default, descriptor=None):
            self._params[name] = copy.deepcopy(bus.params.get(self._name, {}).get(name, default))

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self._params[name])

        def create_subscription(self, kind, topic, callback, qos):
            bus.subscribe(topic, callback)

        def create_publisher(self, kind, topic, qos):
            return Publisher(topic)

        def create_service(self, kind, name, callback):
            bus.services[name] = callback

        def create_timer(self, period, callback):
            period_ns = round(period * 1e9)
            bus.timers.append([bus.ns + period_ns, period_ns, callback])

        def get_clock(self):
            return Clock()

        def get_logger(self):
            return Logger(self._name)

        def destroy_node(self):
            pass   # Callbacks stay registered; the harness owns the lifetime.

    class Subscriber:
        def __init__(self, node, kind, topic, qos_profile=None):
            self.topic = topic

    class ApproximateTimeSynchronizer:
        def __init__(self, subscribers, queue_size, slop):
            self.slop_ns = round(slop * 1e9)
            self.queues = [deque(maxlen=queue_size) for _ in subscribers]
            self.callbacks = []
            for index, subscriber in enumerate(subscribers):
                bus.subscribe(subscriber.topic, lambda m, i=index: self._add(i, m))

        def registerCallback(self, callback):
            self.callbacks.append(callback)

        @staticmethod
        def _stamp(message):
            return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

        def _add(self, index, message):
            self.queues[index].append(message)
            if not all(self.queues):
                return
            pivot = self._stamp(message)
            chosen = [min(q, key=lambda m: abs(self._stamp(m) - pivot)) for q in self.queues]
            stamps = [self._stamp(m) for m in chosen]
            if max(stamps) - min(stamps) <= self.slop_ns:
                for q, m in zip(self.queues, chosen):
                    while q and self._stamp(q[0]) <= self._stamp(m):
                        q.popleft()
                for callback in self.callbacks:
                    callback(*chosen)

    class Buffer:
        def lookup_transform(self, target, source, at, timeout=None):
            if (target, source) not in bus.tf:
                raise LookupError(f'no transform {target} <- {source}')
            return copy.deepcopy(bus.tf[(target, source)])

    class CvBridge:
        _types = {'mono8': ('uint8', 1), '16UC1': ('uint16', 1), '32FC1': ('float32', 1)}

        def encoding_to_dtype_with_channels(self, encoding):
            return self._types[encoding]

        def imgmsg_to_cv2(self, message, desired_encoding='passthrough'):
            dtype, _ = self._types[message.encoding]
            return np.frombuffer(bytes(message.data), dtype=dtype).reshape(message.height, message.width)

    def create_cloud_xyz32(header, points):
        rows = np.ascontiguousarray(points, dtype=np.float32).reshape(-1, 3)
        return PointCloud2(header=copy.deepcopy(header), width=len(rows), height=1,
                           fields=[PointField(name=n, offset=4 * i) for i, n in enumerate('xyz')],
                           is_bigendian=sys.byteorder != 'little', point_step=12, row_step=12 * len(rows),
                           data=array('B', rows.tobytes()), is_dense=False)

    def module(name, **members):
        result = types.ModuleType(name)
        result.__dict__.update(members)
        return result

    px4 = {name: globals()[name] for name in (
        'VehicleStatus', 'VehicleLocalPosition', 'VehicleAttitude', 'VehicleLandDetected', 'BatteryStatus',
        'OffboardControlMode', 'TrajectorySetpoint', 'VehicleCommand', 'VehicleCommandAck', 'VehicleOdometry',
        'DistanceSensor')}
    qos = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, VOLATILE=1, TRANSIENT_LOCAL=2, KEEP_LAST=1)
    return {
        'rclpy': module('rclpy', init=lambda args=None: None, shutdown=lambda: None, spin=lambda node: None),
        'rclpy.node': module('rclpy.node', Node=Node),
        'rclpy.time': module('rclpy.time', Time=types.SimpleNamespace(from_msg=copy.deepcopy)),
        'rclpy.duration': module('rclpy.duration', Duration=lambda **kw: types.SimpleNamespace(**kw)),
        'rclpy.qos': module('rclpy.qos', QoSProfile=lambda **kw: kw, ReliabilityPolicy=qos,
                            DurabilityPolicy=qos, HistoryPolicy=qos, QoSReliabilityPolicy=qos,
                            QoSDurabilityPolicy=qos, QoSHistoryPolicy=qos, qos_profile_sensor_data='sensor'),
        'rcl_interfaces': module('rcl_interfaces'),
        'rcl_interfaces.msg': module('rcl_interfaces.msg', ParameterDescriptor=ParameterDescriptor),
        'px4_msgs': module('px4_msgs'), 'px4_msgs.msg': module('px4_msgs.msg', **px4),
        'geometry_msgs': module('geometry_msgs'),
        'geometry_msgs.msg': module('geometry_msgs.msg', PointStamped=PointStamped, Point=Point,
                                    PoseStamped=PoseStamped, TwistStamped=TwistStamped),
        'nav_msgs': module('nav_msgs'), 'nav_msgs.msg': module('nav_msgs.msg', Odometry=Odometry),
        'sensor_msgs': module('sensor_msgs'),
        'sensor_msgs.msg': module('sensor_msgs.msg', Image=Image, CameraInfo=CameraInfo, PointCloud2=PointCloud2),
        'sensor_msgs_py': module('sensor_msgs_py'),
        'sensor_msgs_py.point_cloud2': module('sensor_msgs_py.point_cloud2', create_cloud_xyz32=create_cloud_xyz32),
        'std_msgs': module('std_msgs'),
        'std_msgs.msg': module('std_msgs.msg', String=String, Bool=Bool, Header=Header, Float32=Float32),
        'diagnostic_msgs': module('diagnostic_msgs'),
        'diagnostic_msgs.msg': module('diagnostic_msgs.msg', DiagnosticArray=DiagnosticArray,
                                      DiagnosticStatus=DiagnosticStatus, KeyValue=KeyValue),
        'std_srvs': module('std_srvs'), 'std_srvs.srv': module('std_srvs.srv', Trigger=Trigger),
        'cv_bridge': module('cv_bridge', CvBridge=CvBridge),
        'message_filters': module('message_filters', Subscriber=Subscriber,
                                  ApproximateTimeSynchronizer=ApproximateTimeSynchronizer),
        'tf2_ros': module('tf2_ros', Buffer=Buffer, TransformListener=lambda *a, **k: None,
                          TransformException=LookupError),
        'ament_index_python': module('ament_index_python'),
        'ament_index_python.packages': module('ament_index_python.packages',
                                              get_package_share_directory=lambda name: PX4_MSGS_SHARE
                                              if name == 'px4_msgs' else bus.share_dir),
    }


@contextlib.contextmanager
def installed(bus, share_dir):
    """Patch sys.modules and the nodes' monotonic clocks for the block (the
    old race stack's nodes, where they are installed; goalnav needs none)."""
    import importlib
    nodes = []
    for name in ('drone_ref.mission_node', 'drone_ref.vision_node'):
        try:
            nodes.append(importlib.import_module(name))
        except ImportError:
            pass
    bus.share_dir = str(share_dir)
    clock = lambda: bus.seconds + 1000.   # Monotonic, offset from ROS time.
    with contextlib.ExitStack() as stack:
        stack.enter_context(mock.patch.dict(sys.modules, _modules(bus)))
        for node in nodes:
            stack.enter_context(mock.patch.object(node.time, 'monotonic', side_effect=clock))
        yield
