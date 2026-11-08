"""The held heading reaches PX4 unchanged: our ENU quaternion through the organizers' own
flight_interface conversion (frames.py) gives back PX4's heading."""
from pathlib import Path
import math
import sys
import unittest

HERE = Path(__file__).resolve().parent
try:
    from skybeta_flight_interface import frames
except ImportError:      # laptop: the drone's source from the repo's backup of skybeta-03
    sys.path.insert(0, str(sorted((HERE.parents[3] / 'jetson-backups').glob(
        'skybeta-03-*/skybeta/src/skybeta_flight_interface'))[-1]))
    from skybeta_flight_interface import frames


def node_helper():
    """overwall_nav.mission_node.yaw_enu_from_ned, without importing rclpy."""
    src = (HERE.parent / 'overwall_nav' / 'mission_node.py').read_text(encoding='utf-8')
    ns = {'math': math}
    exec(src[src.index('def yaw_enu_from_ned'):src.index('def main')], ns)
    return ns['yaw_enu_from_ned']


class YawTest(unittest.TestCase):
    def test_px4_heading_round_trips_through_flight_interface(self):
        yaw_enu_from_ned = node_helper()
        for deg in (-179.0, -90.0, -30.0, 0.0, 0.1, 45.0, 90.0, 135.0, 179.0):
            heading = math.radians(deg)
            yaw = yaw_enu_from_ned(heading)
            quat_xyzw = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))   # as the node builds it
            back = frames.yaw_ned_from_quat_xyzw(quat_xyzw)
            self.assertAlmostEqual(math.remainder(back - heading, 2 * math.pi), 0.0, places=6, msg=deg)


if __name__ == '__main__':
    unittest.main()
