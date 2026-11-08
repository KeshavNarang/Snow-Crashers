"""Evals for driver_params.py: python3 -m unittest -v test_driver_params

The candidate driver configuration is the RUNNING driver's own parameter dump plus a small overlay. These tests
pin the safety invariants: only the pipeline type and stereo.* may change; nothing cuVSLAM depends on (left.*,
right.*, imu.*, driver.*) may change or disappear; and the stereo settings whose defaults would give wrong
geometry on this camera (alignment to the absent RGB camera, a stereo size that disagrees with the rectified
camera_info) must be set explicitly.
"""
import unittest

import yaml

import driver_params as dp

DUMP = """\
/oak:
  ros__parameters:
    driver:
      i_enable_ir: false
      i_publish_tf_from_calibration: true
      i_tf_cam_pos_x: '0.0'
      i_tf_parent_frame: oak_parent_frame
    imu:
      i_acc_freq: 200
      i_gyro_freq: 200
      i_sync_method: LINEAR_INTERPOLATE_ACCEL
    left:
      i_enable_lazy_publisher: false
      i_fps: 30.0
      i_height: 640
      i_publish_topic: true
      i_width: 1024
    pipeline_gen:
      i_enable_imu: true
      i_nn_type: none
      i_pipeline_type: stereo
    right:
      i_enable_lazy_publisher: false
      i_fps: 30.0
      i_height: 640
      i_publish_topic: true
      i_width: 1024
    use_sim_time: false
"""


class Dump(unittest.TestCase):
    def test_parse_and_flatten(self):
        flat = dp.parse_dump(DUMP)
        self.assertEqual(flat['left.i_width'], 1024)
        self.assertEqual(flat['pipeline_gen.i_pipeline_type'], 'stereo')
        self.assertEqual(flat['driver.i_tf_cam_pos_x'], '0.0')           # string stays a string
        self.assertIs(flat['use_sim_time'], False)

    def test_dump_must_be_for_one_node(self):
        with self.assertRaises(ValueError):
            dp.parse_dump('/a:\n  ros__parameters: {x: 1}\n/b:\n  ros__parameters: {y: 2}\n')
        with self.assertRaises(ValueError):
            dp.parse_dump('just: text\n')

    def test_param_file_round_trip(self):
        flat = dp.parse_dump(DUMP)
        text = dp.to_param_file(flat, node='/oak')
        self.assertEqual(dp.parse_dump(text), flat)
        self.assertIn('/oak:', text)
        self.assertIsInstance(yaml.safe_load(text)['/oak']['ros__parameters']['driver']['i_tf_cam_pos_x'], str)


class Overlay(unittest.TestCase):
    def test_default_overlay_is_explicit_about_geometry(self):
        o = dp.depth_overlay()
        self.assertEqual(o['pipeline_gen.i_pipeline_type'], 'depth')
        for key in ('stereo.i_aligned', 'stereo.i_board_socket_id', 'stereo.i_width', 'stereo.i_height',
                    'stereo.i_fps', 'stereo.i_publish_topic'):
            self.assertIn(key, o)
        self.assertIs(o['stereo.i_publish_topic'], True)
        self.assertIsInstance(o['stereo.i_width'], int)
        self.assertIsInstance(o['stereo.i_fps'], float)

    def test_alignment_choices(self):
        self.assertEqual((dp.depth_overlay(align='left')['stereo.i_aligned'],
                          dp.depth_overlay(align='left')['stereo.i_board_socket_id']), (True, 1))
        self.assertEqual((dp.depth_overlay(align='right')['stereo.i_aligned'],
                          dp.depth_overlay(align='right')['stereo.i_board_socket_id']), (True, 2))
        none = dp.depth_overlay(align='none')
        self.assertIs(none['stereo.i_aligned'], False)
        self.assertIn(none['stereo.i_board_socket_id'], (1, 2))           # never CAM_A (0): RGB is not running
        with self.assertRaises(ValueError):
            dp.depth_overlay(align='rgb')

    def test_bad_values_are_refused(self):
        with self.assertRaises(ValueError):
            dp.depth_overlay(size=(640, 480))                  # 4:3 on a 16:10 sensor: CROP would cut the view
        with self.assertRaises(ValueError):
            dp.depth_overlay(preset='TURBO')
        with self.assertRaises(ValueError):
            dp.depth_overlay(fps=0)

    def test_options_pass_through(self):
        o = dp.depth_overlay(size=(1024, 640), preset='ROBOTICS', extended=True, rect=True)
        self.assertEqual((o['stereo.i_width'], o['stereo.i_height']), (1024, 640))
        self.assertEqual(o['stereo.i_depth_preset'], 'ROBOTICS')
        self.assertIs(o['stereo.i_extended_disp'], True)
        self.assertIs(o['stereo.i_left_rect_publish_topic'], True)
        self.assertIs(o['stereo.i_right_rect_publish_topic'], True)


class Invariants(unittest.TestCase):
    def setUp(self):
        self.dump = dp.parse_dump(DUMP)

    def test_good_candidate_passes(self):
        cand = dp.build_candidate(self.dump, dp.depth_overlay())
        self.assertEqual(dp.check_candidate(self.dump, cand), [])
        for k, v in self.dump.items():                         # everything else carried over verbatim
            if k != 'pipeline_gen.i_pipeline_type':
                self.assertEqual(cand[k], v, k)

    def test_vio_critical_change_is_refused(self):
        cand = dp.build_candidate(self.dump, {**dp.depth_overlay(), 'left.i_width': 1280})
        problems = dp.check_candidate(self.dump, cand)
        self.assertTrue(any('left.i_width' in p for p in problems), problems)

    def test_dropped_key_is_refused(self):
        cand = dp.build_candidate(self.dump, dp.depth_overlay())
        del cand['imu.i_acc_freq']
        self.assertTrue(any('imu.i_acc_freq' in p for p in dp.check_candidate(self.dump, cand)))

    def test_missing_explicit_geometry_is_refused(self):
        o = dp.depth_overlay()
        del o['stereo.i_aligned']
        problems = dp.check_candidate(self.dump, dp.build_candidate(self.dump, o))
        self.assertTrue(any('stereo.i_aligned' in p for p in problems), problems)

    def test_pipeline_without_depth_is_refused(self):
        cand = dp.build_candidate(self.dump, {**dp.depth_overlay(), 'pipeline_gen.i_pipeline_type': 'Stereo'})
        self.assertTrue(any('pipeline' in p for p in dp.check_candidate(self.dump, cand)))
        ok = dp.build_candidate(self.dump, {**dp.depth_overlay(), 'pipeline_gen.i_pipeline_type': 'RGBD'})
        self.assertFalse(any('pipeline' in p for p in dp.check_candidate(self.dump, ok)))

    def test_depth_aligned_to_the_absent_rgb_camera_is_refused(self):
        # the driver's own default (i_board_socket_id = CAM_A) in a DEPTH pipeline, where CAM_A never runs
        cand = dp.build_candidate(self.dump, {**dp.depth_overlay(), 'stereo.i_board_socket_id': 0})
        self.assertTrue(any('RGB' in p for p in dp.check_candidate(self.dump, cand)))
        rgbd = dp.build_candidate(self.dump, {**dp.depth_overlay(pipeline='rgbd'), 'stereo.i_board_socket_id': 0})
        self.assertFalse(any('RGB camera' in p for p in dp.check_candidate(self.dump, rgbd)))   # RGBD runs it

    def test_stereo_aspect_must_match_the_cameras(self):
        cand = dp.build_candidate(self.dump, dp.depth_overlay())
        cand['stereo.i_height'] = 480
        self.assertTrue(any('aspect' in p for p in dp.check_candidate(self.dump, cand)))

    def test_rect_outputs_need_the_camera_size(self):
        # rectified outputs take their camera_info size from left/right i_width/i_height (stereo.cpp), so a smaller
        # stereo size would publish images that disagree with their own camera_info
        cand = dp.build_candidate(self.dump, dp.depth_overlay(size=(640, 400), rect=True))
        self.assertTrue(any('rect' in p for p in dp.check_candidate(self.dump, cand)))
        ok = dp.build_candidate(self.dump, dp.depth_overlay(size=(1024, 640), rect=True))
        self.assertEqual(dp.check_candidate(self.dump, ok), [])


if __name__ == '__main__':
    unittest.main()
