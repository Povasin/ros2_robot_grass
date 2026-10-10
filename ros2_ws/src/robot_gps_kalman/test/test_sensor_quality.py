import math
import json
import time
import unittest

import numpy as np

from robot_gps_kalman.kalman_core import KalmanGPSVelocity
from robot_gps_kalman.status_core import RobotStatusEstimator


class GPSQualityTests(unittest.TestCase):
    def test_gps_spike_cannot_change_prediction_or_covariance(self):
        f = KalmanGPSVelocity()
        f.initialize((0, 0, 0), (1, 0, 0))
        f.predict(.1)
        state, covariance = f.state.copy(), f.covariance.copy()
        accepted = f.update_gps((100, -100, 50))
        np.testing.assert_array_equal(f.state, state)
        np.testing.assert_array_equal(f.covariance, covariance)
        self.assertIs(accepted, False)

    def test_good_gps_still_corrects_after_rejected_spike(self):
        f = KalmanGPSVelocity()
        f.initialize((0, 0, 0), (0, 0, 0))
        f.update_gps((100, 0, 0))
        self.assertIs(f.update_gps((.1, 0, 0)), True)
        self.assertGreater(f.position[0], 0)
        self.assertLess(f.position[0], .1)

    def test_nonfinite_gps_cannot_partially_correct_state(self):
        f = KalmanGPSVelocity()
        f.initialize((0, 0, 0))
        before = f.state.copy()
        with self.assertRaises(ValueError):
            f.update_gps((1, math.nan, 0))
        np.testing.assert_array_equal(f.state, before)


try:
    import rclpy
    from geometry_msgs.msg import PointStamped, Vector3
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from robot_gps_kalman.gps_filter_node import RobotGPSFilterNode
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS 2 runtime is required for callback tests')
class NodeQualityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.now = 1_000_000_000
        self.node = RobotGPSFilterNode()
        self.node._clock_ns = lambda: self.now

    def tearDown(self):
        self.node.destroy_node()

    def imu(self, stamp=None):
        msg = Imu()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            self.now if stamp is None else stamp, 1_000_000_000)
        msg.orientation.w = 1.
        return msg

    def gps(self, x=0., stamp=None):
        msg = PointStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            self.now if stamp is None else stamp, 1_000_000_000)
        msg.point.x = float(x)
        return msg

    def test_rejected_gps_does_not_publish_or_refresh_motion(self):
        self.node._on_imu(self.imu())
        self.node._on_gps(self.gps())
        self.now += 100_000_000
        self.node._on_imu(self.imu())
        self.node._on_gps(self.gps(100))
        self.assertEqual(self.node.published_count, 1)
        self.assertAlmostEqual(self.node.status_estimator.last_motion_s, 1.)
        self.now += 250_000_000
        self.assertIsNone(self.node.status_estimator.current_status(self.now*1e-9))

    def test_future_gps_is_rejected_even_inside_max_age(self):
        self.node._on_gps(self.gps(stamp=self.now+200_000_000))
        self.assertEqual(self.node.published_count, 0)

    def test_stamped_gps_with_zero_time_is_not_replaced_by_arrival_time(self):
        self.node._on_gps(self.gps(stamp=0))
        self.assertEqual(self.node.published_count, 0)

    def test_old_gps_cannot_reset_filter_while_clock_moves_forward(self):
        self.node._on_gps(self.gps(1))
        self.now += 100_000_000
        self.node._on_gps(self.gps(50, stamp=900_000_000))
        self.assertEqual(self.node.segment_id, 0)
        self.assertEqual(self.node.published_count, 1)
        self.assertAlmostEqual(self.node.filter.position[0], 1.)

    def test_malformed_quaternion_invalidates_previous_imu(self):
        self.node._on_imu(self.imu())
        self.now += 10_000_000
        bad = self.imu()
        bad.orientation.w = 2.
        self.node._on_imu(bad)
        self.assertIsNone(self.node.status_estimator.last_imu_s)
        quality = self.node._quality_report()
        self.assertEqual(quality['imu']['state'], 'INVALID')
        self.assertAlmostEqual(quality['imu']['age_sec'], .01)
        self.assertFalse(quality['status_valid'])
        self.now += 10_000_000
        self.node._on_imu(self.imu())
        quality = self.node._quality_report()
        self.assertEqual(quality['imu']['state'], 'OK')
        self.assertAlmostEqual(quality['imu']['age_sec'], 0.)

    def test_nonfinite_orientation_covariance_is_rejected(self):
        bad = self.imu()
        bad.orientation_covariance[4] = math.nan
        self.node._on_imu(bad)
        self.assertIsNone(self.node.status_estimator.last_imu_s)

    def test_negative_orientation_variance_is_rejected(self):
        bad = self.imu()
        bad.orientation_covariance[4] = -0.1
        self.node._on_imu(bad)
        self.assertIsNone(self.node.status_estimator.last_imu_s)

    def test_future_imu_is_rejected(self):
        self.node._on_imu(self.imu(stamp=self.now+200_000_000))
        self.assertIsNone(self.node.status_estimator.last_imu_s)

    def test_imu_zero_timestamp_is_rejected(self):
        self.node._on_imu(self.imu(stamp=0))
        self.assertIsNone(self.node.status_estimator.last_imu_s)

    def test_missing_imu_stops_new_status_but_gps_still_works(self):
        self.node._on_imu(self.imu())
        self.node._on_velocity(Vector3(x=1.))
        self.node._on_gps(self.gps())
        self.assertEqual(self.node.published_status_count, 1)
        self.now += 400_000_000
        self.node._on_gps(self.gps(.4))
        self.assertEqual(self.node.published_count, 2)
        self.assertEqual(self.node.published_status_count, 1)

    def receive_quality(self, predicate):
        received = []
        probe = Node('sensor_quality_test_probe')
        sub = probe.create_subscription(String, '/robot/sensor_quality',
                                       lambda msg: received.append(json.loads(msg.data)), 10)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        executor.add_node(probe)
        try:
            deadline = time.monotonic()+1.5
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
                for item in received:
                    if predicate(item):
                        return item
            return None
        finally:
            executor.shutdown()
            probe.destroy_node()

    def test_quality_topic_reports_missing_sensors_without_gps_callbacks(self):
        result = self.receive_quality(lambda q: q['gps']['state']=='MISSING'
                                      and q['imu']['state']=='MISSING')
        self.assertIsNotNone(result, 'Missing sensors must be observable on the quality topic')

    def test_quality_topic_reports_outage_without_any_new_messages(self):
        self.node._on_imu(self.imu())
        self.node._on_gps(self.gps())
        self.now += 400_000_000
        result = self.receive_quality(lambda q: q['gps']['state']=='STALE'
                                      and q['imu']['state']=='STALE')
        self.assertIsNotNone(result, 'A timer must detect loss of both input streams')
        self.assertFalse(result['status_valid'])

    def test_clock_restart_accepts_new_timeline_without_old_imu(self):
        self.node._on_imu(self.imu())
        self.node._on_gps(self.gps())
        self.now = 500_000_000
        self.node._on_gps(self.gps(2))
        self.assertEqual(self.node.published_count, 2)
        self.assertEqual(self.node.segment_id, 1)
        self.assertIsNone(self.node.status_estimator.last_imu_s)

    def test_unavailable_orientation_invalidates_previous_imu(self):
        self.node._on_imu(self.imu())
        self.now += 10_000_000
        msg = self.imu()
        msg.orientation_covariance[0] = -1.
        self.node._on_imu(msg)
        self.assertIsNone(self.node.status_estimator.last_imu_s)

    def test_older_valid_imu_cannot_replace_newer_invalid_orientation(self):
        self.node._on_imu(self.imu())
        self.now += 20_000_000
        bad = self.imu()
        bad.orientation.w = 0.
        self.node._on_imu(bad)
        self.node._on_imu(self.imu(stamp=1_010_000_000))
        self.assertIsNone(self.node.status_estimator.last_imu_s)

    def test_small_quaternion_norm_error_and_unknown_covariance_are_allowed(self):
        msg = self.imu()
        msg.orientation.w = .99
        self.node._on_imu(msg)
        self.assertAlmostEqual(self.node.status_estimator.last_tilt_deg, 0.)
        self.assertIsNotNone(self.node.status_estimator.last_imu_s)

    def test_gps_vector_interface_still_accepts_arrival_timestamp(self):
        self.node._on_gps_vector(Vector3(x=1., y=2., z=3.))
        self.assertEqual(self.node.published_count, 1)
        np.testing.assert_array_equal(self.node.filter.position, [1, 2, 3])

    def test_first_gps_waits_for_clock_and_is_published_once(self):
        self.node.set_parameters([rclpy.Parameter('use_sim_time', value=True)])
        self.now = 0
        self.node._on_gps(self.gps(1., stamp=1_000_000_000))
        self.assertEqual(self.node.published_count, 0)
        self.assertEqual(self.node.quality['gps']['rejected'], 0)
        self.now = 1_000_000_000
        self.node._publish_quality()
        self.node._publish_quality()
        self.assertEqual(self.node.published_count, 1)
        self.assertEqual(self.node.quality['gps']['accepted'], 1)

    def test_startup_messages_keep_order_before_next_gps(self):
        self.node.set_parameters([rclpy.Parameter('use_sim_time', value=True)])
        self.now = 0
        self.node._on_imu(self.imu(stamp=990_000_000))
        self.node._on_gps(self.gps(0., stamp=1_000_000_000))
        self.node._on_gps(self.gps(.01, stamp=1_010_000_000))
        self.now = 1_020_000_000
        self.node._on_velocity(Vector3(x=1.))
        self.node._on_gps(self.gps(.02))
        self.assertEqual(self.node.published_count, 3)
        self.assertEqual(self.node.published_status_count, 1)
        self.assertEqual(self.node.quality['gps']['rejected'], 0)
        self.assertAlmostEqual(self.node.status_estimator.last_imu_s, .99)
        self.assertGreater(self.node.filter.velocity[0], .9)

    def test_waiting_for_clock_does_not_make_stale_gps_fresh(self):
        self.node.set_parameters([rclpy.Parameter('use_sim_time', value=True)])
        self.now = 0
        self.node._on_gps(self.gps(stamp=1_000_000_000))
        self.now = 2_000_000_000
        self.node._publish_quality()
        self.assertEqual(self.node.published_count, 0)
        self.assertEqual(self.node.quality['gps']['rejected'], 1)
        self.node._on_gps(self.gps())
        self.assertEqual(self.node.published_count, 1)

    def test_zero_header_is_still_invalid_after_clock_starts(self):
        self.node.set_parameters([rclpy.Parameter('use_sim_time', value=True)])
        self.now = 0
        self.node._on_gps(self.gps(stamp=0))
        self.now = 1_000_000_000
        self.node._publish_quality()
        self.assertEqual(self.node.published_count, 0)
        self.assertEqual(self.node.quality['gps']['rejected'], 1)

    def test_unstamped_inputs_before_clock_are_not_retimed_as_fresh(self):
        self.node.set_parameters([rclpy.Parameter('use_sim_time', value=True)])
        self.now = 0
        self.node._on_velocity(Vector3(x=1.))
        self.node._on_gps_vector(Vector3(x=100.))
        self.now = 1_000_000_000
        self.node._publish_quality()
        self.assertEqual(self.node.published_count, 0)
        self.assertIsNone(self.node.last_velocity_time_ns)
        self.node._on_gps_vector(Vector3(x=1.))
        self.assertEqual(self.node.published_count, 1)
        self.assertAlmostEqual(self.node.filter.position[0], 1.)


if __name__ == '__main__':
    unittest.main()
