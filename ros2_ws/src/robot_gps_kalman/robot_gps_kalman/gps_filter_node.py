from __future__ import annotations
import math
import json
import uuid
from collections import deque
import rclpy
from geometry_msgs.msg import PointStamped, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from .kalman_core import KalmanGPSVelocity
from .status_core import RobotStatusEstimator, StatusConfig
from .telemetry_csv import TelemetryCSVRecorder, finite, imu_angles, ns_from_stamp


class RobotGPSFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_gps_kalman')
        self.declare_parameter('gps_topic', '/RoboMowerRealistic/gps')
        self.declare_parameter('gps_vector_topic', '/robot/gps')
        self.declare_parameter('speed_vector_topic', '/RoboMowerRealistic/gps/speed_vector')
        self.declare_parameter('filtered_topic', '/robot/gps_filtered')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('status_topic', '/robot/status')
        self.declare_parameter('quality_topic', '/robot/sensor_quality')
        self.declare_parameter('rotated_enter_deg', 110.0)
        self.declare_parameter('rotated_exit_deg', 95.0)
        self.declare_parameter('rotated_hold_sec', 0.30)
        self.declare_parameter('rotated_release_sec', 0.30)
        self.declare_parameter('stop_speed', 0.10)
        self.declare_parameter('resume_speed', 0.20)
        self.declare_parameter('stop_hold_sec', 0.80)
        self.declare_parameter('stop_max_displacement', 0.25)
        self.declare_parameter('resume_displacement', 0.40)
        self.declare_parameter('imu_max_age_sec', 0.30)
        self.declare_parameter('motion_max_age_sec', 0.30)
        self.declare_parameter('gps_sigma', 0.60)
        self.declare_parameter('gps_rejection_threshold', 5.0)
        self.declare_parameter('imu_quaternion_norm_tolerance', 0.10)
        self.declare_parameter('accel_sigma', 1.50)
        self.declare_parameter('speed_sigma', 0.15)
        self.declare_parameter('max_speed_age_sec', 0.15)
        self.declare_parameter('max_time_gap_sec', 2.0)
        self.declare_parameter('input_reliability', 'reliable')
        self.declare_parameter('csv_output', '')
        self.declare_parameter('csv_append', True)
        self.declare_parameter('scenario', '')

        def par(name):
            return self.get_parameter(name).value

        self.filter = KalmanGPSVelocity(
            gps_sigma=float(par('gps_sigma')),
            accel_sigma=float(par('accel_sigma')),
            speed_sigma=float(par('speed_sigma')),
            gps_rejection_threshold=float(par('gps_rejection_threshold')),
        )
        self.status_estimator = RobotStatusEstimator(StatusConfig(**{
            name: float(par(name)) for name in (
                'rotated_enter_deg', 'rotated_exit_deg', 'rotated_hold_sec',
                'rotated_release_sec', 'stop_speed', 'resume_speed',
                'stop_hold_sec', 'stop_max_displacement',
                'resume_displacement', 'imu_max_age_sec',
                'motion_max_age_sec'
            )
        }))
        self.max_speed_age_ns = int(float(par('max_speed_age_sec')) * 1e9)
        self.max_time_gap_ns = int(float(par('max_time_gap_sec')) * 1e9)
        if self.max_speed_age_ns <= 0 or self.max_time_gap_ns <= 0:
            raise ValueError('max_speed_age_sec and max_time_gap_sec must be > 0')
        self.imu_norm_tolerance = float(par('imu_quaternion_norm_tolerance'))
        if not math.isfinite(self.imu_norm_tolerance) or not 0 < self.imu_norm_tolerance < 1:
            raise ValueError('imu_quaternion_norm_tolerance must be between 0 and 1')

        reliability_name = str(par('input_reliability')).lower()
        if reliability_name not in ('reliable', 'best_effort'):
            raise ValueError('input_reliability must be reliable or best_effort')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if reliability_name == 'reliable'
                else ReliabilityPolicy.BEST_EFFORT
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=30,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Vector3, str(par('filtered_topic')), output_qos)
        self.status_publisher = self.create_publisher(
            String, str(par('status_topic')), output_qos
        )
        self.quality_publisher = self.create_publisher(
            String, str(par('quality_topic')), output_qos
        )
        self.gps_sub = self.create_subscription(
            PointStamped, str(par('gps_topic')), self._on_gps, qos
        )
        self.gps_vector_sub = self.create_subscription(
            Vector3, str(par('gps_vector_topic')), self._on_gps_vector, qos
        )
        self.velocity_sub = self.create_subscription(
            Vector3, str(par('speed_vector_topic')), self._on_velocity, qos
        )
        self.imu_sub = self.create_subscription(
            Imu, str(par('imu_topic')), self._on_imu, qos
        )
        self.recorder = None
        self.scenario = str(par('scenario')).strip()
        csv_output = str(par('csv_output')).strip()
        if csv_output:
            if not self.scenario:
                raise ValueError('Set -p scenario:=NAME when csv_output is enabled')
            self.recorder = TelemetryCSVRecorder(csv_output, bool(par('csv_append')))
            self.speed_sub = self.create_subscription(
                Float32, '/RoboMowerRealistic/gps/speed', self._on_speed, qos
            )
            self.get_logger().info(
                'Live telemetry CSV: %s (scenario=%s)' % (self.recorder.path, self.scenario)
            )
        self.run_id = uuid.uuid4().hex[:12]
        self.segment_id = 0
        self.segment_start_ns = None
        self.last_csv_status = None
        self.last_imu_message = None
        self.last_imu_stamp_ns = None
        self.last_imu_receive_ns = None
        self.last_imu_input_ns = None
        self.last_scalar_speed = None
        self.last_scalar_speed_ns = None
        self.raw_gps_window = deque(maxlen=4096)
        self.last_filter_time_ns = None
        self.last_velocity = None
        self.last_velocity_time_ns = None
        self.velocity_seq = 0
        self.last_applied_velocity_seq = 0
        self.published_count = 0
        self.gps_only_count = 0
        self.velocity_update_count = 0
        self.last_published_status = None
        self.published_status_count = 0
        self.last_clock_ns = None
        self.pending_clock_messages = deque(maxlen=120)
        self.clock_wait_dropped = 0
        self.quality = {
            name: dict(last_good_ns=None, accepted=0, rejected=0, last_rejection='')
            for name in ('gps', 'imu')
        }
        self.last_quality_states = None
        self.quality_timer = self.create_timer(0.10, self._publish_quality)

        self.get_logger().info(
            'GPS Kalman online: %s + %s -> %s; '
            'sigma_gps=%.2f, sigma_accel=%.2f, sigma_velocity=%.2f; '
            'input QoS=%s'
            % (
                par('gps_topic'), par('speed_vector_topic'), par('filtered_topic'),
                self.filter.gps_sigma, self.filter.accel_sigma,
                self.filter.speed_sigma, reliability_name,
            )
        )

    def _clock_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _sample_is_fresh(self, stamp_ns: int, max_age_sec: float) -> bool:
        now_ns = self._clock_ns()
        return (stamp_ns > 0 and now_ns > 0
                and -50_000_000 <= now_ns-stamp_ns <= max_age_sec * 1e9)

    def _check_clock(self) -> None:
        now_ns = self._clock_ns()
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self._clear('ROS clock moved backwards (bag loop or reset)')
        self.last_clock_ns = now_ns

    def _defer_until_clock(self, callback=None, msg=None) -> bool:
        """Keep stamped samples until ROS time starts, then recheck them in order."""
        if self._clock_ns() <= 0:
            if callback is None or not self.get_parameter('use_sim_time').value:
                return False
            if len(self.pending_clock_messages) == self.pending_clock_messages.maxlen:
                if self.clock_wait_dropped == 0:
                    self.get_logger().warn('Clock wait queue is full; dropping oldest samples')
                self.clock_wait_dropped += 1
            self.pending_clock_messages.append((callback, msg))
            return True
        # Clear before calling handlers, so each queued message is processed once.
        pending = list(self.pending_clock_messages)
        self.pending_clock_messages.clear()
        for handler, sample in pending:
            handler(sample)
        return False

    def _reject(self, sensor: str, reason: str) -> None:
        quality = self.quality[sensor]
        quality['rejected'] += 1
        if quality['last_rejection'] != reason:
            self.get_logger().warn('Rejected %s sample: %s' % (sensor, reason))
        quality['last_rejection'] = reason

    def _accept(self, sensor: str, stamp_ns: int) -> None:
        self.quality[sensor]['last_good_ns'] = stamp_ns
        self.quality[sensor]['accepted'] += 1

    def _quality_report(self) -> dict:
        now_ns = self._clock_ns()
        report = dict(ros_time_ns=now_ns, status_valid=False,
                      clock_wait_dropped=self.clock_wait_dropped)
        for name, max_age in (
                ('gps', self.status_estimator.config.motion_max_age_sec),
                ('imu', self.status_estimator.config.imu_max_age_sec)):
            quality = self.quality[name]
            stamp = quality['last_good_ns']
            age = None if stamp is None else (now_ns-stamp)*1e-9
            if now_ns <= 0:
                state = 'WAITING_CLOCK'
            elif (name == 'imu' and self.last_imu_input_ns is not None
                  and self.status_estimator.last_imu_s is None):
                state = 'INVALID'
            elif stamp is None:
                state = 'MISSING'
            elif -0.05 <= age <= max_age:
                state = 'OK'
            else:
                state = 'STALE'
            report[name] = dict(state=state, age_sec=age,
                                accepted=quality['accepted'], rejected=quality['rejected'],
                                last_rejection=quality['last_rejection'])
        report['gps']['innovation_distance'] = self.filter.last_gps_distance
        report['status_valid'] = self.status_estimator.current_status(now_ns*1e-9) is not None
        return report

    def _publish_quality(self) -> None:
        self._check_clock()
        self._defer_until_clock()
        report = self._quality_report()
        states = (report['gps']['state'], report['imu']['state'])
        if states != self.last_quality_states:
            self.get_logger().info('Sensor quality: GPS=%s, IMU=%s' % states)
            self.last_quality_states = states
        if not report['status_valid']:
            self.last_published_status = None
        self.quality_publisher.publish(String(data=json.dumps(report, allow_nan=False)))

    def _clear(self, why: str) -> None:
        self.filter.reset()
        self.last_filter_time_ns = None
        self.last_velocity = None
        self.last_velocity_time_ns = None
        self.last_applied_velocity_seq = self.velocity_seq
        self.status_estimator.reset()
        self.last_published_status = None
        self.segment_id += 1
        self.segment_start_ns = None
        self.last_csv_status = None
        self.last_imu_message = None
        self.last_imu_stamp_ns = None
        self.last_imu_receive_ns = None
        self.last_imu_input_ns = None
        self.last_scalar_speed = None
        self.last_scalar_speed_ns = None
        self.raw_gps_window.clear()
        self.pending_clock_messages.clear()
        for quality in self.quality.values():
            quality['last_good_ns'] = None
        self.get_logger().warn('Reset Kalman and status detector: %s' % why)

    def _advance(self, stamp_ns: int) -> bool:
        if self.last_filter_time_ns is None:
            self.last_filter_time_ns = stamp_ns
            return True
        delta_ns = stamp_ns - self.last_filter_time_ns
        if delta_ns > self.max_time_gap_ns:
            self._clear('gap in ROS time > max_time_gap_sec')
            self.last_filter_time_ns = stamp_ns
            return True
        if delta_ns > 0 and self.filter.initialized:
            self.filter.predict(delta_ns * 1e-9)
            self.last_filter_time_ns = stamp_ns
        return False

    def _on_imu(self, msg: Imu) -> None:
        self._check_clock()
        if self._defer_until_clock(self._on_imu, msg):
            return
        q = msg.orientation
        stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec))
        if not self._sample_is_fresh(
                stamp_ns, self.status_estimator.config.imu_max_age_sec):
            self._reject('imu', 'invalid, stale or future timestamp')
            return
        if self.last_imu_input_ns is not None and stamp_ns <= self.last_imu_input_ns:
            self._reject('imu', 'duplicate or out-of-order timestamp')
            return
        self.last_imu_input_ns = stamp_ns
        try:
            if msg.orientation_covariance[0] == -1:
                raise ValueError('IMU orientation is unavailable')
            covariance = msg.orientation_covariance
            if (not all(math.isfinite(v) for v in covariance)
                    or any(covariance[i] < 0 for i in (0, 4, 8))):
                raise ValueError('Invalid IMU orientation covariance')
            norm = math.hypot(q.x, q.y, q.z, q.w)
            if not math.isfinite(norm) or abs(norm-1.0) > self.imu_norm_tolerance:
                raise ValueError('IMU quaternion norm is not close to 1')
            self.status_estimator.update_imu(
                (q.x, q.y, q.z, q.w), stamp_ns * 1e-9
            )
            self._accept('imu', stamp_ns)
            if (self.recorder is not None
                    and self.status_estimator.last_imu_s == stamp_ns * 1e-9):
                self.last_imu_message = msg
                self.last_imu_stamp_ns = stamp_ns
                self.last_imu_receive_ns = self._clock_ns()
        except (ValueError, TypeError) as exc:
            self.status_estimator.reset_imu()
            self.last_imu_message = None
            self.last_imu_stamp_ns = None
            self.last_imu_receive_ns = None
            self._reject('imu', str(exc))

    def _on_speed(self, msg: Float32) -> None:
        self._check_clock()
        self._defer_until_clock()
        if self._clock_ns() <= 0:
            return
        value = float(msg.data)
        if math.isfinite(value):
            self.last_scalar_speed = value
            self.last_scalar_speed_ns = self._clock_ns()

    def _on_velocity(self, msg: Vector3) -> None:
        self._check_clock()
        self._defer_until_clock()
        try:
            xyz = self.filter.validated_xyz((msg.x, msg.y, msg.z))
        except ValueError as exc:
            self.get_logger().warn('Skipped speed_vector: %s' % exc)
            return
        now_ns = self._clock_ns()
        if now_ns <= 0:
            return
        self.last_velocity = xyz
        self.last_velocity_time_ns = now_ns
        self.velocity_seq += 1

    def _on_gps_vector(self, msg: Vector3) -> None:
        self._check_clock()
        self._defer_until_clock()
        converted = PointStamped()
        stamp_ns = self._clock_ns()
        if stamp_ns <= 0:
            return
        sec, nanosec = divmod(stamp_ns, 1_000_000_000)
        converted.header.stamp.sec = sec
        converted.header.stamp.nanosec = nanosec
        converted.point.x, converted.point.y, converted.point.z = msg.x, msg.y, msg.z
        self._on_gps(converted)

    def _on_gps(self, msg: PointStamped) -> None:
        self._check_clock()
        if self._defer_until_clock(self._on_gps, msg):
            return
        try:
            gps = self.filter.validated_xyz((msg.point.x, msg.point.y, msg.point.z))
        except ValueError as exc:
            self._reject('gps', str(exc))
            return
        header_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        stamp_ns = header_stamp_ns
        if not self._sample_is_fresh(
                stamp_ns, self.status_estimator.config.motion_max_age_sec):
            self._reject('gps', 'invalid, stale or future timestamp')
            return
        if self.last_filter_time_ns is not None and stamp_ns <= self.last_filter_time_ns:
            self._reject('gps', 'duplicate or out-of-order timestamp')
            return
        self._advance(stamp_ns)
        if self.segment_start_ns is None:
            self.segment_start_ns = stamp_ns
        if self.recorder is not None:
            self.raw_gps_window.append((stamp_ns, gps[0], gps[1]))
            target_ns = stamp_ns - 1_000_000_000
            while (len(self.raw_gps_window) > 1
                   and self.raw_gps_window[1][0] <= target_ns):
                self.raw_gps_window.popleft()
        fresh_velocity = None
        if (self.last_velocity is not None and self.last_velocity_time_ns is not None and self.velocity_seq > self.last_applied_velocity_seq and abs(stamp_ns - self.last_velocity_time_ns) <= self.max_speed_age_ns):
            fresh_velocity = self.last_velocity
        if not self.filter.initialized:
            self.filter.initialize(gps, fresh_velocity)
            if fresh_velocity is not None:
                self.last_applied_velocity_seq = self.velocity_seq
                self.velocity_update_count += 1
        else:
            accepted = self.filter.update_gps(gps)
            if fresh_velocity is not None:
                self.filter.update_velocity(fresh_velocity)
                self.last_applied_velocity_seq = self.velocity_seq
                self.velocity_update_count += 1
            if not accepted:
                self._reject('gps', 'innovation exceeds gps_rejection_threshold')
                return

        self._accept('gps', stamp_ns)

        filtered = self.filter.position
        velocity = self.filter.velocity
        self.status_estimator.update_motion(filtered, velocity, stamp_ns * 1e-9)
        self.publisher.publish(
            Vector3(x=float(filtered[0]), y=float(filtered[1]), z=float(filtered[2]))
        )
        self.published_count += 1
        now_ns = self._clock_ns()
        status = self.status_estimator.current_status(now_ns * 1e-9) if now_ns > 0 else None
        if status is not None:
            self.status_publisher.publish(String(data=status))
            self.published_status_count += 1
            if status != self.last_published_status:
                self.get_logger().info(
                    'State: %s; tilt=%.1f deg; filtered_speed_xy=%.3f; '
                    'filtered_displacement_xy=%s' % (
                        status, self.status_estimator.last_tilt_deg,
                        self.status_estimator.last_speed_xy,
                        ('%.3f' % self.status_estimator.last_displacement_xy
                         if self.status_estimator.last_displacement_xy is not None
                         else 'pending'),
                    )
                )
                self.last_published_status = status
        if self.recorder is not None:
            self._record_row(msg, stamp_ns, gps, filtered, velocity, status)
        if fresh_velocity is None:
            self.gps_only_count += 1
        if self.published_count == 1:
            self.get_logger().info('First filtered GPS published (frame=%s)' % msg.header.frame_id)
        if self.published_count % 500 == 0:
            self.get_logger().info(
                'Published %d filtered GPS positions; velocity updates=%d'
                % (self.published_count, self.velocity_update_count)
            )

    def _record_row(self, msg, stamp_ns, gps, filtered, velocity, status):
        row = dict(
            scenario=self.scenario, run_id=self.run_id, segment_id=self.segment_id,
            t_s=round((stamp_ns - self.segment_start_ns) * 1e-9, 6),
            gps_header_ns=ns_from_stamp(msg.header.stamp),
            gps_receive_ros_ns=self._clock_ns(), gps_frame=msg.header.frame_id,
            gps_x=gps[0], gps_y=gps[1], gps_z=gps[2],
            filtered_x=float(filtered[0]), filtered_y=float(filtered[1]),
            filtered_z=float(filtered[2]),
            filtered_vx=float(velocity[0]), filtered_vy=float(velocity[1]),
            filtered_vz=float(velocity[2]),
            filtered_speed_xy=self.status_estimator.last_speed_xy,
            filtered_displacement_xy_08s=self.status_estimator.last_displacement_xy,
            is_rotated=int(self.status_estimator.rotated),
            is_stationary=int(self.status_estimator.stationary),
            robot_status=status or '',
            status_changed=int(status is not None and status != self.last_csv_status),
            status_valid=int(status is not None),
        )
        if self.raw_gps_window:
            target_ns = stamp_ns - 1_000_000_000
            nearest = min(
                list(self.raw_gps_window)[:2],
                key=lambda item: abs(item[0] - target_ns),
            )
            if abs(nearest[0] - target_ns) <= 120_000_000:
                row['gps_xy_displacement_1s'] = math.hypot(
                    gps[0] - nearest[1], gps[1] - nearest[2]
                )
        if self.last_imu_message is not None:
            imu = self.last_imu_message
            row.update(
                imu_header_ns=self.last_imu_stamp_ns,
                imu_receive_ros_ns=self.last_imu_receive_ns,
                imu_time_diff_ms=round((self.last_imu_stamp_ns - stamp_ns) * 1e-6, 3),
                imu_frame=imu.header.frame_id,
                q_x=finite(imu.orientation.x), q_y=finite(imu.orientation.y),
                q_z=finite(imu.orientation.z), q_w=finite(imu.orientation.w),
                angular_x=finite(imu.angular_velocity.x),
                angular_y=finite(imu.angular_velocity.y),
                angular_z=finite(imu.angular_velocity.z),
                accel_x=finite(imu.linear_acceleration.x),
                accel_y=finite(imu.linear_acceleration.y),
                accel_z=finite(imu.linear_acceleration.z),
            )
            row.update(imu_angles(imu.orientation))
        if self.last_scalar_speed is not None:
            row.update(
                speed_receive_ros_ns=self.last_scalar_speed_ns,
                speed_time_diff_ms=round((self.last_scalar_speed_ns - stamp_ns) * 1e-6, 3),
                speed=self.last_scalar_speed,
            )
        if self.last_velocity is not None:
            row.update(
                speed_vector_receive_ros_ns=self.last_velocity_time_ns,
                speed_vector_time_diff_ms=round((self.last_velocity_time_ns - stamp_ns) * 1e-6, 3),
                speed_vector_x=self.last_velocity[0],
                speed_vector_y=self.last_velocity[1],
                speed_vector_z=self.last_velocity[2],
            )
        self.recorder.write(row)
        if status is not None:
            self.last_csv_status = status

    def destroy_node(self):
        if self.recorder is not None:
            if rclpy.ok():
                self.get_logger().info(
                    'Live CSV closed: %s (%d rows this run)' % (
                        self.recorder.path, self.recorder.count
                    )
                )
            self.recorder.close()
            self.recorder = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RobotGPSFilterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
