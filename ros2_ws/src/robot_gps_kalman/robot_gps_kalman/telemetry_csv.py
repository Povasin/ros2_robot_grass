from __future__ import annotations
import csv
import math
from pathlib import Path

FIELDS = [
    'scenario', 'run_id', 'segment_id', 't_s',
    'gps_header_ns', 'gps_receive_ros_ns', 'gps_frame',
    'gps_x', 'gps_y', 'gps_z', 'gps_xy_displacement_1s',
    'imu_header_ns', 'imu_receive_ros_ns', 'imu_time_diff_ms', 'imu_frame',
    'q_x', 'q_y', 'q_z', 'q_w',
    'roll_deg', 'pitch_deg', 'yaw_deg', 'tilt_deg', 'up_z',
    'angular_x', 'angular_y', 'angular_z', 'accel_x', 'accel_y', 'accel_z',
    'speed_receive_ros_ns', 'speed_time_diff_ms', 'speed',
    'speed_vector_receive_ros_ns', 'speed_vector_time_diff_ms',
    'speed_vector_x', 'speed_vector_y', 'speed_vector_z',
    'filtered_x', 'filtered_y', 'filtered_z',
    'filtered_vx', 'filtered_vy', 'filtered_vz',
    'filtered_speed_xy', 'filtered_displacement_xy_08s',
    'is_rotated', 'is_stationary',
    'robot_status', 'status_changed', 'status_valid',
]

def finite(value):
    if value is None:
        return ''
    number = float(value)
    return number if math.isfinite(number) else ''

def ns_from_stamp(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

def imu_angles(orientation):
    vals = [float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w)]
    if not all(math.isfinite(v) for v in vals):
        return {}
    scale = max(abs(v) for v in vals)
    if scale == 0:
        return {}
    vals = [v / scale for v in vals]
    magnitude = math.sqrt(sum(v * v for v in vals))
    if scale * magnitude < 1e-10:
        return {}
    x, y, z, w = (v / magnitude for v in vals)
    clamp = lambda v: max(-1.0, min(1.0, v))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(clamp(2 * (w * y - z * x)))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    up_z = clamp(1 - 2 * (x * x + y * y))
    return dict(roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch),yaw_deg=math.degrees(yaw), tilt_deg=math.degrees(math.acos(up_z)),up_z=up_z)


class TelemetryCSVRecorder:
    def __init__(self, filename: str, append: bool = True):
        path = Path(filename).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        exists = path.exists() and path.stat().st_size > 0
        if append and exists:
            with path.open('r', encoding='utf-8-sig', newline='') as stream:
                previous_header = next(csv.reader(stream), None)
            if previous_header != FIELDS:
                raise ValueError(f'CSV has incompatible header: {path}. ' 'Choose a new csv_output or set csv_append:=false.')
        self.path = path
        self.stream = path.open(
            'a' if append and exists else 'w',
            newline='', encoding='utf-8' if append and exists else 'utf-8-sig'
        )
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS, extrasaction='raise')
        if not (append and exists):
            self.writer.writeheader()
            self.stream.flush()
        self.count = 0

    def write(self, data: dict):
        row = {key: data.get(key, '') for key in FIELDS}
        for key in ('scenario', 'run_id', 'gps_frame', 'imu_frame', 'robot_status'):
            value = str(row[key])
            if value.lstrip().startswith(('=', '+', '-', '@')) or value[:1] in ('\t', '\r', '\n'):
                row[key] = '_' + value
        self.writer.writerow(row)
        self.stream.flush() 
        self.count += 1

    def close(self):
        if not self.stream.closed:
            self.stream.close()
