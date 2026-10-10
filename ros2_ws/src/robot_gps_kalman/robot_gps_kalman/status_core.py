from __future__ import annotations
from collections import deque
from dataclasses import dataclass
from math import acos, degrees, hypot, isfinite, sqrt
from typing import Deque, Optional, Sequence, Tuple


def tilt_deg_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    q = (float(x), float(y), float(z), float(w))
    if not all(isfinite(u) for u in q):
        raise ValueError('Non-finite IMU quaternion')
    scale = max(abs(u) for u in q)
    if scale == 0:
        raise ValueError('Zero-length IMU quaternion')
    q = tuple(u / scale for u in q)
    length = sqrt(sum(u * u for u in q))
    if scale * length < 1e-9:
        raise ValueError('Zero-length IMU quaternion')
    x, y, z, w = (u / length for u in q)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return degrees(acos(max(-1.0, min(1.0, up_z))))


@dataclass(frozen=True)
class StatusConfig:
    rotated_enter_deg: float = 110.0
    rotated_exit_deg: float = 95.0 
    rotated_hold_sec: float = 0.30
    rotated_release_sec: float = 0.30
    stop_speed: float = 0.10
    resume_speed: float = 0.20
    stop_hold_sec: float = 0.80
    stop_max_displacement: float = 0.25
    resume_displacement: float = 0.40
    imu_max_age_sec: float = 0.30
    motion_max_age_sec: float = 0.30
    max_gap_sec: float = 2.0

    def __post_init__(self) -> None:
        fields = (self.rotated_enter_deg, self.rotated_exit_deg,self.rotated_hold_sec, self.rotated_release_sec, self.stop_speed, self.resume_speed, self.stop_hold_sec, self.stop_max_displacement, self.resume_displacement, self.imu_max_age_sec, self.motion_max_age_sec, self.max_gap_sec)
        if any((not isfinite(v) or v < 0) for v in fields):
            raise ValueError('StatusConfig: parameters must be nonnegative and finite')
        if not (0 < self.rotated_exit_deg < self.rotated_enter_deg < 180):
            raise ValueError('Require 0 < rotated_exit < rotated_enter < 180')
        if not 0 < self.stop_speed < self.resume_speed:
            raise ValueError('Require 0 < stop_speed < resume_speed')
        if not 0 < self.stop_max_displacement < self.resume_displacement:
            raise ValueError('Require 0 < stop_max_displacement < resume_displacement')
        if any(v <= 0 for v in (self.rotated_hold_sec, self.stop_hold_sec,self.imu_max_age_sec, self.motion_max_age_sec, self.max_gap_sec)):
            raise ValueError('Durations and max ages must be positive')


class RobotStatusEstimator:
    def __init__(self, config: StatusConfig | None = None):
        self.config = config or StatusConfig()
        self.reset()

    def reset(self) -> None:
        self.last_imu_s: Optional[float] = None
        self.last_motion_s: Optional[float] = None
        self.last_tilt_deg: Optional[float] = None
        self.rotated: bool = False
        self._rotated_since: Optional[float] = None
        self._upright_since: Optional[float] = None
        self.stationary: bool = False
        self._low_speed_since: Optional[float] = None
        self._positions: Deque[Tuple[float, float, float]] = deque(maxlen=4096)
        self.last_speed_xy: Optional[float] = None
        self.last_displacement_xy: Optional[float] = None

    def update_imu(self, quaternion: Sequence[float], time_s: float) -> float:
        c = self.config
        t = float(time_s)
        if not isfinite(t):
            raise ValueError('Invalid IMU timestamp')
        if len(quaternion) != 4:
            raise ValueError('Quaternion must be (x,y,z,w)')
        tilt = tilt_deg_from_quaternion(*quaternion)
        if self.last_imu_s is not None:
            dt = t - self.last_imu_s
            if dt < -0.05 or dt > min(c.max_gap_sec, c.imu_max_age_sec):
                self.reset_imu()
            elif dt <= 0:
                return tilt
        self.last_imu_s = t
        self.last_tilt_deg = tilt
        if not self.rotated:
            self._upright_since = None
            if tilt >= c.rotated_enter_deg:
                if self._rotated_since is None:
                    self._rotated_since = t
                elif t - self._rotated_since >= c.rotated_hold_sec:
                    self.rotated = True
                    self._rotated_since = None
            else:
                self._rotated_since = None
        else:
            self._rotated_since = None
            if tilt <= c.rotated_exit_deg:
                if self._upright_since is None:
                    self._upright_since = t
                elif t - self._upright_since >= c.rotated_release_sec:
                    self.rotated = False
                    self._upright_since = None
            else:
                self._upright_since = None
        return tilt

    def reset_imu(self) -> None:
        self.last_imu_s = None
        self.last_tilt_deg = None
        self.rotated = False
        self._rotated_since = None
        self._upright_since = None

    def _reset_motion(self) -> None:
        self.last_motion_s = None
        self.stationary = False
        self._low_speed_since = None
        self._positions.clear()
        self.last_speed_xy = None
        self.last_displacement_xy = None

    def update_motion(self, position: Sequence[float], velocity: Sequence[float], time_s: float) -> None:
        c = self.config
        t = float(time_s)
        p, v = tuple(map(float, position)), tuple(map(float, velocity))
        if not isfinite(t) or len(p) != 3 or len(v) != 3 or not all(map(isfinite, p+v)):
            raise ValueError('Non-finite motion input')
        if self.last_motion_s is not None:
            dt = t - self.last_motion_s
            if dt < -0.05 or dt > min(c.max_gap_sec, c.motion_max_age_sec):
                self._reset_motion()
            elif dt <= 0:
                return
        self.last_motion_s = t
        speed = hypot(v[0], v[1])
        self.last_speed_xy = speed
        self._positions.append((t, p[0], p[1]))
        target = t - c.stop_hold_sec
        while len(self._positions) > 1 and self._positions[1][0] <= target:
            self._positions.popleft()
        displacement = None
        if self._positions and self._positions[0][0] <= target:
            a = self._positions[0]
            b = self._positions[1] if len(self._positions) > 1 else a
            fraction = 0 if b[0] == a[0] else (target - a[0]) / (b[0] - a[0])
            px = a[1] + fraction * (b[1] - a[1])
            py = a[2] + fraction * (b[2] - a[2])
            displacement = hypot(p[0] - px, p[1] - py)
        self.last_displacement_xy = displacement

        if speed >= c.resume_speed or (self.stationary and displacement is not None
                                       and displacement >= c.resume_displacement):
            self.stationary = False
            self._low_speed_since = None
            return
        if speed < c.stop_speed:
            if self._low_speed_since is None:
                self._low_speed_since = t
            if (t - self._low_speed_since >= c.stop_hold_sec
                    and displacement is not None
                    and displacement <= c.stop_max_displacement):
                self.stationary = True
        else:
            self._low_speed_since = None
            if not self.stationary:
                return

    def current_status(self, time_s: float) -> Optional[str]:
        t = float(time_s)
        c = self.config
        if self.last_imu_s is None or self.last_motion_s is None:
            return None
        imu_age, motion_age = t - self.last_imu_s, t - self.last_motion_s
        if not (-0.05 <= imu_age <= c.imu_max_age_sec):
            return None
        if not (-0.05 <= motion_age <= c.motion_max_age_sec):
            return None
        if self.rotated:
            return 'ROTATED'
        if self.stationary:
            return 'MOVEMENT_ON_PLACE'
        if (self._low_speed_since is not None and
                self.last_motion_s - self._low_speed_since < c.stop_hold_sec and
                self.last_displacement_xy is None):
            return None
        return 'ROBOT DRIVING'
