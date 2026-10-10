from __future__ import annotations
import math
from typing import Optional
import numpy as np

class KalmanGPSVelocity:
    def __init__(
        self,
        gps_sigma: float = 0.60,
        accel_sigma: float = 1.50,
        speed_sigma: float = 0.15,
        initial_speed_sigma: float = 2.0,
        gps_rejection_threshold: float = 5.0,
    ) -> None:
        for name, value in (
            ('gps_sigma', gps_sigma),
            ('accel_sigma', accel_sigma),
            ('speed_sigma', speed_sigma),
            ('initial_speed_sigma', initial_speed_sigma),
            ('gps_rejection_threshold', gps_rejection_threshold),
        ):
            if not math.isfinite(value) or not 1e-150 <= value <= 1e150:
                raise ValueError(f'{name} must be positive and numerically safe')
        self.gps_sigma = float(gps_sigma)
        self.accel_sigma = float(accel_sigma)
        self.speed_sigma = float(speed_sigma)
        self.initial_speed_sigma = float(initial_speed_sigma)
        self.gps_rejection_threshold = float(gps_rejection_threshold)
        self.reset()

    def reset(self) -> None:
        self.state: Optional[np.ndarray] = None 
        self.covariance: Optional[np.ndarray] = None
        self.last_gps_distance: Optional[float] = None

    @staticmethod
    def validated_xyz(values) -> np.ndarray:
        arr = np.asarray(values, dtype=float)
        if (arr.shape != (3,) or not np.all(np.isfinite(arr))
                or np.any(np.abs(arr) > 1e150)):
            raise ValueError('XYZ measurement must be three numerically safe numbers')
        return arr

    @property
    def initialized(self) -> bool:
        return self.state is not None

    @property
    def position(self) -> np.ndarray:
        if self.state is None:
            raise RuntimeError('Filter has not been initialized')
        return self.state[:, 0].copy()

    @property
    def velocity(self) -> np.ndarray:
        if self.state is None:
            raise RuntimeError('Filter has not been initialized')
        return self.state[:, 1].copy()

    def initialize(self, gps_xyz, velocity_xyz=None) -> None:
        pos = self.validated_xyz(gps_xyz)
        vel = (
            np.zeros(3, dtype=float)
            if velocity_xyz is None
            else self.validated_xyz(velocity_xyz)
        )
        self.state = np.stack((pos, vel), axis=1)
        self.covariance = np.zeros((3, 2, 2), dtype=float)
        self.covariance[:, 0, 0] = self.gps_sigma ** 2
        self.covariance[:, 1, 1] = self.initial_speed_sigma ** 2

    def predict(self, dt_seconds: float) -> None:
        if not self.initialized:
            raise RuntimeError('Filter has not been initialized')
        if not math.isfinite(dt_seconds) or dt_seconds < 0:
            raise ValueError('dt must be finite and nonnegative')
        if dt_seconds == 0:
            return
        F = np.array([[1.0, dt_seconds], [0.0, 1.0]], dtype=float)
        G = np.array([0.5 * dt_seconds**2, dt_seconds], dtype=float)
        Q = self.accel_sigma**2 * np.outer(G, G)
        for axis in range(3):
            self.state[axis] = F @ self.state[axis]
            self.covariance[axis] = F @ self.covariance[axis] @ F.T + Q

    def _correct(self, xyz, H: np.ndarray, sigma: float) -> None:
        if not self.initialized:
            raise RuntimeError('Filter has not been initialized')
        observation = self.validated_xyz(xyz)
        noise_var = sigma**2
        I = np.eye(2)
        for axis in range(3):
            x = self.state[axis]
            P = self.covariance[axis]
            innovation = observation[axis] - float(H @ x)
            S = float(H @ P @ H + noise_var)
            K = (P @ H) / S
            corrected_state = x + K * innovation
            A = I - np.outer(K, H)
            corrected_covariance = A @ P @ A.T + noise_var * np.outer(K, K)
            self.state[axis] = corrected_state
            self.covariance[axis] = 0.5 * (
                corrected_covariance + corrected_covariance.T
            )

    def update_gps(self, gps_xyz) -> bool:
        if not self.initialized:
            raise RuntimeError('Filter has not been initialized')
        observation = self.validated_xyz(gps_xyz)
        residual = observation - self.position
        variance = self.covariance[:, 0, 0] + self.gps_sigma**2
        # Check all three axes before any correction: reject the entire sample.
        distance = float(np.linalg.norm(residual / np.sqrt(variance)))
        self.last_gps_distance = distance if math.isfinite(distance) else None
        if not math.isfinite(distance) or distance > self.gps_rejection_threshold:
            return False
        self._correct(observation, np.array([1.0, 0.0]), self.gps_sigma)
        return True

    def update_velocity(self, velocity_xyz) -> None:
        self._correct(velocity_xyz, np.array([0.0, 1.0]), self.speed_sigma)
