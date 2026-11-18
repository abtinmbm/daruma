"""Tilt estimation: complementary filter, written to mirror firmware/components/fusion/comp_filter.c.

    angle_k = alpha (angle_{k-1} + gyro_k dt) + (1 - alpha) accel_angle_k,   alpha = tau / (tau + dt)

The gyro path is integrated (accurate over short times, drifts with bias); the accelerometer angle is
absolute but corrupted whenever the robot accelerates. The filter high-passes the first and low-passes the
second with the same time constant tau, so the two add up to a unity-gain estimate.

The arithmetic is float32 in the same order as the C code, so CI checks it bit for bit. accel_angle() uses
atan2, whose last bit can differ between libm and numpy, so that one function is checked to within 1e-6 rad.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

F32 = np.float32

# Chosen jointly with the PID gains by daruma.tuning; the tuning cost changes little between 0.3 and 0.6 s,
# so the exact value isn't critical.
DEFAULT_TAU_S = 0.4


def accel_angle(accel_fwd: float, accel_up: float) -> float:
    """Tilt from the accelerometer alone (any consistent unit): atan2(-forward, up). Mirrors comp_filter_accel_angle."""
    return float(np.arctan2(-F32(accel_fwd), F32(accel_up), dtype=F32))


@dataclass
class ComplementaryFilter:
    """Mirrors comp_filter_t / comp_filter_init / comp_filter_update."""

    tau_s: float
    dt: float
    angle0: float = 0.0
    alpha: np.float32 = field(init=False)
    angle: np.float32 = field(init=False)

    def __post_init__(self) -> None:
        tau, h = F32(self.tau_s), F32(self.dt)
        self.alpha = F32(tau / (tau + h))
        self.angle = F32(self.angle0)

    def update(self, gyro_rate: float, accel_angle_rad: float) -> float:
        w, acc, h, a = F32(gyro_rate), F32(accel_angle_rad), F32(self.dt), self.alpha
        predicted = self.angle + w * h
        self.angle = F32(a * predicted + (F32(1.0) - a) * acc)
        return float(self.angle)
