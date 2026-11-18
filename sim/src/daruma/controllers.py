"""Controllers, written to mirror the firmware's C code exactly.

`Pid` reproduces `firmware/components/control/pid.c` operation by operation in float32, so the C
library can be checked against it with golden vectors (see tests/test_c_control.py).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

F32 = np.float32


@dataclass
class Pid:
    """PID with the derivative taken from a measured rate and a clamped integrator (anti-windup).

    out = kp*error + integral + kd*rate,  integral += ki*error*dt  (clamped to +-i_limit),
    out clamped to [out_min, out_max]. Mirrors pid_ctrl_update() in pid.c.
    """

    kp: float
    ki: float
    kd: float
    i_limit: float
    out_min: float
    out_max: float
    integral: np.float32 = field(default=F32(0.0))

    def reset(self) -> None:
        self.integral = F32(0.0)

    def update(self, error: float, rate: float, dt: float) -> float:
        e, r, h = F32(error), F32(rate), F32(dt)
        integral = self.integral + F32(self.ki) * e * h
        integral = min(max(integral, F32(-self.i_limit)), F32(self.i_limit))
        self.integral = F32(integral)
        out = F32(self.kp) * e + self.integral + F32(self.kd) * r
        out = min(max(out, F32(self.out_min)), F32(self.out_max))
        return float(out)


SPEED_WINDOW = 8  # control periods the wheel speed is differenced over (19 ms at 416 Hz)
SPEED_EST_MAX_WINDOW = 32  # matches speed_est.h


class WheelSpeedEstimator:
    """Wheel speed relative to the body from encoder counts, differenced over `window` periods.

    Mirrors speed_est_init / speed_est_update in firmware/components/control/speed_est.c, including the int32
    wrap-around of the count difference and the float32 operation order.
    """

    def __init__(self, window: int, rad_per_count: float, dt: float, counts0: int) -> None:
        self.window = min(max(window, 1), SPEED_EST_MAX_WINDOW)
        self.scale = F32(F32(rad_per_count) / (F32(self.window) * F32(dt)))
        self.history = [counts0] * self.window
        self.head = 0

    def update(self, counts: int) -> float:
        delta = (counts - self.history[self.head] + 2**31) % 2**32 - 2**31  # int32 wrap, like the C code
        self.history[self.head] = counts
        self.head = (self.head + 1) % self.window
        return float(F32(delta) * self.scale)


@dataclass
class CascadeGains:
    """Gains of the cascaded balance controller (outer speed loop -> inner angle loop).

    Defaults come from `python -m daruma.tuning search` (Nelder-Mead over the scenario set in tuning.py,
    IMU + complementary-filter sensing, 8-period speed window), rounded, except speed_ki, which was raised by
    hand from the search's 0.059 to 0.15. The speed integral acts on distance, so a level error delta (IMU
    mounted crooked, centre of mass not over the axle) is trimmed out after rolling about delta / speed_ki;
    the higher value roughly halves that distance for slightly more tilt while standing. Tuned on ESTIMATE
    parameters, so these are a starting point for the robot, not final values.
    """

    angle_kp: float = 67.0  # V/rad
    angle_ki: float = 380.0  # V/(rad s)
    angle_kd: float = 3.4  # V/(rad/s)
    speed_kp: float = 0.39  # rad/(m/s)
    speed_ki: float = 0.15  # rad/m (the speed integral acts on distance: lean correction per metre rolled)
    max_lean: float = 0.25  # rad, limit on the angle setpoint


class CascadeController:
    """Outer loop turns a wheel-speed error into a lean setpoint; inner loop turns lean error into volts.

    To slow down from forward motion the robot must lean back, so the lean setpoint is
    -(speed PI output). The inner loop then (correctly) drives the wheels forward first to
    tip the body back: the non-minimum-phase behaviour of any balancing robot.
    """

    def __init__(self, gains: CascadeGains, v_max: float) -> None:
        self.gains = gains
        self.angle = Pid(gains.angle_kp, gains.angle_ki, gains.angle_kd, v_max, -v_max, v_max)
        self.speed = Pid(gains.speed_kp, gains.speed_ki, 0.0, gains.max_lean, -gains.max_lean, gains.max_lean)

    def update(self, theta: float, theta_rate: float, speed: float, speed_ref: float, dt: float) -> float:
        lean_ref = -self.speed.update(speed - speed_ref, 0.0, dt)
        return self.angle.update(theta - lean_ref, theta_rate, dt)
