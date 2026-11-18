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


@dataclass
class CascadeGains:
    """Gains of the cascaded balance controller (outer speed loop -> inner angle loop)."""

    angle_kp: float = 80.0  # V/rad
    angle_ki: float = 300.0  # V/(rad s)
    angle_kd: float = 3.0  # V/(rad/s)
    speed_kp: float = 0.12  # rad/(m/s)
    speed_ki: float = 0.08  # rad/m
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
