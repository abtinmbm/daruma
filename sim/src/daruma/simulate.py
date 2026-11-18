"""Closed-loop simulation: continuous plant, discrete controller, realistic sensing.

The plant is integrated with fixed-step RK4 (4 sub-steps per control period) so runs are fast and
bit-for-bit repeatable. Sensing mirrors the firmware: tilt and gyro rate with Gaussian noise, wheel
speed from quantised encoder counts differenced over one period, and a one-period actuation delay
(the voltage computed from sample k is applied during period k+1).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from daruma.controllers import CascadeController, CascadeGains
from daruma.model import State, derivative
from daruma.params import RobotParams

SUBSTEPS = 4


@dataclass
class SimResult:
    t: NDArray[np.float64]
    state: NDArray[np.float64]  # shape (n, 4): x, x_dot, theta, theta_dot
    voltage: NDArray[np.float64]

    @property
    def theta_deg(self) -> NDArray[np.float64]:
        return np.degrees(self.state[:, 2])

    def settling_time(self, band_deg: float = 1.0) -> float:
        """First time after which |theta| stays inside the band for the rest of the run (inf if never)."""
        outside = np.nonzero(np.abs(self.theta_deg) > band_deg)[0]
        if len(outside) == 0:
            return 0.0
        last = outside[-1]
        return math.inf if last == len(self.t) - 1 else float(self.t[last + 1])

    @property
    def fell(self) -> bool:
        return bool(np.any(np.abs(self.state[:, 2]) > math.radians(45.0)))


def rk4_step(state: State, v_cmd: float, h: float, p: RobotParams) -> State:
    k1 = derivative(state, v_cmd, p)
    k2 = derivative(state + 0.5 * h * k1, v_cmd, p)
    k3 = derivative(state + 0.5 * h * k2, v_cmd, p)
    k4 = derivative(state + h * k3, v_cmd, p)
    return state + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def encoder_counts(state: State, p: RobotParams) -> int:
    """Quantised wheel angle relative to the body (what the quadrature encoders count)."""
    x, _, theta, _ = state
    rel_angle = x / p.r_wheel - theta
    return math.floor(rel_angle / (2 * math.pi) * p.encoder_counts_per_rev)


def simulate(
    p: RobotParams,
    gains: CascadeGains,
    theta0_deg: float = 10.0,
    duration_s: float = 5.0,
    speed_ref: float = 0.0,
    seed: int = 0,
    push_impulse_ns: float = 0.0,
    push_time_s: float = 1.0,
) -> SimResult:
    """Runs one closed-loop experiment from rest at an initial tilt.

    push_impulse_ns is a horizontal impulse (N s) applied to the body COM at push_time_s, modelled as
    an instant change of velocity, for push-recovery studies.
    """
    rng = np.random.default_rng(seed)
    dt = p.dt
    n = int(round(duration_s / dt))
    ctrl = CascadeController(gains, p.v_battery)

    state = np.array([0.0, 0.0, math.radians(theta0_deg), 0.0])
    prev_counts = encoder_counts(state, p)
    v_applied = 0.0
    push_step = int(round(push_time_s / dt)) if push_impulse_ns else -1

    t_log = np.empty(n)
    s_log = np.empty((n, 4))
    v_log = np.empty(n)
    for k in range(n):
        if k == push_step:
            state = apply_push(state, push_impulse_ns, p)
        # Sample sensors at the start of the period.
        theta_meas = state[2] + rng.normal(0.0, p.angle_noise_rad)
        rate_meas = state[3] + rng.normal(0.0, p.rate_noise_rad_s)
        counts = encoder_counts(state, p)
        wheel_rel_rate = (counts - prev_counts) / p.encoder_counts_per_rev * 2 * math.pi / dt
        prev_counts = counts
        speed_meas = (wheel_rel_rate + rate_meas) * p.r_wheel  # ground speed = r (phi_rel' + theta')
        v_next = ctrl.update(theta_meas, rate_meas, speed_meas, speed_ref, dt)

        t_log[k], s_log[k], v_log[k] = k * dt, state, v_applied
        for _ in range(SUBSTEPS):
            state = rk4_step(state, v_applied, dt / SUBSTEPS, p)
        v_applied = v_next  # one-period actuation delay
        if abs(state[2]) > math.pi / 2:  # lying on the floor: stop integrating
            t_log, s_log, v_log = t_log[: k + 1], s_log[: k + 1], v_log[: k + 1]
            s_log[-1, 2] = math.copysign(math.pi / 2, state[2])
            break
    return SimResult(t_log, s_log, v_log)


def apply_push(state: State, impulse_ns: float, p: RobotParams) -> State:
    """Instantaneous horizontal impulse at the body COM, solved through the mass matrix.

    An impulse J at the COM gives generalised impulses (J, J l cos(theta)) on (x, theta); the
    velocity jump is M(q)^-1 times that vector.
    """
    _, _, theta, _ = state
    c = math.cos(theta)
    a11 = p.m_wheels + p.j_wheels / p.r_wheel**2 + p.m_body
    a12 = p.m_body * p.l_com * c
    a22 = p.j_body + p.m_body * p.l_com**2
    m = np.array([[a11, a12], [a12, a22]])
    dv = np.linalg.solve(m, np.array([impulse_ns, impulse_ns * p.l_com * c]))
    new = state.copy()
    new[1] += dv[0]
    new[3] += dv[1]
    return new
