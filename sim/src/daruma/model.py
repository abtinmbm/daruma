"""Equations of motion of the planar two-wheeled inverted pendulum driven by two DC gearmotors.

Sign convention (used everywhere, firmware included):
    x      wheel position along the floor, positive = forward            [m]
    theta  body tilt from vertical, positive = leaning forward           [rad]
    V      motor voltage, positive drives the wheels forward             [V]

Derivation (Lagrange, no wheel slip, no friction other than the motor's own back-EMF):
    body COM at (x + l sin(theta), r + l cos(theta)); wheels roll so phi = x / r.
    The motors apply torque tau to the wheels and -tau to the body, so the generalised forces are
    Q_x = tau / r and Q_theta = -tau. This gives

    (m_w + J_w/r^2 + m_b) x''  + m_b l cos(theta) theta''   = tau/r + m_b l sin(theta) theta'^2
    m_b l cos(theta) x''       + (J_b + m_b l^2) theta''    = m_b g l sin(theta) - tau

Motor model (both motors together, inductance neglected because its time constant is ~1 ms):
    omega_rel = x'/r - theta'   (wheel speed relative to the body, what the encoders measure)
    i         = (V_eff - k_e omega_rel) / R
    tau       = 2 k_t i
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
from numpy.typing import NDArray

from daruma.params import RobotParams

State = NDArray[np.float64]  # [x, x_dot, theta, theta_dot]


def effective_voltage(v_cmd: float, p: RobotParams) -> float:
    """Applies battery saturation and the motor deadband to a commanded voltage."""
    v = max(-p.v_battery, min(p.v_battery, v_cmd))
    if abs(v) <= p.motor_deadband_v:
        return 0.0
    return v - math.copysign(p.motor_deadband_v, v)


def motor_torque(v_eff: float, x_dot: float, theta_dot: float, p: RobotParams) -> float:
    """Total wheel torque from both motors for an effective voltage and the current speeds."""
    omega_rel = x_dot / p.r_wheel - theta_dot
    current = (v_eff - p.motor_ke * omega_rel) / p.motor_r
    return 2.0 * p.motor_kt * current


def accelerations(state: State, tau: float, p: RobotParams) -> tuple[float, float]:
    """Solves the 2x2 mass-matrix system for (x'', theta'')."""
    _, _, theta, theta_dot = state
    s, c = math.sin(theta), math.cos(theta)
    a11 = p.m_wheels + p.j_wheels / p.r_wheel**2 + p.m_body
    a12 = p.m_body * p.l_com * c
    a22 = p.j_body + p.m_body * p.l_com**2
    b1 = tau / p.r_wheel + p.m_body * p.l_com * s * theta_dot**2
    b2 = p.m_body * p.g * p.l_com * s - tau
    det = a11 * a22 - a12 * a12
    x_ddot = (b1 * a22 - a12 * b2) / det
    theta_ddot = (a11 * b2 - a12 * b1) / det
    return x_ddot, theta_ddot


def derivative(state: State, v_cmd: float, p: RobotParams) -> State:
    """Time derivative of the state for a commanded motor voltage (held constant over the step)."""
    _, x_dot, _, theta_dot = state
    tau = motor_torque(effective_voltage(v_cmd, p), x_dot, theta_dot, p)
    x_ddot, theta_ddot = accelerations(state, tau, p)
    return np.array([x_dot, x_ddot, theta_dot, theta_ddot])


def linearise(p: RobotParams) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Linear model x' = A x + B V about upright and stationary, ignoring the deadband.

    Computed by central finite differences of `derivative`, with the deadband removed so the
    model is smooth at V = 0. Used for LQR design and for checking the hand-derived physics.
    """
    smooth = replace(p, motor_deadband_v=0.0)
    eps = 1e-6
    x0 = np.zeros(4)
    a = np.zeros((4, 4))
    for j in range(4):
        dx = np.zeros(4)
        dx[j] = eps
        a[:, j] = (derivative(x0 + dx, 0.0, smooth) - derivative(x0 - dx, 0.0, smooth)) / (2 * eps)
    b = ((derivative(x0, eps, smooth) - derivative(x0, -eps, smooth)) / (2 * eps)).reshape(4, 1)
    return a, b
