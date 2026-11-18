"""Physics checks: the model must agree with independent hand calculations, not just run."""

from __future__ import annotations

import math

import numpy as np
import pytest

from daruma.model import accelerations, derivative, effective_voltage, linearise
from daruma.params import RobotParams

P = RobotParams()


def test_upright_at_rest_is_an_equilibrium() -> None:
    assert np.allclose(derivative(np.zeros(4), 0.0, P), 0.0)


def test_linearisation_matches_hand_derived_matrices() -> None:
    # Independent derivation at theta = 0: M q'' = [tau/r, m g l theta - tau],
    # tau = cv V - cw (x'/r - theta'), cv = 2 kt / R, cw = 2 kt ke / R.
    m_b, lc, r = P.m_body, P.l_com, P.r_wheel
    mass = np.array([[P.m_wheels + P.j_wheels / r**2 + m_b, m_b * lc], [m_b * lc, P.j_body + m_b * lc**2]])
    cv = 2 * P.motor_kt / P.motor_r
    cw = 2 * P.motor_kt * P.motor_ke / P.motor_r
    # Generalised force as a linear function of [x, x', theta, theta', V].
    tau = np.array([0.0, -cw / r, 0.0, cw, cv])
    forces = np.vstack([tau / r, np.array([0.0, 0.0, m_b * P.g * lc, 0.0, 0.0]) - tau])
    acc = np.linalg.solve(mass, forces)
    a_hand = np.array([[0, 1, 0, 0], acc[0, :4], [0, 0, 0, 1], acc[1, :4]])
    b_hand = np.array([0.0, acc[0, 4], 0.0, acc[1, 4]])

    a, b = linearise(P)
    assert np.allclose(a, a_hand, rtol=1e-5, atol=1e-6)
    assert np.allclose(b.ravel(), b_hand, rtol=1e-5, atol=1e-6)


def test_open_loop_has_one_unstable_pole_like_a_pendulum() -> None:
    a, _ = linearise(P)
    eig = np.linalg.eigvals(a)
    unstable = eig[eig.real > 1e-6]
    assert len(unstable) == 1
    # Upper bound: a free pendulum on a frictionless cart; motor damping can only slow the fall.
    assert 0.5 * math.sqrt(P.g / P.l_com) < unstable[0].real < 2 * math.sqrt(P.g / P.l_com)


def test_unpowered_robot_falls_forward_and_wheels_roll_back() -> None:
    _, theta_ddot = accelerations(np.array([0.0, 0.0, 0.05, 0.0]), 0.0, P)
    x_ddot, _ = accelerations(np.array([0.0, 0.0, 0.05, 0.0]), 0.0, P)
    assert theta_ddot > 0  # forward tilt grows
    assert x_ddot < 0  # reaction pushes the wheels backward


def test_positive_voltage_drives_wheels_forward_and_tips_body_back() -> None:
    d = derivative(np.zeros(4), 5.0, P)
    assert d[1] > 0
    assert d[3] < 0


@pytest.mark.parametrize(("v_cmd", "expected"), [(0.3, 0.0), (-0.3, 0.0), (5.0, 4.6), (-5.0, -4.6), (50.0, 9.2)])
def test_effective_voltage_applies_deadband_and_saturation(v_cmd: float, expected: float) -> None:
    assert effective_voltage(v_cmd, P) == pytest.approx(expected)


def test_no_load_speed_is_plausible_for_a_333_rpm_motor_on_96_v() -> None:
    assert P.no_load_speed_rpm == pytest.approx(333 * 9.6 / 12, rel=0.01)
