"""Closed-loop requirements the controller must meet in simulation before it goes to firmware."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from daruma.controllers import CascadeGains
from daruma.params import RobotParams
from daruma.simulate import simulate

P = RobotParams()
GAINS = CascadeGains()


@pytest.mark.parametrize("seed", range(5))
def test_recovers_from_10_degrees(seed: int) -> None:
    r = simulate(P, GAINS, theta0_deg=10.0, duration_s=5.0, seed=seed)
    assert not r.fell
    assert r.settling_time(band_deg=1.0) < 1.5
    assert abs(r.state[-1, 0]) < 0.2  # comes back near where it started


@pytest.mark.parametrize(("name", "factor"), [(n, f) for n in ("m_body", "l_com", "j_body", "motor_kt") for f in (0.8, 1.2)])
def test_robust_to_20_percent_parameter_error(name: str, factor: float) -> None:
    p = replace(P, **{name: getattr(P, name) * factor})
    r = simulate(p, GAINS, theta0_deg=5.0, duration_s=4.0)
    assert not r.fell
    assert r.settling_time(band_deg=1.0) < 1.5


def test_survives_a_moderate_push() -> None:
    r = simulate(P, GAINS, theta0_deg=0.0, duration_s=5.0, push_impulse_ns=0.5, push_time_s=1.0)
    assert not r.fell
    assert np.abs(r.theta_deg[-int(P.loop_hz) :]).max() < 1.0  # upright again in the final second


def test_simulation_is_deterministic_for_a_seed() -> None:
    a = simulate(P, GAINS, theta0_deg=10.0, duration_s=1.0, seed=3)
    b = simulate(P, GAINS, theta0_deg=10.0, duration_s=1.0, seed=3)
    assert np.array_equal(a.state, b.state)
