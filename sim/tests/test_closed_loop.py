"""Closed-loop requirements the controller must meet in simulation before it goes to firmware.

All runs use the default sensing: simulated IMU readings fused by the complementary filter, so the
controller sees estimate lag, acceleration-corrupted accelerometer angles and gyro bias, like the robot.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from daruma.controllers import CascadeGains
from daruma.params import RobotParams
from daruma.simulate import simulate
from daruma.tuning import UNCERTAIN, Candidate, max_recovered_push

P = RobotParams()
GAINS = CascadeGains()
# The gains before tuning (hand-picked with ideal sensing), kept as the baseline the tuned ones must beat.
UNTUNED = Candidate(CascadeGains(80.0, 300.0, 3.0, 0.12, 0.08, 0.25), 0.5)


def tail_rms_deg(theta_deg: np.ndarray, n: int) -> float:
    return float(np.sqrt(np.mean(theta_deg[-n:] ** 2)))


@pytest.mark.parametrize("seed", range(5))
def test_recovers_from_10_degrees(seed: int) -> None:
    r = simulate(P, GAINS, theta0_deg=10.0, duration_s=5.0, seed=seed)
    assert not r.fell
    assert r.settling_time(band_deg=1.0) < 1.5
    assert abs(r.state[-1, 0]) < 0.2  # comes back near where it started


PARAM_CASES = [(n, f) for n in UNCERTAIN for f in (0.8, 1.2)] + [("motor_deadband_v", 0.5), ("motor_deadband_v", 2.0)]


@pytest.mark.parametrize(("name", "factor"), PARAM_CASES)
def test_robust_to_parameter_error(name: str, factor: float) -> None:
    """+-20 % on every physical parameter the controller relies on, and the deadband at half / twice the estimate."""
    p = replace(P, **{name: getattr(P, name) * factor})
    r = simulate(p, GAINS, theta0_deg=5.0, duration_s=4.0, seed=1)
    assert not r.fell
    assert tail_rms_deg(r.theta_deg, 2 * int(P.loop_hz)) < 1.0


@pytest.mark.parametrize("v_battery", [8.0, 11.2])
def test_robust_to_battery_voltage(v_battery: float) -> None:
    """Nearly flat (8 x 1.0 V) to freshly charged (8 x 1.4 V) NiMH, with no voltage compensation yet."""
    r = simulate(replace(P, v_battery=v_battery), GAINS, theta0_deg=10.0, duration_s=4.0, seed=2)
    assert not r.fell
    assert r.settling_time(band_deg=1.0) < 1.5


def test_survives_a_moderate_push() -> None:
    r = simulate(P, GAINS, theta0_deg=0.0, duration_s=5.0, push_impulse_ns=0.5, push_time_s=1.0)
    assert not r.fell
    assert np.abs(r.theta_deg[-int(P.loop_hz) :]).max() < 1.0  # upright again in the final second


def test_standing_still_is_quiet() -> None:
    """10 s standing with sensor noise only: small tilt, little wandering, and an honest tilt estimate."""
    r = simulate(P, GAINS, theta0_deg=0.0, duration_s=10.0, seed=4)
    assert tail_rms_deg(r.theta_deg, 5 * int(P.loop_hz)) < 0.75
    assert np.abs(r.state[:, 0]).max() < 0.10
    assert r.estimate_error_rms_deg < 0.5


def test_standing_still_does_not_chatter() -> None:
    """Encoder speed quantisation once caused ~0.9 V RMS step-to-step voltage changes; the 8-period window fixes it."""
    r = simulate(P, GAINS, theta0_deg=0.0, duration_s=6.0, seed=4)
    standing = r.voltage[2 * int(P.loop_hz) :]
    assert np.sqrt(np.mean(np.diff(standing) ** 2)) < 0.3
    one_period = simulate(P, GAINS, theta0_deg=0.0, duration_s=6.0, seed=4, speed_window=1)
    assert np.sqrt(np.mean(np.diff(one_period.voltage[2 * int(P.loop_hz) :]) ** 2)) > 0.5  # the problem is real


@pytest.mark.parametrize("offset_deg", [-2.0, 2.0])
def test_trims_out_a_level_error(offset_deg: float) -> None:
    """IMU mounted 2 deg off (or COM not over the axle): the speed integral must stop the robot rolling away."""
    p = replace(P, imu_mount_offset_rad=np.radians(offset_deg))
    r = simulate(p, GAINS, theta0_deg=0.0, duration_s=10.0, seed=6)
    assert not r.fell
    assert np.abs(r.state[:, 0]).max() < 0.35
    assert np.abs(r.state[-int(P.loop_hz) :, 1]).mean() < 0.01  # stopped by the end


def test_tuned_gains_recover_a_bigger_push_than_untuned() -> None:
    tuned = max_recovered_push(P, Candidate())
    untuned = max_recovered_push(P, UNTUNED)
    assert tuned > 1.1 * untuned, (tuned, untuned)


def test_simulation_is_deterministic_for_a_seed() -> None:
    a = simulate(P, GAINS, theta0_deg=10.0, duration_s=1.0, seed=3)
    b = simulate(P, GAINS, theta0_deg=10.0, duration_s=1.0, seed=3)
    assert np.array_equal(a.state, b.state)
    assert np.array_equal(a.theta_est, b.theta_est)
