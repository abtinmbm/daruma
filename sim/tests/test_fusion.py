"""Physics of the simulated IMU and behaviour of the complementary filter (Python reference)."""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest

from daruma.fusion import ComplementaryFilter, accel_angle
from daruma.imu import quantise, read_imu, specific_force
from daruma.params import RobotParams

P = RobotParams()
DT = P.dt
QUIET = replace(P, accel_noise_g=0.0, rate_noise_rad_s=0.0, gyro_bias_rad_s=0.0)


@pytest.mark.parametrize("theta_deg", [-30.0, -5.0, 0.0, 12.0, 40.0])
def test_accelerometer_at_rest_reads_gravity_in_body_axes(theta_deg: float) -> None:
    th = math.radians(theta_deg)
    f_fwd, f_up = specific_force(np.array([0.0, 0.0, th, 0.0]), 0.0, 0.0, P)
    assert f_fwd == pytest.approx(-P.g * math.sin(th), abs=1e-12)
    assert f_up == pytest.approx(P.g * math.cos(th), abs=1e-12)
    assert accel_angle(f_fwd, f_up) == pytest.approx(th, abs=1e-6)


def test_forward_acceleration_tilts_the_accelerometer_angle_backwards() -> None:
    """Upright, accelerating forward at a: the IMU reads (a, g), so the accel-only angle is -atan(a/g)."""
    a = 2.0
    f_fwd, f_up = specific_force(np.zeros(4), a, 0.0, P)
    assert (f_fwd, f_up) == pytest.approx((a, P.g))
    assert accel_angle(f_fwd, f_up) == pytest.approx(-math.atan(a / P.g), abs=1e-6)


def test_centripetal_acceleration_towards_the_axle_lowers_the_up_reading() -> None:
    """Spinning at w about the axle, upright: the IMU accelerates h w^2 down towards the axle, so it reads g - h w^2."""
    w = 3.0
    f_fwd, f_up = specific_force(np.array([0.0, 0.0, 0.0, w]), 0.0, 0.0, P)
    assert f_fwd == pytest.approx(0.0, abs=1e-12)
    assert f_up == pytest.approx(P.g - P.imu_height * w**2)


def test_quantise_rounds_to_counts_and_clips_at_full_scale() -> None:
    assert quantise(0.30, 0.1, 4.0) == pytest.approx(0.3)
    assert quantise(0.34, 0.1, 4.0) == pytest.approx(0.3)
    assert quantise(9.0, 0.1, 4.0) == pytest.approx(4.0)
    assert quantise(-9.0, 0.1, 4.0) == pytest.approx(-4.0)


def test_filter_with_constant_gyro_bias_settles_at_tau_times_bias() -> None:
    """Closed form: the fixed point of the update is angle = acc + alpha/(1-alpha) b dt = acc + tau b."""
    tau, bias, true = 0.5, math.radians(1.0), math.radians(5.0)
    f = ComplementaryFilter(tau, DT, angle0=true)
    for _ in range(int(10 * tau / DT)):
        est = f.update(bias, true)
    assert est - true == pytest.approx(tau * bias, rel=1e-3)


def test_filter_tracks_a_rotation_from_the_gyro_alone() -> None:
    """With the accelerometer stuck at 0, a fast rotation is followed by the gyro, then decays with tau."""
    f = ComplementaryFilter(1.0, DT)
    w, steps = math.radians(90.0), int(0.1 / DT)  # 41 samples, 8.87 deg of true rotation
    for _ in range(steps):
        est = f.update(w, 0.0)
    expected = w * DT * sum(f.alpha ** (k + 1) for k in range(steps))  # exact sum of the recursion
    assert est == pytest.approx(expected, rel=1e-4)
    assert est > 0.94 * w * steps * DT  # over 0.1 s with tau = 1 s the leak is only ~5 %


def test_filter_attenuates_an_acceleration_burst() -> None:
    """A 0.1 s forward burst at 2 m/s^2 fools the raw accel angle by ~11.5 deg; the filter by far less."""
    tau = 0.5
    f = ComplementaryFilter(tau, DT)
    raw_peak = est_peak = 0.0
    for k in range(int(1.0 / DT)):
        x_ddot = 2.0 if k * DT < 0.1 else 0.0
        imu = read_imu(np.zeros(4), x_ddot, 0.0, QUIET, np.random.default_rng(k))
        acc = accel_angle(imu.accel_fwd_g, imu.accel_up_g)
        raw_peak = max(raw_peak, abs(acc))
        est_peak = max(est_peak, abs(f.update(imu.gyro_rad_s, acc)))
    first_order_bound = (1 - math.exp(-0.1 / tau)) * raw_peak  # a first-order low-pass after 0.1 s
    assert math.degrees(raw_peak) == pytest.approx(math.degrees(math.atan(2.0 / P.g)), abs=0.01)
    assert est_peak <= first_order_bound * 1.01
    assert math.degrees(est_peak) < 2.5


def test_noisy_static_estimate_is_unbiased_apart_from_tau_b() -> None:
    """Held still at 5 deg with the default noise and bias: mean error is tau*b, scatter well below 1 deg."""
    rng = np.random.default_rng(1)
    tau, true = 0.5, math.radians(5.0)
    state = np.array([0.0, 0.0, true, 0.0])
    f = ComplementaryFilter(tau, DT, angle0=true)
    errs = []
    for k in range(int(8.0 / DT)):
        imu = read_imu(state, 0.0, 0.0, P, rng)
        est = f.update(imu.gyro_rad_s, accel_angle(imu.accel_fwd_g, imu.accel_up_g))
        if k * DT > 3.0:
            errs.append(est - true)
    errs = np.array(errs)
    assert errs.mean() == pytest.approx(tau * P.gyro_bias_rad_s, abs=math.radians(0.05))
    assert math.degrees(errs.std()) < 0.3
