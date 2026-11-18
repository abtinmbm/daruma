"""Golden-vector check: firmware/components/fusion must match the Python reference in daruma.fusion.

comp_filter_init and comp_filter_update are pure float32 arithmetic in the same order as the Python code,
built without FMA contraction, so they must be bit-identical. comp_filter_accel_angle calls atan2f, whose
last bit may differ between C libraries and numpy, so it is checked to within ATAN2_TOL_RAD.
"""

from __future__ import annotations

import ctypes

import numpy as np
import pytest

from daruma.controllers import CascadeGains
from daruma.fusion import DEFAULT_TAU_S, ComplementaryFilter, accel_angle
from daruma.params import RobotParams
from daruma.simulate import simulate

ATAN2_TOL_RAD = 1e-6
DT = 1.0 / 416.0


class CompFilter(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("alpha", "dt", "angle")]


@pytest.fixture(scope="module")
def lib(build_c_lib) -> ctypes.CDLL:
    library = build_c_lib("daruma_fusion", ["fusion/comp_filter.c"])
    library.comp_filter_init.argtypes = [ctypes.POINTER(CompFilter), ctypes.c_float, ctypes.c_float, ctypes.c_float]
    library.comp_filter_init.restype = None
    library.comp_filter_accel_angle.argtypes = [ctypes.c_float, ctypes.c_float]
    library.comp_filter_accel_angle.restype = ctypes.c_float
    library.comp_filter_update.argtypes = [ctypes.POINTER(CompFilter), ctypes.c_float, ctypes.c_float]
    library.comp_filter_update.restype = ctypes.c_float
    return library


def c_filter(lib: ctypes.CDLL, tau: float, angle0: float) -> CompFilter:
    f = CompFilter()
    lib.comp_filter_init(ctypes.byref(f), tau, DT, angle0)
    return f


@pytest.mark.parametrize("tau", [0.05, 0.5, 2.0])
def test_c_init_matches_python_bit_for_bit(lib: ctypes.CDLL, tau: float) -> None:
    f = c_filter(lib, tau, 0.1)
    ref = ComplementaryFilter(tau, DT, angle0=0.1)
    assert np.float32(f.alpha) == ref.alpha
    assert np.float32(f.angle) == ref.angle


@pytest.mark.parametrize(("seed", "tau"), [(0, 0.05), (1, 0.5), (2, 2.0)])
def test_c_update_matches_python_bit_for_bit(lib: ctypes.CDLL, seed: int, tau: float) -> None:
    rng = np.random.default_rng(seed)
    gyro = rng.normal(0.0, 2.0, 5000).astype(np.float32)
    acc = rng.normal(0.0, 0.3, 5000).astype(np.float32)
    f = c_filter(lib, tau, float(acc[0]))
    ref = ComplementaryFilter(tau, DT, angle0=float(acc[0]))
    for w, a in zip(gyro, acc, strict=True):
        got = np.float32(lib.comp_filter_update(ctypes.byref(f), float(w), float(a)))
        expected = np.float32(ref.update(float(w), float(a)))
        assert got == expected, f"C {got!r} != Python {expected!r}"


def test_c_accel_angle_matches_python_within_tolerance(lib: ctypes.CDLL) -> None:
    rng = np.random.default_rng(7)
    fwd = rng.uniform(-4.0, 4.0, 20000).astype(np.float32)
    up = rng.uniform(-4.0, 4.0, 20000).astype(np.float32)
    worst = 0.0
    for x, z in zip(fwd, up, strict=True):
        got = lib.comp_filter_accel_angle(float(x), float(z))
        worst = max(worst, abs(got - accel_angle(float(x), float(z))))
    assert worst <= ATAN2_TOL_RAD


def test_c_filter_reproduces_the_simulated_estimate_over_a_balance_run(lib: ctypes.CDLL) -> None:
    """Replays the gyro and accel-angle stream of a 3 s closed-loop run: C must reproduce the sim's estimate."""
    run = simulate(RobotParams(), CascadeGains(), theta0_deg=10.0, duration_s=3.0, push_impulse_ns=0.3)
    # The sim's filter starts from an IMU sample drawn before the loop, which isn't logged, so the C filter
    # starts from the sim's first logged estimate and replays every sample after it.
    f = c_filter(lib, DEFAULT_TAU_S, float(run.theta_est[0]))
    estimates = [float(run.theta_est[0])]
    for w, a in zip(run.gyro[1:], run.acc_angle[1:], strict=True):
        estimates.append(lib.comp_filter_update(ctypes.byref(f), float(w), float(a)))
    assert np.array_equal(np.float32(estimates), np.float32(run.theta_est))
