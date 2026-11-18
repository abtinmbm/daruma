"""Golden-vector check: the firmware's C control code must match the Python reference exactly.

The C sources are compiled on the host into a shared library and called through ctypes. Both sides
use float32 with the same operation order and no FMA contraction, so results must be bit-identical.
In CI a missing C compiler is a failure; locally the test is skipped with a message.
"""

from __future__ import annotations

import ctypes

import numpy as np
import pytest

from daruma.controllers import Pid, WheelSpeedEstimator


class PidCtrl(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("kp", "ki", "kd", "i_limit", "out_min", "out_max", "integral")]


class SpeedEst(ctypes.Structure):
    _fields_ = [
        ("history", ctypes.c_int32 * 32),
        ("window", ctypes.c_uint32),
        ("head", ctypes.c_uint32),
        ("scale", ctypes.c_float),
    ]


@pytest.fixture(scope="module")
def lib(build_c_lib) -> ctypes.CDLL:
    library = build_c_lib("daruma_control", ["control/pid.c", "control/speed_est.c"])
    library.speed_est_init.argtypes = [
        ctypes.POINTER(SpeedEst),
        ctypes.c_uint32,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_int32,
    ]
    library.speed_est_init.restype = None
    library.speed_est_update.argtypes = [ctypes.POINTER(SpeedEst), ctypes.c_int32]
    library.speed_est_update.restype = ctypes.c_float
    library.pid_ctrl_update.argtypes = [ctypes.POINTER(PidCtrl), ctypes.c_float, ctypes.c_float, ctypes.c_float]
    library.pid_ctrl_update.restype = ctypes.c_float
    return library


@pytest.mark.parametrize("seed", range(3))
def test_c_pid_matches_python_bit_for_bit(lib: ctypes.CDLL, seed: int) -> None:
    rng = np.random.default_rng(seed)
    gains = dict(kp=80.0, ki=300.0, kd=3.0, i_limit=9.6, out_min=-9.6, out_max=9.6)
    ref = Pid(**gains)
    c_pid = PidCtrl(**gains, integral=0.0)
    errors = rng.normal(0.0, 0.1, 5000).astype(np.float32)  # enough to hit both clamps
    rates = rng.normal(0.0, 1.0, 5000).astype(np.float32)
    dt = 1.0 / 416.0
    for e, r in zip(errors, rates, strict=True):
        expected = np.float32(ref.update(float(e), float(r), dt))
        got = np.float32(lib.pid_ctrl_update(ctypes.byref(c_pid), float(e), float(r), dt))
        assert got == expected, f"C {got!r} != Python {expected!r}"
        assert np.float32(c_pid.integral) == ref.integral


RAD_PER_COUNT = float(np.float32(2 * np.pi / 1320))


@pytest.mark.parametrize(
    ("window", "start", "low", "high"),
    [
        (1, 0, -40, 41),
        (8, 12345, -40, 41),
        (32, 2**31 - 100, 0, 41),  # drifts forward across INT32_MAX
        (8, -(2**31) + 100, -40, 1),  # drifts backward across INT32_MIN
        (40, -7, -40, 41),  # window above the maximum: clamped to 32
    ],
)
def test_c_speed_estimator_matches_python_bit_for_bit(
    lib: ctypes.CDLL, window: int, start: int, low: int, high: int
) -> None:
    """Random-walk counts, including runs across both int32 limits and a window above the maximum (clamped)."""
    rng = np.random.default_rng(window)
    steps = rng.integers(low, high, 3000)
    unwrapped = start + np.cumsum(steps)
    if start > 2**30:
        assert unwrapped.max() > 2**31 - 1  # the run really crosses the limit
    if start < -(2**30):
        assert unwrapped.min() < -(2**31)
    # A continuous 32-bit accumulated count (what the PCNT driver must provide, see speed_est.h), wrapping at int32.
    counts = (unwrapped + 2**31) % 2**32 - 2**31
    c_est = SpeedEst()
    lib.speed_est_init(ctypes.byref(c_est), window, RAD_PER_COUNT, 1.0 / 416.0, start)
    ref = WheelSpeedEstimator(window, RAD_PER_COUNT, 1.0 / 416.0, start)
    assert c_est.window == ref.window and np.float32(c_est.scale) == ref.scale
    for k in counts:
        got = np.float32(lib.speed_est_update(ctypes.byref(c_est), int(k)))
        assert got == np.float32(ref.update(int(k)))


def test_speed_estimator_is_exact_for_a_constant_speed() -> None:
    """A steady N counts per period reads N counts per period once the window is full, and is ramping before."""
    est = WheelSpeedEstimator(8, RAD_PER_COUNT, 1.0 / 416.0, 0)
    readings = [est.update(5 * (k + 1)) for k in range(20)]
    expected = 5 * RAD_PER_COUNT * 416.0
    assert readings[-1] == pytest.approx(expected, rel=1e-6)
    assert readings[3] == pytest.approx(expected * 4 / 8, rel=1e-6)
