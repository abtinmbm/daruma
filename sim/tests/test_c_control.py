"""Golden-vector check: the firmware's C control code must match the Python reference exactly.

The C sources are compiled on the host into a shared library and called through ctypes. Both sides
use float32 with the same operation order and no FMA contraction, so results must be bit-identical.
In CI a missing C compiler is a failure; locally the test is skipped with a message.
"""

from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from daruma.controllers import Pid

CONTROL = Path(__file__).resolve().parents[2] / "firmware" / "components" / "control"


class PidCtrl(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in ("kp", "ki", "kd", "i_limit", "out_min", "out_max", "integral")]


@pytest.fixture(scope="module")
def lib(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    cc = shutil.which("gcc") or shutil.which("cc")
    if cc is None:
        if os.environ.get("CI"):
            pytest.fail("no C compiler in CI")
        pytest.skip("no C compiler on PATH")
    suffix = ".dll" if sys.platform == "win32" else ".so"
    out = tmp_path_factory.mktemp("cbuild") / f"daruma_control{suffix}"
    cmd = [cc, "-std=c99", "-O2", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC"]
    cmd += ["-I", str(CONTROL / "include"), str(CONTROL / "pid.c"), "-o", str(out)]
    subprocess.run(cmd, check=True)
    library = ctypes.CDLL(str(out))
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
