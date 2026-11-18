"""Hand-worked checks of the PID reference implementation."""

from __future__ import annotations

import pytest

from daruma.controllers import Pid


def test_proportional_integral_and_derivative_terms() -> None:
    pid = Pid(kp=2.0, ki=10.0, kd=0.5, i_limit=100.0, out_min=-100.0, out_max=100.0)
    # integral = 10 * 1.5 * 0.01 = 0.15; out = 2*1.5 + 0.15 + 0.5*(-4) = 1.15
    assert pid.update(error=1.5, rate=-4.0, dt=0.01) == pytest.approx(1.15, abs=1e-6)
    # integral = 0.15 + 0.15 = 0.30; out = 3 + 0.30 - 2 = 1.30
    assert pid.update(error=1.5, rate=-4.0, dt=0.01) == pytest.approx(1.30, abs=1e-6)


def test_integrator_is_clamped_so_it_cannot_wind_up() -> None:
    pid = Pid(kp=0.0, ki=100.0, kd=0.0, i_limit=0.5, out_min=-10.0, out_max=10.0)
    for _ in range(100):
        pid.update(error=1.0, rate=0.0, dt=0.01)
    assert float(pid.integral) == pytest.approx(0.5)
    # Without the clamp the integral would be 10.0 and one opposite step would leave it at 9.0.
    # With it, one step of opposite error moves it from +0.5 to 0.5 - 100*1*0.01 = -0.5.
    assert pid.update(error=-1.0, rate=0.0, dt=0.01) == pytest.approx(-0.5, abs=1e-6)


def test_output_is_clamped() -> None:
    pid = Pid(kp=100.0, ki=0.0, kd=0.0, i_limit=1.0, out_min=-9.6, out_max=9.6)
    assert pid.update(error=1.0, rate=0.0, dt=0.01) == pytest.approx(9.6)
    assert pid.update(error=-1.0, rate=0.0, dt=0.01) == pytest.approx(-9.6)
