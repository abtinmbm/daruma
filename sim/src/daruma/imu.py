"""Simulated LSM6DSOX readings for an IMU mounted on the body at height h above the axle.

Kinematics (same sign convention as model.py). The IMU sits at (x + h sin(theta), r + h cos(theta)), so
its acceleration in the world frame is

    a_fwd_world = x'' + h (theta'' cos(theta) - theta'^2 sin(theta))
    a_up_world  =     - h (theta'' sin(theta) + theta'^2 cos(theta))

An accelerometer measures specific force f = a - g_vec, with g_vec = (0, -g), so at rest it reads +1 g "up".
Projected onto the body axes (forward e = (cos, -sin), up u = (sin, cos)):

    f_fwd = a_fwd cos(theta) - (a_up + g) sin(theta)
    f_up  = a_fwd sin(theta) + (a_up + g) cos(theta)

At rest this gives f_fwd = -g sin(theta), f_up = g cos(theta), so tilt = atan2(-f_fwd, f_up). When the robot
accelerates, x'' corrupts that estimate, which is why the gyro is fused in (fusion.py).

The gyro axis is the axle, so it reads theta' plus a bias and noise. Both sensors are quantised to the
configured LSB and clipped at full scale, like the real registers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from daruma.model import State
from daruma.params import RobotParams


@dataclass(frozen=True)
class ImuSample:
    accel_fwd_g: float  # body-forward specific force, g
    accel_up_g: float  # body-up specific force, g
    gyro_rad_s: float  # rate about the axle, rad/s (positive = tipping forward)


def specific_force(state: State, x_ddot: float, theta_ddot: float, p: RobotParams) -> tuple[float, float]:
    """Noise-free specific force at the IMU in the sensor's axes (forward, up), in m/s^2.

    The sensor axes are the body axes tilted forward by imu_mount_offset_rad, so a level-mounted IMU (offset 0)
    reads in body axes.
    """
    _, _, theta, theta_dot = state
    s, c = math.sin(theta), math.cos(theta)
    h = p.imu_height
    a_fwd = x_ddot + h * (theta_ddot * c - theta_dot**2 * s)
    a_up = -h * (theta_ddot * s + theta_dot**2 * c)
    sm, cm = math.sin(theta + p.imu_mount_offset_rad), math.cos(theta + p.imu_mount_offset_rad)
    f_fwd = a_fwd * cm - (a_up + p.g) * sm
    f_up = a_fwd * sm + (a_up + p.g) * cm
    return f_fwd, f_up


def quantise(value: float, lsb: float, full_scale: float) -> float:
    """Rounds to the nearest register count and clips at full scale, as the sensor does."""
    counts = round(value / lsb)
    limit = math.floor(full_scale / lsb)
    return max(-limit, min(limit, counts)) * lsb


def read_imu(state: State, x_ddot: float, theta_ddot: float, p: RobotParams, rng: np.random.Generator) -> ImuSample:
    """One IMU sample: true motion plus noise and bias, quantised like the LSM6DSOX output registers."""
    f_fwd, f_up = specific_force(state, x_ddot, theta_ddot, p)
    ax = f_fwd / p.g + rng.normal(0.0, p.accel_noise_g)
    az = f_up / p.g + rng.normal(0.0, p.accel_noise_g)
    w = state[3] + p.gyro_bias_rad_s + rng.normal(0.0, p.rate_noise_rad_s)
    return ImuSample(
        accel_fwd_g=quantise(ax, p.accel_lsb_g, p.accel_full_scale_g),
        accel_up_g=quantise(az, p.accel_lsb_g, p.accel_full_scale_g),
        gyro_rad_s=quantise(w, p.gyro_lsb_rad_s, p.gyro_full_scale_rad_s),
    )
