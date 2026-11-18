"""Physical parameters of the robot.

Every value is either an ESTIMATE (datasheet figure or rough calculation, with its source) or MEASURED
(with the name of its row in results/results.csv). Estimates are replaced only by measurements, never
silently.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


@dataclass(frozen=True)
class RobotParams:
    """Lumped parameters of the planar two-wheeled inverted pendulum, in SI units."""

    g: float = 9.81
    # Body: everything except the wheels, lumped at its centre of mass (COM).
    m_body: float = 0.92  # kg
    l_com: float = 0.06  # m, axle to body COM
    j_body: float = 0.0035  # kg m^2, about the body COM
    # Both wheels together.
    m_wheels: float = 0.10  # kg
    r_wheel: float = 0.040  # m
    j_wheels: float = 8.0e-5  # kg m^2, about the axle
    # One motor, referred to the gearbox output shaft (gear ratio already included).
    motor_ke: float = 0.344  # V s/rad (back-EMF constant)
    motor_kt: float = 0.172  # N m/A (torque constant x gearbox efficiency)
    motor_r: float = 3.43  # ohm (winding resistance)
    motor_deadband_v: float = 0.4  # V, applied voltage below which the motor doesn't turn
    encoder_counts_per_rev: int = 1320  # quadrature counts per wheel revolution
    # Power.
    v_battery: float = 9.6  # V, 8x AA NiMH nominal
    # Sensing and control.
    loop_hz: float = 416.0  # control rate = IMU output data rate
    angle_noise_rad: float = math.radians(0.2)  # std of the tilt in "ideal" sensing mode only
    rate_noise_rad_s: float = math.radians(0.5)  # std of the gyro rate
    # IMU (LSM6DSOX) as mounted on the body; used by the "imu" sensing mode (see imu.py).
    imu_height: float = 0.08  # m, axle to IMU along the body axis
    imu_mount_offset_rad: float = 0.0  # IMU axes tilted forward relative to the body (also stands in for a COM offset)
    accel_noise_g: float = 0.03  # std per sample, in g
    gyro_bias_rad_s: float = math.radians(0.5)  # residual bias after the startup calibration
    accel_lsb_g: float = 0.122e-3  # g per count at the +-4 g full scale
    accel_full_scale_g: float = 4.0
    gyro_lsb_rad_s: float = math.radians(17.5e-3)  # rad/s per count at the +-500 dps full scale
    gyro_full_scale_rad_s: float = math.radians(500.0)
    provenance: dict[str, str] = field(
        default_factory=lambda: {
            "m_body": "ESTIMATE: 2 motors+brackets 0.40, 8xAA+holder 0.27, plates 0.12, electronics 0.08, hardware 0.05",
            "l_com": "ESTIMATE: batteries mounted on the middle plate",
            "j_body": "ESTIMATE: box 0.18 m x 0.08 m, uniform density",
            "m_wheels": "ESTIMATE: 2 x 50 g, 80 mm rubber wheels",
            "r_wheel": "ESTIMATE: 80 mm wheel",
            "j_wheels": "ESTIMATE: 2 x uniform disc 50 g, r 40 mm",
            "motor_ke": "ESTIMATE: JGB37-520 12 V 30:1, 333 rpm no-load at 12 V",
            "motor_kt": "ESTIMATE: ke x 0.5 gearbox efficiency (gives ~0.6 N m stall, matches listing)",
            "motor_r": "ESTIMATE: 12 V / 3.5 A listed stall current",
            "motor_deadband_v": "ESTIMATE: guess, to be measured with a slow PWM ramp",
            "encoder_counts_per_rev": "ESTIMATE: 11 PPR x 4 (quadrature) x 30 gear ratio",
            "v_battery": "ESTIMATE: 8 x 1.2 V nominal",
            "angle_noise_rad": "ESTIMATE: pessimistic fused-angle noise",
            "rate_noise_rad_s": "ESTIMATE: pessimistic gyro noise incl. vibration",
            "imu_height": "ESTIMATE: IMU on the lower plate, ~8 cm above the axle; set from the CAD",
            "imu_mount_offset_rad": "ESTIMATE: 0 nominal; tuning and tests use +-2 deg (board mounting, COM not over axle)",
            "accel_noise_g": "ESTIMATE: pessimistic, motor/gearbox vibration dominates the sensor's own noise",
            "gyro_bias_rad_s": "ESTIMATE: datasheet zero-rate level is +-1 dps typ.; assume half remains after calibration",
            "accel_lsb_g": "ESTIMATE: datasheet typ. sensitivity at +-4 g (LSM6DSOX); config choice",
            "accel_full_scale_g": "ESTIMATE: config choice, +-4 g full scale (not a physical estimate)",
            "gyro_lsb_rad_s": "ESTIMATE: datasheet typ. sensitivity at +-500 dps (LSM6DSOX); config choice",
            "gyro_full_scale_rad_s": "ESTIMATE: config choice, +-500 dps full scale (not a physical estimate)",
        }
    )

    @property
    def dt(self) -> float:
        return 1.0 / self.loop_hz

    @property
    def no_load_speed_rpm(self) -> float:
        """Wheel speed at full battery voltage with no load, for sanity checks."""
        return self.v_battery / self.motor_ke / RPM_TO_RAD_S
