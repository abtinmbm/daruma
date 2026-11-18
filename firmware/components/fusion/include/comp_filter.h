/*
 * Complementary filter for the body tilt: fuses the gyro rate (integrated) with the accelerometer
 * angle (absolute but disturbed by acceleration).
 *
 *   angle_k = alpha * (angle_{k-1} + gyro_k * dt) + (1 - alpha) * accel_angle_k
 *   alpha   = tau / (tau + dt)
 *
 * Plain C99 with no ESP-IDF dependency, so it is compiled and tested on the host against the Python
 * reference (sim/src/daruma/fusion.py). Keep the two in step: same float32 operation order, built
 * with -ffp-contract=off.
 */
#ifndef DARUMA_COMP_FILTER_H
#define DARUMA_COMP_FILTER_H

typedef struct {
    float alpha; /* tau / (tau + dt), set by comp_filter_init */
    float dt;    /* control period in seconds */
    float angle; /* state: tilt estimate in rad, positive = leaning forward */
} comp_filter_t;

/*
 * Sets the time constant and period, and starts the estimate at angle0 (normally the accel angle).
 * Preconditions: dt > 0 and tau_s >= 0. Not checked here: tau_s + dt == 0 would make alpha NaN, and a
 * NaN tilt slips past threshold checks, so the control loop must treat a non-finite estimate as a fault.
 */
void comp_filter_init(comp_filter_t *f, float tau_s, float dt, float angle0);

/*
 * Tilt from the accelerometer alone: atan2(-forward, up) in body axes, any consistent unit.
 * Only valid when the robot isn't accelerating; the filter limits the damage when it is.
 */
float comp_filter_accel_angle(float accel_fwd, float accel_up);

/*
 * One step. gyro_rate: rad/s about the axle, positive = d(angle)/dt (pitching forward), so the IMU axis
 * mapping must give that sign at bring-up. accel_angle: rad, from comp_filter_accel_angle. Returns the new
 * estimate.
 */
float comp_filter_update(comp_filter_t *f, float gyro_rate, float accel_angle);

#endif /* DARUMA_COMP_FILTER_H */
