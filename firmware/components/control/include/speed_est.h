/*
 * Wheel speed from quadrature encoder counts, differenced over a window of control periods.
 *
 * Differencing over one 2.4 ms period quantises the speed to one count per period (0.079 m/s on this
 * robot), which the cascade turns into ~2 V of voltage chatter. Over a window of N periods the step is
 * N times finer, at the cost of about N/2 periods of delay in the (slow) outer speed loop.
 *
 * Plain C99 with no ESP-IDF dependency; mirrored by WheelSpeedEstimator in
 * sim/src/daruma/controllers.py and checked bit for bit on the host.
 */
#ifndef DARUMA_SPEED_EST_H
#define DARUMA_SPEED_EST_H

#include <stdint.h>

#define SPEED_EST_MAX_WINDOW 32

typedef struct {
    int32_t history[SPEED_EST_MAX_WINDOW]; /* ring buffer of the last `window` count readings */
    uint32_t window;                       /* periods to difference over, 1..SPEED_EST_MAX_WINDOW */
    uint32_t head;                         /* index of the oldest reading */
    float scale;                           /* rad per count / (window * dt) */
} speed_est_t;

/*
 * Prepares the estimator, treating the wheel as stopped at counts0 (call when the motors are armed).
 * window is clamped to 1..SPEED_EST_MAX_WINDOW. Precondition: dt > 0 (dt == 0 gives an infinite scale and
 * NaN speeds; the control loop must treat a non-finite value as a fault).
 */
void speed_est_init(speed_est_t *s, uint32_t window, float rad_per_count, float dt, int32_t counts0);

/*
 * Takes this period's count reading; returns the wheel's angular speed relative to the body in rad/s.
 * counts must be a continuous 32-bit accumulated count. The ESP32-S3 PCNT counter itself is 16-bit and
 * resets at its limits, so the driver must accumulate overflows (pcnt_unit_config_t.flags.accum_count with
 * watch points at the limits); a raw 16-bit reading would give a false speed spike at every reset. Only
 * the 32-bit wrap-around is handled here.
 */
float speed_est_update(speed_est_t *s, int32_t counts);

#endif /* DARUMA_SPEED_EST_H */
