#include "comp_filter.h"

#include <math.h>

void comp_filter_init(comp_filter_t *f, float tau_s, float dt, float angle0) {
    f->alpha = tau_s / (tau_s + dt);
    f->dt = dt;
    f->angle = angle0;
}

float comp_filter_accel_angle(float accel_fwd, float accel_up) {
    return atan2f(-accel_fwd, accel_up);
}

float comp_filter_update(comp_filter_t *f, float gyro_rate, float accel_angle) {
    /* Same operation order as the Python reference, so float32 results match bit for bit. */
    float predicted = f->angle + gyro_rate * f->dt;
    f->angle = f->alpha * predicted + (1.0f - f->alpha) * accel_angle;
    return f->angle;
}
