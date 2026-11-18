#include "pid.h"

static float clampf(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void pid_ctrl_reset(pid_ctrl_t *pid) {
    pid->integral = 0.0f;
}

float pid_ctrl_update(pid_ctrl_t *pid, float error, float rate, float dt) {
    /* Same operation order as the Python reference, so float32 results match bit for bit. */
    float integral = pid->integral + pid->ki * error * dt;
    pid->integral = clampf(integral, -pid->i_limit, pid->i_limit);
    float out = pid->kp * error + pid->integral + pid->kd * rate;
    return clampf(out, pid->out_min, pid->out_max);
}
