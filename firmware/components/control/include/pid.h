/*
 * PID controller with a measured-rate derivative and a clamped integrator.
 *
 * Plain C99 with no ESP-IDF dependency, so it is compiled and tested on the host against the
 * Python reference (sim/src/daruma/controllers.py, class Pid). Keep the two in step.
 */
#ifndef DARUMA_PID_H
#define DARUMA_PID_H

typedef struct {
    float kp;      /* proportional gain */
    float ki;      /* integral gain (per second) */
    float kd;      /* gain on the measured rate of the controlled variable */
    float i_limit; /* integral term is clamped to +-i_limit (anti-windup) */
    float out_min;
    float out_max;
    float integral; /* state: the integral term, already multiplied by ki */
} pid_ctrl_t;

/* Clears the integrator. Call when the motors are (re-)enabled. */
void pid_ctrl_reset(pid_ctrl_t *pid);

/*
 * One controller step.
 *   error: setpoint error in the controlled variable's units
 *   rate:  measured rate of the controlled variable (e.g. gyro rate), so setpoint jumps
 *          don't cause a derivative kick
 *   dt:    step in seconds
 * Returns kp*error + integral + kd*rate, clamped to [out_min, out_max].
 */
float pid_ctrl_update(pid_ctrl_t *pid, float error, float rate, float dt);

#endif /* DARUMA_PID_H */
