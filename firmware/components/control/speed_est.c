#include "speed_est.h"

void speed_est_init(speed_est_t *s, uint32_t window, float rad_per_count, float dt, int32_t counts0) {
    if (window < 1u) {
        window = 1u;
    }
    if (window > SPEED_EST_MAX_WINDOW) {
        window = SPEED_EST_MAX_WINDOW;
    }
    s->window = window;
    s->head = 0u;
    s->scale = rad_per_count / ((float)window * dt);
    for (uint32_t i = 0u; i < window; i++) {
        s->history[i] = counts0;
    }
}

float speed_est_update(speed_est_t *s, int32_t counts) {
    /* Unsigned subtraction so a counter wrap-around gives the right small difference, not overflow. */
    int32_t delta = (int32_t)((uint32_t)counts - (uint32_t)s->history[s->head]);
    s->history[s->head] = counts;
    s->head = (s->head + 1u) % s->window;
    return (float)delta * s->scale;
}
