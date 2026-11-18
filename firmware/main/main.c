/*
 * Daruma firmware entry point.
 *
 * Skeleton for milestone M0: boots, reports the build, and keeps the motors disabled. Drivers
 * (IMU, encoders, motors, battery), the control task and BLE telemetry are added in M1-M2.
 */
#include "comp_filter.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid.h"

static const char *TAG = "daruma";

void app_main(void) {
    ESP_LOGI(TAG, "Daruma firmware skeleton, built %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "Motors disabled: no drivers yet");

    /* Exercise the control library once so the component is linked and checked by the build. */
    pid_ctrl_t angle = {
        .kp = 67.0f, .ki = 380.0f, .kd = 3.4f, .i_limit = 9.6f, .out_min = -9.6f, .out_max = 9.6f};
    pid_ctrl_reset(&angle);
    float volts = pid_ctrl_update(&angle, 0.01f, 0.0f, 1.0f / 416.0f);
    ESP_LOGI(TAG, "PID self-check: 0.01 rad error -> %.4f V", (double)volts);

    /* Same for the tilt filter: level accelerometer, still gyro, so the estimate stays at zero. */
    comp_filter_t tilt;
    comp_filter_init(&tilt, 0.4f, 1.0f / 416.0f, 0.0f);
    float tilt_rad = comp_filter_update(&tilt, 0.0f, comp_filter_accel_angle(0.0f, 1.0f));
    ESP_LOGI(TAG, "Tilt filter self-check: level -> %.4f rad", (double)tilt_rad);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
