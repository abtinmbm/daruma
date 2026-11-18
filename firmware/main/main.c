/*
 * Daruma firmware entry point.
 *
 * Skeleton for milestone M0: boots, reports the build, and keeps the motors disabled. Drivers
 * (IMU, encoders, motors, battery), the control task and BLE telemetry are added in M1-M2.
 */
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
        .kp = 80.0f, .ki = 300.0f, .kd = 3.0f, .i_limit = 9.6f, .out_min = -9.6f, .out_max = 9.6f};
    pid_ctrl_reset(&angle);
    float volts = pid_ctrl_update(&angle, 0.01f, 0.0f, 1.0f / 416.0f);
    ESP_LOGI(TAG, "PID self-check: 0.01 rad error -> %.4f V", (double)volts);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
