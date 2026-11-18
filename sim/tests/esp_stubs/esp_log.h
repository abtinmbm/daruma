/* Host stand-in for ESP-IDF's esp_log.h, only so firmware/main compiles in the host test suite. */
#pragma once
#include <stdio.h>
#define ESP_LOGI(tag, fmt, ...) printf("%s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("%s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("%s: " fmt "\n", tag, ##__VA_ARGS__)
