/* Host stand-in for freertos/task.h, only so firmware/main compiles in the host test suite. */
#pragma once
#include "freertos/FreeRTOS.h"
void vTaskDelay(TickType_t ticks);
