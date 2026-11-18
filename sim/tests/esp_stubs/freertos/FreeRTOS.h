/* Host stand-in for FreeRTOS.h, only so firmware/main compiles in the host test suite. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
