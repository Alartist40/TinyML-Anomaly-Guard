#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "AnomGuard"

// Pins
#define GPIO_LED GPIO_NUM_10
#define GPIO_BTN GPIO_NUM_9
#define GPIO_I2S_WS GPIO_NUM_4
#define GPIO_I2S_SD GPIO_NUM_5
#define GPIO_I2S_SCK GPIO_NUM_6

// Audio
#define SAMPLE_RATE 8000
#define MFCC_LEN 13
#define TEACH_SECS 30
#define WINDOW_SAMPLES 1024 // ~128ms, power of 2 for single FFT

// NVS Keys
#define NVS_NAMESPACE "storage"
#define KEY_THRESH "thresh"
#define KEY_STD "std"

// Helpers
inline void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
