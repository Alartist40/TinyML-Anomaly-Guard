#include "train.h"
#include "capture.h"
#include "utils.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <math.h>

float mean_err = 0;
float std_err = 0;
bool trained = false;

static void save_threshold() {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        nvs_set_blob(h, KEY_THRESH, &mean_err, sizeof(mean_err));
        nvs_set_blob(h, KEY_STD, &std_err, sizeof(std_err));
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Threshold saved: Mean=%.4f, Std=%.4f", mean_err, std_err);
    } else {
        ESP_LOGE(TAG, "NVS Open Failed");
    }
}

void load_threshold() {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        size_t sz = sizeof(mean_err);
        if (nvs_get_blob(h, KEY_THRESH, &mean_err, &sz) == ESP_OK) {
            sz = sizeof(std_err);
            nvs_get_blob(h, KEY_STD, &std_err, &sz);
            trained = true;
            ESP_LOGI(TAG, "Threshold loaded: Mean=%.4f, Std=%.4f", mean_err, std_err);
        }
        nvs_close(h);
    }
}

void teach() {
    ESP_LOGI(TAG, "Starting Teach Mode...");
    
    // Visual indicator: Fast Blink logic is complex in blocking loop?
    // PRD says: "Press TEACH (30 s) -> blue fast blink"
    // We'll turn off LED first
    gpio_set_level(GPIO_LED, 0);

    // Collect stats
    // We'll use a dynamic array or fixed max? 
    // 30 seconds / ~0.13s per sample ~= 230 samples.
    // Let's alloc for 300 to be safe
    float *errs = (float*)malloc(300 * sizeof(float));
    if (!errs) {
        ESP_LOGE(TAG, "OOM in teach");
        return;
    }

    int count = 0;
    int64_t start = esp_timer_get_time(); // microseconds
    int64_t duration = TEACH_SECS * 1000000LL;

    while (esp_timer_get_time() - start < duration) {
        if (count >= 300) break;
        
        // Blink LED toggle every sample? (~128ms toggle = ~8Hz blink)
        gpio_set_level(GPIO_LED, count % 2);

        errs[count++] = inference_window();
        
        // ~128ms delay is inherent in inference_window
    }
    
    gpio_set_level(GPIO_LED, 0); // Off

    // Calculate stats
    double sum = 0;
    double sum2 = 0;
    for (int i = 0; i < count; i++) {
        sum += errs[i];
        sum2 += errs[i] * errs[i];
    }
    
    if (count > 0) {
        mean_err = (float)(sum / count);
        std_err = sqrtf((float)((sum2 - sum * sum / count) / count));
        save_threshold();
        trained = true;
    }

    free(errs);
    ESP_LOGI(TAG, "Teach complete. Count=%d", count);
}
