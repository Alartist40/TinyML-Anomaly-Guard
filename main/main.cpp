#include "utils.h"
#include "capture.h"
#include "train.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// LED PWM Config
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (GPIO_LED) // GPIO 10
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          5000 // Frequency in Hertz. Set frequency at 5 kHz

static void config_led() {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

// Helper to set brightness 0-255 map to 0-8191
void led_set(uint8_t val) {
    uint32_t duty = ((uint32_t)val * 8191) / 255;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Patterns (Blocking for simplicity or task based? PRD used delays in loop so blocking visual effects are fine if short)
// But 'Breathe' is a loop. We need main loop to run.
// We'll update LED in the main loop or helper.

// ESP-NOW Broadcast
void send_mqtt_broadcast(float err) {
    uint8_t peer_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    // Construct JSON payload
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"time\":%lld,\"err\":%.2f}", esp_timer_get_time()/1000, err);
    
    // Send
    esp_now_send(peer_addr, (uint8_t *)buf, strlen(buf));
}

void alarm(float err) {
    ESP_LOGW(TAG, "ANOMALY: %.2f", err);
    // Solid On
    led_set(255);
    send_mqtt_broadcast(err);
    delay_ms(5000);
    led_set(0);
}

extern "C" void app_main(void) {
    // 1. Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. IO Config
    config_led();
    
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << GPIO_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // PULLUP as per wiring
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

    // 3. I2S Config
    i2s_config();

    // 4. WiFi / ESP-NOW
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    if (esp_now_init() == ESP_OK) {
        // Add broadcast peer
        esp_now_peer_info_t peerInfo = {};
        memset(&peerInfo, 0, sizeof(peerInfo));
        memset(peerInfo.peer_addr, 0xFF, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    } else {
        ESP_LOGE(TAG, "ESP-NOW Init Failed");
    }

    // 5. Load Threshold
    load_threshold();

    ESP_LOGI(TAG, "System Started. Trained=%d", trained);

    // UI State Vars
    int breathe_val = 0;
    int breathe_dir = 1;
    uint32_t last_heartbeat = 0;

    // Main Loop
    while (1) {
        // Button Check
        if (gpio_get_level(GPIO_BTN) == 0) {
            // Debounce
            delay_ms(50);
            if (gpio_get_level(GPIO_BTN) == 0) {
                teach(); // Blocks for 30s
                // reset state
                last_heartbeat = pdTICKS_TO_MS(xTaskGetTickCount());
                continue;
            }
        }

        // Logic
        if (!trained) {
            // Breathe Pattern (Blue/Red)
            // Update every loop? Loop is slow due to inference?
            // Inference takes ~130ms. That's too choppy for smooth breathe.
            // But we only infer if trained? PRD says: "Power on -> blue LED breathe... Press TEACH".
            // It doesn't say "Infer while not trained".
            // So if NOT trained, we iterate fast and breathe smoothly.
            
            breathe_val += 5 * breathe_dir;
            if (breathe_val >= 255) { breathe_val = 255; breathe_dir = -1; }
            if (breathe_val <= 0) { breathe_val = 0; breathe_dir = 1; }
            led_set(breathe_val);
            delay_ms(20);
        } else {
            // Heartbeat + Inference
            // "Normal mode -> green heartbeat every 5 s"
            // "Latency ... <200ms" implies we must be checking constantly?
            // "flash red LED ... the moment the acoustic pattern changes"
            // So we must infer constantly.
            
            // Heartbeat Logic: every 5s, blip.
            uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
            if (now - last_heartbeat > 5000) {
                // Heartbeat Pulse
                led_set(255); delay_ms(100); led_set(0); delay_ms(100);
                led_set(255); delay_ms(100); led_set(0); 
                last_heartbeat = now;
            } else {
                led_set(0); // Off most of the time
            }

            float err = inference_window(); // Takes ~130ms
            
            // Check Anomaly
            if (err > mean_err + 3 * std_err) {
                alarm(err);
            }
        }
    }
}
