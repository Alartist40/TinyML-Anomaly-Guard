# PRD: TinyML Anomaly Guard (MVP)

**Vision**  
A **$5 ESP32-C3** board that learns **normal room noise** for 30 s, then flashes a red LED and sends an MQTT alert the moment the acoustic pattern changes (unknown voice, breaking glass, 2 am typing).  
Ship in one weekend; recruiters see “battery-powered AI break-in detector, 128 kB, <200 ms reaction”.

---

## 1. Core Job Stories
- **As** you **I** press the TEACH button **So** the device learns my room’s keyboard/printer/loudness pattern.  
- **As** a recruiter **I** see a video of LED blinking when a new voice speaks **So** I know you shrunk ML to the edge.

---

## 2. MVP Scope (Pareto cut)
| Feature | In MVP | Later |
|---------|--------|-------|
| 1-button 30 s learning | ✅ | — |
| On-device MFCC + auto-encoder | ✅ | — |
| Reconstruction-error threshold | ✅ | — |
| Red LED + MQTT on anomaly | ✅ | — |
| Wi-Fi cam, cloud dashboard, solar | ❌ | v2 |

---

## 3. Functional Spec
- **Hardware**: ESP32-C3 dev board (<$5), 150 mAh Li-Po, LED on GPIO 10, push-button on GPIO 9.  
- **Audio**: I²S digital mic (INMP441) 8 kHz 16-bit → 1 s windows.  
- **Feature**: 13 MFCC coefficients → 128-byte vector.  
- **Model**: Tiny auto-encoder 13 → 8 → 13, int8 weights, 18 kB.  
- **Threshold**: mean + 3σ of training errors; > threshold = anomaly.  
- **Latency**: capture + inference <200 ms.  
- **Power**: 15 mA average → 10 days on 150 mAh.  
- **No microphone data ever leaves chip** (privacy sticker on box).

---

## 4. End-to-End Flow
1. Power on → blue LED breathe.  
2. Press TEACH (30 s) → blue fast blink → collect 300 vectors → compute μ, σ → store in NVS.  
3. Normal mode → green heartbeat every 5 s.  
4. Anomaly → red LED solid + MQTT `{"time":123456789, "event":"anomaly"}` → 5 s hold → back to green.

---

## 5. Success Criteria
- Teach 30 s → clap hands → no alarm.  
- New voice/breaking glass → red LED <200 ms.  
- Battery lasts 10 days continuous.  
- Binary <40 kB; total flash used <100 kB.

---

## 6. File Layout
```
tiny-anomaly/
├── train/
│   ├── capture.cpp      // I²S driver + MFCC
│   ├── model.cpp        // 13-8-13 auto-encoder
│   └── train.cpp        // 30 s learn + μσ calc
├── detect.cpp           // main loop
├── model.tflite         // 18 kB quantized model (ship binary)
├── partitions.csv       // 1 MB NVS for thresholds
└── README.md            // wiring pic + one-liner flash
```

---

## 7. BOM & Wiring
| Part | Price |
|------|-------|
| ESP32-C3 DevKit | $4 |
| INMP441 I²S mic | $1 |
| 5 mm red LED + 220 Ω | ¢10 |
| Push button | ¢10 |
| 150 mAh Li-Po | $3 |

**Wiring**  
- Mic VCC → 3V3, GND → G, WS → GPIO 4, SD → GPIO 5, SCK → GPIO 6  
- LED → GPIO 10 (sink)  
- Button → GPIO 9 (pull-up, press = LOW)

---

# Code Skeleton (Arduino / ESP-IDF style)

## detect.cpp (main)
```cpp
#include <driver/i2s.h>
#include "model.h"       // auto-encoder weights
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_wifi.h"

#define I2S_PORT I2S_NUM_0
#define GPIO_LED 10
#define GPIO_BTN 9
#define SAMPLE_RATE 8000
#define WINDOW_MS 1000
#define MFCC_LEN 13
#define TEACH_SECS 30
#define ERROR_THRESHOLD_NV "thresh"

float mean_err = 0, std_err = 0;
bool trained = false;

void setup() {
  Serial.begin(115200);
  pinMode(GPIO_LED, OUTPUT);
  pinMode(GPIO_BTN, INPUT_PULLUP);
  nvs_flash_init();
  i2s_config();
  load_threshold();   // sets trained=true if exists
  if (!trained) breathe_blue(); else heartbeat_green();
}

void loop() {
  if (digitalRead(GPIO_BTN) == LOW) {  // pressed
    teach();
  }
  float err = inference_window();
  if (err > mean_err + 3 * std_err && trained) {
    alarm(err);
  }
  delay(50);
}

// ---------- I²S ----------
void i2s_config() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_pin_config_t pin_cfg = {
    .bck_io_num = 6, .ws_io_num = 4, .data_out_num = -1, .data_in_num = 5
  };
  i2s_set_pin(I2S_PORT, &pin_cfg);
}

// ---------- MFCC + inference ----------
float inference_window() {
  int16_t samples[SAMPLE_RATE];
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, samples, sizeof(samples), &bytes_read, portMAX_DELAY);
  float mfcc[MFCC_LEN];
  compute_mfcc(samples, mfcc);        // 13 coeffs
  float reconstructed[MFCC_LEN];
  autoencoder(mfcc, reconstructed);   // tiny model
  float err = 0;
  for (int i = 0; i < MFCC_LEN; i++) err += fabs(mfcc[i] - reconstructed[i]);
  return err / MFCC_LEN;
}

// ---------- 30 s teach ----------
void teach() {
  digitalWrite(GPIO_LED, LOW);
  float errs[300];
  int idx = 0;
  uint32_t start = millis();
  while (millis() - start < TEACH_SECS * 1000) {
    errs[idx++] = inference_window();
    delay(100);
    if (idx == 300) break;
  }
  float sum = 0, sum2 = 0;
  for (int i = 0; i < idx; i++) { sum += errs[i]; sum2 += errs[i] * errs[i]; }
  mean_err = sum / idx;
  std_err = sqrt((sum2 - sum * sum / idx) / idx);
  save_threshold();
  trained = true;
  heartbeat_green();
}

// ---------- threshold storage ----------
void save_threshold() {
  nvs_handle_t h;
  nvs_open("storage", NVS_READWRITE, &h);
  nvs_set_blob(h, ERROR_THRESHOLD_NV, &mean_err, sizeof(mean_err));
  nvs_set_blob(h, "std", &std_err, sizeof(std_err));
  nvs_commit(h);
  nvs_close(h);
}

void load_threshold() {
  nvs_handle_t h;
  nvs_open("storage", NVS_READONLY, &h);
  size_t sz = sizeof(mean_err);
  if (nvs_get_blob(h, ERROR_THRESHOLD_NV, &mean_err, &sz) == ESP_OK) {
    sz = sizeof(std_err);
    nvs_get_blob(h, "std", &std_err, &sz);
    trained = true;
  }
  nvs_close(h);
}

// ---------- alarms ----------
void alarm(float err) {
  digitalWrite(GPIO_LED, HIGH);   // red solid
  send_mqtt(err);
  delay(5000);
  digitalWrite(GPIO_LED, LOW);
}

void send_mqtt(float err) {
  // quick ESP-NOW broadcast (no Wi-Fi setup for MVP)
  uint8_t peer[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast
  esp_now_init();
  esp_now_add_peer(peer, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"time\":%lu,\"err\":%.2f}", millis(), err);
  esp_now_send(peer, (uint8_t *)buf, strlen(buf));
}

// ---------- LEDs ----------
void breathe_blue() {
  for (int i = 0; i < 50; i++) { analogWrite(GPIO_LED, i); delay(20); }
  for (int i = 50; i > 0; i--) { analogWrite(GPIO_LED, i); delay(20); }
}
void heartbeat_green() {
  digitalWrite(GPIO_LED, LOW); delay(200);
  digitalWrite(GPIO_LED, HIGH); delay(200);
}
```

## model.cpp (tiny auto-encoder, int8 weights)
```cpp
const int8_t W1[13*8] = { /* quantized weights, 104 bytes */ };
const int8_t B1[8] = {0};
const int8_t W2[8*13] = { /* 104 bytes */ };
const int8_t B2[13] = {0};

void autoencoder(const float *in, float *out) {
  int8_t x[13], h[8];
  // quantise input
  for (int i = 0; i < 13; i++) x[i] = (int8_t)(in[i] * 64);
  // h = relu(W1*x + B1)
  for (int j = 0; j < 8; j++) {
    int32_t sum = B1[j];
    for (int i = 0; i < 13; i++) sum += W1[i*8+j] * x[i];
    h[j] = sum > 0 ? (int8_t)(sum >> 6) : 0;
  }
  // out = W2*h + B2 (de-quantise)
  for (int i = 0; i < 13; i++) {
    int32_t sum = B2[i];
    for (int j = 0; j < 8; j++) sum += W2[j*13+i] * h[j];
    out[i] = sum / 4096.0f;
  }
}
```

## MFCC (header-only snippet)
```cpp
void compute_mfcc(const int16_t *samples, float *mfcc) {
  // super-light: Hamming window → FFT → mel filter-bank → DCT
  // returns 13 coeffs; code ~60 lines, all fixed-point
}
```

---

# Build & Flash
1. **Install ESP-IDF v5**  
   `git clone --recursive https://github.com/espressif/esp-idf && cd esp-idf && ./install.sh esp32c3`
2. **Create project**  
   `idf.py create-project tiny-anomaly && drop files above`
3. **Menuconfig** → set GPIOs, enable `CONFIG_ARDUINO_LOOP_STACK_SIZE=4096`
4. **Flash**  
   `idf.py -p /dev/ttyUSB0 build flash monitor`

---

# Ship Checklist
1. Record **30-sec video**: teach → green heartbeat → clap (no alarm) → new voice → **red LED <200 ms**.  
2. GitHub release: `firmware.bin + wiring.png + README` with power-consumption table.  
3. Badge: `![CI](https://github.com/you/tiny-anomaly/actions/workflows/build.yml/badge.svg)`

**Impact line for résumé**  
“Shrunk anomaly-detection NN to 18 kB; ESP32-C3 battery guard detects break-in in <200 ms, zero cloud.”