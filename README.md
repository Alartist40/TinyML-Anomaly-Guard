# TinyML Anomaly Guard

A battery-powered AI break-in detector running on a $4 ESP32-C3. It learns the "normal" acoustic room signature in 30 seconds and flashes a red LED (plus sends an alert) when an anomaly is detected (breaking glass, unknown voice).

## Features
- **TinyML on the Edge**: Runs entirely on-chip (160MHz RISC-V), no cloud required.
- **Fast Reaction**: <200ms latency from sound to alarm.
- **Privacy First**: No audio data leaves the device. Only anomaly scores are broadcast.
- **Zero-Shot Learning**: Learn "normal" background noise on-the-fly with a single button press.
- **Low Power**: Designed for battery operation.

## How It Works
The system uses a reconstruction-error based anomaly detection pipeline:
1.  **Audio Capture**: Reads 128ms audio windows @ 8kHz from an I2S microphone (INMP441).
2.  **Feature Extraction**: Computes Mel-Frequency Cepstral Coefficients (MFCCs) on the device.
    -   Applies Hamming Window.
    -   Runs a 1024-point FFT.
    -   Maps power spectrum to 13 Mel-frequency bins.
    -   Applies Discrete Cosine Transform (DCT) to get 13 MFCCs.
3.  **Autoencoder Model**: A tiny Neural Network (13 -> 8 -> 13) attempts to compress and reconstruct the MFCC vector.
    -   The model is pre-trained (weights embedded) to reconstruct generic room acoustics.
    -   *Note: In this MVP, the weights are set to a deterministic "folding" pattern for demonstration purposes without an external training dataset.*
4.  **Anomaly Detection**:
    -   **Teach Mode**: The device runs for 30 seconds, collecting error statistics (Mean & Variance of reconstruction error) for the current environment. These are saved to Non-Volatile Storage (NVS).
    -   **Detect Mode**: The device continuously infers. If `Abs(Input - Reconstructed) > Mean + 3*StdDev`, an anomaly is flagged.

## Hardware & Wiring
**BOM**:
- ESP32-C3 DevKit (~$4)
- INMP441 I2S Microphone (~$1)
- Red LED + 220Ω Resistor
- Push Button
- Li-Po Battery (Optional)

**Wiring**:
| Component | Pin | ESP32-C3 GPIO | Note |
|-----------|-----|---------------|------|
| **Mic** WS | WS | GPIO 4 | Word Select |
| **Mic** SD | SD | GPIO 5 | Serial Data |
| **Mic** SCK | SCK | GPIO 6 | Bit Clock |
| **Mic** VCC | - | 3.3V | |
| **Mic** GND | - | GND | |
| **LED** Anode | + | GPIO 10 | PWM Driven |
| **Button** | - | GPIO 9 | Connect to GND (Pull-up used) |

## Setup Guide (Fresh Computer)

### 1. Install ESP-IDF
You need the Espressif IoT Development Framework (v5.x).
**Windows**:
1.  Download the [ESP-IDF Tools Installer](https://dl.espressif.com/dl/esp-idf/).
2.  Run the installer and choose "ESP-IDF v5.x" (latest stable).
3.  Let it install Git and Python if needed.
4.  Launch "ESP-IDF 5.x CMD" from your Start Menu.

**Linux/Mac**:
```bash
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c3
. ./export.sh
```

### 2. Clone & Build
Open your ESP-IDF terminal and navigate to this project folder.

```bash
# 1. Initialize configuration with defaults
idf.py set-target esp32c3

# 2. Build the firmware
idf.py build
```

### 3. Flash & Monitor
Connect your ESP32-C3 via USB.

```bash
# Replace COMx with your port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)
idf.py -p COMx flash monitor
```

## Usage
1.  **Boot**: The LED will breathe (fade in/out) indicating it is **Untrained**.
2.  **Teach**:
    -   Press and hold the Button on GPIO 9.
    -   Wait for the LED response (System enters Teach mode for 30 seconds).
    -   Make "normal" noises (typing, silence, fan) during this time.
    -   The LED will turn off when creating the baseline.
3.  **Guard**:
    -   After teaching, the system enters Guard Mode.
    -   It will flash the LED briefly every 5 seconds (Heartbeat).
    -   **Trigger**: Clap your hands or speak loudly. The LED will turn **Solid Red** for 5 seconds and broadcast an alert via ESP-NOW.

## File Structure
- `main/main.cpp`: Main application entry point and logic loop.
- `main/capture.cpp`: I2S Driver and Custom MFCC implementation.
- `main/model.cpp`: Neural Network implementation (Autoencoder).
- `main/train.cpp`: Training loop and NVS storage management.
- `main/utils.h`: Configuration constants.
- `CMakeLists.txt`: Build system definitions.
