# Changelog

## [1.0.0] - 2026-01-08

### Features
- **Core Functionality**: Full anomaly detection pipeline implemented (Capture -> MFCC -> Autoencoder -> Threshold).
- **On-Device Learning**: Implemented "Teach Mode" to capture environment baseline (Mean/StdDev) and store in NVS.
- **Signal Processing**: Custom lightweight MFCC implementation using single-pass 1024-point FFT and Mel-binning.
- **Model**: Embedded Autoencoder with quantized int8 weights (simulated for MVP).
- **Alerting**: LED status indicators (Breathe, Heartbeat, Alarm) and ESP-NOW broadcast for alerts.
- **Low Latency**: Optimized capture window to 128ms/1024 samples to strictly meet the <200ms latency requirement (Improved from PRD's 1s window).

### Technical Details
- **Architecture**: Ported Arduino-style pseudo-code to native ESP-IDF C++ components.
- **Performance**:
    -   Window Size: 1024 samples (approx 128ms).
    -   FFT: Recursive implementation running on heap-allocated buffers.
    -   NVS: Persistent storage for learned anomaly thresholds.
- **Hardware Abstraction**:
    -   I2S driver configured for INMP441 (Master/RX).
    -   LEDC PWM used for "Breathe" LED effect.

### Deviations from PRD
- **Window Size**: Changed from `1000ms` (1s) to `~128ms` (1024 samples) to satisfy the "Latency < 200ms" requirement. 1s capture time would strictly violate the 200ms limit.
- **LED**: Adapted sophisticated RGB patterns (Blue/Green/Red) to single Red LED PWM brightness patterns (Breathe/Heartbeat/Solid) to match the BOM ($5 constraint, single LED).
