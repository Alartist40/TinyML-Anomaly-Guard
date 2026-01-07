#pragma once

// Init I2S driver
void i2s_config();

// Captures ~1 sec of audio, computes MFCC, and reconstructs utilizing the model.
// Returns the reconstruction error (anomaly score).
float inference_window();

// Internal function exposed for testing if needed
void compute_mfcc(const int16_t *samples, float *mfcc);
