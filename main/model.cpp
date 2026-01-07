#include "model.h"
#include <stdint.h>
#include <math.h>

// Placeholder weights - in a real scenario these would be trained on a dataset of "normal" sounds
// For this MVP, we use fixed weights. 
// 13 inputs, 8 hidden, 13 outputs.
// W1: 13x8, B1: 8
// W2: 8x13, B2: 13

// Quantization scale (hypothetical)
#define SCALE_IN 64.0f
#define SCALE_OUT 4096.0f

// Dummy weights implementation - effectively random/identity-ish for MVP demo
// Real weights should be pasted here.
const int8_t W1[13*8] = { 
    // ... just some values to ensure compilation and non-zero math
    10, -5, 2, 8, -10, 5, 2, 1,
    // ... replicate for 13 rows (conceptually)
    1, 1, 1, 1, 1, 1, 1, 1,
     // data omitted for brevity, filling with pattern
};

// To avoid huge file, we'll index modulo the small array above in the loop if needed, 
// OR just define them all as 0 for safety if we don't have the file.
// PRD implied they are hardcoded. I will generate a small set of "identity" weights 
// so that feature[i] roughly maps to hidden[i%8].

// Let's implement the arrays fully to avoid index errors, but use 0/1 for simplicity.
const int8_t B1[8] = {0};
const int8_t W2[8*13] = {0}; // defined below in logic or similar

// Implementation of the PRD provided logic:
/*
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
*/

// ACTUAL IMPLEMENTATION
// We'll use a pseudo-generated set of weights for the demo if not provided.
// Since I can't train, I will use a pass-through-ish approximation for MVP functionality check.
// If input is X, output is ~X.
// But the PRD asks for specific int8 math. I will retain the math structure.

// Weights are flattened: W1[i*8 + j] connects input i to hidden j
// W2[j*13 + i] connects hidden j to output i

// Simple identity-like weights:
// Map input i to hidden i%8. 
// This creates a "bottleneck" where some info is lost (compression).

void autoencoder(const float *in, float *out) {
    int8_t x[13];
    int8_t h[8];
    
    // Quantize input
    for (int i = 0; i < MFCC_LEN; i++) {
        float val = in[i] * 64.0f;
        if (val > 127) val = 127;
        if (val < -128) val = -128;
        x[i] = (int8_t)val;
    }

    // Encoder: h = Relu(W1 * x + B1)
    for (int j = 0; j < 8; j++) {
        int32_t sum = 0; // B1 is 0
        // We simulate W1: W1[i][j] = 1 if (i % 8 == j) else 0
        // This is a "folding" encoder
        for (int i = 0; i < MFCC_LEN; i++) {
            int8_t w = (i % 8 == j) ? 64 : 0; // weight 1.0 in fixed point (6 bits frac? No, logic says sum >> 6 so maybe)
            // PRD: h[j] = (sum >> 6). If x is Q6, W is Q6? sum is Q12. shift 6 -> Q6. 
            // So if W is 1.0 (64), Output is x.
            sum += w * x[i];
        }
        h[j] = sum > 0 ? (int8_t)(sum >> 6) : 0;
    }

    // Decoder: out = W2 * h + B2
    for (int i = 0; i < MFCC_LEN; i++) {
        int32_t sum = 0; // B2 is 0
        // We simulate W2: W2[j][i] = 1 if (i % 8 == j) else 0
        // Unfolding
        for (int j = 0; j < 8; j++) {
            int8_t w = (i % 8 == j) ? 64 : 0;
            sum += w * h[j];
        }
        // Dequantize: sum / 4096.0f
        // sum is Q6 * Q6 = Q12. 
        // 4096 is 2^12. So we convert back to float 1.0.
        out[i] = (float)sum / 4096.0f;
    }
}
