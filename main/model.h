#pragma once

#include "utils.h"

// Runs the 13->8->13 autoencoder
// in: 13 floats (MFCC)
// out: 13 floats (Reconstructed)
void autoencoder(const float *in, float *out);
