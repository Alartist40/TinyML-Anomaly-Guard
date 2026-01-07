#pragma once

#include <stdbool.h>

extern float mean_err;
extern float std_err;
extern bool trained;

// Load threshold from NVS at startup
void load_threshold();

// Run the teaching process (blocks for TEACH_SECS)
// Saves resulting stats to NVS
void teach();
