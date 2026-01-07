#include "capture.h"
#include "utils.h"
#include "driver/i2s.h"
#include "model.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define I2S_PORT I2S_NUM_0

// Optimization: Use single FFT for the whole window (1024 samples)
// 1024 samples @ 8kHz = 128ms
#define N_FFT WINDOW_SAMPLES
#define N_MFCC MFCC_LEN
#define N_MELS 13

void fft(float *real, float *imag, int n) {
    if (n <= 1) return;

    // Use a simple iterative or recursive. For 1024, recursive is fine on ESP32 stock stack?
    // 1024 depth might be risky if stack is small. 
    // Let's use iteratively or allocate on heap if recursive.
    // Actually, let's keep the recursive one but ensure stack size in main is large enough (CONFIG_ARDUINO_LOOP_STACK_SIZE=4096 mentioned in PRD).
    
    // ... (Same FFT code as before, safe enough for <2k depth, but let's be careful)
    // Actually, bit-reversal iterative is better but longer to write.
    // I will stick to recursive but allocated arrays.
    
    float *even_r = (float*)calloc(n/2, sizeof(float));
    float *even_i = (float*)calloc(n/2, sizeof(float));
    float *odd_r = (float*)calloc(n/2, sizeof(float));
    float *odd_i = (float*)calloc(n/2, sizeof(float));
    
    if(!even_r) { 
        // fallback
        return; 
    }

    // copy
    for (int i = 0; i < n/2; i++) {
        even_r[i] = real[2*i];
        even_i[i] = imag[2*i];
        odd_r[i] = real[2*i+1];
        odd_i[i] = imag[2*i+1];
    }

    fft(even_r, even_i, n/2);
    fft(odd_r, odd_i, n/2);

    for (int k = 0; k < n/2; k++) {
        float theta = -2.0f * M_PI * k / n;
        float wr = cosf(theta);
        float wi = sinf(theta);
        
        float tr = wr * odd_r[k] - wi * odd_i[k];
        float ti = wr * odd_i[k] + wi * odd_r[k];
        
        real[k] = even_r[k] + tr;
        imag[k] = even_i[k] + ti;
        
        real[k + n/2] = even_r[k] - tr;
        imag[k + n/2] = even_i[k] - ti;
    }

    free(even_r); free(even_i); free(odd_r); free(odd_i);
}

void dct(const float *in, float *out, int N) {
    for (int k = 0; k < N; k++) {
        float sum = 0.0f;
        for (int n = 0; n < N; n++) {
            sum += in[n] * cosf((M_PI / N) * (n + 0.5f) * k);
        }
        out[k] = sum;
    }
}

void compute_mfcc(const int16_t *samples, float *mfcc) {
    // 1. Convert to float and apply window
    float real[N_FFT];
    float imag[N_FFT];

    for (int i = 0; i < N_FFT; i++) {
        // Hamming window
        float w = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (N_FFT - 1));
        real[i] = samples[i] * w;
        imag[i] = 0;
    }

    // 2. FFT
    fft(real, imag, N_FFT);

    // 3. Power Spectrum
    float mag[N_FFT/2];
    for (int i = 0; i < N_FFT/2; i++) {
        mag[i] = sqrtf(real[i]*real[i] + imag[i]*imag[i]);
    }

    // 4. Mel Binning (Linear approx)
    // 512 bins -> 13 mels. ~39 bins per mel.
    float mels[N_MELS] = {0};
    int bin_width = (N_FFT/2) / N_MELS;
    
    for (int i = 0; i < N_MELS; i++) {
        for (int j = 0; j < bin_width; j++) {
            int idx = i * bin_width + j;
            if(idx < N_FFT/2) mels[i] += mag[idx];
        }
        if (mels[i] < 1e-6) mels[i] = 1e-6; 
        mels[i] = logf(mels[i]);
    }

    // 5. DCT
    dct(mels, mfcc, MFCC_LEN);
}

void i2s_config() {
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 512, // Needs to adequately hold buffers
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    
    i2s_pin_config_t pin_cfg = {
        .bck_io_num = GPIO_I2S_SCK, 
        .ws_io_num = GPIO_I2S_WS, 
        .data_out_num = -1, 
        .data_in_num = GPIO_I2S_SD
    };
    i2s_set_pin(I2S_PORT, &pin_cfg);
}

float inference_window() {
    // Read WINDOW_SAMPLES
    // We allocate here to be safe
    int16_t *samples = (int16_t*)calloc(WINDOW_SAMPLES, sizeof(int16_t));
    if(!samples) return 0;

    size_t bytes_read = 0;
    i2s_read(I2S_PORT, samples, WINDOW_SAMPLES * sizeof(int16_t), &bytes_read, portMAX_DELAY);
    
    // Zero pad if read less? (Shouldn't happen with portMAX_DELAY)
    
    float mfcc[MFCC_LEN];
    compute_mfcc(samples, mfcc);
    
    float reconstructed[MFCC_LEN];
    autoencoder(mfcc, reconstructed);

    float err = 0;
    for (int i = 0; i < MFCC_LEN; i++) {
        err += fabs(mfcc[i] - reconstructed[i]);
    }
    
    free(samples);
    return err / MFCC_LEN;
}
