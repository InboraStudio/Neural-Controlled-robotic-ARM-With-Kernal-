  /******************************************************************************
 * Digital Filter Implementations
 * IIR and moving average filters for EMG signal processing
 *****************************************************************************/

#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include "fixed_point.h"

/*-----------------------------------------------------------------------------
 * 1st-order IIR Low-Pass Filter (Exponential Moving Average)
 * y[n] = α * x[n] + (1 - α) * y[n-1]
 * 
 * Cutoff frequency: fc = fs * α / (2π * (1 - α))
 * For small α: fc ≈ fs * α / (2π)
 * 
 * Time constant: τ = 1 / (2π * fc) = (1 - α) / (fs * α)
 * Alpha selection: α = 1 - exp(-2π * fc / fs)
 * For low cutoff (fc << fs): α ≈ 2π * fc / fs
 * 
 * Example: fs = 1000 Hz, fc = 5 Hz
 *   α ≈ 2π * 5 / 1000 = 0.0314
 *   In Q15: α = 0.0314 * 32768 ≈ 1029
 *---------------------------------------------------------------------------*/
typedef struct {
    q15_t alpha;        /* Filter coefficient (Q15) */
    q15_t prev_output;  /* Previous output y[n-1] (Q15) */
} iir_lpf_t;

/* Initialize IIR low-pass filter */
static inline void iir_lpf_init(iir_lpf_t *filter, q15_t alpha) {
    filter->alpha = alpha;
    filter->prev_output = 0;
}

/* Process one sample through IIR LPF */
static inline q15_t iir_lpf_process(iir_lpf_t *filter, q15_t input) {
    /* y[n] = α * x[n] + (1 - α) * y[n-1] */
    q15_t term1 = q15_mul(filter->alpha, input);
    q15_t one_minus_alpha = Q15_ONE - filter->alpha;
    q15_t term2 = q15_mul(one_minus_alpha, filter->prev_output);
    q15_t output = q15_add(term1, term2);
    filter->prev_output = output;
    return output;
}

/*-----------------------------------------------------------------------------
 * Moving Average Filter (FIR)
 * y[n] = (1/N) * Σ(x[n-k]) for k = 0 to N-1
 * 
 * Cutoff frequency (first null): fc = fs / N
 * Example: fs = 1000 Hz, N = 16 → fc = 62.5 Hz
 *---------------------------------------------------------------------------*/
#define MA_WINDOW_SIZE  16

typedef struct {
    int16_t buffer[MA_WINDOW_SIZE];
    uint8_t index;
    int32_t sum;
} moving_avg_t;

/* Initialize moving average filter */
void moving_avg_init(moving_avg_t *filter);

/* Process one sample through moving average */
int16_t moving_avg_process(moving_avg_t *filter, int16_t input);

/*-----------------------------------------------------------------------------
 * 2nd-order Butterworth Low-Pass Filter (Biquad)
 * Direct Form I difference equation:
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 * 
 * Design via bilinear transform from analog prototype
 * Analog: H(s) = ωc² / (s² + √2*ωc*s + ωc²)
 * Digital: Use bilinear transform s = (2/T) * (1-z⁻¹)/(1+z⁻¹)
 * 
 For fc = 5 Hz, fs = 1000 Hz:
  ωc = 2π * fc = 31.416 rad/s
 *   K = tan(π * fc / fs) = tan(0.0157) ≈ 0.0157
 *   b0 = K² / (1 + √2*K + K²) ≈ 0.000246
 *   b1 = 2 * b0 ≈ 0.000492
 *   b2 = b0
 *   a1 = 2*(K² - 1) / (1 + √2*K + K²) ≈ -1.955
 *   a2 = (1 - √2*K + K²) / (1 + √2*K + K²) ≈ 0.956
 *---------------------------------------------------------------------------*/
typedef struct {
    /* Filter coefficients (Q15) */
    q15_t b0, b1, b2;
    q15_t a1, a2;
    
    /* State variables */
    int16_t x1, x2;  /* Previous inputs */
    int16_t y1, y2;  /* Previous outputs */
} biquad_lpf_t;

/* Initialize Butterworth biquad filter */
void biquad_lpf_init(biquad_lpf_t *filter, q15_t b0, q15_t b1, q15_t b2, 
                     q15_t a1, q15_t a2);

/* Process one sample through biquad filter */
int16_t biquad_lpf_process(biquad_lpf_t *filter, int16_t input);

#endif /* FILTER_H */
