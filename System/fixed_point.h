/******************************************************************************
 * Fixed-Point Math Library
 * Q15 format: 1 sign bit + 15 fractional bits
 * Range: -1.0 to +0.999969 (1 - 2^-15)
 * Resolution: 2^-15 ≈ 0.000030518
 *****************************************************************************/

#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

/* Q15 format: s.15 (1 sign bit, 15 fractional bits) */
typedef int16_t q15_t;

#define Q15_SHIFT       15
#define Q15_ONE         (1 << Q15_SHIFT)    /* 32768 = 1.0 */
#define Q15_HALF        (Q15_ONE >> 1)      /* 16384 = 0.5 */

/* Convert float to Q15 (compile-time only) */
#define FLOAT_TO_Q15(x) ((q15_t)((x) * Q15_ONE))

/* Convert integer to Q15 */
#define INT_TO_Q15(x)   ((q15_t)((x) << Q15_SHIFT))

/* Convert Q15 to integer (truncate) */
#define Q15_TO_INT(x)   ((int16_t)((x) >> Q15_SHIFT))

/* Multiply two Q15 numbers, result is Q15 */
static inline q15_t q15_mul(q15_t a, q15_t b) {
    /* Product is Q30, shift right 15 to get Q15 */
    int32_t product = (int32_t)a * (int32_t)b;
    return (q15_t)((product + (1 << 14)) >> Q15_SHIFT);  /* Round */
}

/* Add two Q15 numbers with saturation */
static inline q15_t q15_add(q15_t a, q15_t b) {
    int32_t sum = (int32_t)a + (int32_t)b;
    if (sum > 32767) return 32767;
    if (sum < -32768) return -32768;
    return (q15_t)sum;
}

/* Subtract with saturation */  
static inline q15_t q15_sub(q15_t a, q15_t b) {
    int32_t diff = (int32_t)a - (int32_t)b;
    if (diff > 32767) return 32767;
    if (diff < -32768) return -32768;
    return (q15_t)diff;
}

/* Absolute value */
static inline q15_t q15_abs(q15_t x) {
    if (x == -32768) return 32767;  /* Handle special case */
    return (x < 0) ? -x : x;
}

/* Shift right with rounding (divide by power of 2) */
static inline q15_t q15_shr(q15_t x, uint8_t shift) {
    if (shift == 0) return x;
    return (x + (1 << (shift - 1))) >> shift;
}

/*-----------------------------------------------------------------------------
 * Q7.8 format for intermediate calculations
 * 7 integer bits + 8 fractional bits
 * Range: -128.0 to +127.996
 *---------------------------------------------------------------------------*/
typedef int16_t q7_8_t;

#define Q7_8_SHIFT      8
#define Q7_8_ONE        (1 << Q7_8_SHIFT)   /* 256 = 1.0 */

/* Convert uint16_t ADC value (0-1023) to Q7.8 */
static inline q7_8_t adc_to_q7_8(uint16_t adc) {
    /* Scale 10-bit ADC to Q7.8: adc * 256 / 1024 = adc / 4 */
    return (q7_8_t)(adc >> 2);
}

/* Multiply Q7.8 numbers */
static inline q7_8_t q7_8_mul(q7_8_t a, q7_8_t b) {
    int32_t product = (int32_t)a * (int32_t)b;
    return (q7_8_t)((product + 128) >> Q7_8_SHIFT);
}

#endif /* FIXED_POINT_H */
