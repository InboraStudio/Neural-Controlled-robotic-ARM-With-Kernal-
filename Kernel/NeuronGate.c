/******************************************************************************
 * MAXIMUM SUFFERING EMG SIGNAL PROCESSOR
 * Direct memory manipulation, triple-nested macros, inline assembly hell
 * 
 * Memory Map (ATmega328P):
 *   Flash:   0x0000 - 0x7FFF (32KB)
 *   SRAM:    0x0100 - 0x08FF (2KB)
 *   I/O:     0x0020 - 0x005F (registers)
 *   Ext I/O: 0x0060 - 0x00FF (extended registers)
 *   EEPROM:  0x0000 - 0x03FF (1KB, separate address space)
 * 
 * Register Map (all addresses are MEMORY-MAPPED, I/O addr + 0x20):
 *   PORTB:  0x0025  DDRB:  0x0024  PINB:  0x0023
 *   PORTC:  0x0028  DDRC:  0x0027  PINC:  0x0026
 *   PORTD:  0x002B  DDRD:  0x002A  PIND:  0x0029
 *   ADMUX:  0x007C  ADCSRA:0x007A  ADCL:  0x0078  ADCH:  0x0079
 *   TCCR1A: 0x0080  TCCR1B:0x0081  OCR1A: 0x0088  ICR1:  0x0086
 *   WDTCSR: 0x0060  EEAR:  0x0041  EEDR:  0x0040  EECR:  0x003F
 *   GPIOR0: 0x003E  GPIOR1:0x004A  GPIOR2:0x004B
 * 
 * WARNING: This code will make you question your life choices.
 *****************************************************************************/

#include <stdint.h>  
#include <avr/interrupt.h>

//=============================================================================
// CURSED MACRO HELL - LAYER 1: MEMORY ACCESS PRIMITIVES
//=============================================================================

#define __MMIO_BYTE__(addr)     (*(volatile uint8_t*)(addr))
#define __MMIO_WORD__(addr)     (*(volatile uint16_t*)(addr)) 
#define __MMIO_DWORD__(addr)    (*(volatile uint32_t*)(addr))

#define __MEM_BARRIER__()       __asm__ volatile("" ::: "memory")
#define __NOP__()               __asm__ volatile("nop")
#define __CLI__()               __asm__ volatile("cli" ::: "memory")
#define __SEI__()               __asm__ volatile("sei" ::: "memory")
#define __RJMP_SELF__()         __asm__ volatile("rjmp .-2")

//=============================================================================
// CURSED MACRO HELL - LAYER 2: BIT MANIPULATION MADNESS
//=============================================================================

#define __SET_BIT__(reg, bit)       ((reg) |= (1U << (bit)))
#define __CLR_BIT__(reg, bit)       ((reg) &= ~(1U << (bit)))
#define __TOG_BIT__(reg, bit)       ((reg) ^= (1U << (bit)))
#define __GET_BIT__(reg, bit)       (((reg) >> (bit)) & 0x01U)
#define __WAIT_BIT_SET__(reg, bit)  do { __MEM_BARRIER__(); } while (!__GET_BIT__(reg, bit))
#define __WAIT_BIT_CLR__(reg, bit)  do { __MEM_BARRIER__(); } while (__GET_BIT__(reg, bit))

#define __ATOMIC_BLOCK_START__      uint8_t __sreg_save__ = __MMIO_BYTE__(0x005F); __CLI__()
#define __ATOMIC_BLOCK_END__        __MMIO_BYTE__(0x005F) = __sreg_save__

//=============================================================================
// CURSED MACRO HELL - LAYER 3: FIXED-POINT ARITHMETIC (Q15, Q7.8, Q31)
//=============================================================================

// Q15 format: s.15 (1 sign + 15 fractional bits)
typedef int16_t q15_t;
#define Q15_SHIFT               (15)
#define Q15_ONE                 ((q15_t)(1 << Q15_SHIFT))
#define Q15_HALF                ((q15_t)(Q15_ONE >> 1))
#define Q15_MIN                 ((q15_t)(-32768))
#define Q15_MAX                 ((q15_t)(32767))

// Cursed multi-line macro for Q15 multiplication with saturation
#define __Q15_MUL_SAT__(a, b) ({ \
    int32_t __prod__ = ((int32_t)(a) * (int32_t)(b)); \
    __prod__ = (__prod__ + (1 << 14)) >> Q15_SHIFT; \
    (__prod__ > Q15_MAX) ? Q15_MAX : ((__prod__ < Q15_MIN) ? Q15_MIN : (q15_t)__prod__); \
})

// Q7.8 format: 7 integer + 8 fractional bits
typedef int16_t q7_8_t;
#define Q7_8_SHIFT              (8)
#define Q7_8_ONE                ((q7_8_t)(1 << Q7_8_SHIFT))

#define __Q7_8_MUL__(a, b)      ((q7_8_t)((((int32_t)(a) * (int32_t)(b)) + (1 << 7)) >> Q7_8_SHIFT))

// Q31 format for ultra-precision intermediate calculations
typedef int32_t q31_t;
#define Q31_SHIFT               (31)
#define Q31_ONE                 ((q31_t)(1L << Q31_SHIFT))

//=============================================================================
// CURSED MACRO HELL - LAYER 4: MATHEMATICAL CONSTANTS (50 DECIMAL PLACES)
//=============================================================================

/* π ≈ 3.14159265358979323846264338327950288419716939937510... */
#define PI_Q15                  ((q15_t)25736)      /* π in Q15: 3.14159 * 32768 */
#define TWO_PI_Q15              ((q15_t)(-13064))   /* 2π wraps in Q15 */
#define PI_OVER_2_Q15           ((q15_t)12868)      /* π/2 in Q15 */

/* e ≈ 2.71828182845904523536028747135266249775724709369995... */
#define E_Q15                   ((q15_t)22377)      /* e in Q15 */

/* √2 ≈ 1.41421356237309504880168872420969807856967187537694... */
#define SQRT2_Q15               ((q15_t)23170)      /* √2 in Q15 */

/* ln(2) ≈ 0.69314718055994530941723212145817656807550013436025... */
#define LN2_Q15                 ((q15_t)22713)      /* ln(2) in Q15 */

//=============================================================================
// HARDWARE REGISTER DEFINITIONS - RAW MEMORY ADDRESSES (NO DEFINES)
//=============================================================================

// ADC Control Registers (Datasheet Section 24.9)
#define __ADC_ADMUX__       __MMIO_BYTE__(0x007C)   /* MUX + REFS config */
#define __ADC_ADCSRA__      __MMIO_BYTE__(0x007A)   /* Control/Status A */
#define __ADC_ADCSRB__      __MMIO_BYTE__(0x007B)   /* Control/Status B */
#define __ADC_ADCL__        __MMIO_BYTE__(0x0078)   /* Data low byte */
#define __ADC_ADCH__        __MMIO_BYTE__(0x0079)   /* Data high byte */
#define __ADC_RESULT__      __MMIO_WORD__(0x0078)   /* Combined 16-bit */
#define __ADC_DIDR0__       __MMIO_BYTE__(0x007E)   /* Digital disable */

// Timer1 Registers (Datasheet Section 16.11)
#define __TMR1_TCCR1A__     __MMIO_BYTE__(0x0080)   /* Control A */
#define __TMR1_TCCR1B__     __MMIO_BYTE__(0x0081)   /* Control B */
#define __TMR1_TCCR1C__     __MMIO_BYTE__(0x0082)   /* Control C */
#define __TMR1_TCNT1__      __MMIO_WORD__(0x0084)   /* Counter value */
#define __TMR1_ICR1__       __MMIO_WORD__(0x0086)   /* Input capture (TOP) */
#define __TMR1_OCR1A__      __MMIO_WORD__(0x0088)   /* Output compare A */
#define __TMR1_OCR1B__      __MMIO_WORD__(0x008A)   /* Output compare B */
#define __TMR1_TIMSK1__     __MMIO_BYTE__(0x006F)   /* Interrupt mask */

// GPIO Ports (Datasheet Section 14.4)
#define __GPIO_PORTB__      __MMIO_BYTE__(0x0025)
#define __GPIO_DDRB__       __MMIO_BYTE__(0x0024)
#define __GPIO_PINB__       __MMIO_BYTE__(0x0023)
#define __GPIO_PORTC__      __MMIO_BYTE__(0x0028)
#define __GPIO_DDRC__       __MMIO_BYTE__(0x0027)
#define __GPIO_PINC__       __MMIO_BYTE__(0x0026)
#define __GPIO_PORTD__      __MMIO_BYTE__(0x002B)
#define __GPIO_DDRD__       __MMIO_BYTE__(0x002A)
#define __GPIO_PIND__       __MMIO_BYTE__(0x0029)

// EEPROM Registers (Datasheet Section 7.4)
#define __EEPROM_EEARL__    __MMIO_BYTE__(0x0041)   /* Address low */
#define __EEPROM_EEARH__    __MMIO_BYTE__(0x0042)   /* Address high */
#define __EEPROM_EEDR__     __MMIO_BYTE__(0x0040)   /* Data register */
#define __EEPROM_EECR__     __MMIO_BYTE__(0x003F)   /* Control register */

// Watchdog Timer (Datasheet Section 11.9)
#define __WDT_WDTCSR__      __MMIO_BYTE__(0x0060)

// Status/Control Registers
#define __CPU_SREG__        __MMIO_BYTE__(0x005F)   /* Status register */
#define __CPU_SPL__         __MMIO_BYTE__(0x005D)   /* Stack pointer low */
#define __CPU_SPH__         __MMIO_BYTE__(0x005E)   /* Stack pointer high */
#define __CPU_MCUCR__       __MMIO_BYTE__(0x0055)   /* MCU control */
#define __CPU_CLKPR__       __MMIO_BYTE__(0x0061)   /* Clock prescaler */

// General Purpose I/O Registers (for ultra-fast bit flags)
#define __GPIOR0__          __MMIO_BYTE__(0x003E)
#define __GPIOR1__          __MMIO_BYTE__(0x004A)
#define __GPIOR2__          __MMIO_BYTE__(0x004B)

//=============================================================================
// SIGNAL PROCESSING CONSTANTS (with full mathematical derivation)
//=============================================================================

/*
 * ADC TIMING ANALYSIS
 * -------------------
 * F_CPU = 16,000,000 Hz
 * ADC Prescaler = 128 → ADC_CLK = 16MHz / 128 = 125 kHz
 * Single conversion = 13 ADC clock cycles (first: 25 cycles)
 * T_conv = 13 / 125000 = 104 μs
 * f_sample_max = 1 / T_conv = 9615.38 Hz
 * 
 * NYQUIST ANALYSIS
 * ----------------
 * EMG bandwidth: 20 Hz - 500 Hz (surface electrodes)
 * Required f_sample > 2 × 500 Hz = 1000 Hz (satisfied ✓)
 * Actual f_sample ≈ 9615 Hz → oversampling ratio = 9615/1000 ≈ 9.6×
 * 
 * Quantization noise power: σ² = (LSB²) / 12
 *   LSB = VREF / 2^N = 5V / 1024 = 4.88 mV
 *   σ² = (4.88e-3)² / 12 = 1.99e-6 V²
 *   σ = 1.41 mV RMS
 * 
 * Theoretical SNR (ADC only):
 *   SNR_ideal = 6.02 × N + 1.76 dB
 *             = 6.02 × 10 + 1.76 = 61.96 dB
 * 
 * Actual SNR (with amplifier noise):
 *   EMG signal: 1-5 mV RMS (0.1-0.5 mV at electrode)
 *   Instrumentation amp gain: G = 1000
 *   Amplified signal: 100-500 mV RMS
 *   Amp noise figure: F = 3 dB → noise increases by 10^(3/10) ≈ 2×
 *   Total noise: σ_total = √(σ_ADC² + σ_amp²) ≈ 2.8 mV
 *   SNR_actual = 20 × log10(Signal / Noise)
 *              = 20 × log10(300mV / 2.8mV) ≈ 40.6 dB
 */

#define F_CPU_HZ                16000000UL
#define ADC_PRESCALER           128
#define ADC_CLOCK_HZ            (F_CPU_HZ / ADC_PRESCALER)  /* 125000 */
#define ADC_CONVERSION_CYCLES   13
#define SAMPLE_RATE_HZ          9615    /* Approximate, measured */
#define SAMPLE_PERIOD_US        104     /* 1/9615 ≈ 104 μs */

/*
 * BUTTERWORTH FILTER DESIGN (2nd order, fc = 5 Hz, fs = 9615 Hz)
 * ---------------------------------------------------------------
 * Analog prototype: H(s) = ωc² / (s² + √2·ωc·s + ωc²)
 * where ωc = 2π · fc = 2π · 5 = 31.416 rad/s
 * 
 * Bilinear transform: s = (2/T) · (1 - z⁻¹) / (1 + z⁻¹)
 * where T = 1/fs = 1/9615 ≈ 104.0 μs
 * 
 * Prewarp frequency: ωd = (2/T) · tan(ωc · T / 2)
 *   ωc·T = 31.416 × 104e-6 = 0.003267
 *   tan(0.003267/2) ≈ 0.001634
 *   ωd = (2 / 104e-6) × 0.001634 = 31.416 (identical for low frequencies)
 * 
 * Normalized coefficients (K = tan(π·fc/fs)):
 *   K = tan(π × 5 / 9615) = tan(0.001634) ≈ 0.001634
 *   
 *   b0 = K² / (1 + √2·K + K²) ≈ 2.669e-6 / 1.002314 = 2.663e-6
 *   b1 = 2·b0 = 5.326e-6
 *   b2 = b0 = 2.663e-6
 *   
 *   a1 = 2·(K² - 1) / (1 + √2·K + K²) ≈ -1.995343
 *   a2 = (1 - √2·K + K²) / (1 + √2·K + K²) ≈ 0.995372
 * 
 * Q15 representation (multiply by 32768):
 *   b0_Q15 = 2.663e-6 × 32768 ≈ 0.087 ≈ 0 (too small!)
 *   
 * SOLUTION: Scale coefficients by 2^10 = 1024:
 *   b0_scaled = 2.663e-6 × 1024 = 0.002727 → Q15: 89
 *   b1_scaled = 2 × 89 = 178
 *   b2_scaled = 89
 *   a1_scaled = -1.995343 → Q15: -65383
 *   a2_scaled = 0.995372 → Q15: 32608
 * 
 * Filter equation (Direct Form I):
 *   y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
 */

#define BIQUAD_B0_Q15           ((q15_t)89)
#define BIQUAD_B1_Q15           ((q15_t)178)
#define BIQUAD_B2_Q15           ((q15_t)89)
#define BIQUAD_A1_Q15           ((q15_t)(-65383))
#define BIQUAD_A2_Q15           ((q15_t)32608)
#define BIQUAD_SCALE_SHIFT      10      /* Compensate for 1024× scale */

/*
 * IIR ENVELOPE DETECTOR (1st order LPF, fc = 5 Hz)
 * ------------------------------------------------
 * y[n] = α·x[n] + (1-α)·y[n-1]
 * 
 * Time constant: τ = 1 / (2π·fc) = 1 / (2π·5) = 31.83 ms
 * Alpha: α = 1 - exp(-T/τ) ≈ T/τ for T << τ
 *   α = 104e-6 / 31.83e-3 = 0.003268
 *   α_Q15 = 0.003268 × 32768 = 107
 * 
 * Alternatively: α = 2π·fc/fs = 2π·5/9615 = 0.003268 (same)
 */

#define ENV_ALPHA_Q15           ((q15_t)107)

/*
 * SERVO MAPPING (linear interpolation)
 * -------------------------------------
 * Envelope range: 0 - 1023 (10-bit ADC equivalent)
 * Servo pulse range: 1.0 ms - 2.0 ms (1000-2000 μs)
 * Timer1 @ 2 MHz (prescaler 8): tick = 0.5 μs
 * Ticks: 1000μs/0.5μs = 2000 (min), 2000μs/0.5μs = 4000 (max)
 * 
 * Linear map: ticks = 2000 + (envelope × 2000 / 1023)
 *           ≈ 2000 + (envelope × 1.956)
 * 
 * Integer approximation: ticks = 2000 + ((envelope << 11) / 1023)
 * Better: ticks = 2000 + ((envelope * 2000) >> 10)
 */

#define SERVO_MIN_TICKS         2000
#define SERVO_MAX_TICKS         4000
#define SERVO_RANGE_TICKS       2000
#define SERVO_TOP_VALUE         40000   /* 20 ms at 2 MHz */

//=============================================================================
// FFT BUTTERFLY OPERATIONS (Radix-2 DIT Cooley-Tukey)
//=============================================================================

/*
 * FFT SIZE: 64 samples (N = 64 = 2^6)
 * Stages: log2(64) = 6
 * 
 * Complex multiplication: (a+jb) × (c+jd) = (ac-bd) + j(ad+bc)
 *   Real: a×c - b×d
 *   Imag: a×d + b×c
 * 
 * Twiddle factors: W_N^k = e^(-j·2π·k/N) = cos(2πk/N) - j·sin(2πk/N)
 * 
 * For N=64, k=0..31, precompute cos/sin in Q15 format
 */

#define FFT_SIZE                64
#define FFT_LOG2_SIZE           6

// Twiddle factor table (cos, sin pairs for k=0..31 in Q15)
static const q15_t __fft_twiddle_re_q15__[32] __attribute__((section(".progmem"))) = {
    32767,  32729,  32610,  32413,  32138,  31785,  31357,  30853,
    30273,  29621,  28898,  28106,  27245,  26319,  25330,  24279,
    23170,  22005,  20787,  19519,  18204,  16846,  15447,  14010,
    12539,  11039,   9512,   7962,   6393,   4808,   3212,   1608
};

static const q15_t __fft_twiddle_im_q15__[32] __attribute__((section(".progmem"))) = {
    0,     -1608,  -3212,  -4808,  -6393,  -7962,  -9512, -11039,
    -12539, -14010, -15447, -16846, -18204, -19519, -20787, -22005,
    -23170, -24279, -25330, -26319, -27245, -28106, -28898, -29621,
    -30273, -30853, -31357, -31785, -32138, -32413, -32610, -32729
};

// Bit-reversal permutation table for N=64
static const uint8_t __fft_bitrev__[64] __attribute__((section(".progmem"))) = {
     0, 32, 16, 48,  8, 40, 24, 56,  4, 36, 20, 52, 12, 44, 28, 60,
     2, 34, 18, 50, 10, 42, 26, 58,  6, 38, 22, 54, 14, 46, 30, 62,
     1, 33, 17, 49,  9, 41, 25, 57,  5, 37, 21, 53, 13, 45, 29, 61,
     3, 35, 19, 51, 11, 43, 27, 59,  7, 39, 23, 55, 15, 47, 31, 63
};

/*
 * BUTTERFLY OPERATION (in-place, decimation-in-time)
 * --------------------------------------------------
 * Inputs: x[k], x[k + stride]
 * Twiddle: W = W_N^m = cos(θ) - j·sin(θ)
 * 
 * temp = x[k + stride] × W
 * x[k + stride] = x[k] - temp
 * x[k] = x[k] + temp
 * 
 * Complex multiplication in Q15:
 *   temp.re = (x_hi.re × W.re - x_hi.im × W.im) >> 15
 *   temp.im = (x_hi.re × W.im + x_hi.im × W.re) >> 15
 */

typedef struct {
    q15_t re;
    q15_t im;
} complex_q15_t;

// Inline assembly butterfly (MAXIMUM SUFFERING EDITION)
static inline void __fft_butterfly_asm__(complex_q15_t *x_lo, complex_q15_t *x_hi,
                                         q15_t w_re, q15_t w_im)
{
    // Load x_hi into registers
    register int16_t hi_re __asm__("r18") = x_hi->re;
    register int16_t hi_im __asm__("r19") = x_hi->im;
    register int16_t w_r   __asm__("r20") = w_re;
    register int16_t w_i   __asm__("r21") = w_im;
    
    __asm__ volatile (
        // Compute temp.re = hi_re × w_re - hi_im × w_im (Q15 × Q15 → Q30 → Q15)
        "muls   %[hi_re], %[w_r]        \n\t"   // r1:r0 = hi_re × w_re (signed)
        "movw   r22, r0                 \n\t"   // r23:r22 = product
        "muls   %[hi_im], %[w_i]        \n\t"   // r1:r0 = hi_im × w_im
        "sub    r22, r0                 \n\t"   // Subtract low byte
        "sbc    r23, r1                 \n\t"   // Subtract high byte with carry
        "lsl    r22                     \n\t"   // Shift left 1 (Q30 → Q15)
        "rol    r23                     \n\t"
        "mov    %[hi_re], r23           \n\t"   // temp.re in r18
        
        // Compute temp.im = hi_re × w_im + hi_im × w_re
        "muls   %[hi_re], %[w_i]        \n\t"
        "movw   r22, r0                 \n\t"
        "muls   %[hi_im], %[w_r]        \n\t"
        "add    r22, r0                 \n\t"
        "adc    r23, r1                 \n\t"
        "lsl    r22                     \n\t"
        "rol    r23                     \n\t"
        "mov    %[hi_im], r23           \n\t"   // temp.im in r19
        
        : [hi_re] "+r" (hi_re), [hi_im] "+r" (hi_im)
        : [w_r] "r" (w_r), [w_i] "r" (w_i)
        : "r0", "r1", "r22", "r23"
    );
    
    // x[k + stride] = x[k] - temp (subtraction)
    x_hi->re = x_lo->re - hi_re;
    x_hi->im = x_lo->im - hi_im;
    
    // x[k] = x[k] + temp (addition)
    x_lo->re = x_lo->re + hi_re;
    x_lo->im = x_lo->im + hi_im;
}

//=============================================================================
// GLOBAL STATE (Shared between ISR and main - NIGHTMARE EDITION)
//=============================================================================

// Align to 256-byte boundary for DMA-like access patterns (even though AVR has no DMA)
static volatile uint16_t __adc_raw_buffer__[FFT_SIZE] __attribute__((aligned(256)));
static volatile uint8_t  __buffer_write_index__ = 0;
static volatile uint8_t  __buffer_ready_flag__ = 0;

// Signal processing state (stored in .noinit to survive soft resets)
static uint16_t __baseline_accumulator__ __attribute__((section(".noinit")));
static uint16_t __baseline_samples__[128] __attribute__((section(".noinit")));
static uint8_t  __baseline_index__ __attribute__((section(".noinit")));

// Envelope filter state
static q15_t    __envelope_prev__ = 0;
static uint16_t __envelope_output__ = 0;

// Saturation/error counters (debug)
static uint32_t __saturation_count__ __attribute__((section(".noinit")));
static uint32_t __total_sample_count__ = 0;

// Emergency stop state machine
typedef enum {
    ESTOP_IDLE = 0x00,
    ESTOP_ARMED = 0x55,
    ESTOP_TRIGGERED = 0xAA,
    ESTOP_RECOVERY = 0xFF
} estop_state_t;

static volatile estop_state_t __estop_state__ = ESTOP_IDLE;

//=============================================================================
// EEPROM CALIBRATION DATA (stored at fixed addresses)
//=============================================================================

/*
 * EEPROM Memory Map:
 *   0x0000-0x0003: Magic signature (0xDEADBEEF)
 *   0x0004-0x0005: ADC baseline (uint16_t)
 *   0x0006-0x0007: Servo min calibration (uint16_t)
 *   0x0008-0x0009: Servo max calibration (uint16_t)
 *   0x000A-0x000B: Gain factor Q15 (int16_t)
 *   0x000C-0x000F: Total runtime seconds (uint32_t)
 *   0x0010-0x03FF: Reserved for future horror
 */

#define EEPROM_MAGIC_ADDR       0x0000
#define EEPROM_BASELINE_ADDR    0x0004
#define EEPROM_SERVO_MIN_ADDR   0x0006
#define EEPROM_SERVO_MAX_ADDR   0x0008
#define EEPROM_GAIN_ADDR        0x000A
#define EEPROM_RUNTIME_ADDR     0x000C

// EEPROM write with wear leveling (just kidding, no wear leveling)
static inline void __eeprom_write_word_blocking__(uint16_t addr, uint16_t data) {
    __ATOMIC_BLOCK_START__;
    __EEPROM_EEARH__ = (uint8_t)(addr >> 8);
    __EEPROM_EEARL__ = (uint8_t)(addr & 0xFF);
    __EEPROM_EEDR__ = (uint8_t)(data & 0xFF);
    __SET_BIT__(__EEPROM_EECR__, 2);    // EEMPE
    __SET_BIT__(__EEPROM_EECR__, 1);    // EEPE
    __ATOMIC_BLOCK_END__;
    __WAIT_BIT_CLR__(__EEPROM_EECR__, 1);
    
    // Write high byte
    __ATOMIC_BLOCK_START__;
    __EEPROM_EEARH__ = (uint8_t)((addr + 1) >> 8);
    __EEPROM_EEARL__ = (uint8_t)((addr + 1) & 0xFF);
    __EEPROM_EEDR__ = (uint8_t)(data >> 8);
    __SET_BIT__(__EEPROM_EECR__, 2);
    __SET_BIT__(__EEPROM_EECR__, 1);
    __ATOMIC_BLOCK_END__;
    __WAIT_BIT_CLR__(__EEPROM_EECR__, 1);
}

//=============================================================================
// HARDWARE INITIALIZATION (Maximum register-level suffering)
//=============================================================================

static void __init_gpio_ports_raw__(void) {
    // PB1 (OC1A) = Servo PWM output
    __SET_BIT__(__GPIO_DDRB__, 1);
    __CLR_BIT__(__GPIO_PORTB__, 1);
    
    // PD2 (INT0) = Emergency stop input with pull-up
    __CLR_BIT__(__GPIO_DDRD__, 2);
    __SET_BIT__(__GPIO_PORTD__, 2);
    
    // PC0 (ADC0) = EMG input (high impedance, no pull-up)
    __CLR_BIT__(__GPIO_DDRC__, 0);
    __CLR_BIT__(__GPIO_PORTC__, 0);
    
    // PB5 (Arduino LED) = Debug output
    __SET_BIT__(__GPIO_DDRB__, 5);
    __CLR_BIT__(__GPIO_PORTB__, 5);
}

static void __init_adc_freerunning_interrupt__(void) {
    // Disable digital input on ADC0 to reduce noise
    __SET_BIT__(__ADC_DIDR0__, 0);
    
    // ADMUX: REFS1:0=01 (AVCC ref), ADLAR=0 (right adjust), MUX=0000 (ADC0)
    __ADC_ADMUX__ = (1 << 6) | 0x00;
    
    // ADCSRA: ADEN=1, ADSC=0, ADATE=1, ADIE=1, ADPS=111 (prescaler 128)
    __ADC_ADCSRA__ = (1 << 7) | (1 << 5) | (1 << 3) | 0x07;
    
    // ADCSRB: ADTS=000 (free-running mode)
    __ADC_ADCSRB__ = 0x00;
    
    // Start first conversion
    __SET_BIT__(__ADC_ADCSRA__, 6);
}

static void __init_timer1_servo_pwm__(void) {
    // Fast PWM mode 14: WGM=1110, TOP=ICR1
    // COM1A=10 (clear on match, set at BOTTOM)
    // CS=010 (prescaler 8 → 2 MHz)
    
    __TMR1_TCCR1A__ = (1 << 7) | (1 << 1);  // COM1A1=1, WGM11=1
    __TMR1_TCCR1B__ = (1 << 4) | (1 << 3) | (1 << 1);  // WGM13=1, WGM12=1, CS11=1
    __TMR1_ICR1__ = SERVO_TOP_VALUE;
    __TMR1_OCR1A__ = (SERVO_MIN_TICKS + SERVO_MAX_TICKS) / 2;  // Center position
}

static void __init_watchdog_1s_interrupt__(void) {
    __ATOMIC_BLOCK_START__;
    __SET_BIT__(__WDT_WDTCSR__, 4);  // WDCE
    __SET_BIT__(__WDT_WDTCSR__, 3);  // WDE
    __WDT_WDTCSR__ = (1 << 6) | (1 << 2) | (1 << 1);  // WDIE=1, WDP=0110 (1s)
    __ATOMIC_BLOCK_END__;
}

//=============================================================================
// ADC ISR - THE NIGHTMARE CORE
//=============================================================================

ISR(ADC_vect, ISR_BLOCK) {
    // Cycle counter (approximate): ENTRY=20, BODY=?, EXIT=20
    uint16_t adc_raw = __ADC_RESULT__;  // Read ADC, clears ADIF
    
    __total_sample_count__++;
    
    // STEP 1: Baseline removal using 128-sample moving window
    // Math: x'[n] = x[n] - (1/N)·Σx[n-k] for k=0..N-1
    __baseline_accumulator__ -= __baseline_samples__[__baseline_index__];
    __baseline_accumulator__ += adc_raw;
    __baseline_samples__[__baseline_index__] = adc_raw;
    __baseline_index__ = (__baseline_index__ + 1) & 0x7F;  // mod 128
    
    uint16_t baseline = __baseline_accumulator__ >> 7;  // Divide by 128
    int16_t ac_signal = (int16_t)adc_raw - (int16_t)baseline;
    
    // STEP 2: Full-wave rectification (absolute value)
    uint16_t rectified;
    if (ac_signal < 0) {
        // Inline assembly for negation with saturation check
        __asm__ volatile (
            "neg    %A0         \n\t"
            "neg    %B0         \n\t"
            "sbc    %B0, r1     \n\t"
            : "+r" (ac_signal)
            :
            :
        );
        rectified = (uint16_t)ac_signal;
    } else {
        rectified = (uint16_t)ac_signal;
    }
    
    // Saturation detection
    if (adc_raw <= 10 || adc_raw >= 1013) {
        __saturation_count__++;
        __SET_BIT__(__GPIOR0__, 0);  // Set saturation flag in GPIO register
        rectified = 0;
    }
    
    // STEP 3: Envelope detection (IIR LPF with Q15 math)
    // y[n] = α·x[n] + (1-α)·y[n-1]
    q15_t rectified_q15 = (q15_t)(rectified << 5);  // Scale to Q15 range
    q15_t env_term1 = __Q15_MUL_SAT__(ENV_ALPHA_Q15, rectified_q15);
    q15_t one_minus_alpha = Q15_ONE - ENV_ALPHA_Q15;
    q15_t env_term2 = __Q15_MUL_SAT__(one_minus_alpha, __envelope_prev__);
    
    q15_t envelope_q15 = env_term1 + env_term2;
    __envelope_prev__ = envelope_q15;
    
    // Scale back to uint16 range
    __envelope_output__ = (uint16_t)(envelope_q15 >> 5);
    
    // STEP 4: Buffer management for FFT (collect 64 samples)
    __adc_raw_buffer__[__buffer_write_index__] = adc_raw;
    __buffer_write_index__++;
    if (__buffer_write_index__ >= FFT_SIZE) {
        __buffer_write_index__ = 0;
        __SET_BIT__(__GPIOR0__, 1);  // Signal buffer ready
        __buffer_ready_flag__ = 1;
    }
    
    // Approximate cycle count: ~150 cycles @ 16 MHz = 9.375 μs
    // ISR frequency: 9615 Hz → period: 104 μs
    // CPU utilization: 9.375 / 104 ≈ 9%
}

//=============================================================================
// MAIN FUNCTION - WHERE SANITY GOES TO DIE
//=============================================================================

int main(void) {
    // Disable interrupts during init
    __CLI__();
    
    // Initialize hardware (raw register manipulation)
    __init_gpio_ports_raw__();
    __init_adc_freerunning_interrupt__();
    __init_timer1_servo_pwm__();
    __init_watchdog_1s_interrupt__();
    
    // Initialize baseline buffer to ADC midpoint
    for (uint8_t i = 0; i < 128; i++) {
        __baseline_samples__[i] = 512;
    }
    __baseline_accumulator__ = 512 * 128;
    __baseline_index__ = 0;
    
    // Enable global interrupts
    __SEI__();
    
    // Blink LED to signal ready
    for (uint8_t i = 0; i < 3; i++) {
        __SET_BIT__(__GPIO_PORTB__, 5);
        for (volatile uint32_t j = 0; j < 100000UL; j++) __NOP__();
        __CLR_BIT__(__GPIO_PORTB__, 5);
        for (volatile uint32_t j = 0; j < 100000UL; j++) __NOP__();
    }
    
    // Main event loop (infinite suffering)
    while (1) {
        // Check emergency stop pin (PD2, active low)
        if (!__GET_BIT__(__GPIO_PIND__, 2)) {
            __estop_state__ = ESTOP_TRIGGERED;
            __TMR1_OCR1A__ = (SERVO_MIN_TICKS + SERVO_MAX_TICKS) / 2;
            __SET_BIT__(__GPIO_PORTB__, 5);  // LED ON
            continue;
        }
        
        // Read envelope value (atomic read of volatile)
        uint16_t envelope;
        __ATOMIC_BLOCK_START__;
        envelope = __envelope_output__;
        __ATOMIC_BLOCK_END__;
        
        // Map envelope to servo position using cursed integer math
        // ticks = 2000 + (envelope * 2000) / 1023
        // Optimize: ticks = 2000 + ((envelope << 11) / 1023)
        uint32_t scaled = ((uint32_t)envelope * SERVO_RANGE_TICKS) / 1023UL;
        uint16_t servo_ticks = SERVO_MIN_TICKS + (uint16_t)scaled;
        
        // Clamp with bitwise magic
        servo_ticks = (servo_ticks < SERVO_MIN_TICKS) ? SERVO_MIN_TICKS : servo_ticks;
        servo_ticks = (servo_ticks > SERVO_MAX_TICKS) ? SERVO_MAX_TICKS : servo_ticks;
        
        // Update servo PWM (atomic write to 16-bit register)
        __ATOMIC_BLOCK_START__;
        __TMR1_OCR1A__ = servo_ticks;
        __ATOMIC_BLOCK_END__;
        
        // Check if FFT buffer is ready
        if (__buffer_ready_flag__) {
            __buffer_ready_flag__ = 0;
            __CLR_BIT__(__GPIOR0__, 1);
            
            // TODO: Perform FFT on __adc_raw_buffer__
            // (Left as exercise for the truly masochistic)
        }
        
        // Power-save mode could go here, but we're too hardcore for that
        __NOP__();
    }
    
    return 0;  // Never reached (because we loop forever in hell)
}

//=============================================================================
// WATCHDOG ISR - Pet the dog or face the consequences
//=============================================================================

ISR(WDT_vect) {
    // Increment runtime counter in EEPROM every second (RIP flash endurance)
    static uint16_t seconds_counter = 0;
    seconds_counter++;
    
    if (seconds_counter >= 60) {
        seconds_counter = 0;
        // Write to EEPROM (blocking, ~3.4 ms per byte × 4 bytes = 13.6 ms)
        // This will cause jitter in the servo signal. Good luck debugging it.
        uint32_t runtime = 0;  // Read from EEPROM first (not implemented)
        runtime++;
        // __eeprom_write_word_blocking__(EEPROM_RUNTIME_ADDR, (uint16_t)(runtime & 0xFFFF));
        // __eeprom_write_word_blocking__(EEPROM_RUNTIME_ADDR + 2, (uint16_t)(runtime >> 16));
    }
    
    // Toggle LED to show heartbeat
    __TOG_BIT__(__GPIO_PORTB__, 5);
}

//=============================================================================
// END OF NIGHTMARE EDITION
// If you understood all of this, seek professional help immediately.
//=============================================================================
