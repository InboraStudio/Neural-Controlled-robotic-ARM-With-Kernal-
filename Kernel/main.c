/******************************************************************************
 * EMG-to-Servo Bare-Metal Main Application
 * ATmega328P @ 16 MHz
 * 
 * Signal Chain:
 *   1. ADC samples EMG at ~9.6 kHz (free-running, interrupt-driven)
 *   2. Remove DC baseline (running average subtraction)
 *   3. Full-wave rectification
 *   4. Envelope detection (IIR low-pass filter)
 *   5. Map envelope to servo PWM (1-2 ms pulse width)
 *   6. Emergency stop check on external pin
 * 
 * WARNING: FOR DEMO ONLY. Not for medical use.
 *****************************************************************************/

#include <stdint.h>
#include "peripherals.h"
#include "fixed_point.h"
#include "filter.h"

/*-----------------------------------------------------------------------------
 * GLOBAL STATE (Shared between ISR and main)
 *---------------------------------------------------------------------------*/
volatile uint16_t g_adc_raw = 0;            /* Latest ADC reading */
volatile uint16_t g_envelope = 0;           /* Processed envelope value */
volatile uint8_t  g_new_sample_ready = 0;   /* Flag for main loop */
volatile uint32_t g_sample_count = 0;       /* Sample counter (debug) */

/*-----------------------------------------------------------------------------
 * SIGNAL PROCESSING FILTERS
 *---------------------------------------------------------------------------*/
#define BASELINE_WINDOW     64      /* Samples for baseline estimation */
static uint32_t baseline_sum = 0;
static uint16_t baseline_buffer[BASELINE_WINDOW];
static uint8_t baseline_idx = 0;
static uint16_t baseline_avg = 512;  /* Initial guess (ADC midpoint) */

/* Envelope detector: IIR low-pass filter */
/* Alpha for 5 Hz cutoff at 9600 Hz sample rate:
 * α ≈ 2π * fc / fs = 2π * 5 / 9600 ≈ 0.00327
 * In Q15: 0.00327 * 32768 ≈ 107 */
#define ENVELOPE_ALPHA      FLOAT_TO_Q15(0.00327)
static iir_lpf_t envelope_filter;

/*-----------------------------------------------------------------------------
 * ADC INTERRUPT SERVICE ROUTINE
 * Called every ~104 μs (9615 Hz)
 * Cycle budget: 16MHz / 9615Hz ≈ 1664 cycles
 * ISR overhead: ~50 cycles (prologue/epilogue)
 * Available: ~1600 cycles for processing
 *---------------------------------------------------------------------------*/
void ADC_vect_impl(void) __attribute__((signal, used));

void ADC_vect_impl(void) {
    /* Read ADC result (clears ADIF flag automatically) */
    /* ADCL must be read first for 16-bit access */
    /* Datasheet Section 24.9.3.2, Page 259 */
    uint16_t adc_value = ADC_REG;  /* Read 16-bit register (ADCL+ADCH) */
    
    g_adc_raw = adc_value;
    g_sample_count++;
    
    /* STEP 1: Baseline Removal (High-Pass Filter Effect) */
    /* Update rolling average baseline estimate */
    baseline_sum -= baseline_buffer[baseline_idx];
    baseline_sum += adc_value;
    baseline_buffer[baseline_idx] = adc_value;
    baseline_idx = (baseline_idx + 1) % BASELINE_WINDOW;
    baseline_avg = (uint16_t)(baseline_sum / BASELINE_WINDOW);
    
    /* Subtract baseline (centered around 0) */
    int16_t ac_signal = (int16_t)adc_value - (int16_t)baseline_avg;
    
    /* STEP 2: Full-Wave Rectification */
    uint16_t rectified = (ac_signal < 0) ? -ac_signal : ac_signal;
    
    /* Detect saturation (ADC clipping) */
    if (adc_value <= 10 || adc_value >= 1013) {
        /* Signal saturated, possible safety issue */
        rectified = 0;  /* Could set error flag instead */
    }
    
    /* STEP 3: Envelope Detection (IIR Low-Pass Filter) */
    /* Scale rectified value to Q15 for filter */
    q15_t rectified_q15 = (q15_t)((int32_t)rectified * 32);  /* Scale to Q15 range */
    q15_t envelope_q15 = iir_lpf_process(&envelope_filter, rectified_q15);
    
    /* Scale back to ADC range (0-1023) */
    uint16_t envelope = (uint16_t)(envelope_q15 >> 5);
    
    /* Store result for main loop */
    g_envelope = envelope;
    g_new_sample_ready = 1;
    
    /* Watchdog feed could go here if using watchdog interrupt mode */
}

/*-----------------------------------------------------------------------------
 * EMERGENCY STOP CHECK
 * Polls PD2 (INT0) - active low with pull-up
 * Returns 1 if emergency stop is activated
 *---------------------------------------------------------------------------*/
static inline uint8_t check_emergency_stop(void) {
    /* Read PD2 (bit 2 of PIND) */
    return !(PIND & (1 << 2));  /* Return 1 if pin is low (button pressed) */
}

/*-----------------------------------------------------------------------------
 * MAP ENVELOPE TO SERVO POSITION
 * Linear mapping: envelope (0-1023) → servo ticks (2000-4000)
 * 
 * Math: ticks = MIN + (envelope * (MAX - MIN) / ADC_MAX)
 *       ticks = 2000 + (envelope * 2000 / 1023)
 *       ticks ≈ 2000 + (envelope * 2)  (simplified, slightly nonlinear)
 * 
 * Better: ticks = 2000 + (envelope << 1) with saturation
 *---------------------------------------------------------------------------*/
static uint16_t map_envelope_to_servo(uint16_t envelope) {
    /* Linear map with proper scaling */
    /* ticks = 2000 + (envelope * 2000) / 1023 */
    uint32_t scaled = ((uint32_t)envelope * 2000UL) / 1023UL;
    uint16_t ticks = SERVO_MIN_TICKS + (uint16_t)scaled;
    
    /* Clamp to valid range (redundant with servo_set_position, but safe) */
    if (ticks < SERVO_MIN_TICKS) ticks = SERVO_MIN_TICKS;
    if (ticks > SERVO_MAX_TICKS) ticks = SERVO_MAX_TICKS;
    
    return ticks;
}

/*-----------------------------------------------------------------------------
 * MAIN FUNCTION
 *---------------------------------------------------------------------------*/
int main(void) {
    /* Disable interrupts during initialization */
    cli();
    
    /* Initialize baseline buffer */
    for (uint8_t i = 0; i < BASELINE_WINDOW; i++) {
        baseline_buffer[i] = 512;  /* ADC midpoint */
    }
    baseline_sum = 512UL * BASELINE_WINDOW;
    
    /* Initialize envelope filter */
    iir_lpf_init(&envelope_filter, ENVELOPE_ALPHA);
    
    /* Initialize hardware peripherals */
    gpio_init();
    timer1_pwm_init();
    adc_init();
    watchdog_init();
    
    /* Enable global interrupts */
    sei();
    
    /* Main loop */
    while (1) {
        /* Wait for new ADC sample */
        if (g_new_sample_ready) {
            g_new_sample_ready = 0;
            
            /* Check emergency stop */
            if (check_emergency_stop()) {
                /* Set servo to safe middle position */
                servo_set_position(SERVO_MID_TICKS);
                continue;  /* Skip processing this iteration */
            }
            
            /* Read processed envelope (already computed in ISR) */
            uint16_t envelope = g_envelope;
            
            /* Map envelope to servo position */
            uint16_t servo_ticks = map_envelope_to_servo(envelope);
            
            /* Update servo */
            servo_set_position(servo_ticks);
            
            /* Optional: Send debug data over USART (not implemented) */
            /* printf("ADC: %u, Env: %u, Servo: %u\n", g_adc_raw, envelope, servo_ticks); */
        }
        
        /* Main loop runs at sample rate (~9.6 kHz) */
        /* Can add power-saving sleep mode here if needed */
    }
    
    return 0;  /* Never reached */
}

/*-----------------------------------------------------------------------------
 * WATCHDOG ISR (if using interrupt mode)
 *---------------------------------------------------------------------------*/
void __vector_6(void) __attribute__((signal, used));
void __vector_6(void) {
    /* Watchdog timeout - could reset to safe state */
    servo_set_position(SERVO_MID_TICKS);
}
