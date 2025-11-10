/******************************************************************************
 * Peripheral Driver Implementation
 * Direct register manipulation, no HAL/libraries
 *****************************************************************************/

#include "peripherals.h"
#include "filter.h"

/*-----------------------------------------------------------------------------
 * GPIO Initialization
 *---------------------------------------------------------------------------*/
void gpio_init(void) {
    /* PB1 (OC1A, pin 15): Servo PWM output */
    DDRB |= (1 << 1);   /* Set PB1 as output */
    
    /* PD2 (INT0, pin 4): Emergency stop input with pull-up */
    DDRD &= ~(1 << 2);  /* Set PD2 as input */
    PORTD |= (1 << 2);  /* Enable pull-up resistor */
    
    /* PC0 (ADC0, pin 23): EMG signal input (configured in adc_init) */
}

/*-----------------------------------------------------------------------------
 * ADC Initialization
 * Configure for free-running mode with interrupt
 * Datasheet Section 24, Pages 242-266
 *---------------------------------------------------------------------------*/
void adc_init(void) {
    /* Disable digital input buffer on ADC0 to reduce noise and power */
    /* DIDR0 register at 0x7E, Datasheet Section 24.9.5, Page 265 */
    DIDR0 |= (1 << 0);  /* Disable digital input on ADC0 (PC0) */
    
    /* Set ADC reference to AVCC (5V) and select ADC0 */
    /* ADMUX register at 0x7C, Datasheet Section 24.9.1, Page 262 */
    /* REFS[1:0] = 01 → AVCC with external cap on AREF */
    /* MUX[3:0] = 0000 → ADC0 channel */
    ADMUX = (1 << REFS0);  /* AVCC reference, right-adjusted, ADC0 */
    
    /* Configure ADC Control and Status Register A */
    /* ADCSRA register at 0x7A, Datasheet Section 24.9.2, Page 263 */
    /* ADEN = 1: Enable ADC */
    /* ADSC = 0: Don't start yet */
    /* ADATE = 1: Auto-trigger enable (free-running) */
    /* ADIE = 1: Enable ADC interrupt */
    /* ADPS[2:0] = 111: Prescaler 128 → 16MHz/128 = 125kHz ADC clock */
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | 
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    /* Set auto-trigger source to free-running mode */
    /* ADCSRB register at 0x7B, Datasheet Section 24.9.3, Page 264 */
    /* ADTS[2:0] = 000: Free Running mode */
    ADCSRB = 0x00;
    
    /* Start first conversion (subsequent conversions are automatic) */
    ADCSRA |= (1 << ADSC);
}

/*-----------------------------------------------------------------------------
 * ADC Blocking Read (for testing, not used in interrupt-driven mode)
 *---------------------------------------------------------------------------*/
uint16_t adc_read_blocking(uint8_t channel) {
    /* Select channel (keep AVCC reference) */
    ADMUX = (1 << REFS0) | (channel & 0x0F);
    
    /* Start conversion */
    ADCSRA |= (1 << ADSC);
    
    /* Wait for conversion complete (ADIF flag set) */
    while (!(ADCSRA & (1 << ADIF)));
    
    /* Clear ADIF by writing 1 */
    ADCSRA |= (1 << ADIF);
    
    /* Read ADC result (ADCL must be read first for 16-bit access) */
    return ADC_REG;
}

/*-----------------------------------------------------------------------------
 * Timer1 PWM Initialization for Servo Control
 * Mode 14: Fast PWM with ICR1 as TOP
 * Datasheet Section 16.11, Pages 134-145
 *---------------------------------------------------------------------------*/
void timer1_pwm_init(void) {
    /* Set Fast PWM mode 14: WGM[13:10] = 1110 */
    /* WGM13=1 (in TCCR1B), WGM12=1 (in TCCR1B), WGM11=1 (in TCCR1A), WGM10=0 */
    /* TCCR1A at 0x80, Datasheet Section 16.11.1, Page 134 */
    TCCR1A = (1 << COM1A1) |    /* Clear OC1A on compare match, set at BOTTOM */
             (1 << WGM11);       /* WGM11 = 1 */
    
    /* TCCR1B at 0x81, Datasheet Section 16.11.2, Page 136 */
    TCCR1B = (1 << WGM13) |     /* WGM13 = 1 */
             (1 << WGM12) |     /* WGM12 = 1 */
             (1 << CS11);       /* Prescaler = 8 (CS12:10 = 010) */
    
    /* Set TOP value for 50 Hz (20 ms period) */
    /* ICR1 at 0x86, Datasheet Section 16.11.6, Page 139 */
    /* 16MHz / 8 / 50Hz = 40000 ticks */
    ICR1 = SERVO_TOP_VALUE;  /* 40000 */
    
    /* Set initial servo position to middle (1.5 ms) */
    OCR1A = SERVO_MID_TICKS;  /* 3000 ticks = 1.5 ms */
}

/*-----------------------------------------------------------------------------
 * Set Servo Position
 * Input: ticks (2000-4000 for 1-2 ms pulse width)
 *---------------------------------------------------------------------------*/
void servo_set_position(uint16_t ticks) {
    /* Clamp to valid range */
    if (ticks < SERVO_MIN_TICKS) ticks = SERVO_MIN_TICKS;
    if (ticks > SERVO_MAX_TICKS) ticks = SERVO_MAX_TICKS;
    
    /* Update compare register (double-buffered, safe to write anytime) */
    OCR1A = ticks;
}

/*-----------------------------------------------------------------------------
 * Watchdog Timer Initialization
 * Set to 1 second timeout, interrupt mode
 * Datasheet Section 11.9, Pages 54-57
 *---------------------------------------------------------------------------*/
void watchdog_init(void) {
    /* Timed sequence required to change WDT settings */
    /* WDTCSR at 0x60, Datasheet Section 11.9.2, Page 56 */
    cli();
    
    /* Start timed sequence: set WDCE and WDE simultaneously */
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    
    /* Within 4 cycles, set new prescaler and mode */
    /* WDP[3:0] = 0110: ~1 second timeout */
    /* WDIE = 1: Interrupt mode, WDE = 0: No reset */
    WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1);
    
    sei();
}

/*-----------------------------------------------------------------------------
 * Simple Delay (blocking, for initialization only)
 * Uses Timer0 delay or cycle counting
 *---------------------------------------------------------------------------*/
void delay_ms(uint16_t ms) {
    while (ms--) {
        /* 16 MHz: 16000 cycles per ms */
        /* Each iteration ~4 cycles, need 4000 iterations */
        for (volatile uint16_t i = 0; i < 4000; i++) {
            __asm__ volatile ("nop");
        }
    }
}

/*-----------------------------------------------------------------------------
 * Filter Implementation
 *---------------------------------------------------------------------------*/

/* Moving Average Filter */
void moving_avg_init(moving_avg_t *filter) {
    filter->index = 0;
    filter->sum = 0;
    for (uint8_t i = 0; i < MA_WINDOW_SIZE; i++) {
        filter->buffer[i] = 0;
    }
}

int16_t moving_avg_process(moving_avg_t *filter, int16_t input) {
    /* Subtract oldest value from sum */
    filter->sum -= filter->buffer[filter->index];
    
    /* Add new value to sum and buffer */
    filter->sum += input;
    filter->buffer[filter->index] = input;
    
    /* Update index (circular buffer) */
    filter->index = (filter->index + 1) % MA_WINDOW_SIZE;
    
    /* Return average: sum / N (divide by 16 = shift right 4) */
    return (int16_t)(filter->sum >> 4);
}

/* Biquad Filter */
void biquad_lpf_init(biquad_lpf_t *filter, q15_t b0, q15_t b1, q15_t b2,
                     q15_t a1, q15_t a2) {
    filter->b0 = b0;
    filter->b1 = b1;
    filter->b2 = b2;
    filter->a1 = a1;
    filter->a2 = a2;
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

int16_t biquad_lpf_process(biquad_lpf_t *filter, int16_t input) {
    /* Direct Form I: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] 
                             - a1*y[n-1] - a2*y[n-2] */
    
    /* Scale input to Q15 range (input is 10-bit ADC, 0-1023) */
    q15_t x_q15 = (q15_t)((int32_t)input * 32);  /* Scale to use Q15 range */
    
    /* Feedforward terms */
    int32_t acc = (int32_t)filter->b0 * x_q15;
    acc += (int32_t)filter->b1 * filter->x1;
    acc += (int32_t)filter->b2 * filter->x2;
    
    /* Feedback terms (note negative signs) */
    acc -= (int32_t)filter->a1 * filter->y1;
    acc -= (int32_t)filter->a2 * filter->y2;
    
    /* Shift to compensate for Q15 multiplication (Q15 * Q15 = Q30) */
    int16_t output = (int16_t)((acc + (1 << 14)) >> 15);  /* Round and scale */
    
    /* Update state variables */
    filter->x2 = filter->x1;
    filter->x1 = x_q15;
    filter->y2 = filter->y1;
    filter->y1 = output;
    
    /* Scale back to ADC range for mapping to servo */
    return (int16_t)(output >> 5);  /* Scale back from Q15 to ~10-bit range */
}
