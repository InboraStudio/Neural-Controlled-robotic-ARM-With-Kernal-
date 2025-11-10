/******************************************************************************
 * ATmega328P Hardware Register Definitions
 * All addresses from ATmega328P Datasheet Rev. 7810D-AVR-01/15
 * No external dependencies, pure bare-metal
 *****************************************************************************/

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <stdint.h>

/*-----------------------------------------------------------------------------
 * I/O REGISTER ADDRESS MAPPING
 * I/O registers 0x00-0x3F: use IN/OUT instructions
 * Extended I/O 0x60-0xFF: use LDS/STS with 0x20 offset from I/O address
 * Memory-mapped I/O addresses = I/O address + 0x20
 * Datasheet Section 8.3, Page 16; Section 33, Page 425
 *---------------------------------------------------------------------------*/

/* GPIO Ports - Datasheet Section 14.4, Page 92-93 */
#define PORTB   (*(volatile uint8_t*)0x25)  /* Port B Data Register */
#define DDRB    (*(volatile uint8_t*)0x24)  /* Port B Direction Register */
#define PINB    (*(volatile uint8_t*)0x23)  /* Port B Input Pins */

#define PORTC   (*(volatile uint8_t*)0x28)
#define DDRC    (*(volatile uint8_t*)0x27)
#define PINC    (*(volatile uint8_t*)0x26)

#define PORTD   (*(volatile uint8_t*)0x2B)
#define DDRD    (*(volatile uint8_t*)0x2A)
#define PIND    (*(volatile uint8_t*)0x29)

/* ADC Registers - Datasheet Section 24.9, Pages 258-266 */
#define ADMUX   (*(volatile uint8_t*)0x7C)  /* ADC Multiplexer Selection */
    #define REFS1   7   /* Reference Selection bit 1 */
    #define REFS0   6   /* Reference Selection bit 0 */
    #define ADLAR   5   /* ADC Left Adjust Result */
    #define MUX3    3   /* Analog Channel Selection bits */
    #define MUX2    2
    #define MUX1    1
    #define MUX0    0

#define ADCSRA  (*(volatile uint8_t*)0x7A)  /* ADC Control/Status Register A */
    #define ADEN    7   /* ADC Enable */
    #define ADSC    6   /* ADC Start Conversion */
    #define ADATE   5   /* ADC Auto Trigger Enable */
    #define ADIF    4   /* ADC Interrupt Flag */
    #define ADIE    3   /* ADC Interrupt Enable */
    #define ADPS2   2   /* ADC Prescaler Select bits */
    #define ADPS1   1
    #define ADPS0   0

#define ADCSRB  (*(volatile uint8_t*)0x7B)  /* ADC Control/Status Register B */
    #define ACME    6   /* Analog Comparator Multiplexer Enable */
    #define ADTS2   2   /* ADC Auto Trigger Source bits */
    #define ADTS1   1
    #define ADTS0   0

#define ADCL    (*(volatile uint8_t*)0x78)  /* ADC Data Register Low */
#define ADCH    (*(volatile uint8_t*)0x79)  /* ADC Data Register High */
#define ADC_REG (*(volatile uint16_t*)0x78) /* 16-bit access */

#define DIDR0   (*(volatile uint8_t*)0x7E)  /* Digital Input Disable Register */

/* Timer1 (16-bit) - Datasheet Section 16.11, Pages 134-145 */
#define TCCR1A  (*(volatile uint8_t*)0x80)  /* Timer/Counter1 Control Register A */
    #define COM1A1  7   /* Compare Output Mode for Channel A */
    #define COM1A0  6
    #define COM1B1  5   /* Compare Output Mode for Channel B */
    #define COM1B0  4
    #define WGM11   1   /* Waveform Generation Mode */
    #define WGM10   0

#define TCCR1B  (*(volatile uint8_t*)0x81)  /* Timer/Counter1 Control Register B */
    #define ICNC1   7   /* Input Capture Noise Canceler */
    #define ICES1   6   /* Input Capture Edge Select */
    #define WGM13   4   /* Waveform Generation Mode */
    #define WGM12   3
    #define CS12    2   /* Clock Select bits */
    #define CS11    1
    #define CS10    0

#define TCCR1C  (*(volatile uint8_t*)0x82)  /* Timer/Counter1 Control Register C */
#define TCNT1   (*(volatile uint16_t*)0x84) /* Timer/Counter1 (16-bit) */  
#define ICR1    (*(volatile uint16_t*)0x86) /* Input Capture Register 1 (TOP for PWM) */
#define OCR1A   (*(volatile uint16_t*)0x88) /* Output Compare Register 1A */
#define OCR1B   (*(volatile uint16_t*)0x8A) /* Output Compare Register 1B */

#define TIMSK1  (*(volatile uint8_t*)0x6F)  /* Timer/Counter1 Interrupt Mask */
    #define ICIE1   5   /* Input Capture Interrupt Enable */
    #define OCIE1B  2   /* Output Compare B Match Interrupt Enable */
    #define OCIE1A  1   /* Output Compare A Match Interrupt Enable */
    #define TOIE1   0   /* Overflow Interrupt Enable */

/* Watchdog Timer - Datasheet Section 11.9, Page 55 */
#define WDTCSR  (*(volatile uint8_t*)0x60)  /* Watchdog Timer Control Register */
    #define WDIF    7   /* Watchdog Interrupt Flag */
    #define WDIE    6   /* Watchdog Interrupt Enable */
    #define WDP3    5   /* Watchdog Timer Prescaler bits */
    #define WDCE    4   /* Watchdog Change Enable */
    #define WDE     3   /* Watchdog System Reset Enable */
    #define WDP2    2
    #define WDP1    1
    #define WDP0    0

/* Status Register - Datasheet Section 7.3, Page 14 */
#define SREG    (*(volatile uint8_t*)0x5F)  /* Status Register */
    #define SREG_I  7   /* Global Interrupt Enable */

/* Stack Pointer - Datasheet Section 8.2, Page 18 */
#define SPL     (*(volatile uint8_t*)0x5D)  /* Stack Pointer Low */
#define SPH     (*(volatile uint8_t*)0x5E)  /* Stack Pointer High */

/* MCU Control Register - Datasheet Section 10.11, Page 47 */
#define MCUCR   (*(volatile uint8_t*)0x55)
    #define PUD     4   /* Pull-up Disable */

/*-----------------------------------------------------------------------------
 * MEMORY ADDRESSES (from Datasheet Section 8, Page 16-18)
 *---------------------------------------------------------------------------*/
#define RAMSTART    0x0100  /* Start of SRAM */
#define RAMEND      0x08FF  /* End of SRAM (2KB) */
#define FLASHSTART  0x0000  /* Start of Flash */
#define FLASHEND    0x7FFF  /* End of Flash (32KB) */

/*-----------------------------------------------------------------------------
 * CLOCK AND TIMING
 * F_CPU = 16000000 Hz (16 MHz external crystal, typical Arduino)
 * Instruction cycle = 1 / F_CPU = 62.5 ns
 * Datasheet Section 9, Page 29
 *---------------------------------------------------------------------------*/
#define F_CPU       16000000UL
#define CPU_CYCLES_PER_US   (F_CPU / 1000000UL)

/*-----------------------------------------------------------------------------
 * ADC TIMING CALCULATIONS
 * ADC clock must be 50-200 kHz for full 10-bit resolution
 * Datasheet Section 24.4, Page 250
 * 
 * Prescaler options: 2, 4, 8, 16, 32, 64, 128
 * For F_CPU = 16 MHz:
 *   Prescaler 128 → ADC clock = 16MHz / 128 = 125 kHz ✓
 *   Prescaler 64  → ADC clock = 16MHz / 64  = 250 kHz (slightly high)
 * 
 * Single conversion: 13 ADC clock cycles (first conversion 25 cycles)
 * Datasheet Section 24.4, Page 251
 * At 125 kHz: Conversion time = 13 / 125000 = 104 μs
 * Max sample rate = 1 / 104μs ≈ 9615 Hz
 *---------------------------------------------------------------------------*/
#define ADC_PRESCALER   128
#define ADC_CLOCK_HZ    (F_CPU / ADC_PRESCALER)
#define ADC_CONVERSION_CYCLES   13
#define ADC_CONVERSION_TIME_US  ((ADC_CONVERSION_CYCLES * 1000000UL) / ADC_CLOCK_HZ)

/*-----------------------------------------------------------------------------
 * SERVO PWM PARAMETERS
 * Standard servo: 50 Hz (20 ms period), 1-2 ms pulse width
 * Datasheet Section 16.9.4, Page 132 - Fast PWM mode
 * 
 * Timer1 Fast PWM Mode 14: TOP = ICR1, Update OCR1x at BOTTOM
 * Prescaler 8: Timer clock = 16MHz / 8 = 2 MHz
 * Tick duration = 0.5 μs
 * For 20 ms period: ICR1 = 20000 μs / 0.5 μs = 40000 ticks
 * 
 * Pulse width range:
 *   1.0 ms (min) = 2000 ticks
 *   1.5 ms (mid) = 3000 ticks
 *   2.0 ms (max) = 4000 ticks
 *---------------------------------------------------------------------------*/
#define TIMER1_PRESCALER    8
#define TIMER1_TICK_FREQ    (F_CPU / TIMER1_PRESCALER)  /* 2 MHz */
#define SERVO_PERIOD_US     20000UL     /* 20 ms */
#define SERVO_TOP_VALUE     (SERVO_PERIOD_US * TIMER1_TICK_FREQ / 1000000UL)  /* 40000 */
#define SERVO_MIN_TICKS     2000        /* 1.0 ms */
#define SERVO_MID_TICKS     3000        /* 1.5 ms */
#define SERVO_MAX_TICKS     4000        /* 2.0 ms */

/*-----------------------------------------------------------------------------
 * EMG SIGNAL CHAIN PARAMETERS
 * Typical EMG amplitude: 0.1 - 5 mV (surface electrodes)
 * Required amplification: ~1000x to use ADC range
 * ADC resolution: 10-bit (0-1023), VREF = 5V
 * ADC LSB = 5V / 1024 = 4.88 mV/bit
 *---------------------------------------------------------------------------*/
#define ADC_RESOLUTION      10
#define ADC_MAX_VALUE       1023
#define VREF_MV             5000        /* 5V reference */
#define ADC_LSB_MV          (VREF_MV / 1024)  /* 4.88 mV */

/* Baseline removal (high-pass filter cutoff) */
#define HPF_CUTOFF_HZ       20

/* Envelope detector (low-pass filter cutoff) */
#define LPF_CUTOFF_HZ       5

/*-----------------------------------------------------------------------------
 * FUNCTION PROTOTYPES
 *---------------------------------------------------------------------------*/
void gpio_init(void);
void adc_init(void);
void timer1_pwm_init(void);
void watchdog_init(void);
uint16_t adc_read_blocking(uint8_t channel);
void servo_set_position(uint16_t ticks);
void delay_ms(uint16_t ms);

/* Inline functions for interrupt control */
static inline void cli(void) { __asm__ volatile ("cli" ::: "memory"); }
static inline void sei(void) { __asm__ volatile ("sei" ::: "memory"); }

#endif /* PERIPHERALS_H */
