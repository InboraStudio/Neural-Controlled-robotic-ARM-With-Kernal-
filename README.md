# Neural-Controlled-robotic-ARM-With-Kernal

### About
- Low-level EMG robotic arm controller written in pure C/ASM signal capture, filtering, and actuation from memory addresses up


---

## Hardware Summary

| Component | Function | Key Parameters |
|------------|-----------|----------------|
| EMG Electrodes | Signal Source | Surface electrodes detecting muscle potentials |
| Instrumentation Amplifier | Signal Conditioning | Gain 1000×, Band 20–450 Hz |
| ATmega328P | Core Processor | 10-bit ADC, 16 MHz clock |
| Servo Motor | Output Actuator | PWM control at 50 Hz, 1–2 ms pulse width |
| Power Supply | Energy Source | 5V DC regulated |

---

## Memory and Register Mapping

### Microcontroller Memory Layout
| Section | Address Range | Description |
|----------|----------------|-------------|
| Flash (.text) | 0x0000 – 0x7FFF | Program memory |
| SRAM (.data + .bss) | 0x0100 – 0x08FF | Variables, buffers |
| EEPROM | 0x0000 – 0x03FF | Calibration constants |
| Stack | 0x08FF → 0x0100 | Grows downward |

### Key Register Map (ATmega328P)
| Register | Address | Description |
|-----------|----------|-------------|
| ADMUX | 0x7C | ADC Multiplexer Selection |
| ADCSRA | 0x7A | ADC Control and Status Register |
| ADCH / ADCL | 0x79 / 0x78 | ADC Data High / Low |
| DDRB | 0x24 | Data Direction Register (PWM Output) |
| PORTB | 0x25 | Port B Output Register |
| TCCR1A | 0x80 | Timer/Counter1 Control A |
| TCCR1B | 0x81 | Timer/Counter1 Control B |
| ICR1 | 0x86–0x87 | Input Capture Register (TOP for PWM) |
| OCR1A | 0x88–0x89 | Output Compare Register for Servo Pulse |

---

## Signal Processing Mathematics

### 1. EMG Signal Model
The raw EMG voltage signal can be approximated as a stochastic process:

\[
x(t) = A(t) \cdot \cos(2\pi f_c t + \phi) + n(t)
\]

where  
- \( A(t) \) = envelope of muscle activity  
- \( f_c \) = carrier frequency (50–150 Hz)  
- \( n(t) \) = noise (thermal + motion artifacts)

---

### 2. ADC Quantization
For a 10-bit ADC at 5 V reference:

\[
Q = \frac{V_{ref}}{2^{10}} = \frac{5}{1024} = 4.883 \text{ mV/step}
\]

\[
D_{out} = \frac{V_{in}}{V_{ref}} \times 1023
\]

Example:  
Input 2.45 V → \( D_{out} = (2.45/5) \times 1023 ≈ 501 \)

---

### 3. Full-Wave Rectification
Since EMG is bipolar, a full-wave rectifier converts all signals positive:

\[
y(t) = |x(t) - V_{bias}|
\]

Bias voltage: \( V_{bias} = \frac{V_{ref}}{2} = 2.5 \text{ V} \)

---

### 4. Moving Average Envelope Detection
A discrete moving average filter of window length \( N \) smooths the rectified signal:

\[
E[n] = \frac{1}{N} \sum_{k=0}^{N-1} |x[n-k] - 512|
\]

Implementation (fixed-point arithmetic):

```c
E = E + rect - buffer[idx];
buffer[idx] = rect;
idx = (idx + 1) & (N-1);
output = E >> 5; // Divide by 32
```
## 5. Mapping to Servo PWM

Servo pulse width varies between 1 ms (0°) and 2 ms (180°).

Timer1 configured at Fast PWM, prescaler 8:

<img width="481" height="238" alt="image" src="https://github.com/user-attachments/assets/45a70d2f-f737-4942-8696-efd217f08d0f" />

This maps envelope strength linearly to servo angle.
## Numerical Example
| Input Voltage (V) | ADC Value | Filtered Envelope | Pulse Width (µs) | Servo Angle (°) |
| ----------------- | --------- | ----------------- | ---------------- | --------------- |
| 0.5               | 102       | 10                | 1000             | 0               |
| 1.2               | 245       | 80                | 1200             | 36              |
| 2.0               | 409       | 160               | 1400             | 72              |
| 3.2               | 655       | 250               | 1600             | 108             |
| 4.0               | 819       | 320               | 1800             | 144             |
| 5.0               | 1023      | 400               | 2000             | 180             |

## ADC Timing Analysis

- For a prescaler of 128:
<img width="331" height="79" alt="image" src="https://github.com/user-attachments/assets/09182567-72bd-468c-9aaf-8476f7bd230f" />

- Each conversion requires 13 cycles:
<img width="308" height="75" alt="image" src="https://github.com/user-attachments/assets/99007fe2-9844-41b1-879b-9134c1f02d37" />

- Effective filtered sample rate (after averaging): ~1 kHz.

  | Stage                        | Approx. Delay | Notes                            |
| ---------------------------- | ------------- | -------------------------------- |
| ADC Conversion               | 104 µs        | Hardware                         |
| Filter Window (N=32 @ 1 kHz) | 32 ms         | Envelope delay                   |
| PWM Output Update            | < 1 ms        | Timer immediate                  |
| **Total**                    | **≈ 33 ms**   | End-to-end muscle → motion delay |

| Variable     | Size (bytes) | Address Range   |
| ------------ | ------------ | --------------- |
| ADC Buffer   | 64           | 0x0200 – 0x023F |
| Envelope Sum | 2            | 0x0240 – 0x0241 |
| Filter Index | 1            | 0x0242          |
| PWM Command  | 2            | 0x0243 – 0x0244 |
| Stack        | 64           | 0x08BF – 0x08FF |
## Compilation & Flash

- Build with avr-gcc:

avr-gcc -mmcu=atmega328p -Os -DF_CPU=16000000UL main.c -o emg_arm.elf
avr-objcopy -O ihex emg_arm.elf emg_arm.hex
avrdude -c arduino -p m328p -P COM3 -b 115200 -U flash:w:emg_arm.hex

### Safety and Isolation

- Never connect electrodes directly to the MCU input.
- Always isolate EMG amplifiers from the microcontroller power rail.
- Use differential amplifiers and a DC blocking capacitor on input stages.
- Recommended isolation IC: AD620 or INA128 with ±9V rails and 1 MΩ input impedance.



