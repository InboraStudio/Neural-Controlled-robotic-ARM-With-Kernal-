// stm32f407_advanced_control.c
// Advanced motor control, field-oriented control (FOC), and vector drive
// 32-bit floating point DSP operations

#include <stdint.h>
#include <math.h>

// Memory-mapped registers for STM32F407
#define TIM1_BASE           0x40010000
#define TIM8_BASE           0x40010400
#define ADC1_BASE           0x40012000
#define ADC2_BASE           0x40012100
#define ADC3_BASE           0x40012200

// Field-Oriented Control structure
typedef struct {
    float theta;            // Electrical angle
    float omega;            // Angular velocity
    float id;               // D-axis current
    float iq;               // Q-axis current
    float vd;               // D-axis voltage
    float vq;               // Q-axis voltage
} FOCState;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PIDFloat;

// Clarke transformation (3-phase to 2-phase)
void clarke_transform(int16_t ia, int16_t ib, int16_t ic, float *alpha, float *beta) {
    
    float fa = (float)ia * 0.00390625f;  // 1/256
    float fb = (float)ib * 0.00390625f;
    float fc = (float)ic * 0.00390625f;
    
    *alpha = fa;
    *beta = (fa * 0.5f + fb * 0.866025403f);  // cos(60°) + sin(60°) scaled
}

// Park transformation (2-phase rotating to D-Q)
void park_transform(float alpha, float beta, float theta, float *d, float *q) {
    
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    
    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

// Inverse Park transformation (D-Q to rotating 2-phase)
void inverse_park_transform(float d, float q, float theta, float *alpha, float *beta) {
    
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}

// Inverse Clarke transformation (2-phase to 3-phase)
void inverse_clarke_transform(float alpha, float beta, float *a, float *b, float *c) {
    
    *a = alpha;
    *b = -0.5f * alpha - 0.866025403f * beta;
    *c = -0.5f * alpha + 0.866025403f * beta;
}

// PI controller for current loop
float pi_controller(PIDFloat *pid, float setpoint, float feedback) {
    
    float error = setpoint - feedback;
    
    pid->integral += error * 0.001f;  // 1kHz sampling
    
    if (pid->integral > 1.0f) pid->integral = 1.0f;
    if (pid->integral < -1.0f) pid->integral = -1.0f;
    
    float output = pid->kp * error + pid->ki * pid->integral;
    
    if (output > 1.0f) output = 1.0f;
    if (output < -1.0f) output = -1.0f;
    
    return output;
}

// Rotor position observer (MRAS - Model Reference Adaptive System)
void rotor_observer(float alpha, float beta, float id, float iq, FOCState *state) {
    
    const float Ts = 0.001f;  // 1ms sampling
    const float Rs = 1.5f;     // Stator resistance
    const float Ls = 0.005f;   // Stator inductance
    const float Lr = 0.006f;   // Rotor inductance
    const float Lm = 0.004f;   // Mutual inductance
    
    // Slip calculation
    float slip = (state->id * Rs + Ls * (state->id - id)) / (Lm * state->theta);
    
    // Update rotor angle
    state->theta += (state->omega + slip) * Ts;
    
    if (state->theta > 2 * M_PI) state->theta -= 2 * M_PI;
    if (state->theta < 0) state->theta += 2 * M_PI;
}

// Space Vector PWM modulation
typedef struct {
    float vx;
    float vy;
    float vz;
} SVPWMOutput;

void space_vector_pwm(float vd, float vq, SVPWMOutput *output) {
    
    // Calculate sector and time
    float angle = atanf(vq / vd);
    
    // Normalize to unit circle
    float mag = sqrtf(vd * vd + vq * vq);
    if (mag > 1.0f) mag = 1.0f;
    
    // Compute duty cycles
    output->vx = (1.0f + mag * cosf(angle)) * 0.5f;
    output->vy = (1.0f + mag * cosf(angle - 2.0943f)) * 0.5f;  // 120°
    output->vz = (1.0f + mag * cosf(angle - 4.1888f)) * 0.5f;  // 240°
    
    // Clamp to [0, 1]
    if (output->vx > 1.0f) output->vx = 1.0f;
    if (output->vy > 1.0f) output->vy = 1.0f;
    if (output->vz > 1.0f) output->vz = 1.0f;
}

// Torque estimation
float estimate_torque(FOCState *state, float motor_inertia, float friction_coeff) {
    
    // Torque = Km * iq
    const float Km = 0.15f;
    
    float torque = Km * state->iq;
    
    // Subtract friction
    torque -= friction_coeff * state->omega;
    
    return torque;
}

// Efficiency optimization control
typedef struct {
    float efficiency;
    float power_loss;
    float copper_loss;
    float iron_loss;
} EfficiencyMetrics;

void compute_efficiency(FOCState *state, EfficiencyMetrics *metrics) {
    
    const float Rs = 1.5f;
    const float friction = 0.02f;
    
    // Copper losses (I²R)
    metrics->copper_loss = Rs * (state->id * state->id + state->iq * state->iq);
    
    // Iron losses (core saturation losses)
    metrics->iron_loss = 0.01f * state->theta * state->theta;
    
    // Mechanical friction losses
    float mech_loss = friction * state->omega * state->omega;
    
    metrics->power_loss = metrics->copper_loss + metrics->iron_loss + mech_loss;
    
    // Estimated efficiency (percentage)
    float input_power = 100.0f;  // Assume 100W input
    metrics->efficiency = ((input_power - metrics->power_loss) / input_power) * 100.0f;
    
    if (metrics->efficiency < 0) metrics->efficiency = 0;
}

// Flux weakening for speed enhancement
void flux_weakening_control(FOCState *state, PIDFloat *pid, float speed_cmd, float max_voltage) {
    
    const float base_speed = 157.0f;  // rad/s
    
    if (state->omega > base_speed) {
        
        // Reduce flux (increase negative id)
        float speed_ratio = state->omega / base_speed;
        state->id = -10.0f / speed_ratio;
        
        // Limit voltage
        float voltage_mag = sqrtf(state->vd * state->vd + state->vq * state->vq);
        if (voltage_mag > max_voltage) {
            state->vd *= (max_voltage / voltage_mag);
            state->vq *= (max_voltage / voltage_mag);
        }
    } else {
        
        // Normal operation
        state->id = 5.0f;
    }
}

// Deadtime compensation
void deadtime_compensation(SVPWMOutput *pwm, float deadtime_us, float switching_freq) {
    
    const float dead_comp = deadtime_us * switching_freq * 0.000001f;
    
    // Compensate for deadtime
    if (pwm->vx > 0.5f) pwm->vx += dead_comp * 0.5f;
    if (pwm->vy > 0.5f) pwm->vy += dead_comp * 0.5f;
    if (pwm->vz > 0.5f) pwm->vz += dead_comp * 0.5f;
}

// Current measurement with calibration
typedef struct {
    float adc_offset_a;
    float adc_offset_b;
    float adc_offset_c;
    float gain_a;
    float gain_b;
    float gain_c;
} CurrentCalibration;

float read_calibrated_current(uint16_t adc_raw, CurrentCalibration *cal, int phase) {
    
    float voltage;
    
    switch (phase) {
        case 0:
            voltage = ((float)adc_raw * 3.3f / 4095.0f - cal->adc_offset_a) * cal->gain_a;
            break;
        case 1:
            voltage = ((float)adc_raw * 3.3f / 4095.0f - cal->adc_offset_b) * cal->gain_b;
            break;
        case 2:
            voltage = ((float)adc_raw * 3.3f / 4095.0f - cal->adc_offset_c) * cal->gain_c;
            break;
    }
    
    // Convert voltage to current (assuming 100mV/A)
    float current = voltage / 0.1f;
    
    return current;
}

// Complete FOC loop
void foc_control_loop(FOCState *state, PIDFloat *id_pid, PIDFloat *iq_pid,
                      int16_t ia, int16_t ib, int16_t ic, float torque_cmd) {
    
    // Step 1: Clarke transformation
    float alpha, beta;
    clarke_transform(ia, ib, ic, &alpha, &beta);
    
    // Step 2: Park transformation
    float id_actual, iq_actual;
    park_transform(alpha, beta, state->theta, &id_actual, &iq_actual);
    
    // Step 3: Current PI controllers
    state->vd = pi_controller(id_pid, state->id, id_actual);
    state->vq = pi_controller(iq_pid, torque_cmd, iq_actual);
    
    // Step 4: Rotor observer
    rotor_observer(alpha, beta, state->id, state->iq, state);
    
    // Step 5: Space Vector PWM
    SVPWMOutput pwm_output;
    space_vector_pwm(state->vd, state->vq, &pwm_output);
    
    // Step 6: Apply PWM to hardware
    volatile uint16_t *ccr1 = (volatile uint16_t*)(TIM1_BASE + 0x34);
    volatile uint16_t *ccr2 = (volatile uint16_t*)(TIM1_BASE + 0x38);
    volatile uint16_t *ccr3 = (volatile uint16_t*)(TIM1_BASE + 0x3C);
    
    *ccr1 = (uint16_t)(pwm_output.vx * 16800);
    *ccr2 = (uint16_t)(pwm_output.vy * 16800);
    *ccr3 = (uint16_t)(pwm_output.vz * 16800);
}
