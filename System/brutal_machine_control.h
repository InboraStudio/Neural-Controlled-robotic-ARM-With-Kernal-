// brutal_machine_control.h
// Ultra-low-level robotics control declarations

#ifndef BRUTAL_MACHINE_CONTROL_H
#define BRUTAL_MACHINE_CONTROL_H

#include <stdint.h>

// PID Controller structure
typedef struct {
    int32_t p_gain;
    int32_t i_gain;
    int32_t d_gain;
    int16_t setpoint;
    int16_t prev_error;
} PID_Controller;

// 3D Vector for robotics
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Vector3i;

// 3x3 Matrix for transformations
typedef struct {
    int32_t xx, xy, xz;
    int32_t yx, yy, yz;
    int32_t zx, zy, zz;
} Matrix3x3i;

// Sensor fusion with covariance
typedef struct {
    int16_t x_est;
    int16_t y_est;
    int16_t z_est;
    uint16_t cov_xx;
    uint16_t cov_yy;
    uint16_t cov_zz;
} SensorFusion;

// Trajectory controller
typedef struct {
    int16_t pos[4];
    int16_t vel[4];
    int16_t coeffs[8];
    uint16_t time;
    uint16_t duration;
} Trajectory;

// Neural network
typedef struct {
    uint16_t weights_addr;
    uint16_t state_addr;
    uint8_t num_neurons;
    uint8_t num_layers;
} NeuralNet;

// Function declarations
void brutal_pid_controller(PID_Controller *pid_a, PID_Controller *pid_b, 
                          int16_t feedback_a, int16_t feedback_b);

Vector3i vector_add(Vector3i a, Vector3i b);
Vector3i vector_subtract(Vector3i a, Vector3i b);
int32_t vector_dot(Vector3i a, Vector3i b);
Vector3i vector_cross(Vector3i a, Vector3i b);
uint16_t vector_magnitude(Vector3i v);

Vector3i matrix_multiply_vector(Matrix3x3i m, Vector3i v);
Matrix3x3i matrix_multiply(Matrix3x3i a, Matrix3x3i b);

void brutal_sensor_fusion(SensorFusion *fusion, 
                         Vector3i measurement,
                         int16_t measurement_noise);

void update_trajectory(Trajectory *traj);

void neural_forward_pass(NeuralNet *net, int16_t *input, int16_t *output);

void brutal_pwm_sweep(uint16_t min_pwm, uint16_t max_pwm, uint8_t duration_ms);
void brutal_torque_ripple_compensation(int16_t *commutation_table, 
                                      uint8_t position, 
                                      uint16_t *pwm_a, 
                                      uint16_t *pwm_b);
void brutal_fault_injection(void);

void brutal_memcpy_dma(uint16_t *dest, uint16_t *src, uint16_t count);
void brutal_memset_pattern(volatile uint8_t *addr, uint32_t pattern, uint16_t count);
uint32_t brutal_crc32(uint8_t *data, uint16_t length);

void brutal_control_loop(void);

#endif // BRUTAL_MACHINE_CONTROL_H
