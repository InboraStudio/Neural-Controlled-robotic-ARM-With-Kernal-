// fault_diagnosis_engine.c
// Advanced fault detection, diagnosis, and recovery
// Self-healing robotics system with anomaly detection

#include <stdint.h>

// Fault monitoring addresses: 0xA000-0xA0FF
#define FAULT_CODE_REG        (*(volatile uint16_t*)0xA000)
#define FAULT_TIMESTAMP       (*(volatile uint32_t*)0xA002)
#define FAULT_SEVERITY        (*(volatile uint8_t*)0xA006)
#define FAULT_LOCATION        (*(volatile uint8_t*)0xA007)
#define FAULT_COUNTER         (*(volatile uint16_t*)0xA008)

// Health status: 0xA100-0xA1FF
#define MOTOR_A_HEALTH        (*(volatile uint8_t*)0xA100)
#define MOTOR_B_HEALTH        (*(volatile uint8_t*)0xA101)
#define MOTOR_C_HEALTH        (*(volatile uint8_t*)0xA102)
#define SENSOR_HEALTH         (*(volatile uint8_t*)0xA103)
#define POWER_HEALTH          (*(volatile uint8_t*)0xA104)
#define COMM_HEALTH           (*(volatile uint8_t*)0xA105)

// Fault history: 0xA200-0xA2FF (16 fault records)
#define FAULT_LOG_BASE        0xA200
#define FAULT_LOG_SIZE        16

// Anomaly detection thresholds: 0xA300-0xA3FF
#define ANOMALY_THRESHOLD_ACCEL  (*(volatile uint16_t*)0xA300)
#define ANOMALY_THRESHOLD_CURRENT (*(volatile uint16_t*)0xA302)
#define ANOMALY_THRESHOLD_TEMP    (*(volatile uint8_t*)0xA304)

// Fault codes
#define FAULT_MOTOR_STALL         0x0001
#define FAULT_MOTOR_OVERCURRENT   0x0002
#define FAULT_MOTOR_TEMPERATURE   0x0004 
#define FAULT_SENSOR_FAILURE      0x0008
#define FAULT_SENSOR_DRIFT        0x0010
#define FAULT_POWER_BROWNOUT      0x0020
#define FAULT_POWER_OVERLOAD      0x0040
#define FAULT_COMM_TIMEOUT        0x0080
#define FAULT_WATCHDOG_RESET      0x0100
#define FAULT_MEMORY_CORRUPTION   0x0200

typedef struct {
    uint16_t fault_code;
    uint32_t timestamp;
    uint8_t severity;
    uint8_t location;
    uint16_t counter;
} FaultRecord;

typedef struct {
    int16_t accel_x_prev;
    int16_t accel_y_prev;
    int16_t accel_z_prev;
    uint16_t accel_sum_sq;
    uint8_t sensor_reads;
} AnomalyDetectorState;

// Online anomaly detection using statistical methods
int16_t detect_acceleration_anomaly(int16_t accel_x, int16_t accel_y, int16_t accel_z) {
    
    // Compute Euclidean magnitude
    int32_t mag_sq = (int32_t)accel_x * accel_x + 
                     (int32_t)accel_y * accel_y + 
                     (int32_t)accel_z * accel_z;
    
    // Compare against threshold
    if (mag_sq > (int32_t)ANOMALY_THRESHOLD_ACCEL * ANOMALY_THRESHOLD_ACCEL) {
        return 1;  // Anomaly detected
    }
    
    return 0;
}

// Current-based stall detection
int16_t detect_motor_stall(uint8_t motor_id, uint16_t current_ma, uint16_t pwm_signal) {
    
    // Stall detected if high current but low PWM (mechanical resistance)
    if (current_ma > ANOMALY_THRESHOLD_CURRENT && pwm_signal < 100) {
        return 1;
    }
    
    return 0;
}

// Temperature-based fault detection
int16_t detect_thermal_fault(uint8_t temp_celsius) {
    
    if (temp_celsius > ANOMALY_THRESHOLD_TEMP) {
        return 1;
    }
    
    return 0;
}

// Sensor drift detection (slow degradation)
int16_t detect_sensor_drift(int16_t current_reading, int16_t *history, uint16_t history_size) {
    
    // Calculate mean
    int32_t sum = 0;
    for (uint16_t i = 0; i < history_size; i++) {
        sum += history[i];
    }
    int16_t mean = sum / history_size;
    
    // Calculate variance
    int32_t variance = 0;
    for (uint16_t i = 0; i < history_size; i++) {
        int16_t diff = history[i] - mean;
        variance += (int32_t)diff * diff;
    }
    variance /= history_size;
    
    // Check if current reading is statistical outlier
    int16_t diff_from_mean = current_reading - mean;
    
    if ((int32_t)diff_from_mean * diff_from_mean > variance * 4) {
        return 1;
    }
    
    return 0;
}

// Log fault event
void log_fault_event(uint16_t fault_code, uint8_t severity, uint8_t location) {
    
    // Record fault
    FAULT_CODE_REG = fault_code;
    FAULT_SEVERITY = severity;
    FAULT_LOCATION = location;
    
    // Store in fault history
    uint16_t log_index = FAULT_COUNTER % FAULT_LOG_SIZE;
    volatile uint8_t *log_entry = (volatile uint8_t*)(FAULT_LOG_BASE + log_index * 8);
    
    log_entry[0] = (fault_code >> 8) & 0xFF;
    log_entry[1] = fault_code & 0xFF;
    log_entry[2] = severity;
    log_entry[3] = location;
    
    FAULT_COUNTER++;
}

// Automatic recovery actions
void execute_recovery_action(uint16_t fault_code) {
    
    switch (fault_code) {
        
        case FAULT_MOTOR_STALL:
            // Reduce PWM, attempt restart
            volatile uint16_t *pwm = (volatile uint16_t*)0x0622;
            *pwm = (*pwm * 75) / 100;
            break;
            
        case FAULT_MOTOR_OVERCURRENT:
            // Emergency stop motor
            volatile uint8_t *ctrl = (volatile uint8_t*)0x0620;
            *ctrl = 0x00;
            break;
            
        case FAULT_MOTOR_TEMPERATURE:
            // Disable motor, allow cooling
            volatile uint8_t *ctrl2 = (volatile uint8_t*)0x0620;
            *ctrl2 = 0x00;
            
            // Busy-wait for cooldown
            for (volatile uint32_t i = 0; i < 100000; i++);
            break;
            
        case FAULT_SENSOR_FAILURE:
            // Switch to backup sensor or nominal values
            volatile uint16_t *sensor = (volatile uint16_t*)0x0700;
            *sensor = 0x0000;
            break;
            
        case FAULT_POWER_BROWNOUT:
            // Reduce power consumption
            volatile uint8_t *vreg = (volatile uint8_t*)0x1010;
            *vreg = 0x00;
            break;
            
        case FAULT_WATCHDOG_RESET:
            // Pet watchdog and continue
            volatile uint8_t *wd = (volatile uint8_t*)0x4000;
            *wd = 0xFF;
            break;
    }
}

// Health index computation
uint8_t compute_health_index(uint8_t fault_count, uint16_t runtime_hours) {
    
    // Health = 100 - (fault_count * 5 + runtime_hours / 100)
    uint8_t health = 100;
    
    uint8_t fault_penalty = (fault_count > 20) ? 100 : (fault_count * 5);
    uint8_t age_penalty = (runtime_hours > 2000) ? 100 : (runtime_hours / 20);
    
    if (health > fault_penalty) health -= fault_penalty;
    else health = 0;
    
    if (health > age_penalty) health -= age_penalty;
    else health = 0;
    
    return health;
}

// Predictive maintenance check
int16_t predict_maintenance_needed(uint8_t health_index, uint16_t fault_rate_per_hour) {
    
    // Maintenance needed if health < 30% or fault rate increasing
    if (health_index < 30 || fault_rate_per_hour > 5) {
        return 1;
    }
    
    return 0;
}

// Self-test routine
void execute_self_test() {
    
    // Test motor response
    volatile uint16_t *pwm = (volatile uint16_t*)0x0622;
    *pwm = 500;
    
    for (volatile uint32_t i = 0; i < 50000; i++);
    
    // Check encoder feedback
    volatile uint16_t *encoder = (volatile uint16_t*)0x0800;
    uint16_t encoder_val = *encoder;
    
    if (encoder_val > 0) {
        MOTOR_A_HEALTH = 100;
    } else {
        MOTOR_A_HEALTH = 0;
        log_fault_event(FAULT_MOTOR_STALL, 2, 0);
    }
    
    // Stop motor
    pwm = 0;
}

// Fault recovery state machine
void fault_recovery_state_machine(uint16_t *recovery_state) {
    
    switch (*recovery_state) {
        
        case 0:
            // Idle state
            break;
            
        case 1:
            // Detected fault, log it
            log_fault_event(FAULT_CODE_REG, FAULT_SEVERITY, FAULT_LOCATION);
            execute_recovery_action(FAULT_CODE_REG);
            *recovery_state = 2;
            break;
            
        case 2:
            // Recovery in progress
            for (volatile uint32_t i = 0; i < 50000; i++);
            *recovery_state = 3;
            break;
            
        case 3:
            // Verify recovery
            if (FAULT_CODE_REG == 0) {
                *recovery_state = 0;
            } else {
                *recovery_state = 4;
            }
            break;
            
        case 4:
            // Recovery failed, need manual intervention
            volatile uint8_t *led = (volatile uint8_t*)0xA010;
            *led = 0xFF;
            break;
    }
}

// Compute system reliability metrics
typedef struct {
    uint32_t total_uptime_ms;
    uint16_t fault_count;
    uint32_t mean_time_between_failures;
    uint8_t availability_percent;
} ReliabilityMetrics;

void compute_reliability_metrics(ReliabilityMetrics *metrics) {
    
    metrics->fault_count = FAULT_COUNTER;
    
    if (metrics->fault_count > 0) {
        metrics->mean_time_between_failures = 
            metrics->total_uptime_ms / metrics->fault_count;
    } else {
        metrics->mean_time_between_failures = 0xFFFFFFFF;
    }
    
    // Availability = (total_uptime - downtime) / total_uptime * 100
    // Simplified: availability = 100 - (fault_count * 2)
    metrics->availability_percent = (metrics->fault_count < 50) ? 
        (100 - (metrics->fault_count * 2)) : 0;
}
