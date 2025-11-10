

#include <stdint.h>

#define MOTOR_CTRL_REG   (*(volatile uint8_t*)0x0020)
#define SENSOR_DATA_REG  (*(volatile uint16_t*)0x0030)
#define STATUS_FLAG_REG  (*(volatile uint8_t*)0x0040)

void poke_motor(uint8_t value) {
    MOTOR_CTRL_REG = value;
}

uint16_t read_sensor() {
    return SENSOR_DATA_REG;
}

void set_status_flag(uint8_t flag) {
    STATUS_FLAG_REG |= flag;
}

void clear_status_flag(uint8_t flag) {
    STATUS_FLAG_REG &= ~flag;
}

void glitch_hardware() {
    for (volatile int i = 0; i < 1000; ++i) {
        MOTOR_CTRL_REG ^= 0xFF;
    }
}
