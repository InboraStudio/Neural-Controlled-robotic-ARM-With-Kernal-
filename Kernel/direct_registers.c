// direct_registers.c
// Direct register access for robotics fun

#include <stdint.h>
#include "robot_mem_map.h"

volatile uint8_t* const motor_reg = (uint8_t*)ROBOT_MOTOR_ADDR;
volatile uint16_t* const sensor_reg = (uint16_t*)ROBOT_SENSOR_ADDR;
volatile uint8_t* const status_reg = (uint8_t*)ROBOT_STATUS_ADDR;
volatile uint8_t* const debug_reg = (uint8_t*)ROBOT_DEBUG_ADDR;
volatile uint8_t* const fault_reg = (uint8_t*)ROBOT_FAULT_ADDR;

void set_motor(uint8_t val) {
    *motor_reg = val;
}

uint16_t get_sensor() {
    return *sensor_reg;
}

void debug_toggle() {
    *debug_reg ^= 0xAA;
}

void induce_fault() {
    *fault_reg = 0xFF;
}
