// hardware_magic.h
// Header for ultra low-level robotics tricks

#ifndef HARDWARE_MAGIC_H
#define HARDWARE_MAGIC_H

#include <stdint.h>

void poke_motor(uint8_t value);
uint16_t read_sensor(void);
void set_status_flag(uint8_t flag);
void clear_status_flag(uint8_t flag);
void glitch_hardware(void);

#endif // HARDWARE_MAGIC_H
