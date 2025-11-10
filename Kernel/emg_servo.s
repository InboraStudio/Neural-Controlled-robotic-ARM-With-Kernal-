Archive member included to satisfy reference by file (symbol)

Memory Configuration

Name             Origin             Length             Attributes
FLASH            0x0000000000000000 0x0000000000008000 xr
RAM              0x0000000000800100 0x0000000000000800 rw !x
EEPROM           0x0000000000810000 0x0000000000000400 rw !x
*default*        0x0000000000000000 0xffffffffffffffff

Linker script and memory map

                0x0000000000800900                __stack = (ORIGIN (RAM) + LENGTH (RAM))

.vectors        0x0000000000000000       0x68
                0x0000000000000000                KEEP (*(.vectors))
 *(.vectors)
 .vectors       0x0000000000000000       0x68 build/startup.o
                0x0000000000000068                . = ALIGN (0x2)

.init           0x0000000000000068       0x50
                0x0000000000000068                KEEP (*(.init0))
 *(.init0)
 .init0         0x0000000000000068       0x50 build/startup.o
                0x0000000000000068                __reset_handler
                0x00000000000000b8                . = ALIGN (0x2)

.text           0x00000000000000b8      0x6e4
 *(.text)
 .text          0x00000000000000b8        0x0 build/main.o
 .text          0x00000000000000b8        0x0 build/peripherals.o
 .text          0x00000000000000b8        0x0 build/syscalls.o
 .text          0x00000000000000b8        0x0 build/startup.o
 *(.text*)
 .text.gpio_init
                0x00000000000000b8       0x1e build/peripherals.o
                0x00000000000000b8                gpio_init
 .text.adc_init
                0x00000000000000d6       0x36 build/peripherals.o
                0x00000000000000d6                adc_init
 .text.timer1_pwm_init
                0x000000000000010c       0x28 build/peripherals.o
                0x000000000000010c                timer1_pwm_init
 .text.servo_set_position
                0x0000000000000134       0x2a build/peripherals.o
                0x0000000000000134                servo_set_position
 .text.iir_lpf_process
                0x000000000000015e       0x48 build/peripherals.o
 .text.ADC_vect_impl
                0x00000000000001a6      0x124 build/main.o
                0x00000000000001a6                ADC_vect_impl
 .text.main     0x00000000000002ca      0x0d2 build/main.o
                0x00000000000002ca                main
 .text._sbrk    0x000000000000039c       0x1e build/syscalls.o
                0x000000000000039c                _sbrk
 .text._exit    0x00000000000003ba        0x6 build/syscalls.o
                0x00000000000003ba                _exit
                0x000000000000079c                . = ALIGN (0x2)

.rodata         0x000000000000079c        0x0
 *(.rodata)
 *(.rodata*)
                0x000000000000079c                . = ALIGN (0x2)

.data           0x0000000000800100       0x84 load address 0x000000000000079c
                0x0000000000800100                __data_start = .
 *(.data)
 .data          0x0000000000800100        0x0 build/main.o
 .data          0x0000000000800100        0x0 build/peripherals.o
 .data          0x0000000000800100        0x0 build/syscalls.o
 .data          0x0000000000800100        0x0 build/startup.o
 *(.data*)
 .data.baseline_buffer
                0x0000000000800100       0x80 build/main.o
 .data.envelope_filter
                0x0000000000800180        0x4 build/main.o
                0x0000000000800184                . = ALIGN (0x2)
                0x0000000000800184                __data_end = .

                0x000000000000079c                __data_load = LOADADDR (.data)

.bss            0x0000000000800184      0x0d4
                0x0000000000800184                __bss_start = .
 *(.bss)
 .bss           0x0000000000800184        0x0 build/main.o
 .bss           0x0000000000800184        0x0 build/peripherals.o
 .bss           0x0000000000800184        0x0 build/syscalls.o
 .bss           0x0000000000800184        0x0 build/startup.o
 *(.bss*)
 .bss.g_adc_raw
                0x0000000000800184        0x2 build/main.o
                0x0000000000800184                g_adc_raw
 .bss.g_envelope
                0x0000000000800186        0x2 build/main.o
                0x0000000000800186                g_envelope
 .bss.g_new_sample_ready
                0x0000000000800188        0x1 build/main.o
                0x0000000000800188                g_new_sample_ready
 .bss.g_sample_count
                0x000000000080018c        0x4 build/main.o
                0x000000000080018c                g_sample_count
 .bss.baseline_sum
                0x0000000000800190        0x4 build/main.o
 .bss.baseline_idx
                0x0000000000800194        0x1 build/main.o
 .bss.baseline_avg
                0x0000000000800196        0x2 build/main.o
 *(COMMON)
 COMMON         0x0000000000800198       0xc0 build/syscalls.o
                0x0000000000800198                heap_end.0
                0x0000000000800258                . = ALIGN (0x2)
                0x0000000000800258                __bss_end = .

OUTPUT(../output/emg_servo.elf elf32-avr)

Cross Reference Table

Symbol                        File
ADC_vect_impl                 build/main.o
__bad_interrupt               build/startup.o
__reset_handler               build/startup.o
_exit                         build/syscalls.o
_sbrk                         build/syscalls.o
adc_init                      build/peripherals.o
                              build/main.o
g_adc_raw                     build/main.o
g_envelope                    build/main.o
gpio_init                     build/peripherals.o
                              build/main.o
main                          build/main.o
                              build/startup.o
servo_set_position            build/peripherals.o
                              build/main.o
timer1_pwm_init               build/peripherals.o
                              build/main.o

Memory map:
 .vectors       0x00000000    0x68  Interrupt vectors
 .init          0x00000068    0x50  Startup code
 .text          0x000000b8   0x6e4  Program code
 .rodata        0x0000079c     0x0  Constants (empty)
 .data (Flash)  0x0000079c    0x84  Initialized data (load address)
 .data (RAM)    0x00800100    0x84  Initialized data (runtime address)
 .bss           0x00800184    0xd4  Uninitialized data
 Stack top      0x00800900           (grows downward)

Flash usage: 0x820 bytes / 0x8000 (2080 / 32768 = 6.3%)
SRAM usage:  0x158 bytes / 0x800  (344 / 2048 = 16.8%)
