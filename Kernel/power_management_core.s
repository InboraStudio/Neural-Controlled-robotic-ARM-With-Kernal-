; power_management_core.s
; Battery, power distribution, and energy management
; Direct register control with explicit hardware addresses

.section .text

; Power distribution hardware addresses
; Battery monitoring: 0x1000 - 0x100F
; Power supply regulators: 0x1010 - 0x101F
; Current monitors: 0x1020 - 0x102F

.equ BATT_VOLTAGE_H,    0x1000
.equ BATT_VOLTAGE_L,    0x1001
.equ BATT_CURRENT_H,    0x1002
.equ BATT_CURRENT_L,    0x1003
.equ BATT_TEMP,         0x1004
.equ BATT_SOC,          0x1005
.equ BATT_STATUS,       0x1006

.equ VREG_5V_CTRL,      0x1010
.equ VREG_3V3_CTRL,     0x1011
.equ VREG_12V_CTRL,     0x1012
.equ VREG_STATUS,       0x1013

.equ CURRENT_MOTOR_A,   0x1020
.equ CURRENT_MOTOR_B,   0x1021
.equ CURRENT_MOTOR_C,   0x1022
.equ CURRENT_LOGIC,     0x1023
.equ CURRENT_SENSORS,   0x1024

; global read_battery_voltage
.global read_battery_voltage
read_battery_voltage:
    ; Returns battery voltage in mV
    ; r25:r24 = voltage
    
    ldi r26, BATT_VOLTAGE_H
    ld r25, X+
    ld r24, X
    
    ; Convert to mV (multiply by 14.6)
    mov r0, r25
    mov r1, r24
    
    mul r0, 14
    mov r2, r0
    mov r3, r1
    
    ret

; global read_battery_current
.global read_battery_current
read_battery_current:
    ; Returns current in mA
    ; Positive = discharge, Negative = charge
    
    ldi r26, BATT_CURRENT_H
    ld r25, X+
    ld r24, X
    
    ret

; global read_battery_soc
.global read_battery_soc
read_battery_soc:
    ; Returns state of charge (0-100%)
    
    ldi r26, BATT_SOC
    ld r24, X
    
    ret

; global enable_5v_rail
.global enable_5v_rail
enable_5v_rail:
    ; Enable 5V power supply
    
    ldi r26, VREG_5V_CTRL
    ldi r24, 0x01
    st X, r24
    
    ret

; global enable_3v3_rail
.global enable_3v3_rail
enable_3v3_rail:
    ; Enable 3.3V power supply
    
    ldi r26, VREG_3V3_CTRL
    ldi r24, 0x01
    st X, r24
    
    ret

; global enable_12v_rail
.global enable_12v_rail
enable_12v_rail:
    ; Enable 12V boost converter
    
    ldi r26, VREG_12V_CTRL
    ldi r24, 0x01
    st X, r24
    
    ; Wait for stabilization
    ldi r26, 255
wait_12v:
    dec r26
    brne wait_12v
    
    ret

; global disable_all_rails
.global disable_all_rails
disable_all_rails:
    ; Emergency power shutdown
    
    ldi r26, VREG_5V_CTRL
    clr r24
    st X, r24
    
    ldi r26, VREG_3V3_CTRL
    st X, r24
    
    ldi r26, VREG_12V_CTRL
    st X, r24
    
    ret

; global read_motor_currents
.global read_motor_currents
read_motor_currents:
    ; Returns current for motors A, B, C
    ; r25=A, r24=B, r23=C (in 100mA units)
    
    ldi r26, CURRENT_MOTOR_A
    ld r25, X
    
    ldi r26, CURRENT_MOTOR_B
    ld r24, X
    
    ldi r26, CURRENT_MOTOR_C
    ld r23, X
    
    ret

; global compute_power_budget
.global compute_power_budget
compute_power_budget:
    ; Computes remaining available power
    ; Returns power margin in r24 (percentage)
    
    ldi r26, BATT_VOLTAGE_H
    ld r25, X+
    ld r24, X
    
    ldi r26, BATT_CURRENT_H
    ld r16, X+
    ld r17, X
    
    ; Calculate power: V * I
    mul r25, r16
    mov r0, r0
    
    ; Compare against max (arbitrary limit)
    cpi r0, 200
    brlt power_ok
    
    clr r24
    ret
    
power_ok:
    ldi r24, 100
    sub r24, r0
    ret

; global battery_low_warning
.global battery_low_warning
battery_low_warning:
    ; Checks if battery is critically low
    ; Returns non-zero if low
    
    ldi r26, BATT_SOC
    ld r24, X
    
    cpi r24, 20
    brge batt_ok
    
    ldi r24, 0xFF
    ret
    
batt_ok:
    clr r24
    ret
