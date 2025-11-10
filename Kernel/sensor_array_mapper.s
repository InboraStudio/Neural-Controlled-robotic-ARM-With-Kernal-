; sensor_array_mapper.s
; Direct sensor address mapping and raw data acquisition
; Explicit memory addresses for IMU, encoders, analog sensors

.section .text

; Sensor hardware addresses
; IMU Accelerometer: 0x0700 - 0x0705
; IMU Gyroscope: 0x0706 - 0x070B
; IMU Magnetometer: 0x070C - 0x0711

.equ IMU_ACCEL_X_H,     0x0700
.equ IMU_ACCEL_X_L,     0x0701
.equ IMU_ACCEL_Y_H,     0x0702
.equ IMU_ACCEL_Y_L,     0x0703
.equ IMU_ACCEL_Z_H,     0x0704
.equ IMU_ACCEL_Z_L,     0x0705

.equ IMU_GYRO_X_H,      0x0706
.equ IMU_GYRO_X_L,      0x0707
.equ IMU_GYRO_Y_H,      0x0708
.equ IMU_GYRO_Y_L,      0x0709
.equ IMU_GYRO_Z_H,      0x070A
.equ IMU_GYRO_Z_L,      0x070B

.equ IMU_MAG_X_H,       0x070C
.equ IMU_MAG_X_L,       0x070D
.equ IMU_MAG_Y_H,       0x070E
.equ IMU_MAG_Y_L,       0x070F
.equ IMU_MAG_Z_H,       0x0710
.equ IMU_MAG_Z_L,       0x0711

; Encoder addresses: 0x0800 - 0x080B (4 encoders)
.equ ENCODER_0_H,       0x0800
.equ ENCODER_0_L,       0x0801
.equ ENCODER_1_H,       0x0802
.equ ENCODER_1_L,       0x0803
.equ ENCODER_2_H,       0x0804
.equ ENCODER_2_L,       0x0805
.equ ENCODER_3_H,       0x0806
.equ ENCODER_3_L,       0x0807

; Analog sensor inputs: 0x0900 - 0x090F (8 channels)
.equ ANALOG_CH0_H,      0x0900
.equ ANALOG_CH0_L,      0x0901
.equ ANALOG_CH1_H,      0x0902
.equ ANALOG_CH1_L,      0x0903
.equ ANALOG_CH2_H,      0x0904
.equ ANALOG_CH2_L,      0x0905
.equ ANALOG_CH3_H,      0x0906
.equ ANALOG_CH3_L,      0x0907
.equ ANALOG_CH4_H,      0x0908
.equ ANALOG_CH4_L,      0x0909
.equ ANALOG_CH5_H,      0x090A
.equ ANALOG_CH5_L,      0x090B
.equ ANALOG_CH6_H,      0x090C
.equ ANALOG_CH6_L,      0x090D
.equ ANALOG_CH7_H,      0x090E
.equ ANALOG_CH7_L,      0x090F

; Temperature sensors: 0x0A00 - 0x0A0F
.equ TEMP_MOTOR_A,      0x0A00
.equ TEMP_MOTOR_B,      0x0A01
.equ TEMP_MOTOR_C,      0x0A02
.equ TEMP_PCB,          0x0A03
.equ TEMP_BATTERY,      0x0A04

; global read_accelerometer
.global read_accelerometer
read_accelerometer:
    ; Returns 16-bit values in r25:r24 (X), r23:r22 (Y), r21:r20 (Z)
    
    ldi r26, IMU_ACCEL_X_H
    ld r25, X+
    ld r24, X+
    
    ldi r26, IMU_ACCEL_Y_H
    ld r23, X+
    ld r22, X+
    
    ldi r26, IMU_ACCEL_Z_H
    ld r21, X+
    ld r20, X
    
    ret

; global read_gyroscope
.global read_gyroscope
read_gyroscope:
    ; Returns gyro data
    
    ldi r26, IMU_GYRO_X_H
    ld r25, X+
    ld r24, X+
    
    ldi r26, IMU_GYRO_Y_H
    ld r23, X+
    ld r22, X+
    
    ldi r26, IMU_GYRO_Z_H
    ld r21, X+
    ld r20, X
    
    ret

; global read_encoder
.global read_encoder
read_encoder:
    ; r24 = encoder_id (0-3)
    ; Returns encoder count in r25:r24
    
    cpi r24, 0x00
    brne enc_1
    ldi r26, ENCODER_0_H
    jmp enc_read
    
enc_1:
    cpi r24, 0x01
    brne enc_2
    ldi r26, ENCODER_1_H
    jmp enc_read
    
enc_2:
    cpi r24, 0x02
    brne enc_3
    ldi r26, ENCODER_2_H
    jmp enc_read
    
enc_3:
    ldi r26, ENCODER_3_H
    
enc_read:
    ld r25, X+
    ld r24, X
    ret

; global read_analog_channel
.global read_analog_channel
read_analog_channel:
    ; r24 = channel (0-7)
    ; Returns value in r25:r24
    
    lsl r24
    ldi r26, ANALOG_CH0_H
    add r26, r24
    
    ld r25, X+
    ld r24, X
    ret

; global read_all_temperatures
.global read_all_temperatures
read_all_temperatures:
    ; Reads all temperature sensors to buffer
    ; r25:r24 = buffer_address
    
    mov X, r24
    mov r16, r25
    
    ldi r26, TEMP_MOTOR_A
    ldi r23, 5
    
temp_loop:
    ld r0, X
    st X+, r0
    
    ldi r26, r26 + 1
    ld r0, X
    st X+, r0
    
    dec r23
    brne temp_loop
    
    ret

; global calibrate_sensors
.global calibrate_sensors
calibrate_sensors:
    ; Trigger sensor calibration sequences
    ; Stores offset values
    
    ; Accel offset
    ldi r26, IMU_ACCEL_X_H
    clr r24
    st X, r24
    
    ; Gyro offset
    ldi r26, IMU_GYRO_X_H
    st X, r24
    
    ; Mag calibration
    ldi r26, IMU_MAG_X_H
    st X, r24
    
    ret
