; motor_address_engine.s
; Direct motor control using explicit memory addresses
; Ultra-low-level PWM and commutation control

.section .text

; Motor hardware mapping
; Motor A: 0x0620 - 0x062F
; Motor B: 0x0630 - 0x063F
; Motor C: 0x0640 - 0x064F

.equ MOTOR_A_CTRL,      0x0620
.equ MOTOR_A_PWM_H,     0x0621
.equ MOTOR_A_PWM_L,     0x0622
.equ MOTOR_A_DIR,       0x0623
.equ MOTOR_A_STATUS,    0x0624

.equ MOTOR_B_CTRL,      0x0630
.equ MOTOR_B_PWM_H,     0x0631
.equ MOTOR_B_PWM_L,     0x0632
.equ MOTOR_B_DIR,       0x0633
.equ MOTOR_B_STATUS,    0x0634

.equ MOTOR_C_CTRL,      0x0640
.equ MOTOR_C_PWM_H,     0x0641
.equ MOTOR_C_PWM_L,     0x0642
.equ MOTOR_C_DIR,       0x0643
.equ MOTOR_C_STATUS,    0x0644

; global motor_write_pwm
.global motor_write_pwm
motor_write_pwm:
    ; r24 = motor_id, r25:r26 = pwm_value
    ; Motor ID: 0=A, 1=B, 2=C
    
    cpi r24, 0x00
    brne check_motor_b
    
    ; Write to Motor A
    ldi r20, MOTOR_A_PWM_H
    st X, r25
    ldi r20, MOTOR_A_PWM_L
    st X, r26
    jmp pwm_done
    
check_motor_b:
    cpi r24, 0x01
    brne check_motor_c
    
    ; Write to Motor B
    ldi r20, MOTOR_B_PWM_H
    st X, r25
    ldi r20, MOTOR_B_PWM_L
    st X, r26
    jmp pwm_done
    
check_motor_c:
    ; Write to Motor C
    ldi r20, MOTOR_C_PWM_H
    st X, r25
    ldi r20, MOTOR_C_PWM_L
    st X, r26
    
pwm_done:
    ret

; global motor_set_direction
.global motor_set_direction
motor_set_direction:
    ; r24 = motor_id, r25 = direction (0=fwd, 1=rev)
    
    cpi r24, 0x00
    brne dir_motor_b
    
    ldi r20, MOTOR_A_DIR
    st X, r25
    jmp dir_done
    
dir_motor_b:
    cpi r24, 0x01
    brne dir_motor_c
    
    ldi r20, MOTOR_B_DIR
    st X, r25
    jmp dir_done
    
dir_motor_c:
    ldi r20, MOTOR_C_DIR
    st X, r25
    
dir_done:
    ret

; global motor_read_status
.global motor_read_status
motor_read_status:
    ; r24 = motor_id
    ; returns status in r24
    
    cpi r24, 0x00
    brne status_motor_b
    
    ldi r20, MOTOR_A_STATUS
    ld r24, X
    jmp status_done
    
status_motor_b:
    cpi r24, 0x01
    brne status_motor_c
    
    ldi r20, MOTOR_B_STATUS
    ld r24, X
    jmp status_done
    
status_motor_c:
    ldi r20, MOTOR_C_STATUS
    ld r24, X
    
status_done:
    ret

; global motor_emergency_stop
.global motor_emergency_stop
motor_emergency_stop:
    ; Kill all motors immediately
    
    ldi r20, MOTOR_A_CTRL
    ldi r24, 0xFF
    st X, r24
    
    ldi r20, MOTOR_B_CTRL
    st X, r24
    
    ldi r20, MOTOR_C_CTRL
    st X, r24
    
    ret

; global motor_ramp_speed
.global motor_ramp_speed
motor_ramp_speed:
    ; r24 = motor_id, r25 = target_speed, r23 = ramp_rate
    ; Gradually ramp motor to target speed
    
    push r16
    push r17
    
    mov r16, r25
    clr r17
    
ramp_loop:
    cpi r17, r23
    brge ramp_done
    
    ; Write current speed
    mov r25, r17
    call motor_write_pwm
    
    ; Busy wait
    ldi r26, 100
wait_ramp:
    dec r26
    brne wait_ramp
    
    inc r17
    jmp ramp_loop
    
ramp_done:
    pop r17
    pop r16
    ret
