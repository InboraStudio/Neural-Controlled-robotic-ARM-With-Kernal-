; interrupt_vector_handler.s
; Interrupt vectors and exception handling with memory-mapped interrupts
; Direct hardware interrupt management

.section .text

; Interrupt vector table base: 0x5000
.equ INT_VECTOR_BASE,   0x5000
.equ INT_TIMER0,        0x5000
.equ INT_TIMER1,        0x5001
.equ INT_TIMER2,        0x5002
.equ INT_MOTOR_A,       0x5003
.equ INT_MOTOR_B,       0x5004
.equ INT_MOTOR_C,       0x5005
.equ INT_SENSOR_IMU,    0x5006
.equ INT_SENSOR_ENCODER, 0x5007
.equ INT_CAN_RX,        0x5008
.equ INT_I2C_DONE,      0x5009
.equ INT_UART_RX,       0x500A
.equ INT_FAULT,         0x500B

; Interrupt status registers: 0x5100 - 0x510F
.equ INT_STATUS,        0x5100
.equ INT_ACTIVE,        0x5101
.equ INT_PRIORITY_LVL,  0x5102
.equ INT_NESTING_DEPTH, 0x5103

.global timer0_interrupt_handler
timer0_interrupt_handler:
    ; 1kHz system tick
    
    push r24
    
    ldi r26, INT_STATUS
    ld r24, X
    ori r24, 0x01
    st X, r24
    
    ; Increment tick counter
    ldi r26, 0x3001
    ld r24, X
    inc r24
    st X, r24
    
    pop r24
    reti

.global timer1_interrupt_handler
timer1_interrupt_handler:
    ; High-resolution timer (PWM updates)
    
    push r24
    
    ldi r26, 0x0622
    ld r24, X
    
    ; Pulse-width update logic
    inc r24
    st X, r24
    
    pop r24
    reti

.global motor_fault_handler
motor_fault_handler:
    ; Emergency motor fault interrupt
    
    push r16
    
    ldi r26, 0x0620
    clr r24
    st X, r24
    
    ldi r26, 0x0630
    st X, r24
    
    ldi r26, 0x0640
    st X, r24
    
    ldi r26, INT_STATUS
    ldi r24, 0xFF
    st X, r24
    
    pop r16
    reti

.global can_rx_interrupt_handler
can_rx_interrupt_handler:
    ; CAN message received
    
    push r25
    push r24
    
    ldi r26, 0x2010
    ld r25, X+
    ld r24, X
    
    ; Process CAN frame
    ldi r26, INT_STATUS
    ldi r24, 0x08
    or r24, r24
    st X, r24
    
    pop r24
    pop r25
    reti

.global uart_rx_interrupt_handler
uart_rx_interrupt_handler:
    ; UART data received
    
    push r24
    
    ldi r26, 0x2305
    ld r24, X
    
    ; Echo back
    ldi r26, 0x2304
    st X, r24
    
    pop r24
    reti

.global sensor_imu_interrupt_handler
sensor_imu_interrupt_handler:
    ; IMU data ready
    
    push r16
    
    ; Read accelerometer
    ldi r26, 0x0700
    ld r16, X
    
    ; Trigger data processing
    ldi r26, INT_STATUS
    ldi r24, 0x40
    st X, r24
    
    pop r16
    reti

.global fault_recovery_handler
fault_recovery_handler:
    ; System-wide fault recovery
    
    push r24
    
    ; Disable all motors
    ldi r26, 0x0620
    clr r24
    st X, r24
    
    ldi r26, 0x0630
    st X, r24
    
    ldi r26, 0x0640
    st X, r24
    
    ; Log fault
    ldi r26, 0x5100
    ldi r24, 0xFF
    st X, r24
    
    pop r24
    reti

.global register_interrupt_handler
register_interrupt_handler:
    ; r24 = interrupt_id, r25:r26 = handler_address
    
    lsl r24
    ldi r26, INT_VECTOR_BASE
    add r26, r24
    
    st X+, r25
    st X, r26
    
    ret

.global enable_interrupt
enable_interrupt:
    ; r24 = interrupt_id
    
    ldi r26, 0x5102
    st X, r24
    
    ldi r26, INT_STATUS
    ldi r25, 0x01
    st X, r25
    
    ret

.global disable_interrupt
disable_interrupt:
    ; r24 = interrupt_id
    
    ldi r26, 0x5102
    clr r24
    st X, r24
    
    ret
