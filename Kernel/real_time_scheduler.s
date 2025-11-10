; real_time_scheduler.s
; Real-time task scheduling with explicit memory address management
; Microsecond-precise timing and interrupt handling

.section .text

; Real-time scheduler memory: 0x3000 - 0x300F
.equ SCHED_STATUS,      0x3000
.equ SCHED_TICK_CNT,    0x3001
.equ SCHED_TASK_ID,     0x3002
.equ SCHED_PRIORITY,    0x3003

; Task control blocks: 0x3010 - 0x30FF (16 tasks)
.equ TCB_BASE,          0x3010
.equ TCB_SIZE,          0x10

; Timer hardware: 0x4000 - 0x400F
.equ TIMER_CTRL,        0x4000
.equ TIMER_STATUS,      0x4001
.equ TIMER_COUNT_H,     0x4002
.equ TIMER_COUNT_L,     0x4003
.equ TIMER_PERIOD_H,    0x4004
.equ TIMER_PERIOD_L,    0x4005
.equ TIMER_COMPARE,     0x4006

; Interrupt pending: 0x4010 - 0x401F
.equ INT_PENDING,       0x4010
.equ INT_MASK,          0x4011
.equ INT_PRIORITY,      0x4012

.global timer_init
timer_init:
    ; Initialize timer for 1kHz tick (1ms)
    ; Period = 16000 (for 16MHz clock)
    
    ldi r26, TIMER_CTRL
    clr r24
    st X, r24
    
    ; Set period to 16000 (0x3E80)
    ldi r26, TIMER_PERIOD_H
    ldi r24, 0x3E
    st X+, r24
    
    ldi r24, 0x80
    st X, r24
    
    ; Enable timer with interrupt
    ldi r26, TIMER_CTRL
    ldi r24, 0x03
    st X, r24
    
    ret

.global timer_get_microseconds
timer_get_microseconds:
    ; Returns current timer count in r25:r24 (microseconds)
    
    ldi r26, TIMER_COUNT_H
    ld r25, X+
    ld r24, X
    
    ret

.global delay_microseconds
delay_microseconds:
    ; r25:r24 = delay in microseconds
    
    push r16
    push r17
    
    mov r16, r25
    mov r17, r24
    
    ldi r26, TIMER_COUNT_H
    ld r18, X+
    ld r19, X
    
delay_loop:
    ldi r26, TIMER_COUNT_H
    ld r20, X+
    ld r21, X
    
    ; Check if enough time has passed
    cp r20, r16
    brne delay_continue
    cp r21, r17
    brge delay_done
    
delay_continue:
    jmp delay_loop
    
delay_done:
    pop r17
    pop r16
    ret

.global delay_milliseconds
delay_milliseconds:
    ; r24 = delay in milliseconds
    
    mov r20, r24
    
ms_loop:
    ; Delay 1ms (1000 microseconds)
    ldi r25, 0x03
    ldi r24, 0xE8
    call delay_microseconds
    
    dec r20
    brne ms_loop
    
    ret

.global register_task
register_task:
    ; r24 = task_id, r25:r26 = task_function, r23 = priority
    ; Registers a periodic task
    
    ; Calculate TCB address: TCB_BASE + (task_id * TCB_SIZE)
    lsl r24
    lsl r24
    lsl r24
    lsl r24
    
    ldi r26, TCB_BASE
    add r26, r24
    
    ; Store function pointer
    st X+, r25
    st X+, r26
    
    ; Store priority
    st X, r23
    
    ret

.global schedule_task
.global schedule_task
schedule_task:
    ; r24 = task_id
    ; Marks task as ready to run
    
    ldi r26, SCHED_TASK_ID
    st X, r24
    
    ret

.global interrupt_disable
.global interrupt_disable
interrupt_disable:
    ; Disable all interrupts
    
    ldi r26, INT_MASK
    clr r24
    st X, r24
    
    ret

.global interrupt_enable
.global interrupt_enable
interrupt_enable:
    ; Enable all interrupts
    
    ldi r26, INT_MASK
    ldi r24, 0xFF
    st X, r24
    
    ret

.global critical_section_enter
.global critical_section_enter
critical_section_enter:
    ; Enter critical section (disable interrupts)
    ; Returns previous mask in r24
    
    ldi r26, INT_MASK
    ld r24, X
    
    clr r25
    st X, r25
    
    ret

.global critical_section_exit
.global critical_section_exit
critical_section_exit:
    ; Exit critical section
    ; r24 = previous mask
    
    ldi r26, INT_MASK
    st X, r24
    
    ret

.global get_task_runtime
.global get_task_runtime
get_task_runtime:
    ; r24 = task_id
    ; Returns runtime in microseconds (r25:r24)
    
    lsl r24
    lsl r24
    lsl r24
    lsl r24
    
    ldi r26, TCB_BASE
    add r26, r24
    
    ldi r26, 6
    ld r25, X+
    ld r24, X
    
    ret

.global reset_system_tick
.global reset_system_tick
reset_system_tick:
    ; Reset system tick counter
    
    ldi r26, SCHED_TICK_CNT
    clr r24
    st X, r24
    
    ret

.global get_system_tick
.global get_system_tick
get_system_tick:
    ; Returns system tick count in r24
    
    ldi r26, SCHED_TICK_CNT
    ld r24, X
    
    ret

.global watchdog_pet
.global watchdog_pet
watchdog_pet:
    ; Feed the watchdog to prevent reset
    
    ldi r26, TIMER_CTRL
    ldi r24, 0xFF
    st X, r24
    
    ret
