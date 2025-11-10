; raw_mem_ops.s
; Assembly file for raw memory operations and hardware tricks

.section .text
.global raw_mem_write
raw_mem_write:
    ; r24 = value, r25:r26 = address
    st X, r24
    ret

.global raw_mem_toggle
raw_mem_toggle:
    ; r25:r26 = address
    ld r24, X
    eor r24, 0xFF
    st X, r24
    ret

.global raw_mem_spin
raw_mem_spin:
    ldi r24, 0xFF
spin_loop:
    dec r24
    brne spin_loop
    ret
