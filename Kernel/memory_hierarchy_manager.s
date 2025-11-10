; memory_hierarchy_manager.s
; Cache, SRAM, FLASH management with explicit addressing
; Memory hierarchy optimization for real-time systems

.section .text

; Memory regions: 0x6000 - 0x60FF
.equ MEM_STAT_CTRL,     0x6000
.equ MEM_STAT_FREE,     0x6001
.equ MEM_STAT_USED,     0x6002
.equ CACHE_STATUS,      0x6003
.equ SRAM_BASE,         0x6004

; L1 Cache: 0x6100 - 0x61FF (256 bytes)
.equ L1_CACHE_BASE,     0x6100
.equ L1_CACHE_CTRL,     0x6200
.equ L1_CACHE_TAG,      0x6201

; L2 Cache simulation: 0x6300 - 0x63FF
.equ L2_CACHE_BASE,     0x6300
.equ L2_CACHE_CTRL,     0x6400
.equ L2_CACHE_TAG,      0x6401

; SRAM heap: 0x6500 - 0x6FFF
.equ HEAP_START,        0x6500
.equ HEAP_SIZE,         0x0B00
.equ HEAP_PTR,          0x7000
.equ HEAP_FRAGMENTATION, 0x7001

; FLASH storage: 0x8000 - 0x87FF (logging)
.equ FLASH_LOG_BASE,    0x8000
.equ FLASH_LOG_PTR,     0x8001

.global cache_flush
cache_flush:
    ; Flush L1 cache
    
    ldi r26, L1_CACHE_CTRL
    clr r24
    st X, r24
    
    ldi r26, L2_CACHE_CTRL
    st X, r24
    
    ret

.global cache_prefetch
cache_prefetch:
    ; Prefetch data into L1 cache
    ; r25:r24 = address, r23 = size
    
    mov X, r24
    mov r16, r25
    
    ldi r26, L1_CACHE_BASE
    ldi r20, 0
    
prefetch_loop:
    cpi r20, r23
    brge prefetch_done
    
    ld r0, X+
    st Z+, r0
    
    inc r20
    jmp prefetch_loop
    
prefetch_done:
    ret

.global malloc_block
malloc_block:
    ; r24 = size (in bytes)
    ; Returns pointer in r25:r24
    
    ldi r26, HEAP_PTR
    ld r24, X+
    ld r25, X
    
    ; Check if enough space
    mov r0, r25
    mov r1, r24
    add r1, r24
    adc r0, 0x00
    
    cpi r0, 0x6F
    brge malloc_fail
    
    ; Update heap pointer
    mov r25, r1
    mov r24, r0
    
    ldi r26, HEAP_PTR
    st X+, r25
    st X, r24
    
    ret
    
malloc_fail:
    ldi r25, 0xFF
    ldi r24, 0xFF
    ret

.global free_block
free_block:
    ; r25:r24 = pointer
    ; Simple free (mark as available)
    
    mov X, r24
    mov r16, r25
    
    clr r0
    st X, r0
    
    ret

.global memcpy_cached
memcpy_cached:
    ; r25:r24 = dest, r23:r22 = src, r21:r20 = size
    ; Uses cache for acceleration
    
    mov X, r24
    mov Z, r22
    
    mov r0, r21
    mov r1, r20
    
    ldi r26, L1_CACHE_BASE
    
copy_loop:
    tst r0
    brne copy_continue
    tst r1
    breq copy_done
    
copy_continue:
    ld r2, Z+
    st X+, r2
    
    dec r1
    brne copy_loop
    
    dec r0
    jmp copy_loop
    
copy_done:
    ret

.global memory_defragment
memory_defragment:
    ; Compress heap to reduce fragmentation
    
    ldi r26, HEAP_FRAGMENTATION
    ld r24, X
    
    cpi r24, 0x80
    brlt defrag_ok
    
    ; Trigger defragmentation
    ldi r26, HEAP_PTR
    ldi r24, 0x65
    st X+, r24
    
    ldi r24, 0x00
    st X, r24
    
defrag_ok:
    ret

.global get_memory_usage
get_memory_usage:
    ; Returns memory usage percentage in r24
    
    ldi r26, HEAP_PTR
    ld r25, X+
    ld r24, X
    
    ; Calculate percentage
    ; (used / total) * 100
    
    sub r25, 0x65
    mov r0, r25
    ldi r24, 100
    mul r0, r24
    
    mov r24, r0
    ret

.global sram_ecc_check
sram_ecc_check:
    ; Check SRAM error-correcting code
    ; r25:r24 = address
    
    mov X, r24
    
    clr r0
    ldi r26, 256
    
ecc_loop:
    ld r1, X+
    eor r0, r1
    
    dec r26
    brne ecc_loop
    
    mov r24, r0
    ret

.global flash_write_log
flash_write_log:
    ; r24 = data byte to log to FLASH
    
    ldi r26, FLASH_LOG_PTR
    ld r25, X
    
    ldi r26, FLASH_LOG_BASE
    add r26, r25
    
    st X, r24
    
    ; Increment log pointer
    ldi r26, FLASH_LOG_PTR
    inc r25
    cpi r25, 0x08
    brlt flash_log_done
    
    clr r25
    
flash_log_done:
    st X, r25
    ret

.global flash_read_log
flash_read_log:
    ; r24 = log_index (0-7)
    ; Returns data in r24
    
    ldi r26, FLASH_LOG_BASE
    add r26, r24
    
    ld r24, X
    ret

.global memory_barrier
memory_barrier:
    ; Ensure all memory operations complete
    ; (volatile barrier)
    
    ldi r26, MEM_STAT_CTRL
    ldi r24, 0xFF
    st X, r24
    
    clr r24
    st X, r24
    
    ret
