; stm32f407_dma_engine.s
; Direct Memory Access acceleration for high-speed transfers
; Multi-channel DMA with memory-to-peripheral and peripheral-to-memory

.syntax unified
.arm
.section .text

; DMA1: 0x40026000
; DMA2: 0x40026400

.equ DMA1_BASE,         0x40026000
.equ DMA2_BASE,         0x40026400

; DMA Stream base offsets (each stream is 0x18 bytes apart)
.equ DMA_S0CR,          0x00
.equ DMA_S0NDTR,        0x04
.equ DMA_S0PAR,         0x08
.equ DMA_S0M0AR,        0x0C
.equ DMA_S0M1AR,        0x10
.equ DMA_S0FCR,         0x14

.equ DMA_LISR,          0x00
.equ DMA_HISR,          0x04
.equ DMA_LIFCR,         0x08
.equ DMA_HIFCR,         0x0C

.global dma_configure_stream
dma_configure_stream:
    ; r0 = DMA_base, r1 = stream_id, r2 = peripheral_addr, r3 = memory_addr
    ; Stack: [buffer_size]
    
    push {r4-r8, lr}
    
    mov r4, r0
    mov r5, r1
    mov r6, r2
    mov r7, r3
    ldr r8, [sp, #24]
    
    ; Calculate stream offset
    mov r0, r5
    lsl r0, r0, #3
    lsl r0, r0, #1
    lsl r0, r0, #1
    add r0, r0, r4
    
    ; Disable stream first
    ldr r1, [r0, #DMA_S0CR]
    bic r1, r1, #0x00000001
    str r1, [r0, #DMA_S0CR]
    
wait_stream_disable:
    ldr r1, [r0, #DMA_S0CR]
    tst r1, #0x00000001
    bne wait_stream_disable
    
    ; Configure DMA_SxNDTR (number of data items)
    str r8, [r0, #DMA_S0NDTR]
    
    ; Configure DMA_SxPAR (peripheral address)
    str r6, [r0, #DMA_S0PAR]
    
    ; Configure DMA_SxM0AR (memory address)
    str r7, [r0, #DMA_S0M0AR]
    
    ; Configure DMA_SxCR
    ; Channel 7, M2P, medium priority, MINC
    ldr r1, =0x21C40000
    str r1, [r0, #DMA_S0CR]
    
    ; Set FIFO control
    ldr r1, =0x00000000
    str r1, [r0, #DMA_S0FCR]
    
    pop {r4-r8, pc}

.global dma_start_transfer
dma_start_transfer:
    ; r0 = DMA_base, r1 = stream_id
    
    push {r4, lr}
    
    mov r4, r1
    lsl r4, r4, #3
    lsl r4, r4, #1
    lsl r4, r4, #1
    add r4, r4, r0
    
    ldr r1, [r4, #DMA_S0CR]
    orr r1, r1, #0x00000001
    str r1, [r4, #DMA_S0CR]
    
    pop {r4, pc}

.global dma_stop_transfer
dma_stop_transfer:
    ; r0 = DMA_base, r1 = stream_id
    
    push {r4, lr}
    
    mov r4, r1
    lsl r4, r4, #3
    lsl r4, r4, #1
    lsl r4, r4, #1
    add r4, r4, r0
    
    ldr r1, [r4, #DMA_S0CR]
    bic r1, r1, #0x00000001
    str r1, [r4, #DMA_S0CR]
    
wait_dma_stop:
    ldr r1, [r4, #DMA_S0CR]
    tst r1, #0x00000001
    bne wait_dma_stop
    
    pop {r4, pc}

.global dma_check_transfer_complete
dma_check_transfer_complete:
    ; r0 = DMA_base, r1 = stream_id
    ; Returns transfer complete status in r0
    
    push {r4, lr}
    
    mov r4, r1
    lsl r4, r4, #2
    add r4, r4, r0
    
    ldr r0, [r4, #DMA_LISR]
    
    ; Extract TC bit for stream
    mov r1, #5
    mul r1, r1, r4
    add r1, r1, #5
    
    lsr r0, r0, r1
    and r0, r0, #1
    
    pop {r4, pc}

.global dma_set_memory_address
dma_set_memory_address:
    ; r0 = DMA_base, r1 = stream_id, r2 = memory_address
    
    push {r4, lr}
    
    mov r4, r1
    lsl r4, r4, #3
    lsl r4, r4, #1
    lsl r4, r4, #1
    add r4, r4, r0
    
    str r2, [r4, #DMA_S0M0AR]
    
    pop {r4, pc}

.global dma_set_peripheral_address
dma_set_peripheral_address:
    ; r0 = DMA_base, r1 = stream_id, r2 = peripheral_address
    
    push {r4, lr}
    
    mov r4, r1
    lsl r4, r4, #3
    lsl r4, r4, #1
    lsl r4, r4, #1
    add r4, r4, r0
    
    str r2, [r4, #DMA_S0PAR]
    
    pop {r4, pc}

.global dma_set_transfer_size
dma_set_transfer_size:
    ; r0 = DMA_base, r1 = stream_id, r2 = num_items
    
    push {r4, lr}
    
    mov r4, r1
    lsl r4, r4, #3
    lsl r4, r4, #1
    lsl r4, r4, #1
    add r4, r4, r0
    
    str r2, [r4, #DMA_S0NDTR]
    
    pop {r4, pc}

.global dma_get_transfer_count
dma_get_transfer_count:
    ; r0 = DMA_base, r1 = stream_id
    ; Returns remaining transfer count in r0
    
    push {r4, lr}
    
    mov r4, r1
    lsl r4, r4, #3
    lsl r4, r4, #1
    lsl r4, r4, #1
    add r4, r4, r0
    
    ldr r0, [r4, #DMA_S0NDTR]
    and r0, r0, #0x0000FFFF
    
    pop {r4, pc}

.global dma_clear_interrupt_flags
dma_clear_interrupt_flags:
    ; r0 = DMA_base, r1 = stream_id
    
    push {r4, lr}
    
    mov r4, r1
    lsl r4, r4, #4
    
    ; Clear all flags for stream
    ldr r1, =0x3F000000
    lsr r1, r1, r4
    
    str r1, [r0, #DMA_LIFCR]
    
    pop {r4, pc}

.global dma_double_buffer_mode
dma_double_buffer_mode:
    ; r0 = DMA_base, r1 = stream_id, r2 = mem0, r3 = mem1
    
    push {r4-r6, lr}
    
    mov r4, r1
    lsl r4, r4, #3
    lsl r4, r4, #1
    lsl r4, r4, #1
    add r4, r4, r0
    
    ; Set M0AR
    str r2, [r4, #DMA_S0M0AR]
    
    ; Set M1AR
    str r3, [r4, #DMA_S0M1AR]
    
    ; Enable double buffering in CR
    ldr r5, [r4, #DMA_S0CR]
    orr r5, r5, #0x04000000
    str r5, [r4, #DMA_S0CR]
    
    pop {r4-r6, pc}

.global dma_adc_to_memory
dma_adc_to_memory:
    ; r0 = DMA_base, r1 = stream_id, r2 = adc_address, r3 = buffer_address
    ; Stack: [buffer_size, num_conversions]
    
    push {r4-r8, lr}
    
    mov r4, r0
    mov r5, r1
    mov r6, r2
    mov r7, r3
    ldr r8, [sp, #24]
    
    ; Calculate stream offset
    mov r0, r5
    lsl r0, r0, #3
    lsl r0, r0, #1
    lsl r0, r0, #1
    add r0, r0, r4
    
    ; Disable stream
    ldr r1, [r0, #DMA_S0CR]
    bic r1, r1, #0x00000001
    str r1, [r0, #DMA_S0CR]
    
wait_adc_disable:
    ldr r1, [r0, #DMA_S0CR]
    tst r1, #0x00000001
    bne wait_adc_disable
    
    ; Set number of conversions
    str r8, [r0, #DMA_S0NDTR]
    
    ; Set ADC address
    str r6, [r0, #DMA_S0PAR]
    
    ; Set buffer address
    str r7, [r0, #DMA_S0M0AR]
    
    ; Configure for ADC (P2M, circular)
    ldr r1, =0x04C01000
    str r1, [r0, #DMA_S0CR]
    
    pop {r4-r8, pc}

.global dma_memory_to_memory
dma_memory_to_memory:
    ; r0 = DMA_base, r1 = stream_id, r2 = src, r3 = dst
    ; Stack: [size]
    
    push {r4-r8, lr}
    
    mov r4, r0
    mov r5, r1
    mov r6, r2
    mov r7, r3
    ldr r8, [sp, #24]
    
    ; Calculate stream offset
    mov r0, r5
    lsl r0, r0, #3
    lsl r0, r0, #1
    lsl r0, r0, #1
    add r0, r0, r4
    
    ; Disable stream
    ldr r1, [r0, #DMA_S0CR]
    bic r1, r1, #0x00000001
    str r1, [r0, #DMA_S0CR]
    
wait_m2m_disable:
    ldr r1, [r0, #DMA_S0CR]
    tst r1, #0x00000001
    bne wait_m2m_disable
    
    ; Set size
    str r8, [r0, #DMA_S0NDTR]
    
    ; Set source
    str r6, [r0, #DMA_S0PAR]
    
    ; Set destination
    str r7, [r0, #DMA_S0M0AR]
    
    ; Configure for M2M
    ldr r1, =0x21C40003
    str r1, [r0, #DMA_S0CR]
    
    pop {r4-r8, pc}
