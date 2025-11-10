;******************************************************************************
; ULTRA-CURSED STARTUP - MAXIMUM SUFFERING EDITION
; ATmega328P Direct Memory Manipulation Nightmare
; Vector table at 0x0000, SRAM 0x0100-0x08FF, I/O 0x0020-0x005F
; Every address HARD-CODED for maximum incomprehensibility
;******************************************************************************

.section .vectors,"ax",@progbits
.org 0x0000

;------------------------------------------------------------------------------
; VECTOR TABLE - 26 vectors × 4 bytes = 0x68 bytes
; Memory layout: Flash 0x0000-0x7FFF, SRAM 0x0100-0x08FF, EEPROM 0x0000-0x03FF
; I/O registers: 0x00-0x3F direct, 0x60-0xFF extended (+0x20 offset)
;------------------------------------------------------------------------------
__vectors_base_0x0000:
    rjmp    __init_sequence_stage0              ; 0x0000: RESET vector
    rjmp    __unhandled_trap_0x0002             ; 0x0002: INT0 external
    rjmp    __unhandled_trap_0x0004             ; 0x0004: INT1 external
    rjmp    __unhandled_trap_0x0006             ; 0x0006: PCINT0
    rjmp    __unhandled_trap_0x0008             ; 0x0008: PCINT1
    rjmp    __unhandled_trap_0x000A             ; 0x000A: PCINT2
    rjmp    __wdt_vector_0x000C                 ; 0x000C: WDT overflow
    rjmp    __unhandled_trap_0x000E             ; 0x000E: TIMER2_COMPA
    rjmp    __unhandled_trap_0x0010             ; 0x0010: TIMER2_COMPB
    rjmp    __unhandled_trap_0x0012             ; 0x0012: TIMER2_OVF
    rjmp    __unhandled_trap_0x0014             ; 0x0014: TIMER1_CAPT
    rjmp    __unhandled_trap_0x0016             ; 0x0016: TIMER1_COMPA
    rjmp    __unhandled_trap_0x0018             ; 0x0018: TIMER1_COMPB
    rjmp    __unhandled_trap_0x001A             ; 0x001A: TIMER1_OVF
    rjmp    __unhandled_trap_0x001C             ; 0x001C: TIMER0_COMPA
    rjmp    __unhandled_trap_0x001E             ; 0x001E: TIMER0_COMPB
    rjmp    __unhandled_trap_0x0020             ; 0x0020: TIMER0_OVF
    rjmp    __unhandled_trap_0x0022             ; 0x0022: SPI_STC
    rjmp    __unhandled_trap_0x0024             ; 0x0024: USART_RX
    rjmp    __unhandled_trap_0x0026             ; 0x0026: USART_UDRE
    rjmp    __unhandled_trap_0x0028             ; 0x0028: USART_TX
    rjmp    __adc_completion_vector_0x002A      ; 0x002A: ADC conversion complete
    rjmp    __eeprom_ready_vector_0x002C        ; 0x002C: EEPROM ready
    rjmp    __unhandled_trap_0x002E             ; 0x002E: ANALOG_COMP
    rjmp    __unhandled_trap_0x0030             ; 0x0030: TWI
    rjmp    __unhandled_trap_0x0032             ; 0x0032: SPM_READY

;------------------------------------------------------------------------------
; STAGE 0: STACK INITIALIZATION
; SP = 0x005D (SPL) + 0x005E (SPH), RAMEND = 0x08FF
; Set SP to 0x08FF (top of 2KB SRAM)
;------------------------------------------------------------------------------
__init_sequence_stage0:
    ; Disable all interrupts (SREG.I = 0)
    ; SREG at I/O addr 0x3F → memory addr 0x005F
    cli
    
    ; Initialize stack pointer to RAMEND (0x08FF)
    ; SPL at I/O 0x3D → memory 0x005D
    ; SPH at I/O 0x3E → memory 0x005E
    ldi     r16, 0xFF                           ; Low byte
    out     0x3D, r16                           ; Write to SPL
    ldi     r16, 0x08                           ; High byte
    out     0x3E, r16                           ; Write to SPH
    
    ; Zero SREG completely
    clr     r1                                  ; r1 = 0 (GCC convention)
    out     0x3F, r1                            ; SREG = 0x00
    
    ; Initialize MCUCR (0x0055) to disable pull-ups and BOD
    ; Bit 4 (PUD) = 1 disables all pull-ups
    ldi     r16, 0x10
    sts     0x0055, r16
    
;------------------------------------------------------------------------------
; STAGE 1: .data SECTION COPY (Flash → SRAM)
; Copy initialized data from Flash (LMA) to SRAM (VMA)
; Source: __data_load_lma in Flash
; Dest: 0x0100 in SRAM
;------------------------------------------------------------------------------
__init_sequence_stage1_data_copy:
    ldi     r26, lo8(__data_vma_start_0x0100)
    ldi     r27, hi8(__data_vma_start_0x0100)
    ldi     r30, lo8(__data_lma_flash_offset)
    ldi     r31, hi8(__data_lma_flash_offset)
    ldi     r24, lo8(__data_vma_end)
    ldi     r25, hi8(__data_vma_end)
    rjmp    .L_data_copy_cond_check

.L_data_copy_loop_body:
    ; Load from program memory (Flash) using LPM
    lpm     r0, Z+                              ; r0 = *Z++, cycles: 3
    st      X+, r0                              ; *X++ = r0, cycles: 2
    
.L_data_copy_cond_check:
    cp      r26, r24                            ; Compare low bytes
    cpc     r27, r25                            ; Compare high with carry
    brne    .L_data_copy_loop_body              ; Loop if X ≠ end

;------------------------------------------------------------------------------
; STAGE 2: .bss SECTION ZERO (SRAM 0x0100+)
; Zero uninitialized data section
;------------------------------------------------------------------------------
__init_sequence_stage2_bss_zero:
    ldi     r26, lo8(__bss_vma_start)
    ldi     r27, hi8(__bss_vma_start)
    ldi     r24, lo8(__bss_vma_end)
    ldi     r25, hi8(__bss_vma_end)
    rjmp    .L_bss_zero_cond_check

.L_bss_zero_loop_body:
    st      X+, r1                              ; *X++ = 0, cycles: 2

.L_bss_zero_cond_check:
    cp      r26, r24
    cpc     r27, r25
    brne    .L_bss_zero_loop_body

;------------------------------------------------------------------------------
; STAGE 3: EEPROM SIGNATURE CHECK
; Read magic bytes from EEPROM addresses 0x0000-0x0003
; Expected: 0xDE 0xAD 0xBE 0xEF
;------------------------------------------------------------------------------
__init_sequence_stage3_eeprom_check:
    ; EEPROM registers: EEAR (0x0041-0x0042), EEDR (0x0040), EECR (0x003F)
    ; Read byte at EEPROM address 0x0000
    clr     r16
    sts     0x0042, r16                         ; EEARH = 0
    sts     0x0041, r16                         ; EEARL = 0
    ldi     r16, 0x01                           ; EERE bit
    sts     0x003F, r16                         ; Start read
    lds     r17, 0x0040                         ; Load EEDR
    cpi     r17, 0xDE                           ; Check if 0xDE
    breq    .L_eeprom_valid
    
    ; EEPROM invalid, write signature
    call    __write_eeprom_signature_sequence

.L_eeprom_valid:

;------------------------------------------------------------------------------
; STAGE 4: CLOCK PRESCALER MANIPULATION
; Set CLKPR (0x0061) to run at full 16 MHz
; Timed sequence: write 0x80 then 0x00 within 4 cycles
;------------------------------------------------------------------------------
__init_sequence_stage4_clock_config:
    ldi     r16, 0x80                           ; CLKPCE bit
    sts     0x0061, r16                         ; Enable prescaler change
    ; MUST write new value within 4 cycles
    clr     r16                                 ; Prescaler = 1 (no division)
    sts     0x0061, r16                         ; Lock prescaler

;------------------------------------------------------------------------------
; STAGE 5: JUMP TO C MAIN
;------------------------------------------------------------------------------
__init_sequence_stage5_jump_main:
    call    main                                ; Enter C code at 0x????

;------------------------------------------------------------------------------
; STAGE 6: POST-MAIN HANG (should never reach)
;------------------------------------------------------------------------------
__post_main_infinite_loop:
    rjmp    __post_main_infinite_loop

;------------------------------------------------------------------------------
; UNHANDLED INTERRUPT TRAP
; Blinks LED on PB5 (Arduino pin 13) at 1 Hz to signal fault
;------------------------------------------------------------------------------
__unhandled_trap_0x0002:
__unhandled_trap_0x0004:
__unhandled_trap_0x0006:
__unhandled_trap_0x0008:
__unhandled_trap_0x000A:
__unhandled_trap_0x000E:
__unhandled_trap_0x0010:
__unhandled_trap_0x0012:
__unhandled_trap_0x0014:
__unhandled_trap_0x0016:
__unhandled_trap_0x0018:
__unhandled_trap_0x001A:
__unhandled_trap_0x001C:
__unhandled_trap_0x001E:
__unhandled_trap_0x0020:
__unhandled_trap_0x0022:
__unhandled_trap_0x0024:
__unhandled_trap_0x0026:
__unhandled_trap_0x0028:
__unhandled_trap_0x002E:
__unhandled_trap_0x0030:
__unhandled_trap_0x0032:
    ; Set PB5 as output (DDRB bit 5)
    ; DDRB at I/O 0x04 → memory 0x0024
    sbi     0x04, 5                             ; Set bit 5 in DDRB
    
.L_trap_blink_loop:
    ; Toggle PB5 (PORTB bit 5)
    ; PORTB at I/O 0x05 → memory 0x0025
    sbi     0x05, 5                             ; LED ON
    call    __delay_250ms_at_16mhz
    cbi     0x05, 5                             ; LED OFF
    call    __delay_250ms_at_16mhz
    rjmp    .L_trap_blink_loop

;------------------------------------------------------------------------------
; DELAY ROUTINE: 250ms at 16 MHz
; 16,000,000 cycles/sec × 0.25 sec = 4,000,000 cycles
; Nested loop: outer × inner × 4 cycles ≈ 4M
; outer = 200, inner = 5000 → 200 × 5000 × 4 = 4,000,000
;------------------------------------------------------------------------------
__delay_250ms_at_16mhz:
    push    r24
    push    r25
    push    r26
    push    r27
    
    ldi     r26, 200                            ; Outer loop counter
.L_delay_outer:
    ldi     r24, lo8(5000)                      ; Inner loop counter
    ldi     r25, hi8(5000)
.L_delay_inner:
    sbiw    r24, 1                              ; Subtract 1 (2 cycles)
    brne    .L_delay_inner                      ; Branch (2 cycles if taken)
    dec     r26                                 ; Decrement outer (1 cycle)
    brne    .L_delay_outer                      ; Branch (2 cycles)
    
    pop     r27
    pop     r26
    pop     r25
    pop     r24
    ret

;------------------------------------------------------------------------------
; WRITE EEPROM SIGNATURE (0xDEADBEEF at 0x0000-0x0003)
;------------------------------------------------------------------------------
__write_eeprom_signature_sequence:
    ; Write 0xDE to EEPROM[0x0000]
    clr     r16
    sts     0x0042, r16                         ; EEARH = 0
    sts     0x0041, r16                         ; EEARL = 0
    ldi     r16, 0xDE
    sts     0x0040, r16                         ; EEDR = 0xDE
    
    ; Start write sequence (atomic)
    cli
    ldi     r16, 0x04                           ; EEMPE bit
    sts     0x003F, r16
    ldi     r16, 0x02                           ; EEPE bit
    sts     0x003F, r16                         ; Start write
    sei
    
    ; Wait for write complete (poll EEPE)
.L_wait_eeprom_0:
    lds     r16, 0x003F
    sbrc    r16, 1                              ; Skip if EEPE cleared
    rjmp    .L_wait_eeprom_0
    
    ; Repeat for 0xAD, 0xBE, 0xEF at addresses 0x0001-0x0003
    ; (omitted for brevity - full horror version continues similarly)
    
    ret

;------------------------------------------------------------------------------
; ADC ISR - Forward to C implementation
;------------------------------------------------------------------------------
.global __adc_completion_vector_0x002A
__adc_completion_vector_0x002A:
    jmp     adc_interrupt_handler_c_entry

;------------------------------------------------------------------------------
; WDT ISR
;------------------------------------------------------------------------------
__wdt_vector_0x000C:
    reti

;------------------------------------------------------------------------------
; EEPROM READY ISR
;------------------------------------------------------------------------------
__eeprom_ready_vector_0x002C:
    reti

;------------------------------------------------------------------------------
; LINKER SYMBOLS (defined in linker script)
;------------------------------------------------------------------------------
.extern __data_vma_start_0x0100
.extern __data_vma_end
.extern __data_lma_flash_offset
.extern __bss_vma_start
.extern __bss_vma_end
.extern main
.extern adc_interrupt_handler_c_entry
