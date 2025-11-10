 /******************************************************************************
 * ATmega328P Linker Script
 * Memory layout based on datasheet Section 8, Pages 16-18
 * 
 * Flash: 32KB (0x0000 - 0x7FFF) - Program memory
 * SRAM:  2KB  (0x0100 - 0x08FF) - Data memory
 * EEPROM: 1KB (0x0000 - 0x03FF) - Persistent storage (not used here)
 *****************************************************************************/

OUTPUT_FORMAT("elf32-avr")
OUTPUT_ARCH(avr:5)

/* Memory regions from ATmega328P datasheet */
MEMORY
{
    /* Flash memory: 32KB, addresses 0x0000 - 0x7FFF */
    /* Datasheet Section 8.1, Page 16 */
    FLASH (rx)   : ORIGIN = 0x00000000, LENGTH = 32K
    
    /* SRAM: 2KB, addresses 0x0100 - 0x08FF */
    /* Note: First 256 bytes (0x0000-0x00FF) are registers and I/O */
    /* Datasheet Section 8.2, Page 16 */
    RAM (rw!x)   : ORIGIN = 0x00800100, LENGTH = 2K
    
    /* EEPROM: 1KB (not used in this project) */
    EEPROM (rw!x): ORIGIN = 0x00810000, LENGTH = 1K
}

/* Stack pointer initial value (top of RAM) */
__stack = ORIGIN(RAM) + LENGTH(RAM);

/* Heap (not used, but defined for compatibility) */
__heap_start = __bss_end;
__heap_end = __stack - 256;  /* Reserve 256 bytes for stack */

SECTIONS
{
    /*-------------------------------------------------------------------------
     * TEXT SECTION (Flash)
     *-----------------------------------------------------------------------*/
    
    /* Interrupt vectors must be at address 0x0000 */
    .vectors :
    {
        KEEP(*(.vectors))
        . = ALIGN(2);
    } > FLASH
    
    /* Startup code (.init sections) */
    .init :
    {
        KEEP(*(.init0))  /* Reset handler */
        KEEP(*(.init1))
        KEEP(*(.init2))
        KEEP(*(.init3))
        KEEP(*(.init4))
        KEEP(*(.init5))
        KEEP(*(.init6))
        KEEP(*(.init7))
        KEEP(*(.init8))
        KEEP(*(.init9))
    } > FLASH
    
    /* Program code (.text section) */
    .text :
    {
        *(.text)
        *(.text*)
        . = ALIGN(2);
    } > FLASH
    
    /* Finalization code (.fini sections) */
    .fini :
    {
        KEEP(*(.fini9))
        KEEP(*(.fini8))
        KEEP(*(.fini7))
        KEEP(*(.fini6))
        KEEP(*(.fini5))
        KEEP(*(.fini4))
        KEEP(*(.fini3))
        KEEP(*(.fini2))
        KEEP(*(.fini1))
        KEEP(*(.fini0))
    } > FLASH
    
    /* Read-only data (constants) */
    .rodata :
    {
        *(.rodata)
        *(.rodata*)
        . = ALIGN(2);
    } > FLASH
    
    /* Data to be copied to RAM at startup (stored in Flash) */
    .data :
    {
        __data_start = .;
        *(.data)
        *(.data*)
        . = ALIGN(2);
        __data_end = .;
    } > RAM AT > FLASH
    
    /* Store load address of .data in Flash */
    __data_load = LOADADDR(.data);
    
    /*-------------------------------------------------------------------------
     * BSS SECTION (RAM) - Zero-initialized data
     *-----------------------------------------------------------------------*/
    .bss (NOLOAD) :
    {
        __bss_start = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(2);
        __bss_end = .;
    } > RAM
    
    /*-------------------------------------------------------------------------
     * NO-INIT SECTION (RAM) - Uninitialized data (persists across reset)
     *-----------------------------------------------------------------------*/
    .noinit (NOLOAD) :
    {
        __noinit_start = .;
        *(.noinit)
        *(.noinit*)
        . = ALIGN(2);
        __noinit_end = .;
    } > RAM
    
    /*-------------------------------------------------------------------------
     * EEPROM SECTION (optional)
     *-----------------------------------------------------------------------*/
    .eeprom (NOLOAD) :
    {
        *(.eeprom)
        *(.eeprom*)
    } > EEPROM
    
    /*-------------------------------------------------------------------------
     * DEBUG SECTIONS (not loaded to device)
     *-----------------------------------------------------------------------*/
    /* DWARF debug sections */
    .debug_info     0 : { *(.debug_info) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    .debug_ranges   0 : { *(.debug_ranges) }
    
    /* Discard unused sections */
    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.gnu_debuglink)
    }
}
