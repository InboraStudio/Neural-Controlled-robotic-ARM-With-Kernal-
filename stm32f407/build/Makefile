################################################################################
# Makefile for ATmega328P EMG-to-Servo Project
# Bare-metal build with avr-gcc toolchain
# No Arduino libraries or bootloader
################################################################################

# Target MCU and clock frequency
MCU = atmega328p
F_CPU = 16000000UL

# Project name
PROJECT = emg_servo

# Directories
SRC_DIR = ../src
INC_DIR = ../include
LINKER_DIR = ../linker
OUTPUT_DIR = ../output
BUILD_DIR = build

# Source files
SRC_FILES = $(SRC_DIR)/main.c \
            $(SRC_DIR)/peripherals.c \
            $(SRC_DIR)/syscalls.c

ASM_FILES = $(SRC_DIR)/startup.s

# Object files
OBJ_FILES = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRC_FILES)) \
            $(patsubst $(SRC_DIR)/%.s,$(BUILD_DIR)/%.o,$(ASM_FILES))

# Linker script
LINKER_SCRIPT = $(LINKER_DIR)/atmega328p.ld

# Toolchain
CC = avr-gcc
AS = avr-gcc
LD = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
AVRDUDE = avrdude

# Compiler flags
CFLAGS = -mmcu=$(MCU) \
         -DF_CPU=$(F_CPU) \
         -I$(INC_DIR) \
         -Os \
         -Wall \
         -Wextra \
         -std=gnu99 \
         -ffunction-sections \
         -fdata-sections \
         -fno-fat-lto-objects \
         -flto

# Assembler flags
ASFLAGS = -mmcu=$(MCU) \
          -DF_CPU=$(F_CPU) \
          -x assembler-with-cpp

# Linker flags
LDFLAGS = -mmcu=$(MCU) \
          -T$(LINKER_SCRIPT) \
          -Wl,-Map=$(OUTPUT_DIR)/$(PROJECT).map,--cref \
          -Wl,--gc-sections \
          -Wl,--print-memory-usage \
          -flto

# AVRDUDE settings (adjust for your programmer)
# Example for Arduino as ISP on /dev/ttyUSB0
AVRDUDE_MCU = m328p
AVRDUDE_PROGRAMMER = arduino
AVRDUDE_PORT = /dev/ttyUSB0
AVRDUDE_BAUDRATE = 115200
AVRDUDE_FLAGS = -p $(AVRDUDE_MCU) \
                -c $(AVRDUDE_PROGRAMMER) \
                -P $(AVRDUDE_PORT) \
                -b $(AVRDUDE_BAUDRATE)

################################################################################
# Targets
################################################################################

.PHONY: all clean flash size disasm help

# Default target
all: $(OUTPUT_DIR)/$(PROJECT).hex $(OUTPUT_DIR)/$(PROJECT).eep $(OUTPUT_DIR)/$(PROJECT).lss
	@echo "=== Build complete ==="
	@$(SIZE) --format=avr --mcu=$(MCU) $(OUTPUT_DIR)/$(PROJECT).elf

# Create build and output directories
$(BUILD_DIR) $(OUTPUT_DIR):
	mkdir -p $@

# Compile C source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	@echo "Compiling: $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble assembly files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.s | $(BUILD_DIR)
	@echo "Assembling: $<"
	$(AS) $(ASFLAGS) -c $< -o $@

# Link object files to ELF
$(OUTPUT_DIR)/$(PROJECT).elf: $(OBJ_FILES) | $(OUTPUT_DIR)
	@echo "Linking: $@"
	$(LD) $(LDFLAGS) $(OBJ_FILES) -o $@

# Create Intel HEX file (Flash)
$(OUTPUT_DIR)/$(PROJECT).hex: $(OUTPUT_DIR)/$(PROJECT).elf
	@echo "Creating HEX: $@"
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Create EEPROM HEX file
$(OUTPUT_DIR)/$(PROJECT).eep: $(OUTPUT_DIR)/$(PROJECT).elf
	@echo "Creating EEPROM HEX: $@"
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load \
	           --change-section-lma .eeprom=0 $< $@

# Create disassembly listing
$(OUTPUT_DIR)/$(PROJECT).lss: $(OUTPUT_DIR)/$(PROJECT).elf
	@echo "Creating disassembly: $@"
	$(OBJDUMP) -h -S $< > $@

# Flash program to MCU using AVRDUDE
flash: $(OUTPUT_DIR)/$(PROJECT).hex
	@echo "=== Flashing to $(MCU) ==="
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$<:i

# Display size information
size: $(OUTPUT_DIR)/$(PROJECT).elf
	@echo "=== Size Information ==="
	@$(SIZE) --format=avr --mcu=$(MCU) $<
	@$(SIZE) --format=sysv $<

# Generate detailed disassembly with symbol addresses
disasm: $(OUTPUT_DIR)/$(PROJECT).lss
	@echo "=== Disassembly written to $< ==="
	@echo "Opening first 100 lines:"
	@head -n 100 $<

# Clean build artifacts
clean:
	@echo "=== Cleaning build artifacts ==="
	rm -rf $(BUILD_DIR)
	rm -f $(OUTPUT_DIR)/*.elf $(OUTPUT_DIR)/*.hex $(OUTPUT_DIR)/*.eep \
	      $(OUTPUT_DIR)/*.lss $(OUTPUT_DIR)/*.map

# Help target
help:
	@echo "Available targets:"
	@echo "  all     - Build project (default)"
	@echo "  flash   - Flash firmware to MCU using AVRDUDE"
	@echo "  size    - Display size information"
	@echo "  disasm  - Generate and display disassembly"
	@echo "  clean   - Remove build artifacts"
	@echo "  help    - Show this help"
	@echo ""
	@echo "Configuration:"
	@echo "  MCU: $(MCU)"
	@echo "  F_CPU: $(F_CPU) Hz"
	@echo "  Programmer: $(AVRDUDE_PROGRAMMER) on $(AVRDUDE_PORT)"
