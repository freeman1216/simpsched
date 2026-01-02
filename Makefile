CC = arm-none-eabi-gcc
CFLAGS = -ggdb -Wall -fjump-tables  -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LDFLAGS = -Tstm32f411ceu6.ld -nolibc --specs=nosys.specs -nostartfiles
INCLUDES = -Iinc/ -Iinc/app -Iinc/driver/

SRC_DIR = src
BUILD_DIR = build
SOURCES = \
	$(SRC_DIR)/startup_stm32f411ceu6.c\
	$(SRC_DIR)/main.c

BIN = $(BUILD_DIR)/example.elf

all: $(BUILD_DIR)
	rm -f $(BUILD_DIR)/*.elf
	$(CC) $(CFLAGS) $(LDFLAGS) $(INCLUDES) $(SOURCES) -o $(BIN)

clean:
	rm -f $(BUILD_DIR)/*.elf



.PHONY: debug
debug: $(BIN)

	@echo "Flashing $(BIN)..."
	openocd -f /usr/share/openocd/scripts/interface/stlink.cfg \
	-f /usr/share/openocd/scripts/target/stm32f4x.cfg \
		-c "program $(BIN) reset exit"

	@echo "Starting OpenOCD server..."
	openocd -f /usr/share/openocd/scripts/interface/stlink.cfg \
	-f /usr/share/openocd/scripts/target/stm32f4x.cfg & \
	gf2 $(BIN) \
		-ex "target remote localhost:3333" \
		-ex "monitor reset halt" 
	
	pkill openocd

flash: $(BIN)

	@echo "Flashing $(BIN)..."
	openocd -f /usr/share/openocd/scripts/interface/stlink.cfg \
	-f /usr/share/openocd/scripts/target/stm32f4x.cfg \
		-c "program $(BIN) reset exit"

###############
# Build dir   #
###############
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)
