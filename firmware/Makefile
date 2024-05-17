TARGET = project
MCU = attiny814
SOURCES = main.c
OBJECTS = $(SOURCES:.c=.o)
# clock must be configured in main.c accordingly
CLOCK = 10000000

COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(MCU)

# serial port for UPDI programmer
ifeq ($(OS),Windows_NT)
	PROGRAMMER = COM17
else
	PROGRAMMER = /dev/tty.usbserial*
endif

# compiling and linking, target is the finished hex file
all: $(TARGET).hex

# compile source files to object files
.c.o:
	$(COMPILE) -c $< -o $@

# link the object files together
$(TARGET).elf: $(OBJECTS)
	$(COMPILE) $(OBJECTS) -o $(TARGET).elf
	avr-size $(TARGET).elf

# convert elf file to hex file
$(TARGET).hex: $(TARGET).elf
	avr-objcopy -j .text -j .data -j .rodata -O ihex $(TARGET).elf $(TARGET).hex

# flash (call make flash), requires pymcuprog installed
flash: $(TARGET).hex
ifeq ($(OS),Windows_NT)
	python -m pymcuprog.pymcuprog write -d $(MCU) -t uart -u $(PROGRAMMER) --erase -f $(TARGET).hex
else
	python3 -m pymcuprog.pymcuprog write -d $(MCU) -t uart -u $(PROGRAMMER) --erase -f "$(shell pwd)/$(TARGET).hex"
endif

# remove build files
clean:
ifeq ($(OS),Windows_NT)
	del -f $(TARGET).hex $(TARGET).elf $(OBJECTS)
else
	rm -f $(TARGET).hex $(TARGET).elf $(OBJECTS)
endif