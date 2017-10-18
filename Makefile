TARGET		= project
MCU			= attiny841
DEVICE		= t841
CLOCK		= 8000000
PROGRAMMER	= -c stk500 -P /dev/cu.SLAB_USBtoUART
SOURCE		= main
FUSES		= -U hfuse:w:0xdf:m -U lfuse:w:0xe2:m
# project specific definition
ID			= 1 # axis ID. Change this for each motor via: "make ID=x"
STEPS_REV	= 200 # full steps per revolution of the motor

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -DAX_ID=$(ID) -DFSTEP_REV=$(STEPS_REV) -mmcu=$(MCU)

# just do all the usual stuff consecutive
all:
	# compile
	$(COMPILE) -c $(SOURCE).c -o $(SOURCE).o
	# link
	$(COMPILE) $(SOURCE).o -o $(TARGET).elf
	# hex
	avr-objcopy -O ihex -j .data -j .text $(TARGET).elf $(TARGET).hex
	# flash
	$(AVRDUDE) -U flash:w:$(TARGET).hex:i
	# clean
	rm -f $(TARGET).hex $(TARGET).elf $(SOURCE).o

# this is only executed if called via "make fuse"
fuse:
	$(AVRDUDE) $(FUSES)