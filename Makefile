###############################################################################
# Makefile for the project Pepper-midi
###############################################################################

## General Flags
PROJECT = vceg

# DEVICE = atmega168
DEVICE = atmega168
F_CPU   = 20000000	# in Hz
FUSEL = df
FUSEH = de

TARGET = $(PROJECT).elf
DEBUG =
#DEBUG =  -DDEBUG_LEVEL=2
CC = avr-gcc
AVRDUDE = avrdude -c usbtiny -p$(DEVICE)

############################## ATMega48/88/168 ##############################
# ATMega*8 FUSE_L (Fuse low byte):
# 0xdf = 1 1 0 1   1 1 1 1
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (external >8M crystal)
#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
#        | +------------------ CKOUT (if 0: Clock output enabled)
#        +-------------------- CKDIV8 (if 0: divide by 8)
# ATMega*8 FUSE_H (Fuse high byte):
# 0xde = 1 1 0 1   1 1 1 0
#        ^ ^ ^ ^   ^ \-+-/
#        | | | |   |   +------ BODLEVEL 0..2 (110 = 1.8 V)
#        | | | |   + --------- EESAVE (preserve EEPROM over chip erase)
#        | | | +-------------- WDTON (if 0: watchdog always on)
#        | | +---------------- SPIEN (allow serial programming)
#        | +------------------ DWEN (debug wire enable)
#        +-------------------- RSTDISBL (reset pin is enabled)
#

## Options common to compile, link and assembly rules
COMMON = -g -mmcu=$(DEVICE) -DF_CPU=$(F_CPU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -Os -fsigned-char  $(DEBUG)

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += -x assembler-with-cpp -Wa,

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=$(PROJECT).map

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

## Include Directories
INCLUDES = -I"." -Iusbdrv  -Iuart

## Objects that must be built in order to link
OBJECTS = vceg.o

## Objects explicitly added by the user
LINKONLYOBJECTS =

## Build
all: $(TARGET) $(PROJECT).hex $(PROJECT).lss

$(OBJECTS): Makefile

## Compile
vceg.o: vceg.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)
	avr-size -C --mcu=$(DEVICE) $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=$(DEVICE) ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) $(PROJECT).{elf,hex,lss,map}


.PHONY: flash
flash:	all
	atprogram -q -t avrispmk2 -i isp -d $(DEVICE) program -c -fl -f $(TARGET) --format elf --verify
#	 $(AVRDUDE) -U flash:w:$(PROJECT).hex


.PHONY: fuse
fuse:
	atprogram -q -t avrispmk2 -i isp -d $(DEVICE) write --fuses --values $(FUSEL)$(FUSEH)
#	$(AVRDUDE) -U hfuse:w:$(FUSEH):m -U lfuse:w:$(FUSEL):m
