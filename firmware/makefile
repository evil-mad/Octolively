#NOTE: This file changes the fuse values of the chip to enable brown-out detection.
#fuse settings are hard-coded into the bottom lines; change them only with care.

PRG            = octolively
OBJ            = octolively.o
PROGRAMMER     = avrispmkII #	Must be updated with the type of programmer that you use!
#PROGRAMMER     = usbtiny	#	Must be updated with the type of programmer that you use!
PORT		   = usb		#	Must be updated with the location of programmer that you use!
MCU_TARGET     = atmega164p 
#MCU_TARGET     = atmega164a 
AVRDUDE_TARGET = m164p
#AVRDUDE_TARGET = m164a

OPTIMIZE       = -Os
DEFS           =
LIBS           =


# You should not have to change anything below here.

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text #eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak *.hex *.bin *.srec
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec


ehex:  $(PRG)_eeprom.hex
#ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

#%_eeprom.srec: %.elf
#	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@


ISPSPEEDSLOW   = -B 4	
ISPSPEEDFAST   = -B 1

ISPFUSES    = avrdude -p $(AVRDUDE_TARGET) -c $(PROGRAMMER) -P $(PORT) -v -e $(ISPSPEEDSLOW) -U lfuse:w:0xc2:m -U hfuse:w:0xD9:m   -U efuse:w:0xfd:m 
ISPFLASH    = avrdude -p $(AVRDUDE_TARGET) -c $(PROGRAMMER) -P $(PORT) -v $(ISPSPEEDSLOW) -U flash:w:$(PRG).hex

ISPALL =	avrdude -p $(AVRDUDE_TARGET) -c $(PROGRAMMER) -P $(PORT) -v -e $(ISPSPEEDSLOW)  \
	-U lfuse:w:0xC2:m \
	-U hfuse:w:0xD9:m \
	-U efuse:w:0xFD:m \
    -U flash:w:$(PRG).hex	

# command to program chip (invoked by running "make install")
installslow: 
	avrdude -p $(AVRDUDE_TARGET) -c $(PROGRAMMER) -P $(PORT) -v -e -B 5  \
	 -U lfuse:w:0xC2:m \
	 -U hfuse:w:0xD9:m \
	 -U efuse:w:0xfd:m \
     -U flash:w:$(PRG).hex	

install: 
	$(ISPALL)
	
	
#8 MHz internal, JTAG off, boot section off. BOD off. No clock out.

# Original (Default) fuse settings: 	
#	 -U lfuse:w:0x62:m  -U hfuse:w:0x99:m	-U efuse:w:0xff:m	
	
