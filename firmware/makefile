#NOTE: This file changes the fuse values of the chip to enable brown-out detection.
#fuse settings are hard-coded into the bottom lines; change them only with care.

PRG            = octolively
OBJ            = octolively.o
PROGRAMMER     = avrispmkII #	Must be updated with the type of programmer that you use!
#PROGRAMMER     = usbtiny	#	Must be updated with the type of programmer that you use!
PORT		   = usb		#	Must be updated with the location of programmer that you use!
MCU_TARGET     = atmega164p 
AVRDUDE_TARGET = m164p
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


ISPSPEED   = -B 5	 

# command to program chip (invoked by running "make install")
install: 
	avrdude -p $(AVRDUDE_TARGET) -c $(PROGRAMMER) -P $(PORT) -v -e $(ISPSPEED)  \
	 -U lfuse:w:0xE2:m \
	 -U hfuse:w:0xD9:m \
	 -U efuse:w:0xff:m \
     -U flash:w:$(PRG).hex	
 

#8 MHz internal, JTAG off, boot section off. BOD off. No clock out.

# Original (Default) fuse settings: 	
#	 -U lfuse:w:0x62:m  -U hfuse:w:0x99:m	-U efuse:w:0xff:m	
	
