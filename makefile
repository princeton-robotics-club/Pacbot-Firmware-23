ELF_PATH=.vscode/avr.build/
MCU=atmega32u4
DFU-P=dfu-programmer

C_SOURCES:=$(wildcard *.c)

flash: $(ELF_PATH)output.hex
	$(DFU-P) $(MCU) erase
	$(DFU-P) $(MCU) flash $<

hexfile: $(ELF_PATH)output.hex

$(ELF_PATH)output.hex: $(ELF_PATH)output.elf
	avr-objcopy -O ihex $< $@

clean:
	rm $(ELF_PATH)output.hex

#$(ELF_PATH)output.elf: $(C_SOURCES)
#	avr-gcc -x c -funsigned-char -funsigned-bitfields -DDEBUG -O1  -I"C:\Studio 7\7.0\Packs\atmel\ATmega_DFP\1.7.374\include" -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega32u4 -B "C:\Studio 7\7.0\Packs\atmel\ATmega_DFP\1.7.374\gcc\dev\atmega32u4" -c -std=gnu99 -MD -MP -MF -o $@ $< 
		
