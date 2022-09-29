PROJECT_NAME=BNO055SimpleLib

# The worst makefile you've ever seen #

OBJ_PATH=Output/Obj/
MCU=atmega32u4
DFU-P=dfu-programmer

C_SOURCES:=$(wildcard *.c)
O_SOURCES:=$(patsubst %.c, $(OBJ_PATH)%.o, $(C_SOURCES))

CC=avr-gcc

DASH_F_ARGS=-funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections -fpack-struct -fshort-enums
COMPILER_ARGS=-DDEBUG -O1 $(DASH_F_ARGS) -Wall -mmcu=atmega32u4 -std=gnu99


LINKER_ARGS=-Wl,-Map="Output/$(PROJECT_NAME).map" -Wl,-u,vfprintf -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega32u4 -lprintf_flt


flash: Output/output.hex
	@echo flashing chip
	$(DFU-P) $(MCU) erase
	$(DFU-P) $(MCU) flash $<
	@echo

hexfile: Output/output.hex

Output/output.hex: Output/output.elf
	@echo Copying Elf to Hex
	avr-objcopy -O ihex $< $@
	@echo

Output/output.elf: $(O_SOURCES)
	@echo Linking object files into ELF file
	$(CC) -o $@ $(O_SOURCES) $(LINKER_ARGS)  
	@echo 

# Include directories are provided by avr-gcc assuming they are in the default WinAVR location
$(OBJ_PATH)%.o: %.c | output_folder
	@echo Compiling $< into $@
	$(CC) -c $(COMPILER_ARGS) -o $@ $<
	@echo 

# Generates the output folders if need be
output_folder:
	@echo Making output directories
	mkdir -p Output
	mkdir -p Output/Obj
	@echo

clean:
	rm -r Output
	

