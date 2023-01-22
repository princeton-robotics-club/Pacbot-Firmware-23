# The worst makefile ever produced but should work with any similarly structured AVR projects

PROJECT_NAME=PacbotLowLevelCode
MCU=atmega32u4

# add any additional include directories (AIDs).
AID=

OUT_PATH=Output
OBJ_PATH=Output/Obj/
DFU-P=dfu-programmer

# Edit above
# Don't edit below

C_SOURCES:=$(wildcard *.c)
O_SOURCES:=$(patsubst %.c, $(OBJ_PATH)%.o, $(C_SOURCES))

AID_FORMAT:=$(patsubst %, -I%, $(AID))

# Don't edit above
# Edit below

DEBUG=-DDEBUG
CC=avr-gcc

DASH_F_ARGS=-funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections -fpack-struct -fshort-enums
COMPILER_ARGS=$(DEBUG) -O3 -MD $(DASH_F_ARGS) -Wall $(AID_FORMAT) -mmcu=atmega32u4 -std=gnu99

LINKER_ARGS=-Wl,-Map="Output/$(PROJECT_NAME).map" -Wl,-u,vfprintf -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega32u4 -lprintf_flt

# Edit above
# Don't edit below

.Phony: flash

flash: $(OUT_PATH)/$(PROJECT_NAME).hex
	@echo flashing chip
	$(DFU-P) $(MCU) erase
	$(DFU-P) $(MCU) flash $<
	@echo

erase:
	@echo erasing flash on chip
	$(DFU-P) $(MCU) erase
	@echo

launch:
	@echo launching program
	$(DFU-P) $(MCU) launch --no-reset
	@echo

noflash: hexfile

binfile: $(OUT_PATH)/$(PROJECT_NAME).bin
hexfile: $(OUT_PATH)/$(PROJECT_NAME).hex
elffile: $(OUT_PATH)/$(PROJECT_NAME).elf

$(OUT_PATH)/$(PROJECT_NAME).bin: $(OUT_PATH)/$(PROJECT_NAME).elf
	@echo Copying Elf to Bin
	avr-objcopy -O binary $< $@
	@echo

$(OUT_PATH)/$(PROJECT_NAME).hex: $(OUT_PATH)/$(PROJECT_NAME).elf
	@echo Copying Elf to Hex
	avr-objcopy -O ihex $< $@
	@echo

$(OUT_PATH)/$(PROJECT_NAME).elf: $(O_SOURCES)
	@echo Linking object files into ELF file
	$(CC) -o $@ $(O_SOURCES) $(LINKER_ARGS)  
	@echo 

# Include directories are provided by avr-gcc assuming they are in the default WinAVR location
$(OBJ_PATH)%.o: %.c | output_folder
	@echo Compiling $< into $@
	$(CC) $(COMPILER_ARGS) -o $@ -c $<
	@echo 

# Generates the output folders if need be
output_folder:
	@echo Making output directories
	mkdir -p $(OUT_PATH)
	mkdir -p $(OBJ_PATH)
	@echo


print:
	@echo $(AID_FORMAT)

clean:
	rm -r $(OUT_PATH)
	
-include $(O_SOURCES:.o=.d)
