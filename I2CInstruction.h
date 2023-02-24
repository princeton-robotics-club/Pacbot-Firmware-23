#ifndef I2CINSTRUCTION_H_
#define I2CINSTRUCTION_H_

#include <string.h>
#include <stdio.h>

#define I2C_MAX_BUFFER_SIZE     64

#define I2C_WRITE   0
#define I2C_READ    1

/* I2CInstruction_ID is the memory safe way to identify I2CInstructions */
typedef uint32_t I2CInstruction_ID;

/* Moves the I2CBuffer to the next instruction, deleting the current one 
 * Returns the new current instruction's id
 * Sets global interrupt enable */
I2CInstruction_ID I2CBufferMoveToNextInstruction();

/* Returns the current instruction's device address */
int I2CBufferGetCurrentInstructionAddress();

/* Returns the current instruction's length */
int I2CBufferGetCurrentInstructionLength();

/* Returns the current instruction's data offset by offset */
uint8_t I2CBufferGetCurrentInstructionData(int offset);

/* Sets the current instructions data at offset with ddata */
void I2CBufferSetCurrentInstructionData(int offset, int ddata);

/* Returns whether the current instruction is read or write */
int I2CBufferGetCurrentInstructionReadWrite();

/* Returns the current instruction's id */
I2CInstruction_ID I2CBufferGetCurrentInstructionID();

/* Adds a new instruction to the end of buf, where the new instruction has the following data
 * dev_addr = d_add
 * readWrite = rw
 * data = dat
 * length = leng
 * nextInstr = NULL
 * Sets global interrupt enable */
I2CInstruction_ID I2CBufferAddInstruction(int d_add, int rw, uint8_t* dat, int leng);

/* Returns the I2CBuffer's current size */
size_t I2CBufferGetCurrentSize();

/* Returns 1 (true) if buf contains instr, or 0 (false) if buf does not
 * contain instr
 * Sets global interrupt enable */
int I2CBufferContains(I2CInstruction_ID instr);

/* Prints out a human readable form of the I2C Buffer to ostream
 * Returns -1 if fails, 0 if succeeds */
int I2CBufferPrint(FILE * ostream);

#endif /* I2CINSTRUCTION_H_ */