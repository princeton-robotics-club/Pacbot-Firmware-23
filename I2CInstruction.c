/*
 * I2CInstruction.c
 *
 * Created: 5/26/2022 9:21:20 PM
 *  Author: Jack2bs
 */ 

// Other includes
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>

// Custom includes
#include "I2CInstruction.h"
#include "I2CDriver.h"
#include "UsartAsFile.h"

const static uint16_t g_s_I2CMaxBufSize = I2C_MAX_BUFFER_SIZE; 

/* Abstract data types (not visible outside of this file) */
typedef struct I2CInstruction
{
    int dev_addr;
    int readWrite;
    uint8_t* data;
    int length;
    I2CInstruction_ID instrID;
    
} * I2CInstruction_pT;

typedef struct I2CBuffer
{
    struct I2CInstruction buf[I2C_MAX_BUFFER_SIZE];
    int8_t endPt;
    int8_t currPt;
    int currentSize;
} * I2CBuffer_pT;


// This is the actual instance of the I2CBuffer
static struct I2CBuffer buffer = {
    .buf = {{0}},
    .endPt = -1,
    .currPt = -1,
    .currentSize = 0
};
const I2CBuffer_pT ibt = &buffer;

/* If Instruction is a write then it owns its data pointer.
 * This function frees it from ipt */
void I2CInstructionFreeData(I2CInstruction_pT ipt)
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#ifdef DEBUG
    if (!ipt)
    {
        fprintf(usartStream_Ptr, "ipt is null in I2CInstructionFreeData");
    }
#endif//DEBUG
    
    // If this is a write then the instruction owns the data pointer
    if (ipt->readWrite == I2C_WRITE)
    {
        if (ipt->data)
        {
            
            free(ipt->data);
        }
    }
}
}

/* Prints ipt in a human readable form to ostream 
 * Returns -1 if fails, 0 if succeeds */
int I2CInstructionPrint(I2CInstruction_pT ipt, FILE * ostream)
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#ifdef DEBUG
    if (!ipt)
    {
        fprintf(usartStream_Ptr, "ipt is null in I2CInstructionFreeData");
    }
#endif//DEBUG

    size_t ind;
    // Example "I_id 323: Read with Addr: 0x29; Data:"...
    if (fprintf(ostream, "I_id: %lu: %s with Addr: %x; Data: ",
    (uint32_t)ipt->instrID,
    ((ipt->readWrite)?"Read":"Write"),
    ipt->dev_addr) < 0)
    {
        return -1;
    }

    // Example cont. ..."21 4b 21\n"
    for (ind = 0; ind < ipt->length; ind++)
    {
        if (fprintf(ostream, "%x ", ipt->data[ind]) < 0)
        {
            return -1;
        }
    }
    fputc('\n', ostream);
    return 0;
}
}

/* Moves the I2CBuffer to the next instruction, deleting the current one 
 * Returns the new current instruction's id */
I2CInstruction_ID I2CBufferMoveToNextInstruction()
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    
    
    // Exit if there are no instructions in the buffer
    if (ibt->currPt < 0)
    {
#ifdef DEBUG
        fprintf(usartStream_Ptr, "I2CBufferMoveToNextInstruction called on empty buffer");
#endif//Debug
        
        return 0;
    }
    
    // Decrement size + free data var if last instr was a write
    ibt->currentSize--;
    I2CInstructionFreeData(&ibt->buf[ibt->currPt]);

    // If the buffer is now empty, set the currpt and endpt to -1
    if (ibt->endPt == ibt->currPt)
    {
        ibt->endPt = -1;
        ibt->currPt = -1;
    }
    else // Otherwise increment the current pointer
    {
        ibt->currPt++;
        if (ibt->currPt >= g_s_I2CMaxBufSize)
        {
            ibt->currPt = 0;
        }
    }
    
    
    //
    return ibt->buf[ibt->currPt].instrID;

}
}

/* Returns the current instruction's device address */
int I2CBufferGetCurrentInstructionAddress()
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (ibt->currPt < 0)
    {
        return 0;
    }
    
    return ibt->buf[ibt->currPt].dev_addr;
}
}

/* Returns the current instruction's length */
int I2CBufferGetCurrentInstructionLength()
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (ibt->currPt < 0)
    {
        return 0;
    }
    
    return ibt->buf[ibt->currPt].length;
}
}

/* Returns the current instruction's data offset by offset */
uint8_t I2CBufferGetCurrentInstructionData(int offset)
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (ibt->currPt < 0)
    {
        return 0;
    }
    // Return 0 if offset is past the instruction's length
    if (offset >= ibt->buf[ibt->currPt].length)
    {
        return 0;
    }
    return *(ibt->buf[ibt->currPt].data + offset);
}
}

/* Sets the current instructions data at offset with ddata */
void I2CBufferSetCurrentInstructionData(int offset, int ddata)
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#ifdef DEBUG
    if (offset > ibt->buf[ibt->currPt].length)
    {
        fprintf(usartStream_Ptr, "I2CBufferSetCurrentInstructionData set out of bounds data");
    }
#endif//Debug
    *(ibt->buf[ibt->currPt].data + offset) = ddata;
}
}

/* Returns whether the current instruction is read or write */
int I2CBufferGetCurrentInstructionReadWrite()
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (ibt->currPt < 0)
    {
        return -1;
    }
    
    return ibt->buf[ibt->currPt].readWrite;
}
}

/* Returns the current instruction's id */
I2CInstruction_ID I2CBufferGetCurrentInstructionID()
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (ibt->currPt < 0)
    {
        return 0;
    }
    
    return ibt->buf[ibt->currPt].instrID;
}
}

/* Adds a new instruction to the end of buf, where the new instruction has the following data
 * dev_addr = d_add
 * readWrite = rw
 * data = dat
 * length = leng
 * nextInstr = NULL
 * Sets global interrupt enable */
I2CInstruction_ID I2CBufferAddInstruction(int d_add, int rw, uint8_t* dat, int leng)
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    static I2CInstruction_ID g_s_instrIDAssigner = 1;

    

    // Can't add an instruction if there is no room
    if(ibt->currentSize >= g_s_I2CMaxBufSize)
    {
        
        return 0;
    }

    // Increment the buffer's size
    ibt->currentSize++;

    // If there was no instructions previously, set endPt to 0
    if (ibt->endPt < 0)
    {
        ibt->endPt = 0;
    }
    else // Increment endPt
    {
        ibt->endPt++;
        if (ibt->endPt >= g_s_I2CMaxBufSize)
        {
            ibt->endPt = 0;
        }
    }
    
    // Fill in the newInstr's information
    I2CInstruction_pT newInstr = &ibt->buf[ibt->endPt];
    newInstr->dev_addr = d_add;
    newInstr->length = leng;
    newInstr->readWrite = rw;

    // If it is a write, make a defensive copy (instruction owns the data)
    if (rw == I2C_WRITE)
    {
        newInstr->data = malloc(leng);
        if (!newInstr->data)
        {
            
            return 0;
        }
        memcpy(newInstr->data, dat, leng);
    }
    // If it is a read, then keep the passed pointer to add data to (program owns the data)
    else
    {
        newInstr->data = dat;
    }

    // Assign an ID
    newInstr->instrID = g_s_instrIDAssigner;
    g_s_instrIDAssigner++;
    if (!g_s_instrIDAssigner)
    {
        g_s_instrIDAssigner = 1;
    }

    // If there were no instruction previously, set currPt to endPt
    if (ibt->currPt < 0)
    {
        ibt->currPt = ibt->endPt;
    }
    
    return newInstr->instrID;
}
}

/* Returns the I2CBuffer's current size */
size_t I2CBufferGetCurrentSize()
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    return ibt->currentSize;
}
}

/* Returns 1 (true) if buf contains instr, or 0 (false) if buf does not
 * contain instr */
int I2CBufferContains(I2CInstruction_ID instr)
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#ifdef DEBUG
    if (!instr)
    {
        return 0;
    }
#endif//Debug

    

    // If there are no instructions then it doesn't have instr
    if (ibt->currPt < 0)
    {
        
        return 0;
    }

    

    // Loop through all of the instructions, and return 1 if instr is found
    for (int i = ibt->currPt; i < g_s_I2CMaxBufSize; i++)
    {
        if (instr == ibt->buf[i].instrID)
        {
            
            return 1;
        }
        if (i == ibt->endPt)
        {
            
            return 0;
        }
        
    }
    for (int i = 0; i < ibt->endPt; i++)
    {
        if (instr == ibt->buf[i].instrID)
        {
            
            return 1;
        }
    }
    
    
    return 0;
}
}

/* Prints out a human readable form of the I2C Buffer to ostream
 * Returns -1 if fails, 0 if succeeds */
int I2CBufferPrint(FILE * ostream)
{
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    uint8_t TWCRCpy = TWCR;
    TWCR &= ~(1<<TWI_INT_EN);

    // If there are no instructions, then say that it is empty
    if (ibt->currPt < 0)
    {
        fprintf(ostream, "Buffer is empty");
        TWCR = TWCRCpy;
        return 0;
    }

    // Loop through all instructions and print them using the helper function
    fprintf(ostream, "Buffer Contains:\n");
    for (int i = ibt->currPt; i < g_s_I2CMaxBufSize; i++)
    {
        if (I2CInstructionPrint(&ibt->buf[i], usartStream_Ptr) < 0)
        {
            TWCR = TWCRCpy;
            return -1;
        }
        if (i == ibt->endPt)
        {
            TWCR = TWCRCpy;
            return 0;
        }
    }
    for (int i = 0; i < ibt->endPt; i++)
    {
        if (I2CInstructionPrint(&ibt->buf[i], usartStream_Ptr) < 0)
        {
            TWCR = TWCRCpy;
            return -1;
        }
    }
    fputc('\n', ostream);

    TWCR = TWCRCpy;

    return 0;
}
}




/* I2CInstruction API 
 * Not currently used but available if needed 

int I2CInstructionGetAddress(I2CInstruction_pT ipt)
{
#ifdef DEBUG
    if (!ipt)
    {
        fprintf(usartStream_Ptr, "ipt is null in I2CInstructionGetAddress");
    }
#endif//DEBUG
    
    return ipt->dev_addr;
}

int I2CInstructionGetLength(I2CInstruction_pT ipt)
{
#ifdef DEBUG
    if (!ipt)
    {
        fprintf(usartStream_Ptr, "ipt is null in I2CInstructionGetLength");
    }
#endif//DEBUG
    
    return ipt->length;
}

uint8_t * I2CInstructionGetData(I2CInstruction_pT ipt)
{
#ifdef DEBUG
    if (!ipt)
    {
        fprintf(usartStream_Ptr, "ipt is null in I2CInstructionGetData");
    }
#endif//DEBUG
    
    return ipt->data;
}

int I2CInstructionGetReadWrite(I2CInstruction_pT ipt)
{
#ifdef DEBUG
    if (!ipt)
    {
        fprintf(usartStream_Ptr, "ipt is null in I2CInstructionGetReadWrite");
    }
#endif//DEBUG
    
    return ipt->readWrite;
}

I2CInstruction_ID I2CInstructionGetID(I2CInstruction_pT ipt)
{
#ifdef DEBUG
    if (!ipt)
    {
        fprintf(usartStream_Ptr, "ipt is null in I2CInstructionGetID");
    }
#endif//DEBUG
    
    return ipt->instrID;
}

/* End unused API */