/*
 * I2CInstruction.c
 *
 * Created: 5/26/2022 9:21:20 PM
 *  Author: Jack2bs
 */ 

// Other includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>

// Custom includes
#include "I2CInstruction.h"
#include "I2CDriver.h"

static I2CInstruction_ID g_s_instrIDAssigner = 1;

typedef struct I2CInstruction
{
	int dev_addr;
	int readWrite;
	uint8_t* data;
	int length;
	struct I2CInstruction * nextInstr;
	I2CInstruction_ID instrID;
	
}* I2CInstruction_pT;

struct I2CBuffer
{
	I2CInstruction_pT endPt;
	I2CInstruction_pT currPt;
	size_t currentSize;
	
};

I2CInstruction_pT I2CInstructionNew(int d_add, int rw, uint8_t* dat, int leng)
{
	I2CInstruction_pT newInstr = malloc(sizeof(struct I2CInstruction));
	if (!newInstr)
	{
		return NULL;
	}
	
	// If it is a write, make a defensive copy (instruction owns the data)
	if (rw == I2C_WRITE)
	{
		newInstr->data = malloc(sizeof(uint8_t) * leng);
		if (!newInstr->data)
		{
			free(newInstr);
			return NULL;
		}
		memcpy(newInstr->data, dat, leng);
	}
	// If it is a read, then keep the passed pointer to add data to (program owns the data)
	else
	{
		newInstr->data = dat;
	}
	
	newInstr->dev_addr = d_add;
	newInstr->readWrite = rw;
	newInstr->length = leng;
	
	newInstr->instrID = g_s_instrIDAssigner;
	g_s_instrIDAssigner++;
	if (!g_s_instrIDAssigner)
	{
		g_s_instrIDAssigner = 1;
	}
	
	return newInstr;
}

void I2CInstructionFree(I2CInstruction_pT ipt)
{
	if (!ipt)
	{
		return;
	}
	
	// If this is a write then the instruction owns the data pointer
	if (ipt->readWrite == I2C_WRITE)
	{
		if (ipt->data)
		{
			free(ipt->data);
		}
	}
	free(ipt);
	
}

int I2CInstructionGetAddress(I2CInstruction_pT ipt)
{
	if (!ipt)
	{
		return 0;
	}
	
	return ipt->dev_addr;
}

int I2CInstructionGetLength(I2CInstruction_pT ipt)
{
	if (!ipt)
	{
		return 0;
	}
	
	return ipt->length;
}

uint8_t * I2CInstructionGetData(I2CInstruction_pT ipt)
{
	if (!ipt)
	{
		return NULL;
	}
	
	return ipt->data;
}

int I2CInstructionGetReadWrite(I2CInstruction_pT ipt)
{
	if (!ipt)
	{
		return 0;
	}
	
	return ipt->readWrite;
}

I2CInstruction_pT I2CInstructionGetNextInstr(I2CInstruction_pT ipt)
{
	if (!ipt)
	{
		return NULL;
	}
	
	return ipt->nextInstr;
}

I2CInstruction_ID I2CInstructionGetID(I2CInstruction_pT ipt)
{
	if (!ipt)
	{
		return 0;
	}
	
	return ipt->instrID;
}

int I2CInstructionPrint(I2CInstruction_pT ipt, FILE * ostream)
{
	size_t ind;

	if (fprintf(ostream, "I_id: %lu: %s with Addr: %x; Data: ", (uint32_t)ipt->instrID, ((ipt->readWrite)?"Read":"Write"), ipt->dev_addr) < 0)
	{
		return -1;	
	}

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

I2CBuffer_pT I2CBufferNew()
{
	I2CBuffer_pT newBuf = malloc(sizeof(struct I2CBuffer));
	
	if (!newBuf)
	{
		return NULL;
	}
	newBuf->currPt = NULL;
	newBuf->endPt = NULL;
	newBuf->currentSize = 0;
	return newBuf;
}

void I2CBufferFree(I2CBuffer_pT buf)
{
	if (!buf)
	{
		return;
	}
	
	I2CInstruction_pT ipt = buf->currPt;
	buf->currPt = NULL;
	buf->endPt = NULL;
	buf->currentSize = 0;
	while (ipt)
	{
		I2CInstruction_pT next = ipt->nextInstr;
		I2CInstructionFree(ipt);
		ipt = next;
	}
	free(buf);
}

// Moves to the next instruction
I2CInstruction_ID I2CBufferMoveToNextInstruction(I2CBuffer_pT buf)
{
	if (!buf)
	{
		return 0;
	}
	
	cli();
	
	if (!buf->currPt)
	{
		return 0;
	}
	
	buf->currentSize--;
	
	I2CInstruction_pT del = buf->currPt;
	if (buf->endPt == buf->currPt)
	{
		buf->endPt = NULL;
		buf->currPt = NULL;
	}
	else
	{
		buf->currPt = buf->currPt->nextInstr;
	}
	I2CInstructionFree(del);
	
	sei();
	
	// Returns the next instruction (or nullptr if none)
	return buf->currPt->instrID;
}

int I2CBufferGetCurrentInstructionAddress(I2CBuffer_pT ibt)
{
	if (!ibt || !(ibt->currPt))
	{
		return 0;
	}
	
	return ibt->currPt->dev_addr;
}

int I2CBufferGetCurrentInstructionLength(I2CBuffer_pT ibt)
{
	if (!ibt || !(ibt->currPt))
	{
		return 0;
	}
	
	return ibt->currPt->length;
}

int I2CBufferSetCurrentInstructionData(I2CBuffer_pT ibt, int offset, uint8_t data)
{
	if (!ibt || !(ibt->currPt))
	{
		return 0;
	}
	if (offset >= ibt->currPt->length)
	{
		return 0;
	}
	*(ibt->currPt->data + offset) = data;
	return 1;
}

uint8_t I2CBufferGetCurrentInstructionData(I2CBuffer_pT ibt, int offset)
{
	if (!ibt || !(ibt->currPt))
	{
		return 0;
	}
	if (offset >= ibt->currPt->length)
	{
		return 0;
	}
	return *(ibt->currPt->data + offset);
}

int I2CBufferGetCurrentInstructionReadWrite(I2CBuffer_pT ibt)
{
	if (!ibt || !(ibt->currPt))
	{
		return 0;
	}
	
	return ibt->currPt->readWrite;
}

I2CInstruction_ID I2CBufferGetCurrentInstructionID(I2CBuffer_pT ibt)
{
	if (!ibt || !(ibt->currPt))
	{
		return 0;
	}
	
	return ibt->currPt->instrID;
}

I2CInstruction_ID I2CBufferPushInstruction(I2CBuffer_pT buf, I2CInstruction_pT newInstr)
{
	if (!buf)
	{
		return 0;
	}
	if (!newInstr)
	{
		return 0;
	}

	if (buf->currentSize >= I2C_MAX_BUFFER_SIZE)
	{
		return 0;
	}

	cli();
	buf->currentSize++;
	
	if (buf->endPt)
	{
		buf->endPt->nextInstr = newInstr;
		buf->endPt = buf->endPt->nextInstr;
	}
	else
	{
		buf->endPt = newInstr;
	}
	
	buf->endPt->nextInstr = NULL;
	
	if (!buf->currPt)
	{
		buf->currPt = buf->endPt;
	}
	
	sei();
	return buf->endPt->instrID;
}



// Adds an instruction at w_ptr
I2CInstruction_ID I2CBufferAddInstruction(I2CBuffer_pT buf, int d_add, int rw, uint8_t* dat, int leng)
{
	if (!buf)
	{
		return 0;
	}

	I2CInstruction_pT newInstr = I2CInstructionNew(d_add, rw, dat, leng);
	if (newInstr == NULL)
	{
		return 0;
	}
	return I2CBufferPushInstruction(buf, newInstr);
	
}

size_t I2CBufferGetCurrentSize(I2CBuffer_pT buf)
{
	if (!buf)
	{
		return 0;
	}
	return buf->currentSize;
}

int I2CBufferContains(I2CBuffer_pT buf, I2CInstruction_ID instr)
{
	if (!buf)
	{
		return 0;
	}
	if (!instr)
	{
		return 0;
	}
	
	I2CInstruction_pT ipt = buf->currPt;
	while (ipt != NULL)
	{
		if (instr == ipt->instrID)
		{
			return 1;
		}
		ipt = ipt->nextInstr;
	}
	return 0;
}

int I2CBufferRemove(I2CBuffer_pT buf, I2CInstruction_ID instr)
{
	if (!buf)
	{
		return 0;
	}
	if (!instr)
	{
		return 0;
	}
	
	I2CInstruction_pT ipt = buf->currPt;
	I2CInstruction_pT lastPt = NULL;
	while (ipt != NULL)
	{
		if (instr == ipt->instrID)
		{
			if (lastPt == NULL)
			{
				// We cannot remove the current instruction or else havoc will ensue
				return 0;
			}
			else
			{
				lastPt->nextInstr = ipt->nextInstr;
			}
			I2CInstructionFree(ipt);
			return 1;
		}
		lastPt = ipt;
		ipt = ipt->nextInstr;
	}
	return 0;
}

void I2CBufferSendToBack(I2CBuffer_pT buf)
{
	if (!buf)
	{
		return;
	}
	
	I2CBufferPushInstruction(buf, buf->currPt);
	I2CBufferMoveToNextInstruction(buf);
}

int I2CBufferPrint(I2CBuffer_pT ibt, FILE * ostream)
{
	uint8_t TWCRCpy = TWCR;
	TWCR &= ~(1<<TWI_INT_EN);

	if (!ibt->currPt)
	{
		fprintf(ostream, "Buffer is empty");
		return 0;
	}
	

	I2CInstruction_pT ipt = ibt->currPt;

	fprintf(ostream, "Buffer Contains:\n");

	while (ipt)
	{
		if (I2CInstructionPrint(ipt, ostream) < 0)
		{
			return -1;
		}
		ipt = ipt->nextInstr;
	}
	fputc('\n', ostream);

	TWCR = TWCRCpy;

	return 0;
}

/* End I2C instruction array API */