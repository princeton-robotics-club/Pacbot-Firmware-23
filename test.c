/*
 * NonBlockingI2CLib.c
 *
 * Created: 7/19/2022 1:45:52 AM
 * Author : jack2
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>

#include "I2CInstruction.h"
#include "I2CDriver.h"
#include "Usart.h"
#include "BNO055.h"


int main(void)
{
	MCUCR |= (1<<JTD);
	CLKPR = (1 << CLKPCE);
	CLKPR = 0;
	
	I2CInit();
    usartInit();
	
	DDRC |= (1 << DDC6);
	
	
	I2CBuffer_pT ibt = I2CBufferNew();
	if (!ibt)
	{
		// Freak out ahahahahahah
	}
	I2CSetCurBuf(ibt);
	
	bno055EnterNDOF(ibt);
	
	uint8_t getST_RESULT[] = {0x36};
	uint8_t * result = calloc(1, sizeof(uint8_t));
	I2CInstruction_ID ipt1 = I2CBufferAddInstruction(ibt, 0x28, I2C_WRITE, getST_RESULT, 1);
	I2CBufferAddInstruction(ibt, 0x28, I2C_READ, result, 1);
	
	
	uint8_t fusionResult[6] = {0};
	double fusionFormatted[3] = {0};
	I2CInstruction_ID ipt2 = bno055GetAllEuler(ibt, &fusionResult[0]);
	
	long loop = 0;
	
    while (1) 
    {
		I2CTask();
		usartTask();
		
		if (!I2CBufferContains(ibt, ipt1))
		{
			ipt1 = I2CBufferAddInstruction(ibt, 0x28, I2C_WRITE, getST_RESULT, 1);
			I2CBufferAddInstruction(ibt, 0x28, I2C_READ, result, 1);
		}
		if (!I2CBufferContains(ibt, ipt2))
		{
			ipt2 = bno055GetAllEuler(ibt, &fusionResult[0]);
		}
		loop++;
		
		
		
		if (loop > 5000)
		{
			fusionRawToFormatted(fusionResult, fusionFormatted);
			
			char out[100] = "";
			sprintf(out, "r: %d\n%f\n%f\n%f\n", *result, fusionFormatted[0], fusionFormatted[1], fusionFormatted[2]);
			addStringToUsartWriteBuffer(out);
			loop = 0;
		}
    }
}

