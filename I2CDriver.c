/*
 * I2C.c
 *
 * Created: 5/26/2022 4:28:11 PM
 *  Author: Jack2bs
 */ 

// Other includes
#include <avr/interrupt.h>
#include <stdint.h>

// Custom includes
#include "I2CDriver.h"
#include "I2CInstruction.h"
#include "Usart.h"

//Forward declaration
void I2CHandle(void);

// I2C event interrupt
ISR(TWI_vect)
{
	I2CHandle();
}


// Sends a start condition to the I2C bus
inline void sendStartCond()
{
	TWCR = (1 << TWI_INT_FLAG) | (1 << TWI_ACK_EN) | (1 << TWI_START) | (1 << TWI_ENABLE) | (1 << TWI_INT_EN);
}

// Sends a stop condition to the I2C bus
inline void sendStopCond()
{
	TWCR = (1 << TWI_INT_FLAG) | (1 << TWI_ACK_EN) | (1 << TWI_STOP) | (1 << TWI_ENABLE) | (1 << TWI_INT_EN);
}

// Enables ACK
inline void enableACK()
{
	TWCR = (1 << TWI_INT_FLAG) | (1 << TWI_ACK_EN) | (1 << TWI_ENABLE) | (1 << TWI_INT_EN);
}

// Disables ACK
inline void disableAck()
{
	TWCR = (1 << TWI_INT_FLAG) | (1 << TWI_ENABLE) | (1 << TWI_INT_EN);
}

// Load data into TWDR
inline void loadTWDR(uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWI_INT_FLAG) | (1 << TWI_ENABLE) | (1 << TWI_INT_EN);
}

// Read is high on SDA, Write is low on SDA
// Loads the slave address + r/w onto the I2C bus
inline void loadAdress(uint8_t address, uint8_t r_w)
{
	loadTWDR((address << 1) | r_w);
}

// Read is high on SDA
// Loads the slave address + r onto the I2C bus
inline void loadAddressRead(uint8_t address)
{
	loadTWDR((address << 1) | 1);
}

// Write is low on SDA
// Loads the slave address + w onto the I2C bus
inline void loadAddressWrite(uint8_t address)
{
	loadTWDR(address << 1);
}

// This is high when the I2C bus is active and low when its not
static uint8_t g_state = 0;

// This has to exist because we need to access the buffer from interrupts
// Global variables it is :(
static I2CBuffer_pT g_curBuf = NULL;

/*	Must be called to set the buffer for the I2C driver to take instructions from
 *	Param: struct I2CInstruction * buf is a pointer to the the buffer you want to use */
void I2CSetCurBuf(I2CBuffer_pT buf)
{
	g_curBuf = buf;
}

// This handles I2C using info from the I2C-Instructions
void I2CHandle()
{	
	static int dataPtr = 0;								// Holds how many bytes have been written/read

	if (!g_curBuf)
	{
		// LOG ERROR, current buffer is NULL!
		return;
	}
	if (!I2CBufferGetCurrentSize(g_curBuf))
	{
		// LOG ERROR, current buffer is EMPTY!
		return;
	}

	// Switch for the value of the I2C status Reg
	switch(TWSR & 0b11111000)
	{
		// Start or repeated start
		case START_TRA:
		case REP_START_TRA:
			loadAdress(I2CBufferGetCurrentInstructionAddress(g_curBuf), I2CBufferGetCurrentInstructionReadWrite(g_curBuf));	// Load the device address and r/w
			break;
			
		// Slave address + write has been transmitted and ACK received
		case SLA_W_TRA_ACK_REC:
			loadTWDR(I2CBufferGetCurrentInstructionData(g_curBuf, 0));			// Load the first byte to write into TWDR
			dataPtr = 1;											// Update  dataPtr
			break;
			
		// Slave address + write has been transmitted and NACK received
		case SLA_W_TRA_NACK_REC:
			// Could put an error message here
			sendStopCond();						// Send a stop condition
			I2CBufferMoveToNextInstruction(g_curBuf);		// Move to the next instruction (could comment out)
			dataPtr = 0;
			g_state = 0;						// set g_state to 0 (I2C ready/off)
			return;
		
		// A data byte has been transmitted and an ACK received
		case DATA_TRA_ACK_REC:
			// If all of the bytes have been transmitted
			if(dataPtr == I2CBufferGetCurrentInstructionLength(g_curBuf))
			{
				sendStopCond();					// Send a stop condition
				I2CBufferMoveToNextInstruction(g_curBuf);		// Move to the next instruction (could comment out)
				g_state = 0;					// set g_state to 0 (I2C ready/off)
				dataPtr = 0;					// Reset the dataPtr var
				return;
			}
			// Otherwise
			else
			{	
				loadTWDR(I2CBufferGetCurrentInstructionData(g_curBuf,dataPtr));	// Load the next byte to write into TWDR
				dataPtr++;								// Increment the dataPtr
			}
			break;
			
		// A data byte has been transmitted and a NACK received
		case DATA_TRA_NACK_REC:
			sendStopCond();					// Send a stop condition
			if (dataPtr == I2CBufferGetCurrentInstructionLength(g_curBuf))
			{
				I2CBufferMoveToNextInstruction(g_curBuf);		// Move to the next instruction (could comment out)
			}
			else
			{
				I2CBufferMoveToNextInstruction(g_curBuf);
			}
			g_state = 0;					// set g_state to 0 (I2C ready/off)
			dataPtr = 0;
			return;
			
		// Slave address + read transmitted and an ACK received
		case SLA_R_TRA_ACK_REC:
			// If only 1 byte is going to be read
			if(dataPtr == I2CBufferGetCurrentInstructionLength(g_curBuf) - 1)
			{
				disableAck();				// Disable the ACK
			}
			// Otherwise
			else
			{
				enableACK();				// Enable the ACk
			}
			break;
		
		// Slave address + read transmitted and a NACK received
		case SLA_R_TRA_NACK_REC:
			sendStopCond();					// Send a stop condition
			I2CBufferMoveToNextInstruction(g_curBuf);	// Push instruction to back and move to the next instruction (could comment out)
			dataPtr = 0;
			g_state = 0;					// set g_state to 0 (I2C ready/off)
			return;
			
		// Data received and ACK transmitted
		case DATA_REC_ACK_TRA:
			I2CBufferSetCurrentInstructionData(g_curBuf, dataPtr, TWDR);		// Read in the byte
			dataPtr++;							// Increment dataPtr
			// If we've read as much as we want
			if(dataPtr == I2CBufferGetCurrentInstructionLength(g_curBuf) - 1)
			{
				disableAck();					// Disable the ACK
			}
			else								// Otherwise
			{
				enableACK();					// Enable the ACK
			}
			break;
		
		// Data received and NACK transmitted
		case DATA_REC_NACK_TRA:
			I2CBufferSetCurrentInstructionData(g_curBuf, dataPtr, TWDR);	// Read in the byte
			dataPtr = 0;					// Reset the dataPtr var
			sendStopCond();					// Send a stop condition
			I2CBufferMoveToNextInstruction(g_curBuf);		// Move to the next instruction (could comment out)
			dataPtr = 0;
			g_state = 0;					// set g_state to 0 (I2C ready/off)
			return;
			
		// If one of the other statuses pops up
		default:
			sendStopCond();							// Send a stop condition
			I2CBufferMoveToNextInstruction(g_curBuf);			// Push instruction to back and move to the next instruction (could comment out)
			dataPtr = 0;
			g_state = 0;							// set g_state to 0 (I2C ready/off)
			return;
	}
	// If we haven't returned, then make sure g_state is 1
	g_state = 1;
}

// Called every loop to determine when to start I2C transaction
void I2CTask()
{
	
	// If g_state is low and there is an instruction available
	if(!g_state)
	{
		
		if (I2CBufferGetCurrentSize(g_curBuf))
		{
			// Send a start condition and update g_state
			sendStartCond();
			g_state = 1;
		}
	}	
}

#ifndef F_CPU
#define F_CPU 16000000UL
#endif //F_CPU

/* Called to initialize the I2C to a certain frequency
 * Param: long sclFreq is the intended frequency for the I2C peripheral to run at */
void I2CInit(long sclFreq)
{	
	/*
	
	Calculation stems from the following equation:
		SCL_CLK = F_CPU / (16 + 2 * TWBR * (4^TWPS))
	With the assumption that TWPS is 1
	
	For lower SCL_CLKs, it may be necessary to set TWPS to a number other than 1, and so this function should be
	changed or ignored.
	
	*/
	
	long temp1 = (F_CPU / sclFreq);
	long temp2 = temp1 - 16;
	long temp3 = temp2 / 8;
	
	TWBR = (int)temp3;
	TWCR = (1 << TWI_INT_FLAG) | (1 << TWI_ENABLE) | (1 << TWI_INT_EN);
}