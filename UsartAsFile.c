#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include "UsartAsFile.h"

#define MAX_USART_BUFFER_SIZE 256

int USART_putChar(char c, FILE * stream);
int USART_getChar(FILE * stream);

FILE * usartStream_Ptr;

const int g_maxSize = MAX_USART_BUFFER_SIZE;

static volatile char g_receiveBuffer[MAX_USART_BUFFER_SIZE] = {0};
static volatile char * g_read_r_Ptr = g_receiveBuffer;
static volatile char * g_read_w_Ptr = g_receiveBuffer;
static volatile size_t g_readBufSize = 0;

static volatile char g_writeBuffer[MAX_USART_BUFFER_SIZE] = {0};
static volatile char * g_write_r_Ptr = g_writeBuffer;
static volatile char * g_write_w_Ptr = g_writeBuffer;
static volatile size_t g_writeBufSize = 0;

size_t getWriteBufSize()
{
	return g_writeBufSize;
}

size_t getReadBufSize()
{
	return g_readBufSize;
}

/* Initializes the USART and USART Filestream */
void USART_init(long baud)
{
	// Set UBRR1 for baud configuration
	// This is based on formula recovered from atmega32u4 datasheet
	// PG 191
	UBRR1 = (F_CPU / (16 * baud)) - 0.5;
	
	// Enable interrupts on receiving and finishing a transmit
	UCSR1B |= (1<<RXCIE1) | (1<<TXCIE1);	// We don't interrupt on empty buffer because we may not always have something we want to write.
	
	// Enable both the receiver and transmitter
	UCSR1B |= (1<<RXEN1) | (1<<TXEN1);

	// Initialize the filestream
	usartStream_Ptr = fdevopen(USART_putChar, USART_getChar);
}


ISR(USART1_RX_vect)
{
	if (g_readBufSize < g_maxSize)
	{
		*g_read_w_Ptr = UDR1;
		g_read_w_Ptr++;

		if (g_read_w_Ptr >= (g_receiveBuffer + g_maxSize))
		{
			g_read_w_Ptr = g_receiveBuffer;
		}

		g_readBufSize++;
	}
}

ISR(USART1_TX_vect)
{
	if (g_writeBufSize)
	{
		if (UCSR1A & (1<<UDRE1))
		{
			UDR1 = *g_write_r_Ptr;
			g_write_r_Ptr++;
			g_writeBufSize--;
		}

		if (g_write_r_Ptr >= (g_writeBuffer + g_maxSize))
		{
			g_write_r_Ptr = g_writeBuffer;
		}
	}
}

int USART_getChar(FILE * stream)
{
	int retval = -1;
	if (g_readBufSize)
	{
		retval = *g_read_r_Ptr;
		g_read_r_Ptr++;
		g_readBufSize--;

		if (g_read_r_Ptr >= (g_receiveBuffer + g_maxSize))
		{
			g_read_r_Ptr = g_receiveBuffer;
		}
	}
	return retval;
}

int USART_task(void)
{
	UCSR1B &= ~(1<<RXCIE1) & ~(1<<TXCIE1);
	int retval = -1;
	if (g_writeBufSize)
	{
		if (UCSR1A & (1<<UDRE1))
		{
			retval = *g_write_r_Ptr;
			g_write_r_Ptr++;
			g_writeBufSize--;
			UDR1 = retval;
		}

		if (g_write_r_Ptr >= (g_writeBuffer + g_maxSize))
		{
			g_write_r_Ptr = g_writeBuffer;
		}
		
	}
	UCSR1B |= (1<<RXCIE1) | (1<<TXCIE1);
	return retval;
	
}

int USART_putChar(char c, FILE * stream)
{
	if (g_writeBufSize < g_maxSize)
	{
		*g_write_w_Ptr = c;
		g_write_w_Ptr++;

		if (g_write_w_Ptr >= (g_writeBuffer + g_maxSize))
		{
			g_write_w_Ptr = g_writeBuffer;
		}

		g_writeBufSize++;
		USART_task();
		return 0;
	}
	USART_task();
	return -1;
}

