/*
 * Usart.c
 *
 * Created: 7/20/2022 1:50:47 AM
 *  Author: jack2
 */ 

#include <avr/interrupt.h>


/*

	Usart Write Buffer ADT

*/

#define USART_BUFFER_MAX_SIZE 64
static const int g_maxSize = USART_BUFFER_MAX_SIZE;
static int g_r_Ptr = 0;
static int g_wr_Ptr = 0;
static uint8_t g_writeBuffer[USART_BUFFER_MAX_SIZE] = { 0 };
static int g_size = 0;

void usartTask();

int addByteToUsartWriteBuffer(uint8_t data)
{
	if (g_size >= g_maxSize)
	{
		return 0;
	}
	g_writeBuffer[g_wr_Ptr] = data;
	g_wr_Ptr++;
	if (g_wr_Ptr >= g_maxSize)
	{
		g_wr_Ptr = 0;
	}
	g_size++;
	return 1;
}

void addStringToUsartWriteBuffer(uint8_t * str)
{
	uint8_t * i = str;
	for (; *i != '\0'; i++)
	{
		while(!addByteToUsartWriteBuffer(*i))
		{
			usartTask();
		}
	}
}

int getUsartWriteBufferSize()
{
	return g_size;
}

/*

	END Usart Write Buffer ADT

*/

ISR(USART1_RX_vect)
{
	if (UDR1 == '~')
	{
	}
}

ISR(USART1_TX_vect)
{
	if (getUsartWriteBufferSize())
	{
		UDR1 = g_writeBuffer[g_r_Ptr];
		g_r_Ptr++;
		if(g_r_Ptr >= g_maxSize)
		{
			g_r_Ptr = 0;
		}
		g_size--;
	}
}

void usartInit()
{
	// Set the baud rate
	UBRR1 = 8;
	
	// Enable interrupts on receiving and finishing a transmit
	UCSR1B |= (1<<RXCIE1) | (1<<TXCIE1);	// We don't interrupt on empty buffer because we may not always have something we want to write.
	
	// Enable both the receiver and transmitter
	UCSR1B |= (1<<RXEN1) | (1<<TXEN1);
	
}

void usartTask()
{
	// Disable interrupts on receiving and finishing a transmit
	UCSR1B &= ~(1<<RXCIE1) & ~(1<<TXCIE1);
	if (getUsartWriteBufferSize())
	{
		if (UCSR1A & (1<<UDRE1))
		{
			UDR1 = g_writeBuffer[g_r_Ptr];
			g_r_Ptr++;
			if(g_r_Ptr >= g_maxSize)
			{
				g_r_Ptr = 0;
			}
			g_size--;
		}
	}
	// Enable interrupts on receiving and finishing a transmit
	UCSR1B |= (1<<RXCIE1) | (1<<TXCIE1);
}