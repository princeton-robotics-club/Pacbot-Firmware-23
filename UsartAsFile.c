// Library Includes
#include <util/atomic.h>
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>

// Custom Includes
#include "UsartAsFile.h"

#define MAX_USART_BUFFER_SIZE 100

// Publicly available stream to use (kinda like stdout)
FILE * usartStream_Ptr;


// Forward declarations of put and get char functions
static int USART_putChar(char c, FILE * stream);
static int USART_getChar(FILE * stream);

/* ADT for a usart Buffer */
typedef struct usartBuffer {
    char buffer[MAX_USART_BUFFER_SIZE];
    char * r_ptr;
    char * w_ptr;
    size_t size;
} * usartBuffer_pT;


/* The write (transmit) and receive buffers */
static volatile struct usartBuffer g_receiveBuf = {
    .buffer = {0},
    .r_ptr = (char *) g_receiveBuf.buffer,
    .w_ptr = (char *) g_receiveBuf.buffer,
    .size = 0
};

static volatile struct usartBuffer g_writeBuf = {
    .buffer = {0},
    .r_ptr = (char *) g_writeBuf.buffer,
    .w_ptr = (char *) g_writeBuf.buffer,
    .size = 0
};

static const int g_maxSize = MAX_USART_BUFFER_SIZE;

/* Returns the current size of the USART write buffer */
size_t getWriteBufSize()
{
    return g_writeBuf.size;
}

/* Returns the current size of the USART receive buffer */
size_t getReceiveBufSize()
{
    return g_receiveBuf.size;
}

/* Initializes the USART and USART Filestream
 * Param: long baud = the baud rate to initialize the USART with */
void usartInit(long baud)
{
    // Set UBRR1 for baud configuration
    // This is based on formula recovered from atmega32u4 datasheet
    // PG 191
    UBRR1 = (F_CPU / (16.0 * baud)) - 0.5;

    // UBRR1 = 8;

    // Enable interrupts on receiving and finishing a transmit
    UCSR1B |= (1<<RXCIE1) | (1<<TXCIE1);    // We don't interrupt on empty buffer because we may not always have something we want to write.
    
    // Enable both the receiver and transmitter
    UCSR1B |= (1<<RXEN1) | (1<<TXEN1);

    // Initialize the filestream
    usartStream_Ptr = fdevopen(USART_putChar, USART_getChar);
}

/* Run when the USART hardware receives a byte 
 * Moves the byte into the receive buffer for the client */
ISR(USART1_RX_vect)
{
    if (g_receiveBuf.size < g_maxSize)
    {
        *g_receiveBuf.w_ptr = UDR1;
        g_receiveBuf.w_ptr++;

        if (g_receiveBuf.w_ptr >= (g_receiveBuf.buffer + g_maxSize))
        {
            g_receiveBuf.w_ptr = (char *) g_receiveBuf.buffer;
        }

        g_receiveBuf.size++;
    }
}

/* Run when the USART hardware finishes transmitting a byte
 * Moves a byte out of the write buffer */
ISR(USART1_TX_vect)
{
    if (g_writeBuf.size)
    {
        if (UCSR1A & (1<<UDRE1))
        {
            UDR1 = *g_writeBuf.r_ptr;
            g_writeBuf.r_ptr++;
            g_writeBuf.size--;
        }

        if (g_writeBuf.r_ptr >= (g_writeBuf.buffer + g_maxSize))
        {
            g_writeBuf.r_ptr = (char *) g_writeBuf.buffer;
        }
    }
}

/* Sends a byte if there is any data in the USART write buffer
 * and the USART hardware is available.
 * Returns the char sent or -1 if no char was sent
 * Sets global interrupt enable */
int usartTask(void)
{
    int retval = -1;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (g_writeBuf.size)
        {
            if (UCSR1A & (1<<UDRE1))
            {
                retval = *g_writeBuf.r_ptr;
                g_writeBuf.r_ptr++;
                g_writeBuf.size--;
                UDR1 = retval;
            }

            if (g_writeBuf.r_ptr >= (g_writeBuf.buffer + g_maxSize))
            {
                g_writeBuf.r_ptr = (char *) g_writeBuf.buffer;
            }
            
        }
    }
    return retval;
    
}

/* Returns a char from the receive buffer to the client or -1 if empty
 * Sets global interrupt enable */
static int USART_getChar(FILE * stream)
{
    int retval = -1;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {    
        if (g_receiveBuf.size)
        {
            retval = *g_receiveBuf.r_ptr;
            g_receiveBuf.r_ptr++;
            g_receiveBuf.size--;

            if (g_receiveBuf.r_ptr >= (g_receiveBuf.buffer + g_maxSize))
            {
                g_receiveBuf.r_ptr = (char *) g_receiveBuf.buffer;
            }
        }
    }
    return retval;
}

/* Takes a char from the client to the write buffer 
 * Returns 0 if successful or -1 if unsuccessful */
static int USART_putChar(char c, FILE * stream)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (g_writeBuf.size < g_maxSize)
        {
            *g_writeBuf.w_ptr = c;
            g_writeBuf.w_ptr++;

            if (g_writeBuf.w_ptr >= (g_writeBuf.buffer + g_maxSize))
            {
                g_writeBuf.w_ptr = (char *) g_writeBuf.buffer;
            }

            g_writeBuf.size++;
            usartTask();
            return 0;
        }
        // usartTask reenables the interrupts
        usartTask();
        return -1;
    }
    // Suppresses a bs compiler warning
    return -1;
}

