#include <stdio.h>

#ifndef F_CPU
#define F_CPU 16000000
#endif

/* Initializes the USART and USART Filestream
 * Param: long baud = the baud rate to initialize the USART with */
void USART_init(long baud);

/* Sends a byte if there is any data in the USART write buffer
 * Must either be called periodically or immediately after adding
 * bytes to write buffer
 * Returns the char sent or -1 if no char was sent */
int USART_task();

/* Returns the current size of the USART write buffer */
size_t getWriteBufSize();

/* Returns the current size of the USART read buffer */
size_t getReadBufSize();

/* Used for USART output with stdio functions */
FILE * usartStream_Ptr;