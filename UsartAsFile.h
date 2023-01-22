#include <stdio.h>
#include "Defines.h"

/* Initializes the USART and USART Filestream
 * Param: long baud = the baud rate to initialize the USART with */
void usartInit(long baud);

/* Sends a byte if there is any data in the USART write buffer
 * and the USART hardware is available.
 * Returns the char sent or -1 if no char was sent
 * Sets global interrupt enable */
int usartTask();

/* Returns the current size of the USART write buffer */
size_t getWriteBufSize();

/* Returns the current size of the USART receive buffer */
size_t getReceiveBufSize();

/* Used for USART output with stdio functions */
extern FILE * usartStream_Ptr;