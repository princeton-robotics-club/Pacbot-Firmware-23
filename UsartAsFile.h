#include <stdio.h>

void USART_init();
int USART_task();

size_t getWriteBufSize();
size_t getReadBufSize();

FILE * usartStream_Ptr;