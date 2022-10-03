#include <stdio.h>

#ifndef F_CPU
#define F_CPU 16000000
#endif

void USART_init();
int USART_task();

size_t getWriteBufSize();
size_t getReadBufSize();

FILE * usartStream_Ptr;