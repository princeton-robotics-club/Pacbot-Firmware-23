/*
 * Usart.h
 *
 * Created: 7/20/2022 1:51:03 AM
 *  Author: jack2
 */ 


#ifndef USART_H_
#define USART_H_

int addByteToUsartWriteBuffer(uint8_t data);
void addStringToUsartWriteBuffer(uint8_t * str);
int getUsartWriteBufferSize();

void usartInit();
void usartTask();

#endif /* USART_H_ */