#ifndef __USART_H__
#define __USART_H__


void uartOpen(uint32_t uartChan, uint32_t baudrate);
void uartClose(uint32_t uartChan);

uint32_t uartWrite(uint32_t uartChan, uint8_t *pData, uint32_t Size);
uint32_t uartRead(uint32_t uartChan, uint8_t *pData, uint32_t Size);

uint32_t uartWriteStr(uint32_t uartChan, char *str);
  

#endif
