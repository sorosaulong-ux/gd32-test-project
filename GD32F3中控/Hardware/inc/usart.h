#ifndef __USART_H
#define __USART_H

#include "gd32f30x.h"

// 打印调试信息的串口 (PA9/PA10，在GD32中是USART0)
#define USART_DEBUG  USART0

void Usart0_Init(uint32_t baudrate); // 原 Usart1_Init
void Usart1_Init(uint32_t baudrate); // 原 Usart2_Init
void Usart_SendString(uint32_t usart_periph, unsigned char *str, unsigned short len);
void UsartPrintf(uint32_t usart_periph, char *fmt,...);

#endif
