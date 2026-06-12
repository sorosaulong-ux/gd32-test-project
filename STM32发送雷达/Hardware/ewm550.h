#ifndef __EWM550_H
#define __EWM550_H

#include "stm32f10x.h"

void EWM550_Init(uint32_t baudrate);
void EWM550_SendCmd(const char *cmd);
uint8_t EWM550_WaitResponse(char *buf, uint16_t len, uint16_t timeout_ms);
void EWM550_SetRole(uint8_t role);
void EWM550_SetAddress(const char *addr);
void EWM550_Reset(void);

#endif
