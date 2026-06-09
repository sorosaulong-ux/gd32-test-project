#ifndef __CAN_DRIVE_H
#define __CAN_DRIVE_H

#include "gd32f30x.h"

void CAN_Init_Config(void);
uint8_t CAN_Send_Msg(uint32_t id, uint8_t *msg, uint8_t len);
uint8_t CAN_Receive_Msg(uint32_t *id, uint8_t *buf);

#endif
