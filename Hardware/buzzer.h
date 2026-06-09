/*!
 *  \file    buzzer.h
 *  \brief   Buzzer driver — GD32A7, PB13
 */

#ifndef __BUZZER_H
#define __BUZZER_H

#include "gd32a7xx.h"

#define BUZZER_ON   1
#define BUZZER_OFF  0

extern uint8_t BUZZER_Status;

void BUZZER_Init(void);
void BUZZER_Set(uint8_t status);

#endif
