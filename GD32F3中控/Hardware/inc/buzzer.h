#ifndef __BUZZER_H
#define __BUZZER_H

#include "gd32f30x.h"

// 定义开关状态
#define BUZZER_ON  1
#define BUZZER_OFF 0

extern uint8_t BUZZER_Status; // 外部声明状态变量

void BUZZER_Init(void);
void BUZZER_Set(uint8_t status);

#endif
