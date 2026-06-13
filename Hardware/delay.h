/*!
 *  \file    delay.h
 *  \brief   Delay wrapper — 裸机 delay_1ms/delay_1us
 *
 *  ESP8266 SendCmd/GetIPD 依赖 busy-wait 轮询 UART,
 *  不能用 vTaskDelay (yield 会丢失 AT 命令时序)。
 *  业务任务用 vTaskDelay 直接调 FreeRTOS API。
 */

#ifndef __DELAY_H
#define __DELAY_H

#include "systick.h"
#include "uwb_port.h"

#define delay_init()
#define delay_ms(x)       delay_1ms(x)
#define delay_us(x)       delay_1us(x)
#define DelayMs(x)        delay_1ms(x)
#define DelayXms(x)       delay_1ms(x)
#define DelayUs(x)        delay_1us(x)

#endif
