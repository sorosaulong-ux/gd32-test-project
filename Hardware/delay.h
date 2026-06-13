/*!
 *  \file    delay.h
 *  \brief   Delay wrapper — FreeRTOS 下用 vTaskDelay, 裸机用 delay_1ms
 */

#ifndef __DELAY_H
#define __DELAY_H

#include "FreeRTOS.h"
#include "task.h"
#include "systick.h"
#include "uwb_port.h"

/* ── FreeRTOS 运行时用 vTaskDelay (yield CPU) ── */
#define delay_init()
#define delay_ms(x)       vTaskDelay(pdMS_TO_TICKS(x))
#define delay_us(x)       vTaskDelay(pdMS_TO_TICKS(max(1U,(x)/1000U)))
#define DelayMs(x)        vTaskDelay(pdMS_TO_TICKS(x))
#define DelayXms(x)       vTaskDelay(pdMS_TO_TICKS(x))
#define DelayUs(x)        vTaskDelay(pdMS_TO_TICKS(max(1U,(x)/1000U)))

#endif
