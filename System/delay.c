/*!
 *  \file    delay.c
 *  \brief   FreeRTOS 版本的延时函数
 *
 *  硬件初始化阶段: 使用忙等待 delay_1ms()
 *  FreeRTOS 运行后: 使用 vTaskDelay()
 */

#include "gd32a7xx.h"
#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"

/* ── SysTick 计数器 (FreeRTOS 接管前使用) ── */
static volatile uint32_t s_delay_cnt = 0;

/* ── 忙等待延时 (FreeRTOS 启动前使用) ── */
void delay_1ms(uint32_t count)
{
    /* 如果 FreeRTOS 已启动，使用 vTaskDelay */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(count));
        return;
    }

    /* FreeRTOS 未启动，忙等待 */
    s_delay_cnt = count;
    while (s_delay_cnt != 0);
}

void delay_decrement(void)
{
    if (s_delay_cnt != 0) {
        s_delay_cnt--;
    }
}

void delay_1us(uint32_t count)
{
    /* 微秒级延时，忙等待 */
    volatile uint32_t i;
    for (i = 0; i < count * (SystemCoreClock / 1000000U); i++) {
        __NOP();
    }
}
