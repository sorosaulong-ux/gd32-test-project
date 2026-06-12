/*!
 *  \file    key_exti.c
 *  \brief   按键 EXTI 中断配置 — FreeRTOS 版本
 *
 *  KEY1 (PC13) → EXTI13 → 触发 xKey_Semaphore
 *  KEY2 (GPIOL.3) → EXTI3 → 触发 xKey_Semaphore
 *  KEY3 (PL4) → EXTI4 → 触发 xKey_Semaphore
 *  KEY4 (PL5) → EXTI9_5 → 触发 xKey_Semaphore
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "app_tasks.h"
#include <stdio.h>

extern SemaphoreHandle_t xKey_Semaphore;

/* ====================================================================
 *  EXTI 中断初始化
 * ====================================================================*/
void key_exti_init(void)
{
    /* 使能 GPIO 时钟 */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOL);
    rcu_periph_clock_enable(RCU_AF);

    /* ── KEY1: PC13 → EXTI13 ── */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_13);
    exti_init(EXTI_13, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_13);

    /* ── KEY2: PL3 → EXTI3 ── */
    rcu_periph_clock_enable(RCU_GPIOL);
    prpu_periph_unlock(PRPU_GPIOL);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOL, GPIO_PIN_SOURCE_3);
    exti_init(EXTI_3, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_3);

    /* ── KEY3: PL4 → EXTI4 ── */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOL, GPIO_PIN_SOURCE_4);
    exti_init(EXTI_4, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_4);

    /* ── KEY4: PL5 → EXTI9_5 ── */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOL, GPIO_PIN_SOURCE_5);
    exti_init(EXTI_5, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_5);

    /* NVIC 配置 (优先级必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) */
    nvic_irq_enable(EXTI10_5_IRQn, 5, 0);  /* PL5 → EXTI5 */
    nvic_irq_enable(EXTI4_IRQn, 5, 0);     /* PL4 → EXTI4 */
    nvic_irq_enable(EXTI3_IRQn, 5, 0);     /* PL3 → EXTI3 */
    nvic_irq_enable(EXTI15_10_IRQn, 5, 0); /* PC13 → EXTI13 */

    printf("[KEY] EXTI init OK (PC13/PL3/PL4/PL5)\r\n");
}

/* ====================================================================
 *  EXTI 中断服务函数 — 通知 Key 任务
 * ====================================================================*/
void EXTI15_10_IRQHandler(void)
{
    if (exti_interrupt_flag_get(EXTI_13) != RESET) {
        exti_interrupt_flag_clear(EXTI_13);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xKey_Semaphore != NULL) {
            xSemaphoreGiveFromISR(xKey_Semaphore, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void EXTI3_IRQHandler(void)
{
    if (exti_interrupt_flag_get(EXTI_3) != RESET) {
        exti_interrupt_flag_clear(EXTI_3);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xKey_Semaphore != NULL) {
            xSemaphoreGiveFromISR(xKey_Semaphore, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void EXTI4_IRQHandler(void)
{
    if (exti_interrupt_flag_get(EXTI_4) != RESET) {
        exti_interrupt_flag_clear(EXTI_4);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xKey_Semaphore != NULL) {
            xSemaphoreGiveFromISR(xKey_Semaphore, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void EXTI10_5_IRQHandler(void)
{
    if (exti_interrupt_flag_get(EXTI_5) != RESET) {
        exti_interrupt_flag_clear(EXTI_5);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xKey_Semaphore != NULL) {
            xSemaphoreGiveFromISR(xKey_Semaphore, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
