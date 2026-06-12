/*!
 *  \file    key_exti.c
 *  \brief   按键 EXTI 中断 — GD32A7 FreeRTOS
 *
 *  KEY1 (PC13) → EXTI13  | KEY2 (GPIOL.3) → EXTI3
 *  KEY3 (GPIOL.4) → EXTI4 | KEY4 (GPIOL.5) → EXTI5
 *  所有按键通过 Semaphore 通知 vTaskKey
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "app_tasks.h"
#include <stdio.h>

void key_exti_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOL);
    rcu_periph_clock_enable(RCU_SYSCFG);

    prpu_periph_unlock(PRPU_GPIOL);

    /* ── KEY1: PC13 → EXTI13 ── */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN13);
    exti_init(EXTI_13, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_13);

    /* ── KEY2: GPIOL.3 → EXTI3 ── */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN3);
    exti_init(EXTI_3, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_3);

    /* ── KEY3: GPIOL.4 → EXTI4 ── */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN4);
    exti_init(EXTI_4, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_4);

    /* ── KEY4: GPIOL.5 → EXTI5 ── */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN5);
    exti_init(EXTI_5, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_5);

    nvic_irq_enable(EXTI10_15_IRQn, 5, 0);
    nvic_irq_enable(EXTI3_IRQn, 5, 0);
    nvic_irq_enable(EXTI4_IRQn, 5, 0);
    nvic_irq_enable(EXTI5_9_IRQn, 5, 0);

    printf("[KEY] EXTI init OK (PC13/PL3/PL4/PL5)\r\n");
}

static void notify_key_task(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xKey_Semaphore != NULL) {
        xSemaphoreGiveFromISR(xKey_Semaphore, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* GD32A7 ISR names — 合并 UWB (PI.10) + KEY1 (PC13) */
void EXTI10_15_IRQHandler(void)
{
    /* KEY1: PC13 */
    if (SET == exti_interrupt_flag_get(EXTI_13)) {
        exti_interrupt_flag_clear(EXTI_13);
        notify_key_task();
    }
    /* UWB2 IRQ: PI.10 (from uwb_port.c) */
    if (SET == exti_interrupt_flag_get(EXTI_10)) {
        exti_interrupt_flag_clear(EXTI_10);
        process_deca_irq();
    }
}

void EXTI3_IRQHandler(void)
{
    if (SET == exti_interrupt_flag_get(EXTI_3)) {
        exti_interrupt_flag_clear(EXTI_3);
        notify_key_task();
    }
}

void EXTI4_IRQHandler(void)
{
    if (SET == exti_interrupt_flag_get(EXTI_4)) {
        exti_interrupt_flag_clear(EXTI_4);
        notify_key_task();
    }
}

void EXTI5_9_IRQHandler(void)
{
    /* KEY4: GPIOL.5 */
    if (SET == exti_interrupt_flag_get(EXTI_5)) {
        exti_interrupt_flag_clear(EXTI_5);
        notify_key_task();
    }
    /* UWB1 IRQ: PI.8 (from uwb_port.c) */
    if (SET == exti_interrupt_flag_get(EXTI_8)) {
        exti_interrupt_flag_clear(EXTI_8);
        process_deca_irq();
    }
}
