/*!
 *  \file    key_exti.c
 *  \brief   按键 EXTI 中断 — GD32A7 FreeRTOS
 *
 *  KEY1: 官方 gd_eval_key_init (上沿触发 → 释放时通知)
 *  KEY2: 长按模式切换 → 需要 BOTH 沿 (按下记录时间 + 释放判断时长)
 *  KEY3/4: 官方 init
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "app_tasks.h"
#include "uwb_port.h"
#include <stdio.h>

void key_exti_init(void)
{
    /* ★ 全部使用手动配置 (GPIO+EXTI) — 确保与当前引脚映射一致 */

    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOL);
    rcu_periph_clock_enable(RCU_SYSCFG);
    prpu_periph_unlock(PRPU_GPIOL);

    /* ── KEY1: PC13, PULLUP, BOTH ── */
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN13);
    exti_init(EXTI_13, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(EXTI_13);
    nvic_irq_enable(EXTI10_15_IRQn, 2, 0);

    /* ── KEY2: GPIOL.3, PULLUP, BOTH (长按检测) ── */
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN3);
    exti_init(EXTI_3, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(EXTI_3);
    nvic_irq_enable(EXTI3_IRQn, 2, 0);

    /* ── KEY3: GPIOL.4, PULLUP, BOTH ── */
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN4);
    exti_init(EXTI_4, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(EXTI_4);
    nvic_irq_enable(EXTI4_IRQn, 2, 0);

    /* ── KEY4: GPIOL.5, PULLUP, BOTH ── */
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_5);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN5);
    exti_init(EXTI_5, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(EXTI_5);
    nvic_irq_enable(EXTI5_9_IRQn, 2, 0);

    printf("[KEY] EXTI init OK (KEY1-4)\r\n");
}

static void notify_key_task(void)
{
    BaseType_t woken = pdFALSE;
    if (xKey_Semaphore) {
        xSemaphoreGiveFromISR(xKey_Semaphore, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

/* KEY1 (PC13, 上沿释放) + UWB2 (PI10) */
void EXTI10_15_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    if (SET == exti_interrupt_flag_get(EXTI_13)) {
        exti_interrupt_flag_clear(EXTI_13);
        if (xKey_Semaphore) xSemaphoreGiveFromISR(xKey_Semaphore, &woken);
    }
    if (SET == exti_interrupt_flag_get(EXTI_10)) {
        exti_interrupt_flag_clear(EXTI_10);
        process_deca_irq();
    }
    portYIELD_FROM_ISR(woken);
}

/* KEY2 (GPIOL.3, BOTH 沿) */
void EXTI3_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    if (SET == exti_interrupt_flag_get(EXTI_3)) {
        exti_interrupt_flag_clear(EXTI_3);
        if (xKey_Semaphore) xSemaphoreGiveFromISR(xKey_Semaphore, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

/* KEY3 (GPIOL.4, 上沿释放) */
void EXTI4_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    if (SET == exti_interrupt_flag_get(EXTI_4)) {
        exti_interrupt_flag_clear(EXTI_4);
        if (xKey_Semaphore) xSemaphoreGiveFromISR(xKey_Semaphore, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

/* KEY4 (GPIOL.5, 上沿释放) + UWB1 (PI8) */
void EXTI5_9_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    if (SET == exti_interrupt_flag_get(EXTI_5)) {
        exti_interrupt_flag_clear(EXTI_5);
        if (xKey_Semaphore) xSemaphoreGiveFromISR(xKey_Semaphore, &woken);
    }
    if (SET == exti_interrupt_flag_get(EXTI_8)) {
        exti_interrupt_flag_clear(EXTI_8);
        process_deca_irq();
    }
    portYIELD_FROM_ISR(woken);
}
