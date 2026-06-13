/*!
 *  \file    key_exti.c
 *  \brief   按键 EXTI — GD32A7 FreeRTOS
 *
 *  LED3(PE.10) 在每个 ISR 中翻转 → 确认 ISR 硬件触发
 *  Semaphore → vTaskKey
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
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOL);
    rcu_periph_clock_enable(RCU_SYSCFG);
    prpu_periph_unlock(PRPU_GPIOL);

    /* GPIO 输入 + 上拉 */
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_5);

    /* EXTI 线路由 */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN13);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN3);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN4);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN5);

    /* EXTI 触发沿 — 全用 FALLING (按下) */
    exti_init(EXTI_13, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_init(EXTI_3,  EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_init(EXTI_4,  EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_init(EXTI_5,  EXTI_INTERRUPT, EXTI_TRIG_FALLING);

    /* 清除挂起标记 */
    exti_interrupt_flag_clear(EXTI_13);
    exti_interrupt_flag_clear(EXTI_3);
    exti_interrupt_flag_clear(EXTI_4);
    exti_interrupt_flag_clear(EXTI_5);

    /* NVIC — 所有 KEY 中断优先级 = 5 (可调 FreeRTOS API) */
    nvic_irq_enable(EXTI10_15_IRQn, 5, 0);
    nvic_irq_enable(EXTI3_IRQn, 5, 0);
    nvic_irq_enable(EXTI4_IRQn, 5, 0);
    nvic_irq_enable(EXTI5_9_IRQn, 5, 0);

    printf("[KEY] EXTI init OK (PC13/PL3/PL4/PL5)\r\n");
}

/* ── ISR — 按任何键 LED3(PE.10) 翻转一次 ── */

void EXTI10_15_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    if (SET == exti_interrupt_flag_get(EXTI_13)) {
        exti_interrupt_flag_clear(EXTI_13);
        GPIO_TG(GPIOE) = GPIO_PIN_11;   /* ★ LED3 TOGGLE — ISR fired? */
        if (xKey_Semaphore) xSemaphoreGiveFromISR(xKey_Semaphore, &woken);
    }
    if (SET == exti_interrupt_flag_get(EXTI_10)) {
        exti_interrupt_flag_clear(EXTI_10);
        process_deca_irq();
    }
    portYIELD_FROM_ISR(woken);
}

void EXTI3_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    if (SET == exti_interrupt_flag_get(EXTI_3)) {
        exti_interrupt_flag_clear(EXTI_3);
        GPIO_TG(GPIOE) = GPIO_PIN_10;
        if (xKey_Semaphore) xSemaphoreGiveFromISR(xKey_Semaphore, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

void EXTI4_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    if (SET == exti_interrupt_flag_get(EXTI_4)) {
        exti_interrupt_flag_clear(EXTI_4);
        GPIO_TG(GPIOE) = GPIO_PIN_10;
        if (xKey_Semaphore) xSemaphoreGiveFromISR(xKey_Semaphore, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

void EXTI5_9_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    if (SET == exti_interrupt_flag_get(EXTI_5)) {
        exti_interrupt_flag_clear(EXTI_5);
        GPIO_TG(GPIOE) = GPIO_PIN_10;
        if (xKey_Semaphore) xSemaphoreGiveFromISR(xKey_Semaphore, &woken);
    }
    if (SET == exti_interrupt_flag_get(EXTI_8)) {
        exti_interrupt_flag_clear(EXTI_8);
        process_deca_irq();
    }
    portYIELD_FROM_ISR(woken);
}
