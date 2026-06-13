/*!
 *  \file    key_exti.c
 *  \brief   按键 ISR — 直接在中断中处理, 不经过任务
 *
 *  KEY1: FALLING → 蜂鸣器翻转
 *  KEY2: FALLING → 模式切换
 *  KEY3: 刹车状态
 *  KEY4: 手刹状态
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "FreeRTOS.h"
#include "buzzer.h"
#include "app_tasks.h"       /* extern g_mode, g_brake, g_parking_brake */
#include "uwb_port.h"
#include <stdio.h>

/* ── 外部变量 ── */
extern volatile sys_mode_t g_mode;
extern uint8_t g_brake;
extern uint8_t g_parking_brake;

void key_exti_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOL);
    rcu_periph_clock_enable(RCU_SYSCFG);
    prpu_periph_unlock(PRPU_GPIOL);

    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_5);

    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN13);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN3);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN4);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOL, EXTI_SOURCE_PIN5);

    exti_init(EXTI_13, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_init(EXTI_3,  EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_init(EXTI_4,  EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_init(EXTI_5,  EXTI_INTERRUPT, EXTI_TRIG_FALLING);

    exti_interrupt_flag_clear(EXTI_13);
    exti_interrupt_flag_clear(EXTI_3);
    exti_interrupt_flag_clear(EXTI_4);
    exti_interrupt_flag_clear(EXTI_5);

    nvic_irq_enable(EXTI10_15_IRQn, 5, 0);
    nvic_irq_enable(EXTI3_IRQn, 5, 0);
    nvic_irq_enable(EXTI4_IRQn, 5, 0);
    nvic_irq_enable(EXTI5_9_IRQn, 5, 0);

    printf("[KEY] EXTI init OK (ISR-direct)\r\n");
}

/* ── ISR 辅助: 200ms 去抖 ── */
static int debounce(TickType_t *last)
{
    TickType_t now = xTaskGetTickCountFromISR();
    if ((TickType_t)(now - *last) < pdMS_TO_TICKS(200)) return 0;
    *last = now;
    return 1;
}

/* KEY1 (PC13) + UWB2 (PI10) */
void EXTI10_15_IRQHandler(void)
{
    if (SET == exti_interrupt_flag_get(EXTI_13)) {
        exti_interrupt_flag_clear(EXTI_13);
        GPIO_TG(GPIOE) = GPIO_PIN_11;           /* LED3 TOGGLE */

        static TickType_t last;
        if (debounce(&last)) {
            BUZZER_Set(BUZZER_Status == BUZZER_ON ? BUZZER_OFF : BUZZER_ON);
        }
    }
    if (SET == exti_interrupt_flag_get(EXTI_10)) {
        exti_interrupt_flag_clear(EXTI_10);
        process_deca_irq();
    }
}

/* KEY2 (GPIOL.3) */
void EXTI3_IRQHandler(void)
{
    if (SET == exti_interrupt_flag_get(EXTI_3)) {
        exti_interrupt_flag_clear(EXTI_3);
        GPIO_TG(GPIOE) = GPIO_PIN_11;

        static TickType_t last;
        if (debounce(&last)) {
            g_mode = (g_mode == MODE_RADAR) ? MODE_RANGING : MODE_RADAR;
        }
    }
}

/* KEY3 (GPIOL.4) — 刹车状态 */
void EXTI4_IRQHandler(void)
{
    if (SET == exti_interrupt_flag_get(EXTI_4)) {
        exti_interrupt_flag_clear(EXTI_4);
        GPIO_TG(GPIOE) = GPIO_PIN_11;
        g_brake = (RESET == gpio_input_bit_get(GPIOL, GPIO_PIN_4));
    }
}

/* KEY4 (GPIOL.5) + UWB1 (PI8) */
void EXTI5_9_IRQHandler(void)
{
    if (SET == exti_interrupt_flag_get(EXTI_5)) {
        exti_interrupt_flag_clear(EXTI_5);
        GPIO_TG(GPIOE) = GPIO_PIN_11;
        g_parking_brake = (RESET == gpio_input_bit_get(GPIOL, GPIO_PIN_5));
    }
    if (SET == exti_interrupt_flag_get(EXTI_8)) {
        exti_interrupt_flag_clear(EXTI_8);
        process_deca_irq();
    }
}
