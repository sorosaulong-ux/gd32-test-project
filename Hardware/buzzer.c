/*!
 *  \file    buzzer.c
 *  \brief   Buzzer driver — GD32A7, PB13 push-pull, active-low
 *
 *  PB13 → NPN transistor → buzzer
 *  LOW  = ON,  HIGH = OFF
 *
 *  ★ 关键: 切输出模式前预装 ODR=HIGH, 避免瞬间浮空→LOW→蜂鸣器响
 */

#include "buzzer.h"

uint8_t BUZZER_Status;

void BUZZER_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);

    /* ★ 先写 ODR 寄存器把 pin 13 预置为 HIGH,
     * 然后才切输出模式 — pin 变成输出的那一刻就已经是 HIGH */
    GPIO_ODR(GPIOB) |= (uint16_t)GPIO_PIN_13;

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_13);

    BUZZER_Status = BUZZER_OFF;
}

void BUZZER_Set(uint8_t status)
{
    if (status == BUZZER_ON)
        GPIO_BC(GPIOB) = GPIO_PIN_13;       /* LOW → ON */
    else
        GPIO_BOP(GPIOB) = GPIO_PIN_13;      /* HIGH → OFF */

    BUZZER_Status = status;
}
