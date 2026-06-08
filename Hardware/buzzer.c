/*!
 *  \file    buzzer.c
 *  \brief   Buzzer driver — GD32A7, PB13 push-pull, active-low
 *
 *  PB13 → NPN transistor → buzzer
 *  LOW  = buzzer ON  (三极管导通)
 *  HIGH = buzzer OFF
 */

#include "buzzer.h"

uint8_t BUZZER_Status;

void BUZZER_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);

    /* ★ 先切输出模式，再拉 HIGH — pin 必须在 output 模式下才能设输出值 */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_13);

    GPIO_BOP(GPIOB) = GPIO_PIN_13;          /* HIGH → OFF */
    BUZZER_Status = BUZZER_OFF;
}

void BUZZER_Set(uint8_t status)
{
    if (status == BUZZER_ON)
        GPIO_BC(GPIOB) = GPIO_PIN_13;       /* LOW → ON (active-low) */
    else
        GPIO_BOP(GPIOB) = GPIO_PIN_13;      /* HIGH → OFF */

    BUZZER_Status = status;
}
