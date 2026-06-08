/*!
 *  \file    buzzer.c
 *  \brief   Buzzer driver — GD32A7, PB13 push-pull, active-high
 *
 *  PB13 → NPN transistor → buzzer
 *  HIGH = ON, LOW = OFF
 */

#include "buzzer.h"

uint8_t BUZZER_Status;

void BUZZER_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_13);

    GPIO_BC(GPIOB) = GPIO_PIN_13;          /* LOW → off (active-high) */
    BUZZER_Status = BUZZER_OFF;
}

void BUZZER_Set(uint8_t status)
{
    if (status == BUZZER_ON)
        GPIO_BOP(GPIOB) = GPIO_PIN_13;     /* HIGH → ON */
    else
        GPIO_BC(GPIOB) = GPIO_PIN_13;      /* LOW → OFF */

    BUZZER_Status = status;
}
