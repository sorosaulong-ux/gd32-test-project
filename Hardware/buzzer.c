/*!
 *  \file    buzzer.c
 *  \brief   Buzzer driver — GD32A7, PB13 push-pull
 *
 *  PB13 → NPN transistor / ULN2003 → buzzer
 *  HIGH = on, LOW = off (active-high drive)
 */

#include "buzzer.h"

uint8_t BUZZER_Status;

void BUZZER_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);

    GPIO_BC(GPIOB) = GPIO_PIN_13;         /* default LOW → off */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_13);

    BUZZER_Set(BUZZER_OFF);
}

void BUZZER_Set(uint8_t status)
{
    if (status == BUZZER_ON)
        GPIO_BOP(GPIOB) = GPIO_PIN_13;   /* HIGH → on */
    else
        GPIO_BC(GPIOB) = GPIO_PIN_13;    /* LOW → off */

    BUZZER_Status = status;
}
