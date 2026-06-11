/*!
 *  \file    key.c
 *  \brief   KEY1 (PC13) + KEY3 (PL4) + KEY4 (PL5) 按键驱动
 */

#include "key.h"
#include "buzzer.h"
#include "uwb_port.h"

void key1_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
}

void key1_poll(void)
{
    static uint32_t stable_ms;
    static uint8_t  triggered;
    static uint8_t  last_state = 1;
    uint32_t now = uwb_tick_get();
    uint8_t  cur = gpio_input_bit_get(GPIOC, GPIO_PIN_13);

    if (cur == 0) {
        if (last_state == 1) stable_ms = 0;
        last_state = 0;
        if (!triggered && (now - stable_ms) > 500) {
            triggered = 1;
            BUZZER_Set(BUZZER_Status == BUZZER_ON ? BUZZER_OFF : BUZZER_ON);
        }
    } else {
        last_state = 1;
        triggered  = 0;
        stable_ms  = now;
    }
}

/* ── KEY3 (PL4) — 刹车按键 ── */
static uint8_t key3_state;

void key3_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOL);
    prpu_periph_unlock(PRPU_GPIOL);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    key3_state = 0;
}

void key3_poll(void)
{
    key3_state = !gpio_input_bit_get(GPIOL, GPIO_PIN_4);
}

uint8_t key3_pressed(void)
{
    return key3_state;
}

/* ── KEY4 (PL5) — 手刹按键 ── */
static uint8_t key4_state;

void key4_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOL);
    prpu_periph_unlock(PRPU_GPIOL);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_5);
    key4_state = 0;
}

void key4_poll(void)
{
    key4_state = !gpio_input_bit_get(GPIOL, GPIO_PIN_5);
}

uint8_t key4_pressed(void)
{
    return key4_state;
}
