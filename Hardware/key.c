/*!
 *  \file    key.c
 *  \brief   KEY1 (PC13) — 轮询 + 500ms 长按防抖, 切换蜂鸣器
 */

#include "key.h"
#include "buzzer.h"
#include "uwb_port.h"        /* uwb_tick_get */

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
    uint8_t  cur = gpio_input_bit_get(GPIOC, GPIO_PIN_13);  /* LOW = pressed */

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
