#ifndef __LED_H
#define __LED_H

#include "gd32f30x.h"

/* ================= 硬件管脚映射 ================= */
#define LED_PORT    GPIOA
#define LED_PIN     GPIO_PIN_0
#define LED_CLK     RCU_GPIOA

/* ================= 驱动函数声明 ================= */
void led_init(void);    // LED 初始化
void led_on(void);      // 打开 LED
void led_off(void);     // 关闭 LED
void led_toggle(void);  // 翻转 LED 状态

#endif /* __LED_H */