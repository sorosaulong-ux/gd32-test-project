#include "led.h"

/**
 * @brief  初始化 LED 的 GPIO 管脚
 */
void led_init(void)
{
    /* 1. 开启外设时钟 */
    rcu_periph_clock_enable(LED_CLK);

    /* 2. 初始化引脚：推挽输出，50MHz */
    gpio_init(LED_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PIN);

    /* 3. 默认将引脚拉高（通常开发板上是拉低点亮，拉高熄灭） */
    gpio_bit_set(LED_PORT, LED_PIN);
}

/**
 * @brief  点亮 LED
 */
void led_on(void)
{
    // 如果你的板子是高电平点亮，请将 reset 改为 set
    gpio_bit_reset(LED_PORT, LED_PIN); 
}

/**
 * @brief  熄灭 LED
 */
void led_off(void)
{
    // 如果你的板子是高电平点亮，请将 set 改为 reset
    gpio_bit_set(LED_PORT, LED_PIN);
}

/**
 * @brief  翻转 LED 状态
 */
void led_toggle(void)
{
    // 读取当前引脚的输出状态，并进行取反操作
    if (gpio_output_bit_get(LED_PORT, LED_PIN)) {
        gpio_bit_reset(LED_PORT, LED_PIN);
    } else {
        gpio_bit_set(LED_PORT, LED_PIN);
    }
}