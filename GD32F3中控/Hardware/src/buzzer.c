#include "gd32f30x.h"
#include "buzzer.h"

uint8_t BUZZER_Status;

void BUZZER_Init(void)
{
    // 开启 GPIOB 时钟
    rcu_periph_clock_enable(RCU_GPIOB);
    
    // 初始化 PB10 为推挽输出，50MHz
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    
    // 默认关闭蜂鸣器
    BUZZER_Set(BUZZER_OFF);
}

void BUZZER_Set(uint8_t status)
{
    // 低电平触发逻辑：给 0 (reset) 响，给 1 (set) 停
    if(status == BUZZER_ON)
        gpio_bit_reset(GPIOB, GPIO_PIN_10); 
    else
        gpio_bit_set(GPIOB, GPIO_PIN_10);   
    
    BUZZER_Status = status;
}