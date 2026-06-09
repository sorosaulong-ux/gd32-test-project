#include "gd32f30x.h"
#include "key.h"
#include "delay.h"
#include "buzzer.h"  // 改为 buzzer

void Key_Init(void)
{
    // 开启 GPIOA 和 备用功能 (AFIO) 时钟
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    
    // 初始化 PA1 为上拉输入
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    
    // 配置 NVIC 中断优先级
    nvic_irq_enable(EXTI1_IRQn, 2, 2);
    
    // 连接 EXTI 线到 PA1
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_1);
    
    // 初始化 EXTI1，下降沿触发
    exti_init(EXTI_1, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_1);
}

void EXTI1_IRQHandler(void)
{
    DelayXms(10); // 软件消抖
    
    // 判断按键是否真的按下
    if(gpio_input_bit_get(GPIOA, GPIO_PIN_1) == RESET) 
    {
        // 蜂鸣器状态翻转逻辑 (已改名)
        if(BUZZER_Status == BUZZER_ON) 
            BUZZER_Set(BUZZER_OFF);
        else 
            BUZZER_Set(BUZZER_ON);
    }
    
    // 清除中断标志位
    exti_interrupt_flag_clear(EXTI_1);
}