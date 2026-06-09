#include "delay.h"

/* 保存 1us 需要的 SysTick 计数值 */
static uint32_t fac_us = 0;

/**
 * @brief  初始化延迟函数
 * @note   使用内核 SysTick 定时器，时钟源选择为 AHB/8
 */
void delay_init(void)
{
    /* 配置 SysTick 时钟源为系统时钟 (AHB) 的 8 分频 */
    systick_clksource_set(SYSTICK_CLKSOURCE_HCLK_DIV8);
    
    /* * SystemCoreClock 是系统全局变量，存储当前系统主频（如 108MHz 或 120MHz）
     * 如果主频是 120MHz，经过 8 分频后，SysTick 的时钟就是 15MHz。
     * 即：1秒钟计数 15,000,000 次，那么 1微秒(us) 就需要计数 15 次。
     */
    fac_us = SystemCoreClock / 8000000U;
}

/**
 * @brief  微秒级精准延时
 * @param  nus: 需要延时的微秒数
 */
void delay_us(uint32_t nus)
{
    uint32_t temp;
    
    /* 将需要计数的总次数装载入 SysTick 定时器 */
    SysTick->LOAD = (uint32_t)(nus * fac_us); 
    SysTick->VAL = 0x00U;                      /* 清空当前计数值 */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  /* 启动定时器开始倒数 */
    
    /* 循环等待，直到倒数至 0，此时 COUNTFLAG (第16位) 会被硬件置 1 */
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01U) && !(temp & (1U << 16)));
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; /* 关闭定时器 */
    SysTick->VAL = 0x00U;                      /* 再次清空计数值 */
}

/**
 * @brief  毫秒级精准延时
 * @param  nms: 需要延时的毫秒数
 * @note   为了防止 nms 过大导致 24位的 SysTick->LOAD 寄存器溢出，
 * 这里采用循环调用 delay_us 的方式来实现。
 */
void delay_ms(uint32_t nms)
{
    uint32_t i;
    for(i = 0; i < nms; i++) {
        delay_us(1000);  /* 1ms = 1000us */
    }
}