#ifndef __DELAY_H
#define __DELAY_H

#include "gd32f30x.h"

/* ================= 驱动函数声明 ================= */

/* 延时初始化 (配置SysTick时钟源) */
void delay_init(void);

/* 微秒级精准延时 (us) */
void delay_us(uint32_t nus);

/* 毫秒级精准延时 (ms) */
void delay_ms(uint32_t nms);


/* ================= 完美兼容老代码的宏定义 ================= */
/* 这样一来，原代码里的 DelayXms 就会自动被编译器替换成你的 delay_ms */

#define Delay_Init()  delay_init()
#define DelayMs(x)    delay_ms(x)
#define DelayXms(x)   delay_ms(x)
#define DelayUs(x)    delay_us(x)

#endif /* __DELAY_H */
