#ifndef __OLED_H
#define __OLED_H

#include "gd32f30x.h"

// OLED 初始化
void OLED_Init(void);

// OLED 清屏
void OLED_Clear(void);

// OLED 显示单个字符
// Line: 行位置 (1~4)
// Column: 列位置 (1~16)
// Char: 要显示的字符
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);

// OLED 显示字符串
// Line: 行位置 (1~4)
// Column: 起始列位置 (1~16)
// String: 要显示的字符串指针
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

// OLED 显示无符号十进制数字
// Number: 要显示的数字 (0~4294967295)
// Length: 显示数字的长度 (1~10)
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

// OLED 显示有符号十进制数字
// Number: 要显示的数字 (-2147483648~2147483647)
// Length: 显示数字的长度 (1~10)
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);

// OLED 显示十六进制数字
// Number: 要显示的数字 (0~0xFFFFFFFF)
// Length: 显示数字的长度 (1~8)
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

// OLED 显示二进制数字
// Number: 要显示的数字
// Length: 显示数字的长度 (1~16)
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif