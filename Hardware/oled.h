#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "gd32a7xx.h"

/* ---------------------------------------------------------------------------------
 * I2C pin definitions: PB6=SCL, PB7=SDA
 * ---------------------------------------------------------------------------------*/
#define OLED_SCL_RCU        RCU_GPIOB
#define OLED_SCL_PORT       GPIOB
#define OLED_SCL_PIN        GPIO_PIN_6

#define OLED_SDA_RCU        RCU_GPIOB
#define OLED_SDA_PORT       GPIOB
#define OLED_SDA_PIN        GPIO_PIN_7
/* ---------------------------------------------------------------------------------*/

/*
 * I2C bit-bang macros using INPUT/OUTPUT mode switching.
 * This is more reliable than open-drain mode on GD32:
 *   - HIGH: switch pin to input (high-Z), pull-up pulls it high
 *   - LOW:  switch pin to output, drive 0
 */
void OLED_I2C_SCL_High(void);
void OLED_I2C_SCL_Low(void);
void OLED_I2C_SDA_High(void);
void OLED_I2C_SDA_Low(void);

/* public API */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif
