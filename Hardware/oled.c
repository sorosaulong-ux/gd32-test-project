#include "oled.h"
#include "OLED_Font.h"
#include "systick.h"
#include "gd32a7xx.h"

/* ----------------------------------------------------------------------------
 * I2C bit-bang using INPUT/OUTPUT mode switching (no open-drain mode needed)
 *
 * This is the most robust approach for bit-bang I2C:
 *   - HIGH: switch pin to input (high-Z), external or internal pull-up pulls high
 *   - LOW:  switch pin to push-pull output, write 0 to drive low
 *
 * Avoids issues with open-drain mode + pull-up interaction on some GD32 pins.
 * ----------------------------------------------------------------------------*/

/* ~5us delay @160MHz for ~100kHz I2C */
static void i2c_delay(void)
{
    volatile uint32_t i = 200;
    while (i--) {
        __NOP();
    }
}

void OLED_I2C_SCL_High(void)
{
    gpio_mode_set(OLED_SCL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, OLED_SCL_PIN);
}

void OLED_I2C_SCL_Low(void)
{
    gpio_mode_set(OLED_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OLED_SCL_PIN);
    GPIO_BC(OLED_SCL_PORT) = OLED_SCL_PIN;
}

void OLED_I2C_SDA_High(void)
{
    gpio_mode_set(OLED_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, OLED_SDA_PIN);
}

void OLED_I2C_SDA_Low(void)
{
    gpio_mode_set(OLED_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OLED_SDA_PIN);
    GPIO_BC(OLED_SDA_PORT) = OLED_SDA_PIN;
}

/* ----------------------------------------------------------------------------
 * I2C protocol primitives
 * ----------------------------------------------------------------------------*/

static void OLED_I2C_Start(void)
{
    OLED_I2C_SDA_High();
    i2c_delay();
    OLED_I2C_SCL_High();
    i2c_delay();
    OLED_I2C_SDA_Low();
    i2c_delay();
    OLED_I2C_SCL_Low();
    i2c_delay();
}

static void OLED_I2C_Stop(void)
{
    OLED_I2C_SDA_Low();
    i2c_delay();
    OLED_I2C_SCL_High();
    i2c_delay();
    OLED_I2C_SDA_High();
    i2c_delay();
}

static void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (Byte & (0x80 >> i)) {
            OLED_I2C_SDA_High();
        } else {
            OLED_I2C_SDA_Low();
        }
        i2c_delay();
        OLED_I2C_SCL_High();
        i2c_delay();
        OLED_I2C_SCL_Low();
        i2c_delay();
    }
    /* ACK clock pulse (ignore slave ACK) */
    OLED_I2C_SDA_High();
    i2c_delay();
    OLED_I2C_SCL_High();
    i2c_delay();
    OLED_I2C_SCL_Low();
    i2c_delay();
}

/* ----------------------------------------------------------------------------
 * OLED command & data write
 * ----------------------------------------------------------------------------*/

static void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);    /* slave address (SA0=0) */
    OLED_I2C_SendByte(0x00);    /* command mode */
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

static void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);    /* slave address (SA0=0) */
    OLED_I2C_SendByte(0x40);    /* data mode */
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

/* ----------------------------------------------------------------------------
 * Display functions
 * ----------------------------------------------------------------------------*/

static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (X & 0x0F));
}

void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++) {
            OLED_WriteData(0x00);
        }
    }
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    /* upper half of 8x16 font */
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++) {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }
    /* lower half */
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++) {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + ' ' + 16);
    }
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint32_t Number1;
    if (Number >= 0) {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    } else {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    OLED_ShowNum(Line, Column + 1, Number1, Length);
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++) {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10) {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        } else {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/* ----------------------------------------------------------------------------
 * OLED initialization
 * ----------------------------------------------------------------------------*/

void OLED_Init(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(OLED_SCL_RCU);

    /* start with both lines high (input + pull-up) */
    OLED_I2C_SCL_High();
    OLED_I2C_SDA_High();

    delay_1ms(100);   /* wait for OLED power-up */

    OLED_WriteCommand(0xAE);    /* display off */

    OLED_WriteCommand(0xD5);    /* set display clock divide ratio / oscillator frequency */
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8);    /* set multiplex ratio */
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3);    /* set display offset */
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);    /* set display start line */

    OLED_WriteCommand(0xA1);    /* segment remap (left-right direction) */

    OLED_WriteCommand(0xC8);    /* COM scan direction (bottom-up) */

    OLED_WriteCommand(0xDA);    /* set COM pins hardware configuration */
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);    /* set contrast control */
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);    /* set pre-charge period */
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);    /* set VCOMH deselect level */
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);    /* entire display on/off (resume to RAM) */

    OLED_WriteCommand(0xA6);    /* normal/inverse display */

    OLED_WriteCommand(0xAF);    /* display on */

    OLED_Clear();
}
