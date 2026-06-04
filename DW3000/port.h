/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @brief   HW specific definitions and functions for portability (Modified for GD32)
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "gd32f30x.h"

/* =================================================================================
 * DWS3220 模块引脚定义 (双模块复用 SPI，控制引脚绝对纯净版)
 * =================================================================================*/
/* -------- UWB 1 号机引脚 -------- */
#define UWB1_CS_PORT         GPIOA
#define UWB1_CS_PIN          GPIO_PIN_4

#define DW1_RSTn_GPIO        GPIOB
#define DW1_RSTn             GPIO_PIN_1

#define DECA1_IRQ_GPIO       GPIOB
#define DECA1_IRQ            GPIO_PIN_0

/* -------- UWB 2 号机引脚 (已避开 JTAG 陷阱) -------- */
#define UWB2_CS_PORT         GPIOB
#define UWB2_CS_PIN          GPIO_PIN_12  // PB12 (纯净安全)

#define DW2_RSTn_GPIO        GPIOB
#define DW2_RSTn             GPIO_PIN_13  // PB13 (纯净安全)

#define DECA2_IRQ_GPIO       GPIOB
#define DECA2_IRQ            GPIO_PIN_14  // PB14 (纯净安全)

/* -------- 共享唤醒引脚 (如果有连的话) -------- */
#define DW_WAKEUP_GPIO_Port  GPIOA
#define DW_WAKEUP_Pin        GPIO_PIN_8
/* =================================================================================*/

/* 全局双 UWB 切换标志位：1 代表操作 UWB1，2 代表操作 UWB2 */
extern uint8_t Active_UWB;

#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif

/* DW IC IRQ handler type. */
typedef void (*port_dwic_isr_t)(void);

void port_set_dwic_isr(port_dwic_isr_t isr);

/* MACRO function */
#define GPIO_ResetBits(x,y)         gpio_bit_reset(x,y)
#define GPIO_SetBits(x,y)           gpio_bit_set(x,y)
#define GPIO_ReadInputDataBit(x,y)  gpio_input_bit_get(x,y)

/* port function prototypes */
void Sleep(uint32_t Delay);
unsigned long portGetTickCnt(void);

void port_set_dw_ic_spi_slowrate(void);
void port_set_dw_ic_spi_fastrate(void);

void process_dwRSTn_irq(void);
void process_deca_irq(void);

void reset_DWIC(void);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);

void wakeup_device_with_io(void);
void make_very_short_wakeup_io(void);

/* 新增接口：底层硬件函数 */
void UWB_SPI_Init(void);
void UWB_CS_Low(void);
void UWB_CS_High(void);

#define SET_WAKEUP_PIN_IO_LOW       gpio_bit_reset(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin)
#define SET_WAKEUP_PIN_IO_HIGH      gpio_bit_set(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin)

#define WAIT_500uSEC    Sleep(1) 

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */