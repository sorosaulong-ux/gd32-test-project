/*!
 *  \file    uwb_port.h
 *  \brief   DW3000 port layer — GD32A7xx adaptation (software SPI)
 */

#ifndef UWB_PORT_H_
#define UWB_PORT_H_

#include "gd32a7xx.h"
#include "deca_device_api.h"
#include <stdint.h>

/* deca_regs.h redefines GPIO_MODE_MASK → restore GD32's function-like macro */
#undef GPIO_MODE_MASK
#define GPIO_MODE_MASK(n)  ((uint32_t)((uint32_t)0x3U << (2U * (n))))

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  PIN MAP — change these to match your wiring
 * ====================================================================*/

/* Shared SPI bus (software bit‑bang) */
#define UWB_SPI_SCL_PORT     GPIOE
#define UWB_SPI_SCL_PIN      GPIO_PIN_6
#define UWB_SPI_MISO_PORT    GPIOE
#define UWB_SPI_MISO_PIN     GPIO_PIN_5
#define UWB_SPI_MOSI_PORT    GPIOH
#define UWB_SPI_MOSI_PIN     GPIO_PIN_14

/* Module 1 */
#define UWB1_CS_PORT         GPIOL
#define UWB1_CS_PIN          GPIO_PIN_2
#define UWB1_IRQ_PORT        GPIOI
#define UWB1_IRQ_PIN         GPIO_PIN_8
#define UWB1_RST_PORT        GPIOL
#define UWB1_RST_PIN         GPIO_PIN_0

/* Module 2 */
#define UWB2_CS_PORT         GPIOB
#define UWB2_CS_PIN          GPIO_PIN_11
#define UWB2_IRQ_PORT        GPIOI
#define UWB2_IRQ_PIN         GPIO_PIN_10
#define UWB2_RST_PORT        GPIOI
#define UWB2_RST_PIN         GPIO_PIN_11

/* WAKEUP */
#define UWB_WAKE_PORT        GPIOB
#define UWB_WAKE_PIN         GPIO_PIN_10

/* ====================================================================
 *  Globals
 * ====================================================================*/
extern uint8_t Active_UWB;   /* 1 = module 1,  2 = module 2 */

/* ====================================================================
 *  IRQ callback
 * ====================================================================*/
typedef void (*port_dwic_isr_t)(void);
void port_set_dwic_isr(port_dwic_isr_t isr);

/* ====================================================================
 *  Required by deca_device_api.h
 * ====================================================================*/
int  writetospi(uint16_t hdr_len, const uint8_t *hdr,
                uint16_t body_len, const uint8_t *body);
int  writetospiwithcrc(uint16_t hdr_len, const uint8_t *hdr,
                        uint16_t body_len, const uint8_t *body, uint8_t crc);
int  readfromspi(uint16_t hdr_len, uint8_t *hdr,
                  uint16_t rd_len, uint8_t *rd_buf);
void wakeup_device_with_io(void);
void make_very_short_wakeup_io(void);

/* ====================================================================
 *  Application‑layer API
 * ====================================================================*/
void UWB_SPI_Init(void);
void reset_DWIC(void);
void port_set_dw_ic_spi_slowrate(void);
void port_set_dw_ic_spi_fastrate(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
void process_deca_irq(void);

/* ====================================================================
 *  Timing
 * ====================================================================*/
void deca_sleep(unsigned int ms);
void deca_usleep(unsigned long us);
int  usleep(uint32_t usec);
void Sleep(uint32_t x);
void delay_1us(uint32_t count);
unsigned long portGetTickCnt(void);
decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

/* ms tick counter (fed from SysTick_Handler in gd32a7xx_it.c) */
void uwb_tick_inc(void);
uint32_t uwb_tick_get(void);

/* ====================================================================
 *  Convenience macros (compatible with original port.h)
 * ====================================================================*/
#define GPIO_ResetBits(p, pin)          GPIO_BC(p) = (pin)
#define GPIO_SetBits(p, pin)            GPIO_BOP(p) = (pin)
#define GPIO_ReadInputDataBit(p, pin)   gpio_input_bit_get(p, pin)
#define SET_WAKEUP_PIN_IO_LOW           GPIO_BC(UWB_WAKE_PORT) = UWB_WAKE_PIN
#define SET_WAKEUP_PIN_IO_HIGH          GPIO_BOP(UWB_WAKE_PORT) = UWB_WAKE_PIN

#ifdef __cplusplus
}
#endif

#endif
