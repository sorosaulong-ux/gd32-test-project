#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "stm32f10x.h"

/* =================================================================================
 * UWB Ӳ�����Ŷ��� (STM32F103 �ƶ���ǩ��)
 * SPI���� (CS:PA4, SCK:PA5, MISO:PA6, MOSI:PA7) ���� MySPI.c ������
 * =================================================================================*/

/* RST ��λ����: PB0 */
#define DW_RSTn_PORT         GPIOB
#define DW_RSTn_PIN          GPIO_Pin_0
#define DW_RSTn_CLK          RCC_APB2Periph_GPIOB

/* IRQ �ж�����: PB1 */
#define DECA_IRQ_PORT        GPIOB
#define DECA_IRQ_PIN         GPIO_Pin_1
#define DECA_IRQ_CLK         RCC_APB2Periph_GPIOB

/* WAKEUP ����: PB10 */
#define DW_WKUP_PORT         GPIOB
#define DW_WKUP_PIN          GPIO_Pin_10
#define DW_WKUP_CLK          RCC_APB2Periph_GPIOB

typedef void (*port_dwic_isr_t)(void);
void port_set_dwic_isr(port_dwic_isr_t isr);

void Sleep(uint32_t Delay);
void deca_sleep(unsigned int time_ms);
void deca_usleep(unsigned long time_us);

void port_set_dw_ic_spi_slowrate(void);
void port_set_dw_ic_spi_fastrate(void);

void reset_DWIC(void);
void UWB_Hardware_Init(void);

uint32_t port_CheckEXT_IRQ(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
