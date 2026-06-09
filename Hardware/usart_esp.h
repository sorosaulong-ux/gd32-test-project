/*!
 *  \file    usart_esp.h
 *  \brief   USART compatibility layer — maps GD32F3 API to GD32A7 UART3
 *
 *  GD32F3 code uses:  USART_DEBUG, Usart_SendString(), UsartPrintf()
 *  GD32A7 maps to:    LINFlexD_UART3 on EVAL_COMB (PJ7=TX, PJ8=RX)
 *
 *  ESP8266 receives on UART3 → interrupt → esp8266_buf[]
 *  Debug printf goes to EVAL_COMA (PH7=TX, PH8=RX) via existing uart.h
 */

#ifndef __USART_ESP_H
#define __USART_ESP_H

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USART_DEBUG is UART3 in GD32F3 code — ESP8266 channel */
#define USART_DEBUG            EVAL_COMB

/* ---- re-map GD32F3 USART API to GD32A7 ---- */

#define UsartPrintf(periph, fmt, ...)  printf(fmt, ##__VA_ARGS__)

void usart3_esp_init(uint32_t baudrate);
void Usart_SendString(uint32_t periph, unsigned char *str, unsigned short len);
void LINFLEXD_UART3_IRQHandler(void);   /* called from gd32a7xx_it.c if needed */

#ifdef __cplusplus
}
#endif

#endif /* __USART_ESP_H */
