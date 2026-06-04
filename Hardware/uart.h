/*!
 *  \file    uart.h
 *  \brief   LINFlexD_UART driver header (modular)
 *
 *  Based on GD32A7xx LINFlexD_UART1 on EVAL_COMA (GPIOH.7=TX, GPIOH.8=RX).
 *  Supports printf retargeting via fputc.
 */

#ifndef __UART_H
#define __UART_H

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include <stdio.h>

#ifdef __cplusplus
 extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * UART configuration macros
 * ----------------------------------------------------------------------------*/

/* UART instance: EVAL_COMA = LINFlexD_UART1 */
#define UART_PERIPH             EVAL_COMA

/* clock config */
#define UART_BAUDRATE           (115200UL)
#define UART_CLK_DIV            (4U)

/* on GD32A712A-EVB: TX=PH.7(AF13), RX=PH.8(AF13) */
#define UART_TX_PORT            EVAL_COMA_GPIO_PORT
#define UART_TX_PIN             EVAL_COMA_TX_PIN
#define UART_TX_AF              EVAL_COMA_TX_AF
#define UART_RX_PORT            EVAL_COMA_GPIO_PORT
#define UART_RX_PIN             EVAL_COMA_RX_PIN
#define UART_RX_AF              EVAL_COMA_RX_AF

/* ----------------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------------*/

void uart_init(void);
void uart_send_byte(uint8_t data);
void uart_send_string(const char *str);
uint8_t uart_receive_byte(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
