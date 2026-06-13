/*!
 *  \file    usart_esp.c
 *  \brief   UART3 (EVAL_COMB) driver for ESP8266 — GD32A7
 *
 *  PJ7 = TX (AF13), PJ8 = RX (AF13), LINFLEXD_UART3
 *  Interrupt-based RX → fills global esp8266_buf[]
 */

#include "usart_esp.h"
#include <stdio.h>

/* ─── UART3 config (matching ESP8266 default 115200-8-N-1) ─── */
static linflexd_uart_parameter_struct uart3_params = {
    .baudrate      = 115200UL,
    .parityenable  = DISABLE,
    .paritytype    = LINFLEXD_UART_PARITY_EVEN,
    .stopbitscount = LINFLEXD_UART_ONE_STOP_BIT,
    .wordlength    = LINFLEXD_UART_8_BITS,
    .txmodetype    = LINFLEXD_UART_TXRX_BUFFER_MODE,
    .rxmodetype    = LINFLEXD_UART_TXRX_BUFFER_MODE
};

/* ─── Receive buffer (shared with ESP8266 module) ─── */
unsigned char esp8266_buf[512];
unsigned short esp8266_cnt = 0, esp8266_cntPre = 0;

void usart3_esp_init(uint32_t baudrate)
{
    uart3_params.baudrate = baudrate;

    /* UART clock source */
    rcu_linflexd_clock_config(RCU_LINFLEXDSRC_CKSYS, 4U);

    /* Enable clocks */
    rcu_periph_clock_enable(EVAL_COMB_GPIO_CLK);
    rcu_periph_clock_enable(EVAL_COMB_CLK);

    /* GPIO alternate function */
    gpio_af_set(EVAL_COMB_GPIO_PORT, EVAL_COMB_TX_AF, EVAL_COMB_TX_PIN);
    gpio_af_set(EVAL_COMB_GPIO_PORT, EVAL_COMB_RX_AF, EVAL_COMB_RX_PIN);

    /* TX pin */
    gpio_mode_set(EVAL_COMB_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_COMB_TX_PIN);
    gpio_output_options_set(EVAL_COMB_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, EVAL_COMB_TX_PIN);

    /* RX pin */
    gpio_mode_set(EVAL_COMB_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_COMB_RX_PIN);
    gpio_output_options_set(EVAL_COMB_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, EVAL_COMB_RX_PIN);

    /* Init LINFlexD UART3 */
    linflexd_uart_init(EVAL_COMB, &uart3_params);
    linflexd_uart_receiver_config(EVAL_COMB, LINFLEXD_UART_RECEIVE_ENABLE);
    linflexd_uart_transmitter_config(EVAL_COMB, LINFLEXD_UART_TRANSMIT_ENABLE);

    /* Enable RX interrupt (DRIE = data reception interrupt) */
    LINFLEXD_UART_LINIER(EVAL_COMB) |= LINFLEXD_UART_LINIER_DRIE;
    /* 优先级必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (2) */
    nvic_irq_enable(LINFlexD3_IRQn, 5, 0);

    printf("[ESP] UART3 init OK @ %lu baud\r\n", (unsigned long)baudrate);
}

/* ─── Send raw bytes (blocking, per-byte) ─── */
void Usart_SendString(uint32_t periph, unsigned char *str, unsigned short len)
{
    (void)periph;
    for (unsigned short i = 0; i < len; i++) {
        linflexd_uart_byte_transmit(EVAL_COMB, str[i]);
        while (RESET == linflexd_uart_flag_get(EVAL_COMB, LINFLEXD_UART_FLAG_DTF_TFF));
        linflexd_uart_flag_clear(EVAL_COMB, LINFLEXD_UART_FLAG_DTF_TFF);
    }
}

/* ─── UART3 RX interrupt handler ─── */
void LINFlexD3_IRQHandler(void)
{
    /* Check reception complete flag */
    if (linflexd_uart_interrupt_flag_get(EVAL_COMB,
        LINFLEXD_UART_INT_FLAG_DRF) != RESET)
    {
        if (esp8266_cnt >= sizeof(esp8266_buf))
            esp8266_cnt = 0;

        esp8266_buf[esp8266_cnt++] =
            (unsigned char)linflexd_uart_byte_receive(EVAL_COMB);

        /* Release message buffer (clear RMB flag) */
        linflexd_uart_flag_clear(EVAL_COMB, LINFLEXD_UART_FLAG_RMB);
    }
}
