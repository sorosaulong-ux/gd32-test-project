/*!
 *  \file    uart.c
 *  \brief   LINFlexD_UART driver implementation
 *
 *  Uses LINFlexD_UART1 on EVAL_COMA (GPIOH.7=TX, GPIOH.8=RX) at 115200-8-N-1.
 *  printf retargeting is in gd32a712_evb.c (fputc -> eval_com).
 */

#include "uart.h"

/* ----------------------------------------------------------------------------
 * Reference to eval_com in gd32a712_evb.c (used by fputc for printf)
 * ----------------------------------------------------------------------------*/
extern uint32_t eval_com;

/* ----------------------------------------------------------------------------
 * Default UART parameters: 115200-8-N-1, buffer mode
 * ----------------------------------------------------------------------------*/
static linflexd_uart_parameter_struct uart_params = {
    .baudrate     = UART_BAUDRATE,
    .parityenable = DISABLE,
    .paritytype   = LINFLEXD_UART_PARITY_EVEN,  /* ignored when parity disable */
    .stopbitscount = LINFLEXD_UART_ONE_STOP_BIT,
    .wordlength   = LINFLEXD_UART_8_BITS,
    .txmodetype   = LINFLEXD_UART_TXRX_BUFFER_MODE,
    .rxmodetype   = LINFLEXD_UART_TXRX_BUFFER_MODE
};

/* ----------------------------------------------------------------------------
 * uart_init()
 *
 * Configures UART clock, GPIO, LINFlexD and synchronises with the
 * printf retargeting layer (fputc in gd32a712_evb.c drives eval_com).
 * ----------------------------------------------------------------------------*/
void uart_init(void)
{
    /* 1. set LINFlexD clock source = CK_SYS / UART_CLK_DIV */
    rcu_linflexd_clock_config(RCU_LINFLEXDSRC_CKSYS, UART_CLK_DIV);

    /* 2. enable GPIO and UART clocks */
    rcu_periph_clock_enable(EVAL_COMA_GPIO_CLK);
    rcu_periph_clock_enable(EVAL_COMA_CLK);

    /* 3. connect GPIO to UART alternate function */
    gpio_af_set(UART_TX_PORT, UART_TX_AF, UART_TX_PIN);
    gpio_af_set(UART_RX_PORT, UART_RX_AF, UART_RX_PIN);

    /* 4. configure TX pin: AF, push-pull + pull-up */
    gpio_mode_set(UART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, UART_TX_PIN);
    gpio_output_options_set(UART_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, UART_TX_PIN);

    /* 5. configure RX pin: AF, push-pull + pull-up */
    gpio_mode_set(UART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, UART_RX_PIN);
    gpio_output_options_set(UART_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, UART_RX_PIN);

    /* 6. initialise LINFlexD_UART with configured parameters */
    linflexd_uart_init(UART_PERIPH, &uart_params);
    linflexd_uart_receiver_config(UART_PERIPH, LINFLEXD_UART_RECEIVE_ENABLE);
    linflexd_uart_transmitter_config(UART_PERIPH, LINFLEXD_UART_TRANSMIT_ENABLE);

    /* 7. tell printf retargeting which UART to use */
    eval_com = UART_PERIPH;
}

/* ----------------------------------------------------------------------------
 * uart_send_byte()
 *
 * Blocking single-byte transmit on the configured UART.
 * ----------------------------------------------------------------------------*/
void uart_send_byte(uint8_t data)
{
    linflexd_uart_byte_transmit(UART_PERIPH, data);
    while (RESET == linflexd_uart_flag_get(UART_PERIPH, LINFLEXD_UART_FLAG_DTF_TFF));
    linflexd_uart_flag_clear(UART_PERIPH, LINFLEXD_UART_FLAG_DTF_TFF);
}

/* ----------------------------------------------------------------------------
 * uart_send_string()
 *
 * Blocking string transmit (null-terminated).
 * ----------------------------------------------------------------------------*/
void uart_send_string(const char *str)
{
    while (*str) {
        uart_send_byte((uint8_t)*str++);
    }
}

/* ----------------------------------------------------------------------------
 * uart_receive_byte()
 *
 * Blocking single-byte receive.  Waits forever if no data arrives.
 * ----------------------------------------------------------------------------*/
uint8_t uart_receive_byte(void)
{
    while (RESET == linflexd_uart_flag_get(UART_PERIPH, LINFLEXD_UART_FLAG_DRF_RFE));
    return linflexd_uart_byte_receive(UART_PERIPH);
}
