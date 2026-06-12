/*!
 *  \file    ble.c
 *  \brief   ZX-D36 BLE 蓝牙驱动 — GD32A7
 *
 *  UART4: PJ1(TX) PJ0(RX) @ 9600
 *  STATE: PJ2 (连接时高电平)
 */

#include "ble.h"
#include "gd32a712_evb.h"
#include <stdio.h>

void ble_init(void)
{
    /* 1. 使能时钟 */
    rcu_periph_clock_enable(EVAL_BLE_GPIO_CLK);
    rcu_periph_clock_enable(EVAL_BLE_CLK);

    /* 2. GPIO AF */
    gpio_af_set(EVAL_BLE_GPIO_PORT, EVAL_BLE_TX_AF, EVAL_BLE_TX_PIN);
    gpio_af_set(EVAL_BLE_GPIO_PORT, EVAL_BLE_RX_AF, EVAL_BLE_RX_PIN);

    /* 3. TX: PJ1 */
    gpio_mode_set(EVAL_BLE_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_BLE_TX_PIN);
    gpio_output_options_set(EVAL_BLE_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, EVAL_BLE_TX_PIN);

    /* 4. RX: PJ0 */
    gpio_mode_set(EVAL_BLE_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_BLE_RX_PIN);
    gpio_output_options_set(EVAL_BLE_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, EVAL_BLE_RX_PIN);

    /* 5. STATE: PJ2 (输入上拉) */
    gpio_mode_set(EVAL_BLE_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_2);

    /* 6. UART4 配置: 9600-8-N-1 */
    linflexd_uart_parameter_struct uart_params;
    uart_params.baudrate = 9600UL;
    uart_params.parityenable = DISABLE;
    uart_params.paritytype = LINFLEXD_UART_PARITY_EVEN;
    uart_params.stopbitscount = LINFLEXD_UART_ONE_STOP_BIT;
    uart_params.wordlength = LINFLEXD_UART_8_BITS;
    uart_params.txmodetype = LINFLEXD_UART_TXRX_BUFFER_MODE;
    uart_params.rxmodetype = LINFLEXD_UART_TXRX_BUFFER_MODE;

    linflexd_uart_init(EVAL_BLE, &uart_params);
    linflexd_uart_receiver_config(EVAL_BLE, LINFLEXD_UART_RECEIVE_ENABLE);
    linflexd_uart_transmitter_config(EVAL_BLE, LINFLEXD_UART_TRANSMIT_ENABLE);

    printf("[BLE] UART4 init OK (PJ0/PJ1 @ 9600)\r\n");
}

uint8_t ble_is_connected(void)
{
    return gpio_input_bit_get(EVAL_BLE_GPIO_PORT, GPIO_PIN_2);
}
