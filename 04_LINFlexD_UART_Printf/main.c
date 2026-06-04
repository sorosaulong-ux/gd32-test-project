/*!
    \file    main.c
    \brief   LINFlexD_UART printf demo

    \version 2025-08-26, V0.1.0, demo for GD32A7xx
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "systick.h"

static void test_status_led_init(void);
static void led_flash(uint8_t times);
static void linflexd_uart_config(void);
void cache_enable(void);

linflexd_uart_parameter_struct linflexd_uart_initpara = {
    .baudrate = 115200,
    .parityenable = DISABLE,
    .stopbitscount = LINFLEXD_UART_ONE_STOP_BIT,
    .wordlength = LINFLEXD_UART_8_BITS,
    .txmodetype = LINFLEXD_UART_TXRX_BUFFER_MODE,
    .rxmodetype = LINFLEXD_UART_TXRX_BUFFER_MODE
};


/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    /* enable the CPU cache */
    cache_enable();
    /* initialize the LEDs */
    test_status_led_init();

    /* configure systick */
    systick_config();

    /* flash the LEDs for 2 time */
    led_flash(2);

    /* configure LINFlexD_UART1 */
    linflexd_uart_config();

    /* configure KEY_1 key */
    gd_eval_key_init(KEY_1, KEY_MODE_GPIO);

    /* output a message on hyperterminal using printf function */
    printf("\r\n LINFlexD_UART printf example: please press the KEY_1 key. \r\n");

    while(1) {
        /* check if the KEY_1 is pressed */
        if(RESET != gd_eval_key_state_get(KEY_1)) {
            delay_1ms(100);

            if(RESET != gd_eval_key_state_get(KEY_1)) {
                /* turn on LED1 */
                gd_eval_led_on(LED1);
                /* output a message on hyperterminal using printf function */
                printf("\r\n LINFlexD_UART printf example \r\n");
            } else {
                /* turn off LED1 */
                gd_eval_led_off(LED1);
            }
        } else {
            /* turn off LED1 */
            gd_eval_led_off(LED1);
        }
    }
}

/*!
    \brief      test status led initialize
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void test_status_led_init(void)
{
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED3);
    gd_eval_led_init(LED4);
}

/*!
    \brief      flash the LED for test
    \param[in]  times: times to flash the LEDs
    \param[out] none
    \retval     none
*/
static void led_flash(uint8_t times)
{
    uint8_t i;

    for(i = 0; i < times; i++) {
        /* delay 400 ms */
        delay_1ms(400);

        /* turn on LEDs */
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED3);
        gd_eval_led_on(LED4);

        /* delay 400 ms */
        delay_1ms(400);
        /* turn off LEDs */
        gd_eval_led_off(LED1);
        gd_eval_led_off(LED2);
        gd_eval_led_off(LED3);
        gd_eval_led_off(LED4);
    }
}

/*!
    \brief      linflexd_uart configure
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void linflexd_uart_config(void)
{
    /* configure LINFlexD_UART clock */
    rcu_linflexd_clock_config(RCU_LINFLEXDSRC_CKSYS, 4);

    /* enable LINFlexD_UART GPIO clock */
    rcu_periph_clock_enable(EVAL_COMA_GPIO_CLK);

    /* enable LINFlexD_UART clock */
    rcu_periph_clock_enable(EVAL_COMA_CLK);

    /* connect port to LINFlexD_UART TX and RX */
    gpio_af_set(EVAL_COMA_GPIO_PORT, EVAL_COMA_TX_AF, EVAL_COMA_TX_PIN);
    gpio_af_set(EVAL_COMA_GPIO_PORT, EVAL_COMA_RX_AF, EVAL_COMA_RX_PIN);

    /* configure LINFlexD_UART TX and RX as alternate function push-pull */
    gpio_mode_set(EVAL_COMA_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_COMA_TX_PIN);
    gpio_output_options_set(EVAL_COMA_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, EVAL_COMA_TX_PIN);
    gpio_mode_set(EVAL_COMA_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_COMA_RX_PIN);
    gpio_output_options_set(EVAL_COMA_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, EVAL_COMA_RX_PIN);

    /* LINFlexD_UART configure */
    linflexd_uart_parameter_struct* initpara = &linflexd_uart_initpara;
    linflexd_uart_init(EVAL_COMA, initpara);
    /* enable transmitter and receiver */
    linflexd_uart_receiver_config(EVAL_COMA, LINFLEXD_UART_RECEIVE_ENABLE);
    linflexd_uart_transmitter_config(EVAL_COMA, LINFLEXD_UART_TRANSMIT_ENABLE);
}

/*!
    \brief      enable the CPU chache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cache_enable(void)
{
    /* enable i-cache */
    SCB_EnableICache();

    /* enable d-cache */
    SCB_EnableDCache();
}
