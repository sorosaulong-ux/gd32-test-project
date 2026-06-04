/*!
    \file    main.c
    \brief   dual CAN mode and dual CANFD mode communication demo

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
#include "string.h"

#define  BUFFER_SIZE       8U
#define  BUFFER_SIZE_FD    12U
#define CAN_MESSAGE_ID     0xAA

can_transmit_message_struct tx_message;
can_receive_message_struct rx_message;
can_transmit_message_struct fdtx_message;
can_receive_message_struct fdrx_message;
uint16_t dtm_can2_receive_flag;
uint16_t dtm_can5_receive_flag;
uint16_t i;

void cache_enable(void);

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_gpio_config(void)
{
    /* notice to open the other DTM_CAN clock in the same cluster, please refer to user manual for details */
    rcu_dtm_can_clock_config(DTM_CAN0, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN0);
    rcu_dtm_can_clock_config(DTM_CAN1, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN1);
    rcu_dtm_can_clock_config(DTM_CAN2, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN2);
    rcu_dtm_can_clock_config(DTM_CAN3, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN3);
    rcu_dtm_can_clock_config(DTM_CAN4, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN4);
    rcu_dtm_can_clock_config(DTM_CAN5, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN5);
    rcu_dtm_can_clock_config(DTM_CAN6, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN6);
    rcu_dtm_can_clock_config(DTM_CAN7, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN7);
    /* enable CAN port clock */
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOM);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOE);

    /* deinitialize CAN cluster: CAN0, CAN1 and CAN2 */
    can_deinit(DTM_CAN0);
    can_deinit(DTM_CAN2);
    can_deinit(DTM_CAN4);
    can_deinit(DTM_CAN5);
    
    /* configure DTM_CAN0 GPIO */
    /* Tx pin */
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_10);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_af_set(GPIOH, GPIO_AF_4, GPIO_PIN_10);

    /* Rx pin */
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_12);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOH, GPIO_AF_8, GPIO_PIN_12);

    /* configure DTM_CAN2 GPIO */
    /* Tx pin */
    gpio_output_options_set(GPIOM, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_2);
    gpio_mode_set(GPIOM, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_af_set(GPIOM, GPIO_AF_3, GPIO_PIN_2);

    /* Rx pin */
    gpio_output_options_set(GPIOM, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_3);
    gpio_mode_set(GPIOM, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_af_set(GPIOM, GPIO_AF_8, GPIO_PIN_3);

    /* configure DTM_CAN4 GPIO */
    /* Tx pin */
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_11);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_11);
    gpio_af_set(GPIOH, GPIO_AF_1, GPIO_PIN_11);

    /* Rx pin */
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_12);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOB, GPIO_AF_8, GPIO_PIN_12);

    /* configure DTM_CAN5 GPIO */
    /* Tx pin */
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_15);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_15);
    gpio_af_set(GPIOE, GPIO_AF_3, GPIO_PIN_15);

    /* Rx pin */
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_14);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_af_set(GPIOE, GPIO_AF_8, GPIO_PIN_14);
}

/*!
    \brief      configure BSP
    \param[in]  none
    \param[out] none
    \retval     none
*/
void bsp_board_config(void)
{
    /* configure USART */
    gd_eval_com_init(EVAL_COMA);

    /* configure K1 key */
    gd_eval_key_init(KEY_1, KEY_MODE_GPIO);

    /* configure LEDs */
    gd_eval_led_init(LED1);
    gd_eval_led_off(LED1);
    gd_eval_led_init(LED2);
    gd_eval_led_off(LED2);
    gd_eval_led_init(LED3);
    gd_eval_led_off(LED3);
    gd_eval_led_init(LED4);
    gd_eval_led_off(LED4);
}

/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_config(void)
{
    can_parameter_struct can_param;
    can_filter_struct can_filter;
    can_rx_fifo_struct rx_fifo0_config;
    can_tx_buffer_struct tx_buffer_config;
    can_filter_element_struct filter_element;

    /* initialize parameter structure */
    can_struct_para_init(CAN_INIT_STRUCT, &can_param);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    can_struct_para_init(CAN_RX_FIFO_STRUCT, &rx_fifo0_config);
    can_struct_para_init(CAN_TX_BUFFER_STRUCT, &tx_buffer_config);
    can_struct_para_init(CAN_FILTER_ELEMENT_STRUCT, &filter_element);

    /* set CAN parameters */
    can_param.auto_retransmission_enable = ENABLE;
    can_param.transmit_pause_enable = DISABLE;
    can_param.edge_filter_enable = DISABLE;
    can_param.protocol_exception_enable = DISABLE;
    can_param.wide_message_enable = DISABLE;

    /* CAN baudrate = 1 Mbps, 70%, based on CAN clock = 80MHz */
    can_param.prescaler = 4;
    can_param.resync_jump_width = 2;
    can_param.time_segment_1 = 13;
    can_param.time_segment_2 = 6;

    /* setup filter parameters */
    can_filter.non_match_std_frame_accept = CAN_ACCEPT_INTO_RXFIFO0;
    can_filter.non_match_ext_frame_accept = CAN_ACCEPT_INTO_RXFIFO0;
    can_filter.remote_std_frame_accept = CAN_REMOTE_FILTER;
    can_filter.remote_ext_frame_accept = CAN_REMOTE_FILTER;
    can_filter.filter_start_address_std_frame = 0;
    can_filter.filter_start_address_ext_frame = 0;
    can_filter.filter_number_std_frame = 1;
    can_filter.filter_number_ext_frame = 0;
    can_filter.and_mask_ext_frame = 0;

    /* configure RX FIFO 0 */
    rx_fifo0_config.fifo_operation_mode = CAN_RXFIFO_BLOCKING;
    rx_fifo0_config.fifo_watermark = 0;
    rx_fifo0_config.fifo_size = 8;
    rx_fifo0_config.fifo_start_address = 0x100;
    rx_fifo0_config.fifo_element_size = CAN_RXFS_8BYTES;

    /* configure TX buffer */
    tx_buffer_config.tx_buffer_start_address = 0x300;
    tx_buffer_config.dedicate_buffer_size = 4;
    tx_buffer_config.fifo_or_queue_mode = CAN_TXFIFO_OPERATION;
    tx_buffer_config.fifo_or_queue_size = 4;
    tx_buffer_config.tx_buffer_element_size = CAN_TXBS_8BYTES;

    /* link configurations to the parameter structure */
    can_param.filter_config = &can_filter;
    can_param.rx_fifo0_config = &rx_fifo0_config;
    can_param.rx_fifo1_config = NULL;
    can_param.rx_buffer_config = NULL;
    can_param.tx_buffer_config = &tx_buffer_config;
    can_param.tx_event_fifo_config = NULL;

    /* setup a filter for message ID */
    filter_element.filter_type = CAN_FILTER_DUAL;
    filter_element.config = CAN_FILTER_TO_RXFIFO0;
    filter_element.id1 = CAN_MESSAGE_ID;                        /* standard ID to match */
    filter_element.id2_or_mask_or_rxbuffercfg = CAN_MESSAGE_ID;

    /* initialize DTM_CAN4 */
    while(SET != can_sram_init_state_get(DTM_CAN4)) {
    }
    can_init(DTM_CAN4, &can_param);
    /* initialize DTM_CAN5 */
    while(SET != can_sram_init_state_get(DTM_CAN5)) {
    }
    can_init(DTM_CAN5, &can_param);
    
    can_filter_set(DTM_CAN4, CAN_FF_STANDARD, 0, &filter_element);
    can_filter_set(DTM_CAN5, CAN_FF_STANDARD, 0, &filter_element);

    /* enable DTM_CAN5 RX FIFO 0 new message interrupt */
    can_interrupt_enable(DTM_CAN5, CAN_INT_RFIFO0_NEW);
    can_mcan_interrupt_line_config(DTM_CAN5, CAN_INTR_LINE0, CAN_INT_RFIFO0_NEW);
    can_mcan_interrupt_line_enable(DTM_CAN5, CAN_INTR_LINE0);
    /* enable NVIC for DTM_CAN5 */
    nvic_irq_enable(DTM_CAN5_INT0_IRQn, 0, 0);

    /* enable DTM_CAN5 */
    can_operating_mode_enable(DTM_CAN5, CAN_MODE_NORMAL);
    /* enable DTM_CAN4 */
    can_operating_mode_enable(DTM_CAN4, CAN_MODE_NORMAL);
}

/*!
    \brief      initialize CANFD function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void canfd_config(void)
{
    can_parameter_struct can_param;
    can_filter_struct can_filter;
    can_rx_fifo_struct rx_fifo0_config;
    can_tx_buffer_struct tx_buffer_config;
    can_filter_element_struct filter_element;
    can_fd_parameter_struct can_fd_param;

    /* initialize parameter structure */
    can_struct_para_init(CAN_INIT_STRUCT, &can_param);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    can_struct_para_init(CAN_RX_FIFO_STRUCT, &rx_fifo0_config);
    can_struct_para_init(CAN_TX_BUFFER_STRUCT, &tx_buffer_config);
    can_struct_para_init(CAN_FILTER_ELEMENT_STRUCT, &filter_element);
    can_struct_para_init(CAN_FD_INIT_STRUCT, &can_fd_param);

    /* set CAN parameters */
    can_param.auto_retransmission_enable = ENABLE;
    can_param.transmit_pause_enable = DISABLE;
    can_param.edge_filter_enable = ENABLE;
    can_param.protocol_exception_enable = ENABLE;
    can_param.wide_message_enable = DISABLE;

    /* CAN baudrate = 500 Kbps, 70%, based on CAN clock = 80MHz */
    can_param.prescaler = 8;
    can_param.resync_jump_width = 2;
    can_param.time_segment_1 = 13;
    can_param.time_segment_2 = 6;
   
    /* set CAN FD parameters */
    can_fd_param.bitrate_switch_enable = ENABLE;
    can_fd_param.iso_can_fd_enable = ENABLE;
    can_fd_param.tdc_enable = DISABLE;

    /* CAN FD baudrate = 2 Mbps, 70%, based on CAN clock = 80MHz */
    can_fd_param.prescaler = 2;
    can_fd_param.resync_jump_width = 2;
    can_fd_param.time_segment_1 = 13;
    can_fd_param.time_segment_2 = 6;

    /* setup filter parameters */
    can_filter.non_match_std_frame_accept = CAN_ACCEPT_INTO_RXFIFO0;
    can_filter.non_match_ext_frame_accept = CAN_ACCEPT_INTO_RXFIFO0;
    can_filter.remote_std_frame_accept = CAN_REMOTE_FILTER;
    can_filter.remote_ext_frame_accept = CAN_REMOTE_FILTER;
    can_filter.filter_start_address_std_frame = 0;
    can_filter.filter_start_address_ext_frame = 0;
    can_filter.filter_number_std_frame = 1;
    can_filter.filter_number_ext_frame = 0;
    can_filter.and_mask_ext_frame = 0;

    /* configure RX FIFO 0 */
    rx_fifo0_config.fifo_operation_mode = CAN_RXFIFO_BLOCKING;
    rx_fifo0_config.fifo_watermark = 0;
    rx_fifo0_config.fifo_size = 8;
    rx_fifo0_config.fifo_start_address = 0x100;
    rx_fifo0_config.fifo_element_size = CAN_RXFS_16BYTES;

    /* configure TX buffer */
    tx_buffer_config.tx_buffer_start_address = 0x300;
    tx_buffer_config.dedicate_buffer_size = 4;
    tx_buffer_config.fifo_or_queue_mode = CAN_TXFIFO_OPERATION;
    tx_buffer_config.fifo_or_queue_size = 4;
    tx_buffer_config.tx_buffer_element_size = CAN_TXBS_16BYTES;

    /* link configurations to the parameter structure */
    can_param.filter_config = &can_filter;
    can_param.rx_fifo0_config = &rx_fifo0_config;
    can_param.rx_fifo1_config = NULL;
    can_param.rx_buffer_config = NULL;
    can_param.tx_buffer_config = &tx_buffer_config;
    can_param.tx_event_fifo_config = NULL;

    /* setup a filter for message ID */
    filter_element.filter_type = CAN_FILTER_DUAL;
    filter_element.config = CAN_FILTER_TO_RXFIFO0;
    filter_element.id1 = CAN_MESSAGE_ID;                        /* standard ID to match */
    filter_element.id2_or_mask_or_rxbuffercfg = CAN_MESSAGE_ID;
    
    /* initialize DTM_CAN0 */
    while(SET != can_sram_init_state_get(DTM_CAN0)) {
    }
    can_init(DTM_CAN0, &can_param);
    /* initialize DTM_CAN2 */
    while(SET != can_sram_init_state_get(DTM_CAN2)) {
    }
    can_init(DTM_CAN2, &can_param);

    can_filter_set(DTM_CAN0, CAN_FF_STANDARD, 0, &filter_element);
    can_filter_set(DTM_CAN2, CAN_FF_STANDARD, 0, &filter_element);

    /* FD configuration */
    can_operating_mode_enable(DTM_CAN0, CAN_MODE_INIT);
    can_fd_config(DTM_CAN0, &can_fd_param);
    can_operating_mode_enable(DTM_CAN2, CAN_MODE_INIT);
    can_fd_config(DTM_CAN2, &can_fd_param);

    /* enable DTM_CAN2 RX FIFO 0 new message interrupt */
    can_interrupt_enable(DTM_CAN2, CAN_INT_RFIFO0_NEW);
    can_mcan_interrupt_line_config(DTM_CAN2, CAN_INTR_LINE0, CAN_INT_RFIFO0_NEW);
    can_mcan_interrupt_line_enable(DTM_CAN2, CAN_INTR_LINE0);
    /* enable NVIC for DTM_CAN2 */
    nvic_irq_enable(DTM_CAN2_INT0_IRQn, 0, 0);

    /* enable DTM_CAN2 */
    can_operating_mode_enable(DTM_CAN2, CAN_MODE_NORMAL);
    /* enable DTM_CAN0 */
    can_operating_mode_enable(DTM_CAN0, CAN_MODE_NORMAL);
}

/*!
    \brief      check received data
    \param[in]  none
    \param[out] none
    \retval     none
*/
void communication_check(void)
{
    /* DTM_CAN5 receive data correctly, the received data is printed */
    if(1 == dtm_can5_receive_flag) {
        dtm_can5_receive_flag = 0;
        /* receive message from FIFO 0 */
        can_message_receive(DTM_CAN5, CAN_RXFIFO0, &rx_message);

        /* check message content */
        if((CAN_MESSAGE_ID == rx_message.id) && (0 == memcmp(rx_message.data, tx_message.data, tx_message.data_bytes))) {
            printf("\r\nDTM_CAN5 receive data: \r\n");
            for(i = 0; i < rx_message.data_bytes; i++) {
                printf("%02x\r\n", rx_message.data[i]);
            }
            gd_eval_led_toggle(LED3);
        }
    }
    /* DTM_CAN2 receive data correctly, the received data is printed */
    if(1 == dtm_can2_receive_flag) {
        /* clear receive flag */
        dtm_can2_receive_flag = 0;
        /* receive message from FIFO 0 */
        can_message_receive(DTM_CAN2, CAN_RXFIFO0, &fdrx_message);

        /* check message content */
        if((CAN_MESSAGE_ID == fdrx_message.id) && (0 == memcmp(fdrx_message.data, fdtx_message.data, fdtx_message.data_bytes))) {
            printf("\r\nDTM_CAN2 receive data: \r\n");
            for(i = 0; i < fdrx_message.data_bytes; i++) {
                printf("%02x\r\n", fdrx_message.data[i]);
            }
            gd_eval_led_toggle(LED4);
        }
    }
}

/*!
    \brief      enable the CPU cache
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

/*!
    \brief      test DTM_CAN4 message transmission
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus dtm_can4_message_transmit_test(void)
{
    uint8_t transmit_mailbox = 0;
    ErrStatus ret = SUCCESS;
    uint16_t i;

    /* initialize transmit message structure */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx_message);

    /* set up message to send */
    tx_message.id = CAN_MESSAGE_ID;
    tx_message.rtr = CAN_FT_DATA;
    tx_message.xtd = CAN_FF_STANDARD;
    tx_message.esi = 0;
    tx_message.brs = DISABLE;
    tx_message.fdf = DISABLE;
    tx_message.message_marker = 0;
    tx_message.ev_fifo_control = CAN_TXEVENT_FIFO_DISABLE;
    tx_message.data_bytes = 8;
    for(i = 0; i < 8; i++) {
        tx_message.data[i] = i + 1;
    }

    /* transmit message */
    transmit_mailbox = can_message_transmit_prepare(DTM_CAN4, &tx_message);

    /* check if transmission was valid */
    if(transmit_mailbox == 0xFF) {
        ret = ERROR;
    } else {
        can_message_transmit_add(DTM_CAN4, transmit_mailbox);
    }
    return ret;
}

/*!
    \brief      test DTM_CAN0 message transmission
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus dtm_can0_message_transmit_test(void)
{
    uint8_t transmit_mailbox = 0;
    ErrStatus ret = SUCCESS;
    uint16_t i;

    /* initialize transmit message structure */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &fdtx_message);

    /* set up message to send */
    fdtx_message.id = CAN_MESSAGE_ID;
    fdtx_message.rtr = CAN_FT_DATA;
    fdtx_message.xtd = CAN_FF_STANDARD;
    fdtx_message.esi = 0;
    fdtx_message.brs = ENABLE;
    fdtx_message.fdf = ENABLE;
    fdtx_message.message_marker = 0;
    fdtx_message.ev_fifo_control = CAN_TXEVENT_FIFO_DISABLE;
    fdtx_message.data_bytes = 16;
    for(i = 0; i < 16; i++) {
        fdtx_message.data[i] = i + 1;
    }

    /* transmit message */
    transmit_mailbox = can_message_transmit_prepare(DTM_CAN0, &fdtx_message);

    /* check if transmission was valid */
    if(transmit_mailbox == 0xFF) {
        ret = ERROR;
    } else {
        can_message_transmit_add(DTM_CAN0, transmit_mailbox);
    }
    return ret;
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    cache_enable();
    /* configure systick */
    systick_config();
    /* configure board */
    bsp_board_config();
    /* configure GPIO */
    can_gpio_config();
    /* initialize CAN and filter */
    can_config();
    /* initialize CANFD and filter */
    canfd_config();

    printf("\r\ncommunication test CAN and CANFD, please press K1 key to start! \r\n");


    while(1) {
        /* test whether the K1 key is pressed */
        if(SET == gd_eval_key_state_get(KEY_1)) {
            delay_1ms(100);
            if(SET == gd_eval_key_state_get(KEY_1)) {
                /* transmit message */
                if(SUCCESS != dtm_can4_message_transmit_test()) {
                    /* transmission failed, LED1 on */
                    gd_eval_led_on(LED1);
                }

                printf("\r\nDTM_CAN4 transmit data: \r\n");
                for(i = 0; i < tx_message.data_bytes; i++) {
                    printf("%02x\r\n", tx_message.data[i]);
                }

                /* transmit message */
                if(SUCCESS != dtm_can0_message_transmit_test()) {
                    /* transmission failed, LED2 on */
                    gd_eval_led_on(LED2);
                }

                printf("\r\nDTM_CAN0 transmit data: \r\n");
                for(i = 0; i < fdtx_message.data_bytes; i++) {
                    printf("%02x\r\n", fdtx_message.data[i]);
                }
                /* waiting for the K1 key up */
                while(SET == gd_eval_key_state_get(KEY_1));
            }
        }
        communication_check();
    }
}
