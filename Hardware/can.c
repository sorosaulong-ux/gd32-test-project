/*!
 *  \file    can.c
 *  \brief   CAN driver  GD32A712 EVB
 *
 *  CAN2  TX=PM2(AF3)  RX=PM3(AF8)  -> MCP2562FD -> CN4
 *  CAN0  TX=PH10(AF4) RX=PH12(AF8)
 *  CAN4  TX=PH11(AF1) RX=PB12(AF8)
 *  CAN5  TX=PE15(AF3) RX=PE14(AF8)
 */

#include "can.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 *  读 ECR / PSR 的方式：
 *  GD32A7xx SPL 没有暴露 DTM_CANx_BASE，
 *  改用 can_error_count_get() + can_flag_get() 等 SPL API。
 *  如果 SPL 版本里有对应函数则直接调用，否则用下面的方式
 *  通过 can_register_read() 或直接用已知偏移。
 *
 *  GD32A7xx TRM: MCAN ECR offset = 0x040, PSR offset = 0x044
 *  SPL 里 CAN_REG_VAL(canx, offset) 宏通常这样定义：
 *    #define CAN_REG_VAL(canx, offset)  REG32((canx) + (offset))
 *  其中 canx 就是枚举值直接当地址用（SPL常见做法）。
 *
 *  查看你的 gd32a7xx_can.h，找 CAN_REG_VAL 或类似宏，
 *  如果没有就用下面的安全方式：通过 can_error_count_get()。
 * ========================================================================= */

/* SPL 里 can_dtm_canx_enum 的枚举值本身就是寄存器基地址 */
/* 参考 gd32a7xx_can.h：
 *   typedef enum {
 *     DTM_CAN0 = CAN0_BASE,
 *     DTM_CAN1 = CAN1_BASE,
 *     ...
 *   } can_dtm_canx_enum;
 * 所以可以直接把枚举值当地址使用
 */
#define CAN_ECR_OFFSET   0x040U
#define CAN_PSR_OFFSET   0x044U

#define _CAN_ECR(canx)   REG32((uint32_t)(canx) + CAN_ECR_OFFSET)
#define _CAN_PSR(canx)   REG32((uint32_t)(canx) + CAN_PSR_OFFSET)

/* =========================================================================
 *  can_gpio_config()
 * ========================================================================= */
void can_gpio_config(void)
{
    /* 1. CAN 时钟源选择 PCLK2 并使能 */
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

    /* 2. GPIO 时钟 */
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOM);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOE);

    /* 3. 复位 */
    can_deinit(DTM_CAN0);
    can_deinit(DTM_CAN2);
    can_deinit(DTM_CAN4);
    can_deinit(DTM_CAN5);

    /* 4. CAN0  TX=PH10(AF4)  RX=PH12(AF8) */
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_10);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_af_set(GPIOH, GPIO_AF_4, GPIO_PIN_10);

    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_12);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOH, GPIO_AF_8, GPIO_PIN_12);

    /* 5. CAN2  TX=PM2(AF3)  RX=PM3(AF8)  <--- CN4 */
    gpio_output_options_set(GPIOM, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_2);
    gpio_mode_set(GPIOM, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_af_set(GPIOM, GPIO_AF_3, GPIO_PIN_2);

    gpio_output_options_set(GPIOM, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_3);
    gpio_mode_set(GPIOM, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_af_set(GPIOM, GPIO_AF_8, GPIO_PIN_3);

    /* 6. CAN4  TX=PH11(AF1)  RX=PB12(AF8) */
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_11);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_11);
    gpio_af_set(GPIOH, GPIO_AF_1, GPIO_PIN_11);

    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_12);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOB, GPIO_AF_8, GPIO_PIN_12);

    /* 7. CAN5  TX=PE15(AF3)  RX=PE14(AF8) */
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_15);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_15);
    gpio_af_set(GPIOE, GPIO_AF_3, GPIO_PIN_15);

    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_14);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_af_set(GPIOE, GPIO_AF_8, GPIO_PIN_14);
}

/* =========================================================================
 *  can_config()
 *
 *  位时序: tseg1=13  tseg2=6  total_tq=20  SJW=2
 *  波特率 = CAN_CLK / (prescaler * 20)
 *  CAN_CLK 从 rcu_clock_freq_get(CK_APB2) 动态读取
 * ========================================================================= */
void can_config(can_dtm_canx_enum dtm_canx, uint32_t baudrate_khz)
{
    can_parameter_struct        can_param;
    can_filter_struct           can_filter;
    can_rx_fifo_struct          rx_fifo0_cfg;
    can_tx_buffer_struct        tx_buf_cfg;

    const uint32_t tseg1    = 13U;
    const uint32_t tseg2    = 6U;
    const uint32_t total_tq = tseg1 + tseg2 + 1U;  /* = 20 */

    uint32_t can_clk_hz;
    uint32_t prescaler;
    uint32_t actual_baud_khz;

    can_struct_para_init(CAN_INIT_STRUCT,      &can_param);
    can_struct_para_init(CAN_FILTER_STRUCT,    &can_filter);
    can_struct_para_init(CAN_RX_FIFO_STRUCT,   &rx_fifo0_cfg);
    can_struct_para_init(CAN_TX_BUFFER_STRUCT, &tx_buf_cfg);

    /* 动态计算 prescaler */
    can_clk_hz       = rcu_clock_freq_get(CK_APB2);
    prescaler        = (can_clk_hz / 1000U) / baudrate_khz / total_tq;
    if (prescaler == 0U) prescaler = 1U;
    actual_baud_khz  = (can_clk_hz / 1000U) / total_tq / prescaler;

    printf("[CAN_CFG] canx=%d  PCLK2=%u Hz  prescaler=%u  total_tq=%u\r\n",
           (int)dtm_canx, (unsigned)can_clk_hz,
           (unsigned)prescaler, (unsigned)total_tq);
    printf("[CAN_CFG] target=%u kbps  actual=%u kbps  [%s]\r\n",
           (unsigned)baudrate_khz, (unsigned)actual_baud_khz,
           (actual_baud_khz == baudrate_khz) ? "OK" : "MISMATCH !");

    /* CAN 基本参数 */
    can_param.auto_retransmission_enable = ENABLE;
    can_param.transmit_pause_enable      = DISABLE;
    can_param.edge_filter_enable         = DISABLE;
    can_param.protocol_exception_enable  = DISABLE;
    can_param.wide_message_enable        = DISABLE;
    can_param.prescaler                  = prescaler;
    can_param.resync_jump_width          = 2U;
    can_param.time_segment_1             = tseg1;
    can_param.time_segment_2             = tseg2;

    /* 过滤器：全部接收到 FIFO0 */
    can_filter.non_match_std_frame_accept      = CAN_ACCEPT_INTO_RXFIFO0;
    can_filter.non_match_ext_frame_accept      = CAN_ACCEPT_INTO_RXFIFO0;
    can_filter.remote_std_frame_accept         = CAN_REMOTE_FILTER;
    can_filter.remote_ext_frame_accept         = CAN_REMOTE_FILTER;
    can_filter.filter_start_address_std_frame  = 0U;
    can_filter.filter_start_address_ext_frame  = 0U;
    can_filter.filter_number_std_frame         = 1U;
    can_filter.filter_number_ext_frame         = 0U;
    can_filter.and_mask_ext_frame              = 0U;

    /* RX FIFO0 */
    rx_fifo0_cfg.fifo_operation_mode = CAN_RXFIFO_BLOCKING;
    rx_fifo0_cfg.fifo_watermark      = 0U;
    rx_fifo0_cfg.fifo_size           = 8U;
    rx_fifo0_cfg.fifo_start_address  = 0x100U;
    rx_fifo0_cfg.fifo_element_size   = CAN_RXFS_8BYTES;

    /* TX buffer */
    tx_buf_cfg.tx_buffer_start_address = 0x300U;
    tx_buf_cfg.dedicate_buffer_size    = 4U;
    tx_buf_cfg.fifo_or_queue_mode      = CAN_TXFIFO_OPERATION;
    tx_buf_cfg.fifo_or_queue_size      = 4U;
    tx_buf_cfg.tx_buffer_element_size  = CAN_TXBS_8BYTES;

    /* 组装 */
    can_param.filter_config        = &can_filter;
    can_param.rx_fifo0_config      = &rx_fifo0_cfg;
    can_param.rx_fifo1_config      = NULL;
    can_param.rx_buffer_config     = NULL;
    can_param.tx_buffer_config     = &tx_buf_cfg;
    can_param.tx_event_fifo_config = NULL;

    /* 等 SRAM 就绪 */
    while (SET != can_sram_init_state_get(dtm_canx)) {
    }

    can_init(dtm_canx, &can_param);
}

/* =========================================================================
 *  can_print_error_status()
 *  直接读 ECR / PSR 寄存器
 *  can_dtm_canx_enum 枚举值 = 寄存器基地址（GD32A7xx SPL 惯例）
 * ========================================================================= */
void can_print_error_status(can_dtm_canx_enum dtm_canx)
{
    uint32_t ecr_val = _CAN_ECR(dtm_canx);
    uint32_t psr_val = _CAN_PSR(dtm_canx);

    uint32_t tec = (ecr_val >> 16U) & 0xFFU;
    uint32_t rec = (ecr_val >>  0U) & 0x7FU;
    uint32_t lec = (psr_val >>  0U) & 0x7U;
    uint32_t act = (psr_val >>  3U) & 0x3U;
    uint32_t ep  = (psr_val >>  5U) & 0x1U;
    uint32_t ew  = (psr_val >>  6U) & 0x1U;
    uint32_t bo  = (psr_val >>  7U) & 0x1U;

    static const char * const lec_str[] = {
        "NoErr", "Stuff", "Form", "ACK",
        "Bit1",  "Bit0",  "CRC", "NoChange"
    };
    static const char * const act_str[] = {
        "SYNC", "IDLE", "RX", "TX"
    };

    printf("[ERR] TEC=%3u  REC=%3u  LEC=%-8s  ACT=%-4s"
           "  EP=%u EW=%u BO=%u\r\n",
           (unsigned)tec, (unsigned)rec,
           lec_str[lec < 8U ? lec : 7U],
           act_str[act < 4U ? act : 3U],
           (unsigned)ep, (unsigned)ew, (unsigned)bo);

    if (bo) {
        printf("  !! BUS-OFF: TEC 超过 255\r\n"
               "     原因: 1)波特率不匹配  2)PCAN 没有回 ACK\r\n");
    }
    if (ep && !bo) {
        printf("  !! ERROR-PASSIVE: 错误计数 > 127\r\n");
    }
    if (lec == 3U) {
        printf("  !! ACK Error: 发出帧无节点应答\r\n"
               "     检查: PCAN 是否 Listen-only? 波特率是否一致?\r\n");
    }
    if (lec == 1U) {
        printf("  !! Stuff Error: 位填充错误，通常是波特率偏差\r\n");
    }
    if (lec == 2U) {
        printf("  !! Form Error: 帧格式错误，通常是波特率偏差\r\n");
    }
    if (lec == 5U) {
        printf("  !! Bit0 Error: 发 0 采样到 1\r\n");
    }
}

/* =========================================================================
 *  can_send_std_frame()
 * ========================================================================= */
ErrStatus can_send_std_frame(can_dtm_canx_enum dtm_canx,
                              uint32_t std_id,
                              const uint8_t *data, uint8_t len)
{
    can_transmit_message_struct tx_msg;
    uint8_t mailbox;
    uint8_t i;

    if (len > 8U) len = 8U;

    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx_msg);

    tx_msg.id              = std_id;
    tx_msg.xtd             = CAN_FF_STANDARD;
    tx_msg.rtr             = CAN_FT_DATA;
    tx_msg.fdf             = DISABLE;
    tx_msg.brs             = DISABLE;
    tx_msg.ev_fifo_control = CAN_TXEVENT_FIFO_DISABLE;
    tx_msg.message_marker  = 0U;
    tx_msg.esi             = 0U;
    tx_msg.data_bytes      = len;

    for (i = 0U; i < len; i++) {
        tx_msg.data[i] = data[i];
    }

    mailbox = can_message_transmit_prepare(dtm_canx, &tx_msg);
    if (mailbox == 0xFFU) {
        return ERROR;
    }

    can_message_transmit_add(dtm_canx, mailbox);
    return SUCCESS;
}

/* =========================================================================
 *  can_recv_frame_poll()
 * ========================================================================= */
ErrStatus can_recv_frame_poll(can_dtm_canx_enum dtm_canx,
                               can_receive_message_struct *rx_msg)
{
    if (SET != can_flag_get(dtm_canx, CAN_FLAG_RFIFO0_NEW)) {
        return ERROR;
    }

    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, rx_msg);
    can_message_receive(dtm_canx, CAN_RXFIFO0, rx_msg);
    can_fifo_acknowledge(dtm_canx, CAN_FIFO_RX0);

    return SUCCESS;
}

/* =========================================================================
 *  can_flush_rx_fifo0()
 * ========================================================================= */
void can_flush_rx_fifo0(can_dtm_canx_enum dtm_canx)
{
    can_receive_message_struct rx_msg;

    while (SET == can_flag_get(dtm_canx, CAN_FLAG_RFIFO0_NEW)) {
        can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx_msg);
        can_message_receive(dtm_canx, CAN_RXFIFO0, &rx_msg);
        can_fifo_acknowledge(dtm_canx, CAN_FIFO_RX0);
    }
}