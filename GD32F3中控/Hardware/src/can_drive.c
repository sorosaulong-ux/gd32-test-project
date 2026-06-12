#include "can_drive.h"

/*
************************************************************
*	CAN_Init_Config
*	初始化 CAN0, PB8(RX) PB9(TX) partial remap
*	500Kbps
************************************************************
*/
void CAN_Init_Config(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;

    /* 1. 使能时钟 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_CAN0);

    /* 2. 引脚重映射 PB8/PB9 */
    gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);

    /* 3. 配置引脚: PB8(RX), PB9(TX) */
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* 4. CAN 参数初始化 */
    can_deinit(CAN0);
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);

    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    
    /* 5. 波特率 500Kbps: SJW=1 + BS1=14 + BS2=5 = 20 TQ */
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_14TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_5TQ;
    can_parameter.prescaler = 6;

    can_init(CAN0, &can_parameter);

    /* 6. CAN 过滤器: 接收所有帧到 FIFO0 */
    can_filter.filter_number = 0;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);
}

/*
************************************************************
*	CAN_Send_Msg
*	发送标准CAN帧 (最多 8 字节)
* 返回值:   0: 发送成功  1: 发送超时/失败
************************************************************
*/
uint8_t CAN_Send_Msg(uint32_t id, uint8_t *msg, uint8_t len)
{
    can_transmit_message_struct tx_message;
    uint32_t timeout = 0x0FFFFF; 
    
    if(len > 8) len = 8;
    
    tx_message.tx_sfid = id;               
    tx_message.tx_efid = 0x00;
    tx_message.tx_ff = CAN_FF_STANDARD;    
    tx_message.tx_ft = CAN_FT_DATA;        
    tx_message.tx_dlen = len;              
    
    for(uint8_t i = 0; i < len; i++) {
        tx_message.tx_data[i] = msg[i];
    }
    
    uint8_t mailbox = can_message_transmit(CAN0, &tx_message);
    
    while((can_transmit_states(CAN0, mailbox) == CAN_TRANSMIT_PENDING) && (timeout != 0))
    {
        timeout--;
    }
    
    if(can_transmit_states(CAN0, mailbox) == CAN_TRANSMIT_OK)
    {
        return 0;
    }
    
    return 1;
}

/*
************************************************************
*	CAN_Receive_Msg
*	查询接收 CAN 数据
* 返回值:   0: 收到数据  1: 没有数据
************************************************************
*/
uint8_t CAN_Receive_Msg(uint32_t *id, uint8_t *buf)
{
    can_receive_message_struct rx_message;
    
    if(can_receive_message_length_get(CAN0, CAN_FIFO0) == 0) {
        return 1; 
    }
    
    can_message_receive(CAN0, CAN_FIFO0, &rx_message);
    
    *id = rx_message.rx_sfid;
    for(uint8_t i = 0; i < rx_message.rx_dlen; i++) {
        buf[i] = rx_message.rx_data[i];
    }
    return 0; 
}
