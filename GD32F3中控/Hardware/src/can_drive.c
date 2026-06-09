#include "can_drive.h"

/*
************************************************************
*	函数名称：	CAN_Init_Config
*	说明：		初始化 CAN0，引脚 PA11(RX) 和 PA12(TX)
* 波特率 500Kbps (基于 APB1 = 36MHz 计算)
************************************************************
*/
void CAN_Init_Config(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;

    /* 1. 开启时钟：开启 GPIOB、AF复用时钟 和 CAN0 时钟 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_CAN0);

    /* 2. 【核心绝杀】开启 CAN0 的引脚部分重映射，挪到 PB8 和 PB9 上！ */
    gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);

    /* 3. 配置引脚: PB8(RX, 上拉输入), PB9(TX, 复用推挽输出) */
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* 4. CAN 基础参数初始化 */
    can_deinit(CAN0);
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);

    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE; // 保持死磕重传开启
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    
    /* 5. 锁死已证实的 60MHz 匹配 500Kbps 参数 */
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_14TQ;  // 14TQ
    can_parameter.time_segment_2 = CAN_BT_BS2_5TQ;   // 5TQ
    can_parameter.prescaler = 6;                     // 6分频

    can_init(CAN0, &can_parameter);

    /* 6. CAN 过滤器配置 (接收所有数据) */
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
*	函数名称：	CAN_Send_Msg
*	说明：		发送标准数据帧 (最大 8 字节)
* 返回值：    0: 发送成功(收到对方ACK)  1: 发送超时或断线
************************************************************
*/
uint8_t CAN_Send_Msg(uint32_t id, uint8_t *msg, uint8_t len)
{
    can_transmit_message_struct tx_message;
    // 稍微放大一点超时时间，给自动重传留一点点余地
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
    
    // 把报文塞进发送邮箱
    uint8_t mailbox = can_message_transmit(CAN0, &tx_message);
    
    // 阻塞等待：只要还在发送中，就一直等，直到成功、失败或超时
    while((can_transmit_states(CAN0, mailbox) == CAN_TRANSMIT_PENDING) && (timeout != 0))
    {
        timeout--;
    }
    
    // 【终极修复】必须明确判断最终状态是不是 OK！
    if(can_transmit_states(CAN0, mailbox) == CAN_TRANSMIT_OK)
    {
        return 0; // 真成功：发出去且收到了 ACK
    }
    
    // 如果走到这里，说明超时退出了，或者状态是 FAILED
    return 1; // 真失败：没接线、波特率错、或设备不在线
}

/*
************************************************************
*	函数名称：	CAN_Receive_Msg
*	说明：		轮询接收 CAN 数据
* 返回值：    0: 收到数据  1: 没收到数据
************************************************************
*/
uint8_t CAN_Receive_Msg(uint32_t *id, uint8_t *buf)
{
    can_receive_message_struct rx_message;
    
    // 检查 FIFO0 中是否有报文
    if(can_receive_message_length_get(CAN0, CAN_FIFO0) == 0) {
        return 1; 
    }
    
    // 提取报文
    can_message_receive(CAN0, CAN_FIFO0, &rx_message);
    
    *id = rx_message.rx_sfid;
    for(uint8_t i = 0; i < rx_message.rx_dlen; i++) {
        buf[i] = rx_message.rx_data[i];
    }
    return 0; 
}