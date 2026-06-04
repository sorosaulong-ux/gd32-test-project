/*
*********************************************************************************************************
* UWB RANGING MODULE
*
* 双 UWB 模块 SS-TWR 测距封装组件 (GD32F30x)
*
* 文件名 : uwb_ranging.c
*********************************************************************************************************
*/
#include "uwb_ranging.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"        // <--- 核心寄存器宏定义
#include "delay.h"
#include "usart.h"
#include <stdio.h>

// =========================================================================
// 内部私有常量与射频参数配置 (单片机双驱放宽延时版)
// =========================================================================
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

// 【核心修改区】大幅放宽 UWB2 的反应时间和 UWB1 的等待窗口！
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000  // UWB2 收到 POLL 后，延时 2000us 再发送 RESP
#define POLL_TX_TO_RESP_RX_DLY_UUS 1500  // UWB1 发完 POLL 后，等待 1500us 后再开启接收机
#define RESP_RX_TIMEOUT_UUS        1500  // UWB1 接收机的开机超时时间放大到 1500us

#define SPEED_OF_LIGHT             299702547.0
#define TIMEOUT_MAX_LOOP           200000 // 防死机循环加大

// UWB 默认通信配置 (频道 5, 6.8Mbps, 无 STS)
static dwt_config_t uwb_config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, 
    DWT_PHRRATE_STD, (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

// 发射功率配置
static dwt_txconfig_t tx_power_config = { 0x34, 0xfdfdfdfd, 0x0 };

// 测距暗号报文数组
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_buffer[24];
static uint8_t frame_seq_nb = 0;

// =========================================================================
// 内部时间戳转换静态辅助函数
// =========================================================================
static uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    dwt_readrxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) { ts <<= 8; ts |= ts_tab[i]; }
    return ts;
}

static void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts) {
    for (int i = 0; i < 4; i++) { ts_field[i] = (uint8_t) (ts >> (i * 8)); }
}

static void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts) {
    *ts = 0;
    for (int i = 0; i < 4; i++) { *ts |= ((uint32_t)ts_field[i]) << (i * 8); }
}

// =========================================================================
// 内部私有函数：单节点射频级初始化
// =========================================================================
static uint32_t Static_Init_UWB_Node(uint8_t uwb_index) 
{
    Active_UWB = uwb_index;
    reset_DWIC();                   
    DelayMs(15);                    
    port_set_dw_ic_spi_slowrate();  
    
    uint32_t id = dwt_readdevid();
    if(id == 0xFFFFFFFF || id == 0) return 0; 
    
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) return 0;
    port_set_dw_ic_spi_fastrate(); 
    
    dwt_configure(&uwb_config);
    dwt_configuretxrf(&tx_power_config);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE); 
    
    if(uwb_index == 1) {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    }
    return id;
}

// =========================================================================
// 外部公用接口：UWB 全局系统初始化
// =========================================================================
uint8_t UWB_System_Init(void)
{
    uint32_t id1 = 0, id2 = 0;
    
    UWB_SPI_Init(); 
    
    id1 = Static_Init_UWB_Node(1);
    id2 = Static_Init_UWB_Node(2);
    
    if ((id1 == 0xDECA0302 || id1 == 0xDECA0312) && 
        (id2 == 0xDECA0302 || id2 == 0xDECA0312)) 
    {
        UsartPrintf(USART_DEBUG, "[UWB] Both nodes initialized perfectly!\r\n");
        return 0; // 成功
    }
    
    UsartPrintf(USART_DEBUG, "[UWB] Initialization failed! ID1:0x%08X, ID2:0x%08X\r\n", id1, id2);
    return 1; // 失败
}

// =========================================================================
// 外部公用接口：执行一次完整的 SS-TWR 测距状态机调度
// =========================================================================
uint8_t UWB_Execute_Ranging(double *distance_out)
{
    uint32_t status_reg;
    uint32_t timeout_cnt;

    /* ------ 回合 1：配置 UWB2 进入听球状态 ------ */
    Active_UWB = 2;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* ------ 回合 2：配置 UWB1 发射 POLL 报文，随后原地等回信 ------ */
    Active_UWB = 1;
    tx_poll_msg[2] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); 
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); 
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* ------ 回合 3：切换回 UWB2 确认接球状态 ------ */
    Active_UWB = 2;
    timeout_cnt = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
        if(++timeout_cnt > TIMEOUT_MAX_LOOP) break; 
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        return UWB_ERR_UWB2_TIMEOUT; 
    }

    // UWB2 成功接收 POLL，记录时间戳并打包 RESP 发回
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); 
    uint64_t poll_rx_ts = get_rx_timestamp_u64();
    
    uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * 65536)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);
    
    uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    rx_resp_msg[2] = frame_seq_nb;
    resp_msg_set_ts(&rx_resp_msg[10], poll_rx_ts);
    resp_msg_set_ts(&rx_resp_msg[14], resp_tx_ts);
    
    dwt_writetxdata(sizeof(rx_resp_msg), rx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(rx_resp_msg), 0, 1);
    
    // 【核心防弹衣】检测如果单片机处理太慢错过了发送窗口，直接报错返回
    if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_ERROR) {
        return UWB_ERR_UWB2_TIMEOUT; 
    }

    /* ------ 回合 4：切回 UWB1 拦截被弹回来的 RESP 报文 ------ */
    Active_UWB = 1;
    timeout_cnt = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        if(++timeout_cnt > TIMEOUT_MAX_LOOP) break;
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return UWB_ERR_UWB1_TIMEOUT; 
    }

    // UWB1 成功捕获回信，提取 4 个核心时间戳计算物理距离
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t poll_tx_ts_32 = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts_32 = dwt_readrxtimestamplo32();
    
    uint32_t poll_rx_ts_32, resp_tx_ts_32;
    dwt_readrxdata(rx_buffer, sizeof(rx_resp_msg), 0);
    resp_msg_get_ts(&rx_buffer[10], &poll_rx_ts_32);
    resp_msg_get_ts(&rx_buffer[14], &resp_tx_ts_32);

    int32_t rtd_init = resp_rx_ts_32 - poll_tx_ts_32;
    int32_t rtd_resp = resp_tx_ts_32 - poll_rx_ts_32;
    
    float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);
    double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * (15.65e-12); 
    
    *distance_out = tof * SPEED_OF_LIGHT; // 传出物理距离
    
    frame_seq_nb++;
    return UWB_RANGING_OK; 
}