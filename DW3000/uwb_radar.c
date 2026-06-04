#include "uwb_radar.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "delay.h"
#include "usart.h"
#include "can_drive.h"
#include <stdio.h>

// =========================================================================
// 雷达参数与 CIR 截取窗口配置
// =========================================================================
#define RADAR_TAP_COUNT   30   // 截取 30 个复数采样点 (约覆盖车内 4.5 米反射空间)
// DW3000 的每个点是 24-bit实部 + 24-bit虚部 = 6 字节。最前面有 1 个 dummy 字节
#define ACCUM_DATA_LEN    ((RADAR_TAP_COUNT * 6) + 1)  // 181 字节

static uint8_t accum_data[ACCUM_DATA_LEN]; // 存放原始雷达数据的数组
static dwt_rxdiag_t rx_diag;               // 诊断结构体 (存放第一径索引)

// UWB 通信配置 (必须保证两边配置完全一致)
static dwt_config_t uwb_config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, 
    DWT_PHRRATE_STD, (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};
static dwt_txconfig_t tx_power_config = { 0x34, 0xfdfdfdfd, 0x0 };

// 雷达盲发数据包 (不需要任何时间戳)
static uint8_t tx_blind_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'A', 'D', 'A', 'R', 0, 0};
static uint8_t frame_seq_nb = 0;

// =========================================================================
// 私有函数：初始化单节点
// =========================================================================
static uint32_t Init_UWB_Node(uint8_t uwb_index) 
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
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE); 
    
    // 开启诊断功能 (必须开，否则读不到 CIR 寄存器)
    dwt_configciadiag(1);
    
    return id;
}

// =========================================================================
// 接口：雷达系统初始化
// =========================================================================
uint8_t UWB_Radar_Init(void)
{
    UWB_SPI_Init(); 
    uint32_t id1 = Init_UWB_Node(1);
    uint32_t id2 = Init_UWB_Node(2);
    
    if ((id1 == 0xDECA0302 || id1 == 0xDECA0312) && 
        (id2 == 0xDECA0302 || id2 == 0xDECA0312)) 
    {
        UsartPrintf(USART_DEBUG, "[RADAR] UWB1(TX) & UWB2(RX) Ready!\r\n");
        return 0;
    }
    UsartPrintf(USART_DEBUG, "[RADAR] UWB Init Failed! ID1:%X, ID2:%X\r\n", id1, id2);
    return 1; 
}

// =========================================================================
// 接口：发射雷达波、接收、抓取 CIR 并打入 CAN 总线
// =========================================================================
void UWB_Radar_Scan_And_Send(void)
{
    uint32_t status_reg;
    uint32_t timeout_cnt;

    /* 1. UWB2 (雷达接收机) 进入无限制倾听模式 */
    Active_UWB = 2;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* 2. UWB1 (雷达发射机) 发射探测脉冲 */
    Active_UWB = 1;
    tx_blind_msg[2] = frame_seq_nb++;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_blind_msg), tx_blind_msg, 0); 
    dwt_writetxfctrl(sizeof(tx_blind_msg), 0, 1); 
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    /* 3. 切回 UWB2，等待回波阵面到达 */
    Active_UWB = 2;
    timeout_cnt = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
        if(++timeout_cnt > 200000) break; 
    }

    /* 4. 如果成功接收，开始极速提取 CIR 数据 */
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) 
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        
        // 读取芯片内部诊断数据
        dwt_readdiagnostics(&rx_diag);
        
        // 获取第一径 (First Path) 的索引，右移 6 位得到整数部分
        uint16_t fp_int = rx_diag.ipatovFpIndex >> 6;
        
        // 核心：读取包含第一径在内的 30 个 CIR 复数采样点 (稍微往前偏移 4 个点抓取前导底噪)
        dwt_readaccdata(accum_data, ACCUM_DATA_LEN, (fp_int - 4));
        
        // =======================================================
        // 通过 1Mbps 高速 CAN 将 180 字节 CIR 数据传给电脑
        // =======================================================
        uint8_t can_buf[8];
        
        // 帧0：报头 (发送 ID: 0x100)，告诉电脑这一帧的基本信息
        can_buf[0] = 0xAA;                   // 雷达数据包起始帧头
        can_buf[1] = (fp_int >> 8) & 0xFF;   // 第一径索引高 8 位
        can_buf[2] = fp_int & 0xFF;          // 第一径索引低 8 位
        can_buf[3] = RADAR_TAP_COUNT;        // 发送的点数 (30)
        can_buf[4] = 0; can_buf[5] = 0; can_buf[6] = 0; can_buf[7] = 0;
        CAN_Send_Msg(0x100, can_buf, 8);
        
        // 帧1~23：纯雷达回波有效数据 payload (发送 ID: 0x200 + 序号)
        uint16_t payload_len = ACCUM_DATA_LEN - 1; // 丢掉第 0 个无用的 dummy byte
        uint8_t frame_index = 0;
        
        for(uint16_t offset = 1; offset < ACCUM_DATA_LEN; offset += 8) 
        {
            uint8_t bytes_to_send = 8;
            if(offset + 8 > ACCUM_DATA_LEN) bytes_to_send = ACCUM_DATA_LEN - offset; // 最后一帧可能不足 8 字节
            
            // CAN ID 累加 (0x200, 0x201, 0x202...)
            CAN_Send_Msg(0x200 + frame_index, &accum_data[offset], bytes_to_send);
            frame_index++;
        }
    } 
    else 
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    }
}