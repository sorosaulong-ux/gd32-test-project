#include "uwb_tag.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "Delay.h"

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_TX_TO_RESP_RX_DLY_UUS 1500 
#define RESP_RX_TIMEOUT_UUS        1500 
#define SPEED_OF_LIGHT             299702547.0

static dwt_config_t uwb_config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, 
    DWT_PHRRATE_STD, (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};
static dwt_txconfig_t tx_power_config = { 0x34, 0xfdfdfdfd, 0x0 };

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_buffer[24];
static uint8_t frame_seq_nb = 0;

static void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts) {
    *ts = 0;
    for (int i = 0; i < 4; i++) { *ts |= ((uint32_t)ts_field[i]) << (i * 8); }
}

uint8_t UWB_Tag_Init(void)
{
    UWB_Hardware_Init(); // 调用 port.c 的硬件初始化
    
    reset_DWIC();                   
    Delay_ms(15);                    
    port_set_dw_ic_spi_slowrate();  
    
    uint32_t id = dwt_readdevid();
    if(id != 0xDECA0302 && id != 0xDECA0312) return 1; // 初始化失败
    
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) return 1;
    
    port_set_dw_ic_spi_fastrate(); 
    dwt_configure(&uwb_config);
    dwt_configuretxrf(&tx_power_config);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE); 
    
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    
    return 0; // 初始化成功
}

uint8_t UWB_Tag_Execute_Ranging(double *distance_out)
{
    uint32_t status_reg;
    uint32_t timeout_cnt = 0;

    tx_poll_msg[2] = frame_seq_nb++;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); 
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); 
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // 等待对方基站发回来的 RESP 报文
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        if(++timeout_cnt > 200000) return 2; // 超时
    }

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) 
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        uint32_t poll_tx_ts_32 = dwt_readtxtimestamplo32();
        uint32_t resp_rx_ts_32 = dwt_readrxtimestamplo32();
        uint32_t poll_rx_ts_32, resp_tx_ts_32;
        
        dwt_readrxdata(rx_buffer, 20, 0); // 读取 20 字节的回信
        resp_msg_get_ts(&rx_buffer[10], &poll_rx_ts_32);
        resp_msg_get_ts(&rx_buffer[14], &resp_tx_ts_32);

        int32_t rtd_init = resp_rx_ts_32 - poll_tx_ts_32;
        int32_t rtd_resp = resp_tx_ts_32 - poll_rx_ts_32;
        
        float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);
        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * (15.65e-12); 
        
        *distance_out = tof * SPEED_OF_LIGHT;
        return 0; // 测距成功
    }
    
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    return 2; // 接收失败
}