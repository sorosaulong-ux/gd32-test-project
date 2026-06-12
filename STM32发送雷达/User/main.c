/*!
 *  \file    main.c
 *  \brief   STM32F103 — 编译开关切换
 *           MODE_KEY     = DS-TWR Initiator (钥匙测距)
 *           MODE_RADAR_TX = 雷达脉冲发射器 (车内生命检测)
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "port.h"
#include "ble.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include <string.h>
#include <stdio.h>

/* ====================================================================
 *  ★ 编译开关 — 改这里切换模式 ★
 * ====================================================================*/
#define MODE_KEY           /* 钥匙测距 */
//#define MODE_RADAR_TX    /* 雷达发射 */

/* ─── 公共常量 ─── */
#define UUS_TO_DWT_TIME      63898
#define SPEED_OF_LIGHT       299702547.0
#define FRAME_LEN_MAX_EX     1023
#ifndef FCS_LEN
#define FCS_LEN              2
#endif
#define TX_ANT_DLY           16385
#define RX_ANT_DLY           16385

/* ─── 公共无线配置 ─── */
static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

/* ====================================================================
 *  调试串口 (PA9, 115200)
 * ====================================================================*/
static void uinit(void)
{
    GPIO_InitTypeDef g; USART_InitTypeDef u;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    g.GPIO_Mode = GPIO_Mode_AF_PP; g.GPIO_Pin = GPIO_Pin_9;
    g.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOA, &g);
    u.USART_BaudRate = 115200; u.USART_WordLength = USART_WordLength_8b;
    u.USART_StopBits = USART_StopBits_1; u.USART_Parity = USART_Parity_No;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    u.USART_Mode = USART_Mode_Tx; USART_Init(USART1, &u); USART_Cmd(USART1, ENABLE);
}
static void up(const char *s) { while (*s) { while (!(USART1->SR & 0x80)); USART1->DR = *s++; } }

/* ====================================================================
 *  MODE_KEY — DS-TWR Initiator (ex_05a)
 * ====================================================================*/
#ifdef MODE_KEY
#define POLL_TX_TO_RESP_RX_DLY_UUS    400
#define RESP_RX_TIMEOUT_UUS           15000
#define RESP_RX_TO_FINAL_TX_DLY_UUS   5000
#define PRE_TIMEOUT                   0
#define RNG_DELAY_MS                  1000

static uint8_t tx_poll_msg[]  = {0x41,0x88,0,0xCA,0xDE,'W','A','V','E',0x21};
static uint8_t rx_resp_msg[]  = {0x41,0x88,0,0xCA,0xDE,'V','E','W','A',0x10,0x02,0,0};
static uint8_t tx_final_msg[] = {0x41,0x88,0,0xCA,0xDE,'W','A','V','E',0x23,
                                  0,0,0,0, 0,0,0,0, 0,0,0,0};

#define ALL_MSG_COMMON_LEN      10
#define ALL_MSG_SN_IDX          2
#define FINAL_MSG_POLL_TX_IDX   10
#define FINAL_MSG_RESP_RX_IDX   14
#define FINAL_MSG_FINAL_TX_IDX  18
#define FINAL_MSG_TS_LEN        4
#define RX_BUF_LEN              24

static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg;
static uint8_t  frame_seq_nb;
static uint64_t poll_tx_ts, resp_rx_ts, final_tx_ts;

static uint64_t get_tx_timestamp_u64(void) {
    uint8_t t[5]; uint64_t v = 0; dwt_readtxtimestamp(t);
    for (int i = 4; i >= 0; i--) { v <<= 8; v |= t[i]; } return v;
}
static uint64_t get_rx_timestamp_u64(void) {
    uint8_t t[5]; uint64_t v = 0; dwt_readrxtimestamp(t);
    for (int i = 4; i >= 0; i--) { v <<= 8; v |= t[i]; } return v;
}
static void final_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++) { ts_field[i] = (uint8_t)ts; ts >>= 8; }
}

static void key_main(void)
{
    uint32_t id; int ret;

    uinit();
    up("\r\nSTM32 KEY v2.0\r\n");
    OLED_Init();
    OLED_ShowString(1, 1, "UWB: Init...");
    OLED_ShowString(3, 1, "BLE: Init...");

    BLE_Init();
    up("BLE init OK\r\n");

    OLED_ShowString(3, 1, "BLE: Waiting..");
    while (!BLE_IsConnected()) {
        OLED_ShowString(3, 1, "BLE: Waiting..");
        Delay_ms(500);
    }
    OLED_ShowString(3, 1, "BLE: Connected ");

    UWB_Hardware_Init();
    reset_DWIC(); Delay_ms(2);
    while (!dwt_checkidlerc());

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        OLED_ShowString(1, 1, "UWB: INIT ERR "); up("INIT ERR\r\n"); while (1);
    }
    if (dwt_configure(&config)) {
        OLED_ShowString(1, 1, "UWB: CFG ERR  "); up("CFG ERR\r\n"); while (1);
    }

    { dwt_txconfig_t tp = {0x34, 0xfdfdfdfd, 0x0}; dwt_configuretxrf(&tp); }
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    id = dwt_readdevid();
    { char b[16]; snprintf(b,16,"UWB: %08lX",(unsigned long)id); OLED_ShowString(1,1,b); up(b); up("\r\n"); }

    while (1) {
        /* BLE 连接检查 */
        if (!BLE_IsConnected()) {
            OLED_ShowString(3, 1, "BLE: Disconn.  ");
            OLED_ShowString(1, 1, "UWB: Stopped   ");
            dwt_entersleep(0);
            while (!BLE_IsConnected()) { Delay_ms(500); }
            /* 重连后重新初始化UWB (完整复位流程) */
            UWB_Hardware_Init();
            reset_DWIC(); Delay_ms(100);
            while (!dwt_checkidlerc());
            if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
                OLED_ShowString(1, 1, "UWB: RETRY..   ");
                reset_DWIC(); Delay_ms(100);
                while (!dwt_checkidlerc());
                dwt_initialise(DWT_DW_INIT);
            }
            dwt_configure(&config);
            { dwt_txconfig_t tp = {0x34, 0xfdfdfdfd, 0x0}; dwt_configuretxrf(&tp); }
            dwt_setrxantennadelay(RX_ANT_DLY);
            dwt_settxantennadelay(TX_ANT_DLY);
            dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
            dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
            dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
            OLED_ShowString(3, 1, "BLE: Connected ");
            id = dwt_readdevid();
            { char b[16]; snprintf(b,16,"UWB: %08lX",(unsigned long)id); OLED_ShowString(1,1,b); }
        }

        /* Poll */
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* Wait Resp */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {}
        frame_seq_nb++;

        if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
            dwt_write32bitreg(SYS_STATUS_ID,
                SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
            { char dbg[20]; snprintf(dbg,20,"UWB: TO %04lX",(unsigned long)(status_reg & 0xFFFF)); OLED_ShowString(1,1,dbg); }
            up("T"); goto next_key;
        }

        /* Read Resp */
        {
            uint32_t fl = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
            if (fl <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, fl, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) { up("B"); goto next_key; }
        }

        /* Final — NO slow I/O here! */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
        poll_tx_ts = get_tx_timestamp_u64();
        resp_rx_ts = get_rx_timestamp_u64();

        { uint32_t ft = (uint32_t)((resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
          dwt_setdelayedtrxtime(ft);
          final_tx_ts = (((uint64_t)(ft & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY; }

        final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_IDX], poll_tx_ts);
        final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_IDX], resp_rx_ts);
        final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_IDX], final_tx_ts);
        tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
        dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1);

        ret = dwt_starttx(DWT_START_TX_DELAYED);
        if (ret != DWT_SUCCESS) {
            tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
            dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1);
            ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
        }

        OLED_ShowString(2, 1, "UWB: Resp OK  "); up("R");
        if (ret == DWT_SUCCESS) {
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            frame_seq_nb++;
            OLED_ShowString(2, 1, "UWB: Final OK "); up("S\r\n");
        } else {
            OLED_ShowString(2, 1, "UWB: Final FAIL"); up("F\r\n");
        }

next_key:
        Delay_ms(RNG_DELAY_MS);
    }
}
#endif /* MODE_KEY */

/* ====================================================================
 *  MODE_RADAR_TX — 雷达脉冲发射器 (4 Hz, CH5)
 * ====================================================================*/
#ifdef MODE_RADAR_TX
#define RADAR_TX_INTERVAL_MS  250   /* 4 Hz */

static uint8_t tx_radar[] = {0x41,0x88,0,0xCA,0xDE,'R','A','D','A','R',0,0};

static void radar_tx_main(void)
{
    uint32_t id, cnt = 0;

    uinit();
    up("\r\nSTM32 RADAR TX v1.0\r\n");
    OLED_Init();
    OLED_ShowString(1, 1, "UWB: Init...");
    OLED_ShowString(3, 1, "BLE: Init...");

    BLE_Init();
    up("BLE init OK\r\n");

    OLED_ShowString(3, 1, "BLE: Waiting..");
    while (!BLE_IsConnected()) {
        OLED_ShowString(3, 1, "BLE: Waiting..");
        Delay_ms(500);
    }
    OLED_ShowString(3, 1, "BLE: Connected ");

    UWB_Hardware_Init();
    reset_DWIC(); Delay_ms(2);
    while (!dwt_checkidlerc());

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        OLED_ShowString(1, 1, "UWB: INIT ERR "); while (1);
    }
    if (dwt_configure(&config)) {
        OLED_ShowString(1, 1, "UWB: CFG ERR  "); while (1);
    }

    { dwt_txconfig_t tp = {0x34, 0xfdfdfdfd, 0x0}; dwt_configuretxrf(&tp); }
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_PA_ENABLE);

    id = dwt_readdevid();
    { char b[16]; snprintf(b,16,"UWB: %08lX",(unsigned long)id); OLED_ShowString(1,1,b); up(b); up("\r\n"); }

    OLED_ShowString(2, 1, "UWB: Pulsing  ");
    up("RADAR TX running\r\n");

    while (1) {
        /* BLE 连接检查 */
        if (!BLE_IsConnected()) {
            OLED_ShowString(3, 1, "BLE: Disconn.  ");
            OLED_ShowString(1, 1, "UWB: Stopped   ");
            dwt_entersleep(0);
            while (!BLE_IsConnected()) { Delay_ms(500); }
            /* 重连后重新初始化UWB (完整复位流程) */
            UWB_Hardware_Init();
            reset_DWIC(); Delay_ms(100);
            while (!dwt_checkidlerc());
            if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
                OLED_ShowString(1, 1, "UWB: RETRY..   ");
                reset_DWIC(); Delay_ms(100);
                while (!dwt_checkidlerc());
                dwt_initialise(DWT_DW_INIT);
            }
            dwt_configure(&config);
            { dwt_txconfig_t tp = {0x34, 0xfdfdfdfd, 0x0}; dwt_configuretxrf(&tp); }
            dwt_settxantennadelay(TX_ANT_DLY);
            dwt_setlnapamode(DWT_PA_ENABLE);
            OLED_ShowString(3, 1, "BLE: Connected ");
            id = dwt_readdevid();
            { char b[16]; snprintf(b,16,"UWB: %08lX",(unsigned long)id); OLED_ShowString(1,1,b); }
            OLED_ShowString(2, 1, "UWB: Pulsing  ");
        }

        tx_radar[2] = (uint8_t)(cnt & 0xFF);
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(tx_radar), tx_radar, 0);
        dwt_writetxfctrl(sizeof(tx_radar) + FCS_LEN, 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

        cnt++;
        { char b[16]; snprintf(b,16,"UWB: Pulse %lu",(unsigned long)cnt); OLED_ShowString(2,1,b); }
        Delay_ms(RADAR_TX_INTERVAL_MS);
    }
}
#endif /* MODE_RADAR_TX */

/* ====================================================================
 *  main
 * ====================================================================*/
int main(void)
{
#if defined(MODE_KEY)
    key_main();
#elif defined(MODE_RADAR_TX)
    radar_tx_main();
#else
#error "Please define MODE_KEY or MODE_RADAR_TX"
#endif
}
