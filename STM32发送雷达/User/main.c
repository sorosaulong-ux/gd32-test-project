/*!
 *  \file    main.c
 *  \brief   STM32F103 DS-TWR Initiator — EXACT Decawave official ex_05a pattern
 *
 *  STM32 = Initiator (sends Poll → receives Response → sends Final with timestamps)
 *  GD32A7 = Responder (computes & displays distance)
 *
 *  Message flow:
 *   1. Send Poll with DWT_RESPONSE_EXPECTED
 *   2. Wait for Response (RXFCG) or timeout
 *   3. Send Final with poll_tx_ts, resp_rx_ts, final_tx_ts (DWT_START_TX_DELAYED)
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include <string.h>
#include <stdio.h>

/* ─── Official Decawave constants (from shared_defines.h) ─── */
#define UUS_TO_DWT_TIME      63898
#define SPEED_OF_LIGHT       299702547.0
#define FRAME_LEN_MAX_EX     1023
#ifndef FCS_LEN
#define FCS_LEN              2
#endif

/* ─── Communication config (matching GD32A7 Responder) ─── */
static dwt_config_t config = {
    5,               /* Channel 5 */
    DWT_PLEN_128,    /* Preamble length */
    DWT_PAC8,        /* PAC size */
    9,               /* TX preamble code */
    9,               /* RX preamble code */
    1,               /* SFD type: non-standard 8 symbol */
    DWT_BR_6M8,      /* Data rate 6.8 Mbps */
    DWT_PHRMODE_STD, /* PHY header mode */
    DWT_PHRRATE_STD, /* PHY header rate */
    (129 + 8 - 8),   /* SFD timeout */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

/* ─── Antenna delay (64 MHz PRF) ─── */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* ─── Timing — coordinated with GD32A7 software SPI ───
 *  GD32A7 Responder: ~8000µs from Poll RX to Response TX
 *  STM32 Initiator must stay open for Response and have time for Final prep
 *
 *  Timeline (from STM32 Poll RMARKER, t=0):
 *    t=+190µs:  Poll TX ends
 *    t=+590µs:  STM32 RX enables (end + POLL_TX_TO_RESP_RX_DLY_UUS)
 *    t=~8000µs: GD32A7 Response arrives at STM32
 *    t=~13000µs: STM32 sends Final (RESP_RX_TO_FINAL_TX_DLY_UUS=5000 from Resp RX)
 */
#define POLL_TX_TO_RESP_RX_DLY_UUS    400
#define RESP_RX_TIMEOUT_UUS           15000
#define RESP_RX_TO_FINAL_TX_DLY_UUS   5000  /* ★ was 1500 — was too tight */
#define PRE_TIMEOUT                   0
#define RNG_DELAY_MS                  1000

/* ─── Frames — exact official ex_05a format ───
 *  sizeof returns data byte count. +FCS_LEN = full frame with CRC.
 *  Poll:  10B data + 2B CRC = 12B  frame
 *  Resp:  13B data + 2B CRC = 15B  frame
 *  Final: 22B data + 2B CRC = 24B  frame */
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
static uint32_t status_reg = 0;
static uint8_t  frame_seq_nb = 0;

/* ─── Timestamps (40-bit in 64-bit storage) ─── */
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

/* ====================================================================
 *  Helper functions (from official shared_functions.c)
 * ====================================================================*/

/* Get TX timestamp as 64-bit (5-byte register → 40-bit value) */
static uint64_t get_tx_timestamp_u64(void)
{
    uint8_t t[5]; uint64_t v = 0;
    dwt_readtxtimestamp(t);
    for (int i = 4; i >= 0; i--) { v <<= 8; v |= t[i]; }
    return v;
}

/* Get RX timestamp as 64-bit */
static uint64_t get_rx_timestamp_u64(void)
{
    uint8_t t[5]; uint64_t v = 0;
    dwt_readrxtimestamp(t);
    for (int i = 4; i >= 0; i--) { v <<= 8; v |= t[i]; }
    return v;
}

/* Fill 4-byte timestamp field in Final message (LSB first) */
static void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8_t)ts;
        ts >>= 8;
    }
}

/* ====================================================================
 *  Debug UART (PA9, 115200)
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
 *  main — DS-TWR Initiator (ex_05a pattern)
 * ====================================================================*/
int main(void)
{
    uint32_t id;
    int      ret;

    /* Init debug UART & OLED */
    uinit();
    up("\r\nSTM32 DS-TWR Init v1.0 (ex_05a)\r\n");
    OLED_Init();
    OLED_ShowString(1, 1, "DS-TWR Init v1.0");

    /* ── DW3000 init (exact official sequence) ── */
    UWB_Hardware_Init();
    reset_DWIC();
    Delay_ms(2);

    while (!dwt_checkidlerc());

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        OLED_ShowString(2, 1, "INIT ERR");
        up("INIT ERR\r\n");
        while (1);
    }

    if (dwt_configure(&config)) {
        OLED_ShowString(2, 1, "CFG ERR");
        up("CFG ERR\r\n");
        while (1);
    }

    /* TX power config */
    {
        dwt_txconfig_t tp = {0x34, 0xfdfdfdfd, 0x0};
        dwt_configuretxrf(&tp);
    }

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set response delay and timeout (once — same for all exchanges) */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    /* NOTE: no dwt_setpreambledetecttimeout here — we set it per-exchange */

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    id = dwt_readdevid();
    {
        char buf[20];
        snprintf(buf, 20, "ID: %08lX", (unsigned long)id);
        OLED_ShowString(2, 1, buf);
        up(buf); up("\r\n");
    }

    /* ================================================================
     *  DS-TWR Initiator main loop (ex_05a official pattern)
     * ================================================================*/
    while (1)
    {
        /* ── Step 1: Send Poll ── */
        dwt_setpreambledetecttimeout(PRE_TIMEOUT); /* unlimited = 0: wait forever */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);

        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* ── Step 2: Wait for Response ── */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { }

        frame_seq_nb++;

        if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
        {
            /* Timeout or error — clear and retry */
            dwt_write32bitreg(SYS_STATUS_ID,
                SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);

            {
                char dbg[22];
                snprintf(dbg, 22, "Resp: TO S=%08lX", (unsigned long)status_reg);
                OLED_ShowString(3, 1, dbg);
            }
            up("T"); /* debug: timeout */
            goto next_ranging;
        }

        /* ── Got Response frame! ── */
        {
            uint32_t frame_len;

            /* Clear RXFCG + TXFRS */
            dwt_write32bitreg(SYS_STATUS_ID,
                SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

            /* Read Response */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Validate: clear seq#, compare header */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0)
            {
                OLED_ShowString(3, 1, "Resp: BAD");
                up("B"); /* debug: bad frame */
                goto next_ranging;
            }

            /* ── Step 3: Send Final with timestamps ──
             * CRITICAL: NO slow I2C/UART calls here — they eat the delayed TX window!
             * All printf/OLED AFTER dwt_starttx completes. */

            /* Get timestamps */
            poll_tx_ts = get_tx_timestamp_u64();
            resp_rx_ts = get_rx_timestamp_u64();

            /* Compute delayed TX time */
            {
                uint32_t final_tx_time;
                final_tx_time = (uint32_t)((resp_rx_ts +
                    (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
                dwt_setdelayedtrxtime(final_tx_time);
                final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
            }

            /* Fill Final message & send */
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_IDX],  poll_tx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_IDX],  resp_rx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_IDX], final_tx_ts);
            tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
            dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1);

            ret = dwt_starttx(DWT_START_TX_DELAYED);

            /* Fallback: if delayed window missed, try immediate */
            if (ret != DWT_SUCCESS) {
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1);
                ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
            }

            /* NOW safe for slow I/O */
            OLED_ShowString(3, 1, "Resp: OK");
            up("R");

            if (ret == DWT_SUCCESS)
            {
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                frame_seq_nb++;
                OLED_ShowString(4, 1, "Final: OK");
                up("S\r\n");
            }
            else
            {
                OLED_ShowString(4, 1, "Final: FAIL");
                up("F\r\n");
            }
        }

next_ranging:
        /* Delay between ranging exchanges */
        Delay_ms(RNG_DELAY_MS);
    }
}
