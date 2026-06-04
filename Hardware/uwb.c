/*!
 *  \file    uwb.c
 *  \brief   DW3000 UWB application — radar + ranging (adapted for GD32A7xx)
 */

#include "uwb.h"
#include "uwb_port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>

/* ====================================================================
 *  Config
 * ====================================================================*/
#define TX_ANT_DLY   16385
#define RX_ANT_DLY   16385
#define SPEED_OF_LIGHT  299702547.0
#define POLL_RX_TO_RESP_TX_DLY_UUS  2000
#define POLL_TX_TO_RESP_RX_DLY_UUS  1500
#define RESP_RX_TIMEOUT_UUS         1500
#define TIMEOUT_MAX_LOOP            200000

/* ====================================================================
 *  UWB config — CH5, 6.8 Mbps, no STS (matching original)
 * ====================================================================*/
static dwt_config_t uwb_config = {
    5,                        /* channel */
    DWT_PLEN_128,             /* preamble length */
    DWT_PAC8,                 /* PAC size */
    9,                        /* TX preamble code */
    9,                        /* RX preamble code */
    1,                        /* SFD type */
    DWT_BR_6M8,               /* data rate */
    DWT_PHRMODE_STD,          /* PHY header mode */
    DWT_PHRRATE_STD,          /* PHY header rate */
    (129 + 8 - 8),            /* SFD timeout */
    DWT_STS_MODE_OFF,         /* STS mode */
    DWT_STS_LEN_64,           /* STS length */
    DWT_PDOA_M0               /* PDOA mode */
};

static dwt_txconfig_t tx_power = { 0x34, 0xfdfdfdfd, 0x0 };

/* ====================================================================
 *  Ranging message buffers
 * ====================================================================*/
static uint8_t tx_poll[12] = { 0x41,0x88,0, 0xCA,0xDE,'W','A','V','E', 0xE0,0,0 };
static uint8_t rx_resp[20] = { 0x41,0x88,0, 0xCA,0xDE,'V','E','W','A', 0xE1,0,0,0,0,0,0,0,0,0,0 };
static uint8_t rx_buf[24];
static uint8_t frame_seq = 0;

/* ====================================================================
 *  Internal helpers
 * ====================================================================*/
static uint64_t rx_ts_u64(void)
{
    uint8_t ts[5]; uint64_t t = 0;
    dwt_readrxtimestamp(ts);
    for (int i = 4; i >= 0; i--) { t <<= 8; t |= ts[i]; }
    return t;
}

static void ts_to_buf(uint8_t *dst, uint64_t ts) {
    for (int i = 0; i < 4; i++) dst[i] = (uint8_t)(ts >> (i * 8));
}

static void ts_from_buf(uint8_t *src, uint32_t *ts) {
    *ts = 0;
    for (int i = 0; i < 4; i++) *ts |= ((uint32_t)src[i]) << (i * 8);
}

/* ---- initialise a single node ---- */
static uint32_t uwb_node_init(uint8_t idx)
{
    Active_UWB = idx;
    reset_DWIC();
    delay_1ms(15);
    port_set_dw_ic_spi_slowrate();

    uint32_t id = dwt_readdevid();
    if (id == 0xFFFFFFFF || id == 0) return 0;
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) return 0;

    port_set_dw_ic_spi_fastrate();
    dwt_configure(&uwb_config);
    dwt_configuretxrf(&tx_power);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    if (idx == 1) {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    }
    return id;
}

/* ====================================================================
 *  Public API
 * ====================================================================*/
int uwb_init(void)
{
    UWB_SPI_Init();
    uint32_t id1 = uwb_node_init(1);
    uint32_t id2 = uwb_node_init(2);

    printf("[UWB] ID1=0x%08X  ID2=0x%08X\r\n", id1, id2);

    if ((id1 == 0xDECA0302 || id1 == 0xDECA0312) &&
        (id2 == 0xDECA0302 || id2 == 0xDECA0312))
        return UWB_OK;
    return UWB_ERR_NO_DEVICE;
}

void uwb_print_ids(void)
{
    uint32_t id1, id2;

    Active_UWB = 1;
    reset_DWIC(); delay_1ms(15);
    port_set_dw_ic_spi_slowrate();
    id1 = dwt_readdevid();
    port_set_dw_ic_spi_fastrate();

    Active_UWB = 2;
    reset_DWIC(); delay_1ms(15);
    port_set_dw_ic_spi_slowrate();
    id2 = dwt_readdevid();
    port_set_dw_ic_spi_fastrate();

    printf("[UWB] ID1=0x%08X  ID2=0x%08X\r\n", id1, id2);
}

/* ====================================================================
 *  SS‑TWR Ranging
 * ====================================================================*/
int uwb_ranging(double *dist_out)
{
    uint32_t status, timeout;

    /* ── Start UWB2 in RX ── */
    Active_UWB = 2;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* ── UWB1 sends POLL ── */
    Active_UWB = 1;
    tx_poll[2] = frame_seq;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll), tx_poll, 0);
    dwt_writetxfctrl(sizeof(tx_poll), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* ── UWB2 waits for POLL ── */
    Active_UWB = 2;
    timeout = 0;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
        if (++timeout > TIMEOUT_MAX_LOOP) return UWB_ERR_TIMEOUT;
    }
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        return UWB_ERR_TIMEOUT;
    }

    /* ── UWB2 sends RESP ── */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    uint64_t poll_rx_ts = rx_ts_u64();
    uint32_t resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * 65536)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    rx_resp[2] = frame_seq;
    ts_to_buf(&rx_resp[10], poll_rx_ts);
    ts_to_buf(&rx_resp[14], resp_tx_ts);

    dwt_writetxdata(sizeof(rx_resp), rx_resp, 0);
    dwt_writetxfctrl(sizeof(rx_resp), 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_ERROR)
        return UWB_ERR_TIMEOUT;

    /* ── UWB1 waits for RESP ── */
    Active_UWB = 1;
    timeout = 0;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        if (++timeout > TIMEOUT_MAX_LOOP) break;
    }
    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return UWB_ERR_TIMEOUT;
    }

    /* ── Compute distance ── */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    uint32_t poll_tx_ts_32  = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts_32  = dwt_readrxtimestamplo32();

    uint32_t poll_rx_ts_32, resp_tx_ts_32;
    dwt_readrxdata(rx_buf, sizeof(rx_resp), 0);
    ts_from_buf(&rx_buf[10], &poll_rx_ts_32);
    ts_from_buf(&rx_buf[14], &resp_tx_ts_32);

    int32_t rtd_init = resp_rx_ts_32 - poll_tx_ts_32;
    int32_t rtd_resp = resp_tx_ts_32 - poll_rx_ts_32;

    float clk_ratio = ((float)dwt_readclockoffset()) / (uint32_t)(1U << 26);
    double tof = ((rtd_init - rtd_resp * (1 - clk_ratio)) / 2.0) * 15.65e-12;
    *dist_out = tof * SPEED_OF_LIGHT;

    frame_seq++;
    return UWB_OK;
}

/* ====================================================================
 *  Radar — bare scan (print CIR)
 * ====================================================================*/
void uwb_radar_scan(void)
{
    uint32_t status, timeout;
    static uint8_t radar_msg[] = { 0x41,0x88,0, 0xCA,0xDE,'R','A','D','A','R',0,0 };
    static uint8_t s_frame = 0;

    /* UWB2 in RX */
    Active_UWB = 2;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* UWB1 TX blind pulse */
    Active_UWB = 1;
    radar_msg[2] = s_frame++;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(radar_msg), radar_msg, 0);
    dwt_writetxfctrl(sizeof(radar_msg), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    /* UWB2 wait for RX */
    Active_UWB = 2;
    timeout = 0;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
        if (++timeout > 200000) break;
    }

    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        dwt_rxdiag_t diag;
        dwt_readdiagnostics(&diag);
        uint16_t fp = diag.ipatovFpIndex >> 6;
        printf("[RADAR] First Path index = %u\r\n", fp);
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        printf("[RADAR] No signal received\r\n");
    }
}
