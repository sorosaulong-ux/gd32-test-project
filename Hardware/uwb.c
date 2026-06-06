/*!
 *  \file    uwb.c
 *  \brief   DW3000 UWB — DS-TWR Responder (single module, UWB2)
 *
 *  Strictly follows official Decawave ex_05b_ds_twr_responder.c pattern.
 *  GD32A7 = Responder (computes distance), STM32F103 = Initiator.
 */

#include "uwb.h"
#include "uwb_port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>

/* ====================================================================
 *  Shared constants
 * ====================================================================*/
#define TX_ANT_DLY          16385
#define RX_ANT_DLY          16385
#define SPEED_OF_LIGHT      299702547.0
#define UUS_TO_DWT_TIME     63898      /* 1 uus → dtu */
#define RXFL_MASK_1023      0x000003FFUL

/* ====================================================================
 *  Radio config (shared — CH5, 6.8M, PLEN_128)
 * ====================================================================*/
static dwt_config_t uwb_config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

/* ====================================================================
 *  print device ID (module 2 only)
 * ====================================================================*/
void uwb_print_id(void)
{
    Active_UWB = 2;
    reset_DWIC();
    delay_1ms(15);
    port_set_dw_ic_spi_slowrate();
    uint32_t id = dwt_readdevid();
    port_set_dw_ic_spi_fastrate();
    printf("[UWB] ID2=0x%08lX\r\n", (unsigned long)id);
}

/* ====================================================================
 *  ★★★  OFFICIAL DECAWAVE DS-TWR RESPONDER (ex_05b) ★★★
 *
 *  Message flow:
 *   Initiator (STM32)           Responder (GD32A7)
 *     |Poll TX| ────────────────→ |Poll RX|    (record poll_rx_ts)
 *     |Resp RX| ←──────────────── |Resp TX|    (record resp_tx_ts)
 *     |Final TX| ───────────────→ |Final RX|   (record final_rx_ts)
 *
 *  Distance computed by Responder from all 6 timestamps.
 * ====================================================================*/

/* ─── Timing — generous for GD32A7 software SPI ─── */
#define DS_POLL_RX_TO_RESP_TX_DLY_UUS   8000
#define DS_RESP_TX_TO_FINAL_RX_DLY_UUS  2000  /* was 500 - Final arrives ~13ms after Poll */
#define DS_FINAL_RX_TIMEOUT_UUS         5000  /* was 2000 - need longer window */
#define DS_PRE_TIMEOUT                  0

/* ─── Frame definitions (official ex_05b format) ─── */
static uint8_t ds_rx_poll[]  = {0x41,0x88,0,0xCA,0xDE,'W','A','V','E',0x21,0,0};
static uint8_t ds_tx_resp[]  = {0x41,0x88,0,0xCA,0xDE,'V','E','W','A',0x10,0x02,0,0,0,0};
static uint8_t ds_rx_final[] = {0x41,0x88,0,0xCA,0xDE,'W','A','V','E',0x23,
                                 0,0,0,0, 0,0,0,0, 0,0,0,0};

#define DS_RX_BUF_LEN             24
#define DS_ALL_MSG_COMMON_LEN     10
#define DS_ALL_MSG_SN_IDX         2
#define DS_FINAL_MSG_POLL_TX_IDX  10
#define DS_FINAL_MSG_RESP_RX_IDX  14
#define DS_FINAL_MSG_FINAL_TX_IDX 18
#define DS_FINAL_MSG_TS_LEN       4

static uint8_t  ds_rx_buf[DS_RX_BUF_LEN];
static uint8_t  ds_fsn;
static uint32_t ds_status_reg;

static uint64_t ds_poll_rx_ts;
static uint64_t ds_resp_tx_ts;
static uint64_t ds_final_rx_ts;

/* ─── Timestamp helpers (from official shared_functions.c) ─── */
static uint64_t ds_get_rx_timestamp_u64(void) {
    uint8_t t[5]; uint64_t v = 0; dwt_readrxtimestamp(t);
    for (int i = 4; i >= 0; i--) { v <<= 8; v |= t[i]; } return v;
}
static uint64_t ds_get_tx_timestamp_u64(void) {
    uint8_t t[5]; uint64_t v = 0; dwt_readtxtimestamp(t);
    for (int i = 4; i >= 0; i--) { v <<= 8; v |= t[i]; } return v;
}
static void ds_final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
    *ts = 0;
    for (int i = 0; i < DS_FINAL_MSG_TS_LEN; i++)
        *ts += ((uint32_t)ts_field[i]) << (i * 8);
}

/* ====================================================================
 *  uwb_ds_responder_init
 * ====================================================================*/
int uwb_ds_responder_init(void)
{
    uint32_t id;

    UWB_SPI_Init();
    Active_UWB = 2;

    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2);

    while (!dwt_checkidlerc());

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("[DS-RESP] dwt_initialise FAILED\r\n");
        return UWB_ERR_NO_DEVICE;
    }
    if (dwt_configure(&uwb_config)) {
        printf("[DS-RESP] dwt_configure FAILED\r\n");
        return UWB_ERR_NO_DEVICE;
    }

    {
        dwt_txconfig_t tp = { 0x34, 0xfdfdfdfd, 0x0 };
        dwt_configuretxrf(&tp);
    }

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    id = dwt_readdevid();
    printf("[DS-RESP] Responder ready, id=0x%08lX\r\n", (unsigned long)id);
    return UWB_OK;
}

/* ====================================================================
 *  uwb_ds_responder_poll — one DS-TWR exchange
 *  Returns 1 and fills *dist on success.
 * ====================================================================*/
int uwb_ds_responder_poll(double *dist)
{
    uint32_t frame_len;
    int ret;

    /* ── Phase 1: Wait for Poll ── */
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (!((ds_status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { }

    if (!(ds_status_reg & SYS_STATUS_RXFCG_BIT_MASK))
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return 0;
    }

    /* Clear RXFCG */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* Read frame */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFL_MASK_1023;
    if (frame_len <= DS_RX_BUF_LEN)
        dwt_readrxdata(ds_rx_buf, frame_len, 0);

    /* Validate Poll */
    ds_rx_buf[DS_ALL_MSG_SN_IDX] = 0;
    if (memcmp(ds_rx_buf, ds_rx_poll, DS_ALL_MSG_COMMON_LEN) != 0)
        return 0;

    /* ── Phase 2: Send Response (NO printf until after starttx!) ── */

    ds_poll_rx_ts = ds_get_rx_timestamp_u64();

    {
        uint32_t resp_tx_time;
        resp_tx_time = (uint32_t)((ds_poll_rx_ts +
            (DS_POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
        dwt_setdelayedtrxtime(resp_tx_time);
    }

    dwt_setrxaftertxdelay(DS_RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(DS_FINAL_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(DS_PRE_TIMEOUT);

    ds_tx_resp[DS_ALL_MSG_SN_IDX] = ds_fsn;
    dwt_writetxdata(sizeof(ds_tx_resp), ds_tx_resp, 0);
    dwt_writetxfctrl(sizeof(ds_tx_resp), 0, 1);
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    /* Fallback: if delayed TX missed the window, try immediate */
    if (ret == DWT_ERROR)
    {
        dwt_setrxaftertxdelay(DS_RESP_TX_TO_FINAL_RX_DLY_UUS);
        dwt_setrxtimeout(DS_FINAL_RX_TIMEOUT_UUS);
        dwt_setpreambledetecttimeout(DS_PRE_TIMEOUT);
        ds_tx_resp[DS_ALL_MSG_SN_IDX] = ds_fsn;
        dwt_writetxdata(sizeof(ds_tx_resp), ds_tx_resp, 0);
        dwt_writetxfctrl(sizeof(ds_tx_resp), 0, 1);
        ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    }

    if (ret == DWT_ERROR)
    {
        printf("[DS-RESP] starttx RESP FAIL\r\n");
        return 0;
    }

    /* ── Phase 3: Wait for Final ── */

    while (!((ds_status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { }

    ds_fsn++;

    if (!(ds_status_reg & SYS_STATUS_RXFCG_BIT_MASK))
    {
        printf("[DS-RESP] FINAL timeout S=0x%08lX\r\n", (unsigned long)ds_status_reg);
        dwt_write32bitreg(SYS_STATUS_ID,
            SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
        return 0;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFL_MASK_1023;
    if (frame_len <= DS_RX_BUF_LEN)
        dwt_readrxdata(ds_rx_buf, frame_len, 0);

    /* Validate Final */
    ds_rx_buf[DS_ALL_MSG_SN_IDX] = 0;
    if (memcmp(ds_rx_buf, ds_rx_final, DS_ALL_MSG_COMMON_LEN) != 0)
    {
        printf("[DS-RESP] NOT FINAL: %02X %02X func=%02X\r\n",
               ds_rx_buf[0], ds_rx_buf[1], ds_rx_buf[9]);
        return 0;
    }

    /* ── Compute DS-TWR distance ── */

    ds_resp_tx_ts  = ds_get_tx_timestamp_u64();
    ds_final_rx_ts = ds_get_rx_timestamp_u64();

    {
        uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
        double Ra, Rb, Da, Db;
        int64_t tof_dtu;

        ds_final_msg_get_ts(&ds_rx_buf[DS_FINAL_MSG_POLL_TX_IDX],  &poll_tx_ts);
        ds_final_msg_get_ts(&ds_rx_buf[DS_FINAL_MSG_RESP_RX_IDX],  &resp_rx_ts);
        ds_final_msg_get_ts(&ds_rx_buf[DS_FINAL_MSG_FINAL_TX_IDX], &final_tx_ts);

        /* DS-TWR asymmetric double-sided formula (official) */
        Ra = (double)(resp_rx_ts - poll_tx_ts);
        Rb = (double)((uint32_t)ds_final_rx_ts - (uint32_t)ds_resp_tx_ts);
        Da = (double)(final_tx_ts - resp_rx_ts);
        Db = (double)((uint32_t)ds_resp_tx_ts - (uint32_t)ds_poll_rx_ts);
        tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

        double tof = (double)tof_dtu * DWT_TIME_UNITS;
        *dist = tof * SPEED_OF_LIGHT;

        printf("[DS-RESP] FINAL rx, dist=%.3f m\r\n", *dist);
    }

    return 1;
}

/* ====================================================================
 *  Radar RX — bistatic 车内生命检测 (接收 STM32 的雷达脉冲)
 *
 *  STM32 = TX (连续发脉冲 @ 4Hz)
 *  GD32A7 = RX (收脉冲 → 提取 CIR I/Q → FFT 呼吸检测)
 *
 *  CIR 配置: 128 bins, raw I/Q (24-bit sign-extended)
 *  CSV 输出供 Python 分析 (cir_phase.py, cir_breath.py)
 * ====================================================================*/

/* ─── Radar channel config (RX only) ─── */
#define RADAR_TIMEOUT_MS    600  /* wait up to 600ms per frame */

static int32_t radar_s24(const uint8_t *p) {
    int32_t v = ((int32_t)p[0] << 16) | ((int32_t)p[1] << 8) | p[2];
    if (v & 0x800000) v |= (int32_t)0xFF000000U;
    return v;
}

int uwb_radar_rx_init(void)
{
    uint32_t id;

    UWB_SPI_Init();
    Active_UWB = 2;

    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2);

    while (!dwt_checkidlerc());

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("[RADAR-RX] dwt_initialise FAILED\r\n");
        return UWB_ERR_NO_DEVICE;
    }
    if (dwt_configure(&uwb_config)) {
        printf("[RADAR-RX] dwt_configure FAILED\r\n");
        return UWB_ERR_NO_DEVICE;
    }

    /* TX power config (needed even if RX-only — PLL uses it) */
    {
        dwt_txconfig_t tp = { 0x34, 0xfdfdfdfd, 0x0 };
        dwt_configuretxrf(&tp);
    }

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /* ★ Enable CIR accumulator on RX path */
    dwt_configciadiag(1);

    id = dwt_readdevid();
    printf("[RADAR-RX] Receiver ready, id=0x%08lX\r\n", (unsigned long)id);
    return UWB_OK;
}

int uwb_radar_rx_scan(uwb_radar_result_t *res, uint32_t seq, const char *label)
{
    uint32_t status, t0;
    uint8_t  acc_buf[256 * 6 + 1];
    dwt_rxdiag_t diag;

    memset(res, 0, sizeof(*res));
    res->seq = seq;
    if (label) strncpy(res->label, label, UWB_LABEL_LEN - 1);

    /* Quick poll: if RX already active from previous re-arm, check for pending frame */
    status = dwt_read32bitreg(SYS_STATUS_ID);
    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        /* Frame already waiting — skip to read */
    } else {
        /* Need to open/re-open RX */
        dwt_setpreambledetecttimeout(0);
        dwt_setrxtimeout(0);
        dwt_write32bitreg(SYS_STATUS_ID,
            SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        t0 = uwb_tick_get();
        while (1) {
            status = dwt_read32bitreg(SYS_STATUS_ID);
            if (status & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))
                break;
            if (uwb_tick_get() - t0 > RADAR_TIMEOUT_MS)
                break;
        }
    }

    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
        return UWB_ERR_TIMEOUT;
    }

    res->timestamp_ms = uwb_tick_get();

    /* Read diagnostics */
    dwt_readdiagnostics(&diag);
    res->fp_index = diag.ipatovFpIndex >> 6;
    res->rx_power = diag.ipatovPower;

    /* Read CIR accumulator */
    {
        uint16_t fp = res->fp_index;
        int32_t start = (int32_t)fp - 40;
        if (start < 0) start = 0;

        memset(acc_buf, 0, sizeof(acc_buf));
        dwt_readaccdata(acc_buf, sizeof(acc_buf), (uint16_t)start);

        uint32_t noise_sum = 0, max_mag = 0;
        for (int j = 0; j < UWB_RADAR_CIR_BINS; j++) {
            res->cir_i[j] = radar_s24(&acc_buf[1 + j * 6]);
            res->cir_q[j] = radar_s24(&acc_buf[1 + j * 6 + 3]);
            uint32_t mag = (uint32_t)(res->cir_i[j] >= 0 ? res->cir_i[j] : -res->cir_i[j])
                         + (uint32_t)(res->cir_q[j] >= 0 ? res->cir_q[j] : -res->cir_q[j]);
            if (mag > max_mag) max_mag = mag;
            if (j < UWB_RADAR_NOISE_BINS) noise_sum += mag;
        }
        res->noise   = noise_sum / UWB_RADAR_NOISE_BINS;
        res->fp_power = max_mag;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* ★ Immediately re-arm RX — so UART CSV printing (~260ms) doesn't miss next pulse */
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);
    dwt_write32bitreg(SYS_STATUS_ID,
        SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    return UWB_OK;
}

void uwb_radar_csv(const uwb_radar_result_t *res)
{
    if (res == NULL) {
        printf("tms,seq,fp,prx,noise");
        for (int j = 0; j < UWB_RADAR_CIR_BINS; j++) printf(",i%d,q%d", j, j);
        printf(",label\r\n");
        return;
    }
    printf("%lu,%lu,%u,%lu,%lu",
        (unsigned long)res->timestamp_ms, (unsigned long)res->seq,
        (unsigned)res->fp_index, (unsigned long)res->rx_power,
        (unsigned long)res->noise);
    for (int j = 0; j < UWB_RADAR_CIR_BINS; j++)
        printf(",%ld,%ld", (long)res->cir_i[j], (long)res->cir_q[j]);
    printf(",%s\r\n", res->label[0] ? res->label : "none");
}
