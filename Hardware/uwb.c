/*!
 *  \file    uwb.c
 *  \brief   DW3000 UWB radar CIR extraction + ranging (GD32A7xx)
 *
 *  Radar mode (bistatic):
 *    Module 1 = TX  → sends fixed pulse every 200 ms
 *    Module 2 = RX  → receives, captures 128-bin CIR, reads diagnostics
 *
 *  Config: CH9, PRF 64M, PLEN 512, PAC 32, 6.8 Mbps, no STS
 */

#include "uwb.h"
#include "uwb_port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>

/* ====================================================================
 *  Radar PHY configuration
 * ====================================================================*/
static dwt_config_t radar_config = {
    UWB_RADAR_CHANNEL,
    UWB_RADAR_PLEN,
    UWB_RADAR_PAC,
    9,                          /* TX preamble code (64 MHz PRF, code 9) */
    9,                          /* RX preamble code */
    UWB_RADAR_SFD,
    UWB_RADAR_DR,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (128 + 1 + 8 - 8),         /* SFD timeout = preamble + 1 + SFD_len - PAC = 129 */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

/* CH5 TX power — reduced to avoid saturation at close range */
static dwt_txconfig_t radar_tx_power = { 0x34, 0xfdfdfdfd, 0x0 };

/* Fixed radar TX pulse */
static uint8_t  tx_radar_msg[12] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'R','A','D','A','R', 0, 0
};

/* ====================================================================
 *  Internal helpers
 * ====================================================================*/

/* Sign‑extend a 24‑bit value read as 3 bytes (MSB first) */
static int32_t s24(const uint8_t *p)
{
    int32_t v = ((int32_t)p[0] << 16) | ((int32_t)p[1] << 8) | p[2];
    if (v & 0x800000) v |= (int32_t)0xFF000000U;
    return v;
}

/* Initialise one UWB node with radar PHY config */
static uint32_t uwb_init_node(uint8_t idx, uint8_t is_tx)
{
    Active_UWB = idx;
    reset_DWIC();
    delay_1ms(15);
    port_set_dw_ic_spi_slowrate();

    uint32_t id = dwt_readdevid();
    if (id == 0xFFFFFFFF || id == 0) return 0;
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) return 0;

    port_set_dw_ic_spi_fastrate();
    dwt_configure(&radar_config);
    dwt_configuretxrf(&radar_tx_power);
    dwt_setrxantennadelay(16385);
    dwt_settxantennadelay(16385);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /* enable CIA accumulator logging on RX module */
    if (!is_tx) {
        dwt_configciadiag(1);
    }

    return id;
}

/* ====================================================================
 *  uwb_radar_init()
 * ====================================================================*/
int uwb_radar_init(void)
{
    UWB_SPI_Init();

    uint32_t id_tx = uwb_init_node(1, 1);
    uint32_t id_rx = uwb_init_node(2, 0);

    printf("[UWB] TX(ID1)=0x%08X  RX(ID2)=0x%08X\r\n", (unsigned)id_tx, (unsigned)id_rx);

    if ((id_tx != 0xDECA0302 && id_tx != 0xDECA0312) ||
        (id_rx != 0xDECA0302 && id_rx != 0xDECA0312))
        return UWB_ERR_NO_DEVICE;
    return UWB_OK;
}

/* ====================================================================
 *  uwb_radar_scan()
 * ====================================================================*/
int uwb_radar_scan(uwb_radar_result_t *res, uint32_t seq, const char *label)
{
    uint32_t status, t0;
    uint8_t  acc_buf[UWB_RADAR_CIR_BINS * 6 + 1];  /* +1 dummy byte */
    dwt_rxdiag_t diag;

    memset(res, 0, sizeof(*res));
    res->seq = seq;
    if (label) strncpy(res->label, label, UWB_LABEL_LEN - 1);

    /* ── 1. Start RX on module 2 ── */
    Active_UWB = 2;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* ── 2. TX pulse on module 1 ── */
    Active_UWB = 1;
    tx_radar_msg[2] = (uint8_t)(seq & 0xFF);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_radar_msg), tx_radar_msg, 0);
    dwt_writetxfctrl(sizeof(tx_radar_msg), 0, 0);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_ERROR) {
        return UWB_ERR_TX_FAIL;
    }

    /* ── 3. Poll RX on module 2 ── */
    Active_UWB = 2;
    t0 = uwb_tick_get();
    while (1) {
        status = dwt_read32bitreg(SYS_STATUS_ID);
        if (status & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO))
            break;
        if (uwb_tick_get() - t0 > 50)   /* 50 ms timeout */
            break;
    }

    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
        printf("[DBG] seq=%lu status=0x%08lX (no RXFCG)\r\n",
               (unsigned long)seq, (unsigned long)status);
        return UWB_ERR_TIMEOUT;
    }

    res->timestamp_ms = uwb_tick_get();

    /* ── 4. Read diagnostics ── */
    dwt_readdiagnostics(&diag);
    res->fp_index = diag.ipatovFpIndex >> 6;
    res->rx_power = diag.ipatovPower;

    /* ── 5. Read accumulator — store raw I/Q per bin ── */
    {
        memset(acc_buf, 0, sizeof(acc_buf));
        dwt_readaccdata(acc_buf, sizeof(acc_buf), 0);

        int32_t  i_val, q_val;
        uint32_t mag, max_mag = 0, noise_sum = 0;
        int      peak_bin = 0;
        for (int j = 0; j < UWB_RADAR_CIR_BINS; j++) {
            i_val  = s24(&acc_buf[1 + j * 6]);
            q_val  = s24(&acc_buf[1 + j * 6 + 3]);
            res->cir_i[j] = i_val;
            res->cir_q[j] = q_val;
            mag = (i_val >= 0 ? (uint32_t)i_val : (uint32_t)(-i_val))
                + (q_val >= 0 ? (uint32_t)q_val : (uint32_t)(-q_val));
            if (mag > max_mag) { max_mag = mag; peak_bin = j; }
            if (j < UWB_RADAR_NOISE_BINS) noise_sum += mag;
        }
        res->noise    = noise_sum / UWB_RADAR_NOISE_BINS;
        res->fp_power = max_mag;
        res->fp_index = (uint16_t)peak_bin;

        if (seq < 3) {
            printf("[CIR] s=%lu peak=%u mag=%lu noise=%lu "
                   "i[%u]=%ld q[%u]=%ld\r\n",
                   (unsigned long)seq, (unsigned)peak_bin,
                   (unsigned long)max_mag, (unsigned long)res->noise,
                   (unsigned)peak_bin, (long)res->cir_i[peak_bin],
                   (unsigned)peak_bin, (long)res->cir_q[peak_bin]);
        }
    }

    /* ── 6. Clear RX status ── */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    return UWB_OK;
}

/* ====================================================================
 *  uwb_radar_csv()
 *  Print header if res==NULL; otherwise one CSV line.
 * ====================================================================*/
void uwb_radar_csv(const uwb_radar_result_t *res)
{
    if (res == NULL) {
        /* header: ts,seq,fp,rxp,fpp,noise, i0,q0,i1,q1,... i127,q127, label */
        printf("timestamp_ms,seq,fp_index,rx_power,fp_power,noise");
        for (int j = 0; j < UWB_RADAR_CIR_BINS; j++)
            printf(",i%d,q%d", j, j);
        printf(",label\r\n");
        return;
    }

    printf("%lu,%lu,%u,%lu,%lu,%lu",
           (unsigned long)res->timestamp_ms,
           (unsigned long)res->seq,
           (unsigned)res->fp_index,
           (unsigned long)res->rx_power,
           (unsigned long)res->fp_power,
           (unsigned long)res->noise);

    for (int j = 0; j < UWB_RADAR_CIR_BINS; j++)
        printf(",%ld,%ld", (long)res->cir_i[j], (long)res->cir_q[j]);

    printf(",%s\r\n", res->label[0] ? res->label : "none");
}

/* ====================================================================
 *  OG ranging API stubs (unchanged logic, moved to uwb_ranging.c style)
 * ====================================================================*/

static dwt_config_t uwb_config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};
static dwt_txconfig_t tx_power = { 0x34, 0xFFFFFFFF, 0x0 };

static uint8_t tx_poll[12] = { 0x41,0x88,0, 0xCA,0xDE,'W','A','V','E', 0xE0,0,0 };
static uint8_t rx_resp[20] = { 0x41,0x88,0, 0xCA,0xDE,'V','E','W','A', 0xE1,0,0,0,0,0,0,0,0,0,0 };
static uint8_t rx_buf[24];
static uint8_t frame_seq = 0;

static uint64_t rx_ts_u64(void) {
    uint8_t ts[5]; uint64_t t = 0;
    dwt_readrxtimestamp(ts);
    for (int i = 4; i >= 0; i--) { t <<= 8; t |= ts[i]; }
    return t;
}
static void ts_to_buf(uint8_t *d, uint64_t t) { for(int i=0;i<4;i++) d[i]=(uint8_t)(t>>(i*8)); }
static void ts_from_buf(uint8_t *s, uint32_t *t) {
    *t=0; for(int i=0;i<4;i++) *t|=((uint32_t)s[i])<<(i*8);
}

static uint32_t uwb_node_init(uint8_t idx) {
    Active_UWB=idx; reset_DWIC(); delay_1ms(15);
    port_set_dw_ic_spi_slowrate();
    uint32_t id=dwt_readdevid();
    if(id==0xFFFFFFFF||id==0) return 0;
    if(dwt_initialise(DWT_DW_INIT)==DWT_ERROR) return 0;
    port_set_dw_ic_spi_fastrate();
    dwt_configure(&uwb_config); dwt_configuretxrf(&tx_power);
    dwt_setrxantennadelay(16385); dwt_settxantennadelay(16385);
    dwt_setlnapamode(DWT_LNA_ENABLE|DWT_PA_ENABLE);
    if(idx==1){ dwt_setrxaftertxdelay(1500); dwt_setrxtimeout(1500); }
    return id;
}

int uwb_init(void) {
    UWB_SPI_Init();
    uint32_t a=uwb_node_init(1), b=uwb_node_init(2);
    printf("[UWB] ID1=0x%08X ID2=0x%08X\r\n",(unsigned)a,(unsigned)b);
    if((a==0xDECA0302||a==0xDECA0312)&&(b==0xDECA0302||b==0xDECA0312)) return UWB_OK;
    return UWB_ERR_NO_DEVICE;
}

void uwb_print_ids(void) {
    uint32_t a,b;
    Active_UWB=1; reset_DWIC(); delay_1ms(15);
    port_set_dw_ic_spi_slowrate(); a=dwt_readdevid(); port_set_dw_ic_spi_fastrate();
    Active_UWB=2; reset_DWIC(); delay_1ms(15);
    port_set_dw_ic_spi_slowrate(); b=dwt_readdevid(); port_set_dw_ic_spi_fastrate();
    printf("[UWB] ID1=0x%08X ID2=0x%08X\r\n",(unsigned)a,(unsigned)b);
}

int uwb_ranging(double *dist) {
    uint32_t status, timeout;
    Active_UWB=2; dwt_rxenable(DWT_START_RX_IMMEDIATE);
    Active_UWB=1; tx_poll[2]=frame_seq;
    dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll),tx_poll,0); dwt_writetxfctrl(sizeof(tx_poll),0,1);
    dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
    Active_UWB=2; timeout=0;
    while(!((status=dwt_read32bitreg(SYS_STATUS_ID))&(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_ERR)))
        if(++timeout>200000) return UWB_ERR_TIMEOUT;
    if(!(status&SYS_STATUS_RXFCG_BIT_MASK)){ dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_ERR); return UWB_ERR_TIMEOUT; }
    dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXFCG_BIT_MASK);
    uint64_t prx=rx_ts_u64();
    uint32_t rtt=(prx+(2000ULL*65536))>>8;
    dwt_setdelayedtrxtime(rtt);
    uint64_t rtx=(((uint64_t)(rtt&0xFFFFFFFEUL))<<8)+16385;
    rx_resp[2]=frame_seq; ts_to_buf(&rx_resp[10],prx); ts_to_buf(&rx_resp[14],rtx);
    dwt_writetxdata(sizeof(rx_resp),rx_resp,0); dwt_writetxfctrl(sizeof(rx_resp),0,1);
    if(dwt_starttx(DWT_START_TX_DELAYED)==DWT_ERROR) return UWB_ERR_TIMEOUT;
    Active_UWB=1; timeout=0;
    while(!((status=dwt_read32bitreg(SYS_STATUS_ID))&(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR)))
        if(++timeout>200000) break;
    if(!(status&SYS_STATUS_RXFCG_BIT_MASK)){ dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR); return UWB_ERR_TIMEOUT; }
    dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXFCG_BIT_MASK);
    uint32_t ptx=dwt_readtxtimestamplo32(), rrx=dwt_readrxtimestamplo32();
    uint32_t prx32,rtx32; dwt_readrxdata(rx_buf,sizeof(rx_resp),0);
    ts_from_buf(&rx_buf[10],&prx32); ts_from_buf(&rx_buf[14],&rtx32);
    int32_t rtd=rrx-ptx, rtr=rtx32-prx32;
    float clk=((float)dwt_readclockoffset())/(uint32_t)(1U<<26);
    double tof=((rtd-rtr*(1-clk))/2.0)*15.65e-12;
    *dist=tof*299702547.0;
    frame_seq++; return UWB_OK;
}

void uwb_radar_scan_simple(void) {
    uint32_t status,timeout;
    static uint8_t msg[]={0x41,0x88,0,0xCA,0xDE,'R','A','D','A','R',0,0};
    static uint8_t s=0;
    Active_UWB=2; dwt_rxenable(DWT_START_RX_IMMEDIATE);
    Active_UWB=1; msg[2]=s++;
    dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(msg),msg,0); dwt_writetxfctrl(sizeof(msg),0,1);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    Active_UWB=2; timeout=0;
    while(!((status=dwt_read32bitreg(SYS_STATUS_ID))&(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_ERR)))
        if(++timeout>200000) break;
    if(status&SYS_STATUS_RXFCG_BIT_MASK){
        dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXFCG_BIT_MASK);
        dwt_rxdiag_t diag; dwt_readdiagnostics(&diag);
        printf("[RADAR] FP=%u\r\n",diag.ipatovFpIndex>>6);
    }else{ dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_ERR); printf("[RADAR] No RX\r\n"); }
}
