/*!
 *  \file    uwb.h
 *  \brief   DW3000 UWB — radar CIR + DS-TWR ranging + SS-TWR ranging
 */

#ifndef UWB_H_
#define UWB_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  Radar configuration — CH5, 128 bins
 * ====================================================================*/
#define UWB_RADAR_CHANNEL        5
#define UWB_RADAR_PLEN           DWT_PLEN_128
#define UWB_RADAR_PAC            DWT_PAC8
#define UWB_RADAR_SFD            DWT_SFD_IEEE_4Z
#define UWB_RADAR_STS_MODE       DWT_STS_MODE_OFF

#define UWB_RADAR_DR             DWT_BR_6M8
#define UWB_RADAR_STS_LEN        DWT_STS_LEN_64
#define UWB_RADAR_PDOA           DWT_PDOA_M0

#define UWB_RADAR_TX_INTERVAL_MS  250
#define UWB_RADAR_CIR_BINS        128
#define UWB_RADAR_NOISE_BINS      20

/* ====================================================================
 *  Return codes
 * ====================================================================*/
#define UWB_OK                   0
#define UWB_ERR_TIMEOUT          1
#define UWB_ERR_NO_DEVICE        2
#define UWB_ERR_TX_FAIL          3

#define UWB_LABEL_LEN            32

/* ====================================================================
 *  CIR result — raw I/Q per bin
 * ====================================================================*/
typedef struct {
    uint32_t  timestamp_ms;
    uint32_t  seq;
    uint16_t  fp_index;
    uint32_t  rx_power;
    uint32_t  fp_power;
    uint32_t  noise;
    int32_t   cir_i[UWB_RADAR_CIR_BINS];
    int32_t   cir_q[UWB_RADAR_CIR_BINS];
    char      label[UWB_LABEL_LEN];
} uwb_radar_result_t;

/* ====================================================================
 *  API — Bistatic radar (dual-module on GD32A7)
 * ====================================================================*/
int  uwb_radar_init(void);
int  uwb_radar_scan(uwb_radar_result_t *res, uint32_t seq, const char *label);
void uwb_radar_csv(const uwb_radar_result_t *res);

/* ====================================================================
 *  API — SS-TWR (dual-module on GD32A7)
 * ====================================================================*/
int  uwb_init(void);
int  uwb_ranging(double *dist_out);
void uwb_print_ids(void);

/* ====================================================================
 *  API — DS-TWR (single-module UWB2, with STM32 initiator)
 * ====================================================================*/
int  uwb_ds_responder_init(void);
int  uwb_ds_responder_poll(double *dist);

/* ====================================================================
 *  API — Radar RX (single-module UWB2, with STM32 as TX source)
 * ====================================================================*/
int  uwb_radar_rx_init(void);
int  uwb_radar_rx_scan(uwb_radar_result_t *res, uint32_t seq, const char *label);

/* ====================================================================
 *  API — UWB2 ID check (non-destructive)
 * ====================================================================*/
uint32_t uwb_check_id(void);
void     uwb_print_id(void);

#ifdef __cplusplus
}
#endif

#endif
