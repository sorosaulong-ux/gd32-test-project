/*!
 *  \file    uwb.h
 *  \brief   DW3000 UWB — DS-TWR Responder + Radar RX (single module, UWB2)
 */

#ifndef UWB_H_
#define UWB_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  Return codes
 * ====================================================================*/
#define UWB_OK                   0
#define UWB_ERR_TIMEOUT          1
#define UWB_ERR_NO_DEVICE        2
#define UWB_ERR_TX_FAIL          3

/* ====================================================================
 *  DS-TWR Responder (测距)
 * ====================================================================*/
int  uwb_ds_responder_init(void);
int  uwb_ds_responder_poll(double *dist);
void uwb_print_id(void);

/* ====================================================================
 *  Radar RX (车内生命检测)
 * ====================================================================*/
#define UWB_RADAR_CIR_BINS    128
#define UWB_RADAR_NOISE_BINS  20
#define UWB_LABEL_LEN         32

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

int  uwb_radar_rx_init(void);
int  uwb_radar_rx_scan(uwb_radar_result_t *res, uint32_t seq, const char *label);
void uwb_radar_csv(const uwb_radar_result_t *res);  /* NULL → header */

#ifdef __cplusplus
}
#endif

#endif
