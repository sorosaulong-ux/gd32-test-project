/*!
 *  \file    uwb.h
 *  \brief   DW3000 UWB — DS-TWR Responder (single module, UWB2)
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
 *  DS-TWR Responder — GD32A7 as Responder (computes distance)
 *  STM32F103 as Initiator (sends Poll, receives Response, sends Final)
 * ====================================================================*/
int  uwb_ds_responder_init(void);
int  uwb_ds_responder_poll(double *dist);
void uwb_print_id(void);   /* print module 2 device ID */

#ifdef __cplusplus
}
#endif

#endif
