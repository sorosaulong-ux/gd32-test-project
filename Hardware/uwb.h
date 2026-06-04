/*!
 *  \file    uwb.h
 *  \brief   DW3000 UWB simplified API — radar + ranging
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
#define UWB_OK                0
#define UWB_ERR_TIMEOUT       1
#define UWB_ERR_NO_DEVICE     2

/* ====================================================================
 *  API
 * ====================================================================*/

/** Initialise both UWB modules — returns 0 on success */
int uwb_init(void);

/** Execute one SS‑TWR ranging cycle, write distance (m) to *dist_out.
 *  Returns UWB_OK on success. */
int uwb_ranging(double *dist_out);

/** Run one radar scan: TX pulse on module 1, capture CIR on module 2.
 *  Prints results to UART. */
void uwb_radar_scan(void);

/** Print device IDs of both modules to UART */
void uwb_print_ids(void);

#ifdef __cplusplus
}
#endif

#endif
