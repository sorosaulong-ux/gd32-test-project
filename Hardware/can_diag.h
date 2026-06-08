/*!
 *  \file    can_diag.h
 *  \brief   CAN diagnostics protocol — GD32A7 vehicle MCU
 *
 *  CAN2 (PM2/PM3) @ 500 kbps → GD32F3 master controller
 *
 *  Message IDs (11-bit):
 *    0x100 — Heartbeat (1Hz)
 *    0x101 — Ranging result (distance in mm, float*1000)
 *    0x102 — Radar status (child/pet detected)
 *    0x200 — Command from master (lock/unlock/alarm)
 */

#ifndef CAN_DIAG_H_
#define CAN_DIAG_H_

#include "gd32a7xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  CAN message IDs
 * ====================================================================*/
#define CAN_ID_HEARTBEAT    0x100U
#define CAN_ID_RANGING      0x101U
#define CAN_ID_RADAR        0x102U
#define CAN_ID_CMD          0x200U   /* from master */

/* ====================================================================
 *  CAN data structures (big-endian for compatibility)
 * ====================================================================*/

/* 0x100 Heartbeat: [0]=uptime_sec, [1]=mode, [2-3]=error_flags */
#define CAN_HB_MODE_RANGING   0
#define CAN_HB_MODE_RADAR     1
#define CAN_HB_MODE_IDLE      2

/* 0x101 Ranging: [0-3]=distance_mm (uint32 big-endian) */
/* 0x102 Radar:   [0]=detected(0/1), [1]=confidence(0-100) */
/* 0x200 CMD:     [0]=cmd(1=lock 2=unlock 3=alarm) */

/* ====================================================================
 *  API
 * ====================================================================*/
void can_diag_init(void);
void can_diag_send_heartbeat(uint8_t mode, uint16_t error_flags);
void can_diag_send_ranging(float distance_m);
void can_diag_send_radar(uint8_t detected, uint8_t confidence);
int  can_diag_poll_command(uint8_t *cmd, uint8_t *param);

/* Direct CAN access for debugging */
void can_diag_print_status(void);

#ifdef __cplusplus
}
#endif

#endif
