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

/* ====================================================================
 *  0x103 System Error — 异步事件帧，状态变化时发送
 *
 *  byte[0] = err_code   (主类别)
 *  byte[1] = sub_code   (子类别)
 *  byte[2] = 0x00       (reserved)
 *  byte[3] = 0x55=正常 / 0xAA=故障 (signature)
 *
 *  重复相同错误不发，err_code+sub_code=0 表示恢复正常
 * ====================================================================*/
#define CAN_ID_ERROR        0x103U

/* ── 错误码 ── */
#define CAN_ERR_UWB              0x01
#define   CAN_ERR_UWB_INIT       0x01
#define   CAN_ERR_UWB_ID         0x02
#define   CAN_ERR_UWB_TIMEOUT    0x03

#define CAN_ERR_ESP8266          0x02
#define   CAN_ERR_ESP_WIFI       0x01
#define   CAN_ERR_ESP_AT_TIMEOUT 0x02

#define CAN_ERR_CAN              0x03
#define   CAN_ERR_CAN_BUSOFF     0x01
#define   CAN_ERR_CAN_TX_FAIL    0x02

#define CAN_ERR_SYSTEM           0x04
#define   CAN_ERR_SYS_BOOT       0x01   /* boot完成 → GD32F3知道A7已就绪 */
#define   CAN_ERR_SYS_DEGRADED   0x02   /* 部分模块异常，降级运行 */

#define CAN_ERR_ONENET           0x05
#define   CAN_ERR_ONENET_MQTT    0x01   /* MQTT 断连 */

/* ====================================================================
 *  API
 * ====================================================================*/
void can_diag_init(void);
void can_diag_send_heartbeat(uint8_t mode, uint16_t error_flags);
void can_diag_send_ranging(float distance_m);
void can_diag_send_radar(uint8_t detected, uint8_t confidence);
void can_diag_send_error(uint8_t err_code, uint8_t sub_code);
int  can_diag_poll_command(uint8_t *cmd, uint8_t *param);

/* Direct CAN access for debugging */
void can_diag_print_status(void);

#ifdef __cplusplus
}
#endif

#endif
