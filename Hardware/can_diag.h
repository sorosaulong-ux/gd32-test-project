/*!
 *  \file    can_diag.h
 *  \brief   CAN 系统错误上报 — GD32A7 → GD32F3
 *
 *  CAN2 (PM2/PM3) @ 500 kbps
 *  仅发送 0x103 系统错误帧，不发送 UWB/测距/雷达数据
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
#define CAN_ID_RANGING      0x101U
#define CAN_ID_RADAR        0x102U   /* radar detection */
#define CAN_ID_ERROR        0x103U
#define CAN_ID_VEHICLE      0x104U   /* 车辆启动/熄灭 */
#define CAN_ID_BUZZER       0x105U   /* 蜂鸣器状态 */

/* ====================================================================
 *  0x103 System Error — 状态变化时发送
 *
 *  byte[0] = err_code
 *  byte[1] = sub_code
 *  byte[2] = 0x00
 *  byte[3] = 0x55=正常 / 0xAA=故障
 *
 *  去重: 同一 err_code+sub_code 只发一次
 *        err_code=0 表示清除所有错误
 * ====================================================================*/

/* ── 错误码 ── */
#define CAN_ERR_UWB              0x01
#define   CAN_ERR_UWB_ID         0x01    /* device ID 异常 */

#define CAN_ERR_ESP8266          0x02
#define   CAN_ERR_ESP_WIFI       0x01
#define   CAN_ERR_ESP_AT_TIMEOUT 0x02

#define CAN_ERR_CAN              0x03
#define   CAN_ERR_CAN_BUSOFF     0x01
#define   CAN_ERR_CAN_TX_FAIL    0x02

#define CAN_ERR_SYSTEM           0x04
#define   CAN_ERR_SYS_BOOT       0x01
#define   CAN_ERR_SYS_DEGRADED   0x02

#define CAN_ERR_ONENET           0x05
#define   CAN_ERR_ONENET_MQTT    0x01

/* ── 车辆命令 ── */
#define VEHICLE_CMD_START    0x01
#define VEHICLE_CMD_STOP     0x02

/* ====================================================================
 *  API
 * ====================================================================*/
void can_diag_init(void);
void can_diag_send_error(uint8_t err_code, uint8_t sub_code);
void can_diag_send_ranging(float distance_m);
void can_diag_send_radar(uint8_t detected, uint8_t confidence);
void can_diag_send_vehicle_cmd(uint8_t cmd);
void can_diag_send_buzzer(uint8_t on);

#ifdef __cplusplus
}
#endif

#endif
