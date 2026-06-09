/*!
 *  \file    can_diag.c
 *  \brief   CAN 系统错误上报 — GD32A7 → GD32F3
 *
 *  CAN2 @ 500 kbps, 仅发送 0x103 错误帧
 */

#include "can_diag.h"
#include "can.h"
#include <stdio.h>

static uint32_t s_tx_cnt;
static uint32_t s_err_cnt;

static void be32(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val >> 24);
    buf[1] = (uint8_t)(val >> 16);
    buf[2] = (uint8_t)(val >> 8);
    buf[3] = (uint8_t)(val);
}

/* ====================================================================
 *  can_diag_init
 * ====================================================================*/
void can_diag_init(void)
{
    can_gpio_config();
    can_config(DTM_CAN2, 500U);

    printf("[CAN] CAN2 @ 500 kbps ready\r\n");
    can_print_error_status(DTM_CAN2);
}

/* ====================================================================
 *  can_diag_send_error — 系统错误事件 (状态变化时发送)
 *
 *  同一 err_code+sub_code 只发一次。
 *  err_code=0, sub_code=0 → 清除所有错误 (发 0x55 签名)。
 * ====================================================================*/
void can_diag_send_error(uint8_t err_code, uint8_t sub_code)
{
    static uint8_t last_err, last_sub;
    uint8_t data[4];

    /* 去重 */
    if (err_code == last_err && sub_code == last_sub)
        return;

    last_err = err_code;
    last_sub = sub_code;

    data[0] = err_code;
    data[1] = sub_code;
    data[2] = 0x00;
    data[3] = (err_code == 0 && sub_code == 0) ? 0x55 : 0xAA;

    if (SUCCESS == can_send_std_frame(DTM_CAN2, CAN_ID_ERROR, data, 4)) {
        s_tx_cnt++;
        printf("[CAN-ERR] 0x103 err=%02X.%02X %s\r\n",
               err_code, sub_code, (err_code == 0) ? "CLEAR" : "FAULT");
    } else {
        s_err_cnt++;
    }
}

/* ====================================================================
 *  can_diag_send_ranging — distance (m) → mm big-endian, ID 0x101
 * ====================================================================*/
void can_diag_send_ranging(float distance_m)
{
    uint8_t data[4];
    int32_t mm = (int32_t)(distance_m * 1000.0f);
    if (mm < 0) mm = 0;
    be32(data, (uint32_t)mm);

    if (SUCCESS == can_send_std_frame(DTM_CAN2, CAN_ID_RANGING, data, 4)) {
        s_tx_cnt++;
    } else {
        s_err_cnt++;
    }
}

/* ====================================================================
 *  can_diag_send_radar — child/pet detection alert, ID 0x102
 * ====================================================================*/
void can_diag_send_radar(uint8_t detected, uint8_t confidence)
{
    uint8_t data[4];
    data[0] = detected;
    data[1] = confidence;
    data[2] = 0x00;
    data[3] = 0x00;

    if (SUCCESS == can_send_std_frame(DTM_CAN2, CAN_ID_RADAR, data, 4)) {
        s_tx_cnt++;
    } else {
        s_err_cnt++;
    }
}
