/*!
 *  \file    can_diag.c
 *  \brief   CAN diagnostics protocol — GD32A7 vehicle MCU
 *
 *  CAN2 (PM2/PM3) @ 500 kbps → GD32F3 master controller
 */

#include "can_diag.h"
#include "can.h"
#include <stdio.h>

static uint32_t s_tx_cnt;
static uint32_t s_rx_cnt;
static uint32_t s_err_cnt;

/* ─── be32: uint32 → 4 bytes big-endian ─── */
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
 *  can_diag_send_heartbeat — call from 1 Hz timer in main loop
 * ====================================================================*/
void can_diag_send_heartbeat(uint8_t mode, uint16_t error_flags)
{
    uint8_t data[4];
    data[0] = (uint8_t)(mode);
    data[1] = (uint8_t)(error_flags >> 8);
    data[2] = (uint8_t)(error_flags);
    data[3] = 0xAA;  /* signature */

    if (SUCCESS == can_send_std_frame(DTM_CAN2, CAN_ID_HEARTBEAT, data, 4)) {
        s_tx_cnt++;
    } else {
        s_err_cnt++;
    }
}

/* ====================================================================
 *  can_diag_send_ranging — distance (m) → mm big-endian
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
 *  can_diag_send_radar — child/pet detection alert
 * ====================================================================*/
void can_diag_send_radar(uint8_t detected, uint8_t confidence)
{
    uint8_t data[2];
    data[0] = detected;
    data[1] = confidence;

    if (SUCCESS == can_send_std_frame(DTM_CAN2, CAN_ID_RADAR, data, 2)) {
        s_tx_cnt++;
    } else {
        s_err_cnt++;
    }
}

/* ====================================================================
 *  can_diag_send_error — 异步错误事件 (状态变化时发送)
 *
 *  同一 err_code+sub_code 连续调用只发一次，
 *  调用 can_diag_send_error(0,0) 可强制重发。
 * ====================================================================*/
void can_diag_send_error(uint8_t err_code, uint8_t sub_code)
{
    static uint8_t last_err, last_sub;
    uint8_t data[4];

    /* 去重：同一个错误不反复发 */
    if (err_code == last_err && sub_code == last_sub)
        return;

    last_err = err_code;
    last_sub = sub_code;

    data[0] = err_code;
    data[1] = sub_code;
    data[2] = 0x00;
    data[3] = (err_code == 0 && sub_code == 0) ? 0x55 : 0xAA;  /* 55=正常, AA=故障 */

    if (SUCCESS == can_send_std_frame(DTM_CAN2, CAN_ID_ERROR, data, 4)) {
        s_tx_cnt++;
        printf("[CAN-ERR] id=0x%03X err=%02X.%02X %s\r\n",
               CAN_ID_ERROR, err_code, sub_code,
               (err_code == 0) ? "CLEAR" : "FAULT");
    } else {
        s_err_cnt++;
    }
}

/* ====================================================================
 *  can_diag_poll_command — check for master command (non-blocking)
 * ====================================================================*/
int can_diag_poll_command(uint8_t *cmd, uint8_t *param)
{
    can_receive_message_struct rx;

    if (SUCCESS != can_recv_frame_poll(DTM_CAN2, &rx)) {
        return 0;
    }

    if (rx.id != CAN_ID_CMD) {
        return 0;
    }

    s_rx_cnt++;
    if (rx.data_bytes >= 1) *cmd   = rx.data[0];
    if (rx.data_bytes >= 2) *param = rx.data[1];

    return 1;
}

/* ====================================================================
 *  can_diag_print_status
 * ====================================================================*/
void can_diag_print_status(void)
{
    printf("[CAN] TX=%lu RX=%lu ERR=%lu\r\n",
           (unsigned long)s_tx_cnt,
           (unsigned long)s_rx_cnt,
           (unsigned long)s_err_cnt);
    can_print_error_status(DTM_CAN2);
}
