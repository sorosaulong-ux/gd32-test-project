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
