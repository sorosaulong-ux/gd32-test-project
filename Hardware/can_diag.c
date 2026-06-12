/*!
 *  \file    can_diag.c
 *  \brief   CAN 初始化 — GD32A7
 *
 *  CAN2 @ 500 kbps
 *  发送函数已移至 app_tasks.c (通过 FreeRTOS 队列)
 */

#include "can_diag.h"
#include "can.h"
#include <stdio.h>

static uint32_t s_tx_cnt, s_err_cnt;

void can_diag_init(void)
{
    can_gpio_config();
    can_config(DTM_CAN2, 500U);
    printf("[CAN] CAN2 @ 500 kbps ready\r\n");
    can_print_error_status(DTM_CAN2);
}

static void be32(uint8_t *buf, uint32_t val) {
    buf[0]=(uint8_t)(val>>24); buf[1]=(uint8_t)(val>>16);
    buf[2]=(uint8_t)(val>>8);  buf[3]=(uint8_t)(val);
}

void can_diag_send_error(uint8_t err_code, uint8_t sub_code) {
    static uint8_t last_err, last_sub;
    if (err_code==last_err && sub_code==last_sub) return;
    last_err=err_code; last_sub=sub_code;
    uint8_t d[4]={err_code,sub_code,0,(uint8_t)((err_code==0&&sub_code==0)?0x55:0xAA)};
    if (can_send_std_frame(DTM_CAN2,0x103U,d,4)==SUCCESS) s_tx_cnt++; else s_err_cnt++;
}
void can_diag_send_radar(uint8_t detected, uint8_t confidence) {
    uint8_t d[4]={detected,confidence,0,0};
    if (can_send_std_frame(DTM_CAN2,0x102U,d,4)==SUCCESS) s_tx_cnt++; else s_err_cnt++;
}
void can_diag_send_ranging(float distance_m) {
    int32_t mm=(int32_t)(distance_m*1000.0f); if(mm<0)mm=0;
    uint8_t d[4]; be32(d,(uint32_t)mm);
    if (can_send_std_frame(DTM_CAN2,0x101U,d,4)==SUCCESS) s_tx_cnt++; else s_err_cnt++;
}
