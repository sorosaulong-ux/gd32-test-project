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

void can_diag_init(void)
{
    can_gpio_config();
    can_config(DTM_CAN2, 500U);
    printf("[CAN] CAN2 @ 500 kbps ready\r\n");
    can_print_error_status(DTM_CAN2);
}
