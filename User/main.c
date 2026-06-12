/*!
 *  \file    main.c
 *  \brief   GD32A7 整车终端 — FreeRTOS 统一架构
 *
 *  init_task: 创建 app_tasks → 删除自身
 *  业务逻辑全在 User/task/app_tasks.c
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "uart.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_tasks.h"

#include <stdio.h>

#define INIT_PRIO ( tskIDLE_PRIORITY + 6 )

static void init_task(void *pv)
{
    SCB_EnableICache();
    SCB_EnableDCache();

    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);

    uart_init();

    printf("\r\n=== GD32A7 Vehicle Terminal (FreeRTOS) ===\r\n\r\n");

    /* 启动所有任务 (UWB/WiFi/Key/CAN) */
    app_tasks_init();

    gd_eval_led_on(LED1);

    vTaskDelete(NULL);
}

int main(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    xTaskCreate(init_task, "Init", configMINIMAL_STACK_SIZE * 4,
                NULL, INIT_PRIO, NULL);
    vTaskStartScheduler();
    while (1) {}
}
