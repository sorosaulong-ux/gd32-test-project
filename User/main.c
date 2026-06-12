/*!
 *  \file    main.c
 *  \brief   GD32A7 整车诊断终端 — FreeRTOS 版本
 *
 *  UWB Task  (P5): DS-TWR 测距 + 雷达检测
 *  WiFi Task (P3): OneNET 上报 + 云指令
 *  Key Task  (P2): 按键处理
 *  CAN TX    (P4): 队列发送
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "systick.h"
#include "uart.h"
#include "delay.h"

#include "esp8266.h"
#include "onenet.h"
#include "buzzer.h"
#include "key.h"
#include "can_diag.h"
#include "uwb.h"
#include "uwb_port.h"
#include "ml_predict.h"
#include "ble.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "app_tasks.h"

#include <stdio.h>

/* ── 调试开关 ── */
//#define UWB_CSV_OUTPUT

/* ====================================================================
 *  Mode switch
 * ====================================================================*/
typedef enum { MODE_RANGING, MODE_RADAR } sys_mode_t;
sys_mode_t g_mode = MODE_RADAR;

/* ── 车辆控制状态 ── */
uint8_t  g_car_lock = 0;
uint8_t  g_brake = 0;
uint8_t  g_parking_brake = 0;
uint8_t  g_system_status = 0;
float    g_key_distance = 99.0f;
uint8_t  g_ble_connected = 0;

/* ====================================================================
 *  硬件初始化 (在 init_task 中调用)
 * ====================================================================*/
static void hardware_init(void)
{
    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);

    uart_init();
    key1_init();
    ble_init();

    can_diag_init();
    BUZZER_Init();

    printf("\r\n=== GD32A7 FreeRTOS Terminal ===\r\n\r\n");
}

/* ====================================================================
 *  NVIC 优先级配置 (FreeRTOS 要求)
 * ====================================================================*/
static void nvic_config(void)
{
    /* FreeRTOS 要求: 4位抢占，0位子优先级 */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
}

/* ====================================================================
 *  SysTick 配置 (FreeRTOS 使用)
 * ====================================================================*/
static void systick_setup(void)
{
    /* FreeRTOS 会接管 SysTick，这里只做基本配置 */
    systick_config();
}

/* ====================================================================
 *  main
 * ====================================================================*/
int main(void)
{
    /* 1. 启用缓存 */
    SCB_EnableICache();
    SCB_EnableDCache();

    /* 2. 配置 NVIC (必须在 FreeRTOS 之前) */
    nvic_config();

    /* 3. 配置 SysTick */
    systick_setup();

    /* 4. 硬件初始化 */
    hardware_init();

    /* 5. 创建 FreeRTOS 任务 */
    app_tasks_init();

    /* 6. 启动调度器 (不会返回) */
    vTaskStartScheduler();

    /* 不应该到这里 */
    while (1) {}
}

/* ====================================================================
 *  FreeRTOS 钩子函数
 * ====================================================================*/
void vApplicationMallocFailedHook(void)
{
    printf("[FATAL] Malloc failed!\r\n");
    taskDISABLE_INTERRUPTS();
    while (1);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("[FATAL] Stack overflow in %s\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    while (1);
}

void vApplicationIdleHook(void)
{
    /* 空闲时可以进入低功耗 */
}
