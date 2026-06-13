/*!
 *  \file    app_tasks.h
 *  \brief   FreeRTOS 任务定义 — GD32A7
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>

/* ── 任务优先级 ── */
#define TASK_PRIO_KEY           ( tskIDLE_PRIORITY + 5 )  /* 最高 — 按键响应优先 */
#define TASK_PRIO_CAN           ( tskIDLE_PRIORITY + 4 )  /* CAN 不丢帧 */
#define TASK_PRIO_UWB           ( tskIDLE_PRIORITY + 3 )
#define TASK_PRIO_WIFI          ( tskIDLE_PRIORITY + 2 )
#define TASK_PRIO_BLE           ( tskIDLE_PRIORITY + 1 )

/* ── 任务栈大小 ── */
#define TASK_STACK_UWB          ( 512 )
#define TASK_STACK_WIFI         ( 512 )
#define TASK_STACK_KEY          ( 256 )
#define TASK_STACK_BLE          ( 256 )

/* ── 模式 ── */
typedef enum { MODE_RANGING, MODE_RADAR } sys_mode_t;

/* ── 全局变量 (ISR 共享) ── */
extern volatile sys_mode_t g_mode;
extern uint8_t g_brake;
extern uint8_t g_parking_brake;
extern uint8_t g_car_lock;

/* ── CAN TX 队列 ── */
#define CAN_TX_QUEUE_LEN        16

typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
} can_tx_msg_t;

/* ── 任务句柄 ── */
extern TaskHandle_t xTaskUWB_Handle;
extern TaskHandle_t xTaskWiFi_Handle;
extern TaskHandle_t xTaskKey_Handle;

/* ── 队列/信号量 ── */
extern QueueHandle_t xCanTxQueue;
extern SemaphoreHandle_t xKey_Semaphore;

/* ── API ── */
void app_tasks_init(void);
void key_exti_init(void);

/* ── 任务函数 ── */
void vTaskUWB(void *pvParameters);
void vTaskWiFi(void *pvParameters);
void vTaskKey(void *pvParameters);

/* ── CAN 发送 (从队列) ── */
void can_tx_task_init(void);

#endif
