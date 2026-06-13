/*!
 *  \file    app_tasks.c
 *  \brief   FreeRTOS 任务实现 — 统一架构
 *
 *  vTaskUWB  (Prio 5): DS-TWR 测距 + 雷达 ML 检测 + BLE 休眠
 *  vTaskWiFi (Prio 3): WiFi 状态机 + OneNET + Buzzer 状态同步
 *  vTaskKey  (Prio 2): 按键 EXTI (Semaphore 唤醒)
 *  vTaskCanTX(Prio 4): 队列消费 CAN 帧
 *
 *  can_diag_send_* 通过 FreeRTOS 队列 → vTaskCanTX 异步发送
 */

#include "app_tasks.h"
#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "buzzer.h"
#include "can_diag.h"
#include "esp8266.h"
#include "onenet.h"
#include "uwb.h"
#include "uwb_port.h"
#include "key.h"
#include "ble.h"
#include "can.h"
#include "ml_predict.h"
#include <stdio.h>
#include <string.h>

/* ── 任务句柄 ── */
TaskHandle_t xTaskUWB_Handle  = NULL;
TaskHandle_t xTaskWiFi_Handle = NULL;
TaskHandle_t xTaskKey_Handle  = NULL;

/* ── 队列/信号量 ── */
QueueHandle_t xCanTxQueue = NULL;
SemaphoreHandle_t xKey_Semaphore = NULL;

/* ── 外部全局变量 ── */

/* ── 车辆状态 ── */
float   g_key_distance;
uint8_t g_car_lock;
uint8_t g_brake;
uint8_t g_parking_brake;
uint8_t g_system_status;
uint8_t g_ble_connected;

/* ── 模式切换 (KEY2 长按) ── */
typedef enum { MODE_RANGING, MODE_RADAR } sys_mode_t;
static volatile sys_mode_t g_mode = MODE_RANGING;

/* ====================================================================
 *  vTaskCanTX — 从队列取 CAN 帧并发送
 * ====================================================================*/
static void vTaskCanTX(void *pvParameters)
{
    can_tx_msg_t msg;
    while (1)
    {
        if (xQueueReceive(xCanTxQueue, &msg, portMAX_DELAY) == pdTRUE)
        {
            can_send_std_frame(DTM_CAN2, msg.id, msg.data, msg.len);
        }
    }
}

/* ── CAN 发送 API (通过队列) ── */
void can_diag_send_error(uint8_t err_code, uint8_t sub_code)
{
    static uint8_t last_err, last_sub;
    if (err_code == last_err && sub_code == last_sub) return;  /* 去重 */
    last_err = err_code; last_sub = sub_code;

    can_tx_msg_t msg;
    msg.id = 0x103U;
    msg.len = 4;
    msg.data[0] = err_code;
    msg.data[1] = sub_code;
    msg.data[2] = 0x00;
    msg.data[3] = (err_code == 0 && sub_code == 0) ? 0x55 : 0xAA;
    xQueueSend(xCanTxQueue, &msg, pdMS_TO_TICKS(10));
}

void can_diag_send_ranging(float distance_m)
{
    can_tx_msg_t msg;
    int32_t mm = (int32_t)(distance_m * 1000.0f);
    if (mm < 0) mm = 0;
    msg.id = 0x101U;
    msg.len = 4;
    msg.data[0] = (uint8_t)(mm >> 24);
    msg.data[1] = (uint8_t)(mm >> 16);
    msg.data[2] = (uint8_t)(mm >> 8);
    msg.data[3] = (uint8_t)(mm);
    xQueueSend(xCanTxQueue, &msg, pdMS_TO_TICKS(10));
}

void can_diag_send_radar(uint8_t detected, uint8_t confidence)
{
    can_tx_msg_t msg;
    msg.id = 0x102U;
    msg.len = 4;
    msg.data[0] = detected;
    msg.data[1] = confidence;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    xQueueSend(xCanTxQueue, &msg, pdMS_TO_TICKS(10));
}

/* ====================================================================
 *  vTaskUWB — DS-TWR 测距 + 雷达 ML (最高优先级)
 * ====================================================================*/
void vTaskUWB(void *pvParameters)
{
    uint8_t  rad_inited = 0;
    uint8_t  rng_inited = 0;
    uint32_t rad_seq = 0;
    uint32_t rng_cnt = 0;

    printf("[TASK] UWB task started\r\n");

    while (1)
    {
        /* ── BLE 连接检查 ── */
        if (!ble_is_connected()) {
            printf("[UWB] BLE disconnected, sleep\r\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (g_mode == MODE_RADAR) {
            /* ==================== 雷达模式 ==================== */
            static uwb_radar_result_t res;  /* static — ~1KB, 避免栈溢出 */

            if (!rad_inited) {
                if (uwb_radar_init() != UWB_OK) {
                    printf("[RADAR] Init FAIL\r\n");
                    taskYIELD();                              /* 让出 CPU 给按键/WiFi */
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                rad_inited = 1;
                rng_inited = 0;
                printf("[RADAR] Bistatic radar running...\r\n");
            }

            int ret = uwb_radar_scan(&res, rad_seq, "radar");
            if (ret == UWB_OK) {
                gd_eval_led_toggle(LED3);
                rad_seq++;

                /* ── ML 推理 ── */
                float prob;
                int   human = ml_predict(&res, &prob);
                static int last_state = -1;

                if (human != last_state) {
                    last_state = human;
                    can_diag_send_radar(human, (uint8_t)(prob * 100.0f));
                    printf("[DETECT] %s  p=%.2f\r\n",
                           human ? "HUMAN" : "EMPTY", prob);
                }
            }

        } else {
            /* ==================== 测距模式 ==================== */
            double dist;

            if (!rng_inited) {
                if (uwb_ds_responder_init() != UWB_OK) {
                    printf("[RNG] Init FAIL\r\n");
                    taskYIELD();
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                rng_inited = 1;
                rad_inited = 0;
                printf("[RNG] Listening for STM32...\r\n");
            }

            int ret = uwb_ds_responder_poll(&dist);
            if (ret == 1) {
                rng_cnt++;
                g_key_distance = (float)dist;
                gd_eval_led_toggle(LED2);
                printf("[%lu] dist=%.3f m\r\n", (unsigned long)rng_cnt, dist);
                can_diag_send_ranging((float)dist);

                /* 自动锁车逻辑 */
                if (dist < 2.5f && !g_car_lock) {
                    g_car_lock = 1;
                    printf("[LOGIC] Auto UNLOCK\r\n");
                }
                if (g_parking_brake && dist > 5.0f) {
                    printf("[LOGIC] Distance > 5m, stop\r\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ====================================================================
 *  vTaskWiFi — WiFi + OneNET + Buzzer 同步
 * ====================================================================*/
void vTaskWiFi(void *pvParameters)
{
    unsigned short tick_5s = 0;

    printf("[TASK] WiFi task started\r\n");

    /* 首次等待 BLE 连接 */
    while (!ble_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* 非阻塞 WiFi 初始化 */
    wifi_sm_start();

    while (1)
    {
        wifi_sm_tick();

        if (wifi_sm_ready()) {
            if (++tick_5s >= 100) { tick_5s = 0;
                OneNet_SendData(); ESP8266_Clear();
            }
            { unsigned char *p = ESP8266_GetIPD(0);
              if (p) OneNet_RevPro(p); }
        }

        /* BLE 状态同步 */
        g_ble_connected = ble_is_connected();

        /* ── 健康检测: UWB2 ID (每 30s) ── */
        {
            static uint16_t hc_cnt;
            static uint8_t  last_uwb_ok = 1;
            if (++hc_cnt >= 600) { hc_cnt = 0;
                uint32_t id = uwb_check_id();
                uint8_t  ok = (id == 0xDECA0302U || id == 0xDECA0312U);
                if (!ok && last_uwb_ok) {
                    can_diag_send_error(0x01U, 0x01U);
                }
                if (ok && !last_uwb_ok) {
                    can_diag_send_error(0, 0);
                }
                last_uwb_ok = ok;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ====================================================================
 *  vTaskKey — 按键信号量唤醒
 * ====================================================================*/
void vTaskKey(void *pvParameters)
{
    printf("[TASK] Key task started\r\n");

    while (1)
    {
        if (xSemaphoreTake(xKey_Semaphore, portMAX_DELAY) == pdTRUE)
        {
            /* ★ 确认任务执行 */
            printf("[KEY] wake K1=%d K2=%d\r\n",
                   gpio_input_bit_get(GPIOC, GPIO_PIN_13),
                   gpio_input_bit_get(GPIOL, GPIO_PIN_3));
            uint8_t k1 = gpio_input_bit_get(GPIOC, GPIO_PIN_13);
            uint8_t k2 = gpio_input_bit_get(GPIOL, GPIO_PIN_3);
            uint8_t k3 = gpio_input_bit_get(GPIOL, GPIO_PIN_4);
            uint8_t k4 = gpio_input_bit_get(GPIOL, GPIO_PIN_5);

            /* 时间戳去抖 — 同键 200ms 内只响应 1 次 */
            TickType_t now = xTaskGetTickCount();

            /* KEY1 (PC13) — 按下 LOW = 蜂鸣器翻转 */
            {
                static TickType_t last_tm;
                if (k1 == 0 && (TickType_t)(now - last_tm) > pdMS_TO_TICKS(200)) {
                    last_tm = now;
                    BUZZER_Set(BUZZER_Status == BUZZER_ON ? BUZZER_OFF : BUZZER_ON);
                    printf("[KEY] Buzzer toggle\r\n");
                }
            }

            /* KEY2 (GPIOL.3) — 按下 LOW = 模式切换 */
            {
                static TickType_t last_tm;
                if (k2 == 0 && (TickType_t)(now - last_tm) > pdMS_TO_TICKS(200)) {
                    last_tm = now;
                    g_mode = (g_mode == MODE_RADAR) ? MODE_RANGING : MODE_RADAR;
                    printf("\r\n=== SWITCH: %s ===\r\n",
                           (g_mode == MODE_RADAR) ? "RADAR" : "RANGING");
                }
            }

            /* KEY3/4 — 按下 LOW = 刹车/手刹 */
            g_brake         = (k3 == 0);
            g_parking_brake = (k4 == 0);

            gd_eval_led_toggle(LED2);       /* LED2 翻转 = 按键已处理 */

            if (g_car_lock && g_brake) {
                printf("[LOGIC] START\r\n");
            }
        }
    }
}

/* ====================================================================
 *  app_tasks_init — 创建所有任务
 * ====================================================================*/
void app_tasks_init(void)
{
    /* CAN TX 队列 */
    xCanTxQueue = xQueueCreate(CAN_TX_QUEUE_LEN, sizeof(can_tx_msg_t));

    /* 信号量 */
    xKey_Semaphore = xSemaphoreCreateBinary();

    /* ── 按键 EXTI 初始化 ── */
    key_exti_init();

    /* ── BLE 初始化 ── */
    ble_init();
    g_ble_connected = 0;

    /* ── 硬件 ── */
    BUZZER_Init();
    can_diag_init();
    printf("[SYS] Buzzer+CAN ready\r\n\r\n");

    /* ── 创建任务 ── */
    xTaskCreate(vTaskCanTX, "CANTX", 256, NULL, TASK_PRIO_CAN, NULL);
    xTaskCreate(vTaskUWB,   "UWB",   TASK_STACK_UWB,  NULL, TASK_PRIO_UWB,  &xTaskUWB_Handle);
    xTaskCreate(vTaskWiFi,  "WiFi",  TASK_STACK_WIFI, NULL, TASK_PRIO_WIFI, &xTaskWiFi_Handle);
    xTaskCreate(vTaskKey,   "Key",   TASK_STACK_KEY,  NULL, TASK_PRIO_KEY,  &xTaskKey_Handle);

    /* 启动通知 */
    can_diag_send_error(0x04U, 0x01U);  /* Boot 完成 */

    printf("[SYS] All tasks created\r\n");
}

/* ====================================================================
 *  can_diag_send_buzzer — CAN 0x105 蜂鸣器状态
 * ====================================================================*/
void can_diag_send_buzzer(uint8_t on)
{
    can_tx_msg_t msg;
    msg.id = 0x105U;
    msg.len = 4;
    msg.data[0] = on;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    xQueueSend(xCanTxQueue, &msg, pdMS_TO_TICKS(10));
}

/* ====================================================================
 *  can_diag_send_vehicle_cmd — CAN 0x104 车辆控制命令
 * ====================================================================*/
void can_diag_send_vehicle_cmd(uint8_t cmd)
{
    can_tx_msg_t msg;
    msg.id = 0x104U;
    msg.len = 4;
    msg.data[0] = cmd;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    xQueueSend(xCanTxQueue, &msg, pdMS_TO_TICKS(10));
}

/* ====================================================================
 *  FreeRTOS required hooks
 * ====================================================================*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; (void)pcTaskName;
    while (1) {}
}

void vApplicationMallocFailedHook(void)
{
    while (1) {}
}
