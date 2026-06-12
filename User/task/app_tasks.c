/*!
 *  \file    app_tasks.c
 *  \brief   FreeRTOS 任务实现 — GD32A7
 *
 *  UWB Task  (优先级5): DS-TWR 测距 + 雷达检测
 *  WiFi Task (优先级3): OneNET 上报 + 云指令
 *  Key Task  (优先级2): 按键处理
 *  CAN       (无任务):  队列发送，ISR 或其他任务直接写入
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
#include "ble.h"
#include "key.h"
#include <stdio.h>
#include <string.h>

/* ── 全局变量 ── */
TaskHandle_t xTaskUWB_Handle = NULL;
TaskHandle_t xTaskWiFi_Handle = NULL;
TaskHandle_t xTaskKey_Handle = NULL;

QueueHandle_t xCanTxQueue = NULL;
SemaphoreHandle_t xUWB_Semaphore = NULL;
SemaphoreHandle_t xKey_Semaphore = NULL;

/* ── 外部变量 (来自 main.c) ── */
extern uint8_t  g_car_lock;
extern uint8_t  g_brake;
extern uint8_t  g_parking_brake;
extern uint8_t  g_system_status;
extern float    g_key_distance;
extern uint8_t  g_ble_connected;
extern volatile uint8_t BUZZER_Status;

/* ── CAN TX 任务 (从队列读取并发送) ── */
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

void can_diag_send_error(uint8_t err_code, uint8_t sub_code)
{
    can_tx_msg_t msg;
    msg.id = 0x103;
    msg.len = 4;
    msg.data[0] = err_code;
    msg.data[1] = sub_code;
    msg.data[2] = 0x00;
    msg.data[3] = (err_code == 0 && sub_code == 0) ? 0x55 : 0xAA;
    xQueueSend(xCanTxQueue, &msg, 0);
}

void can_diag_send_ranging(float distance_m)
{
    can_tx_msg_t msg;
    msg.id = 0x101;
    msg.len = 4;
    int32_t mm = (int32_t)(distance_m * 1000.0f);
    msg.data[0] = (mm >> 24) & 0xFF;
    msg.data[1] = (mm >> 16) & 0xFF;
    msg.data[2] = (mm >> 8) & 0xFF;
    msg.data[3] = mm & 0xFF;
    xQueueSend(xCanTxQueue, &msg, 0);
}

void can_diag_send_radar(uint8_t detected, uint8_t confidence)
{
    can_tx_msg_t msg;
    msg.id = 0x102;
    msg.len = 4;
    msg.data[0] = detected;
    msg.data[1] = confidence;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    xQueueSend(xCanTxQueue, &msg, 0);
}

void can_diag_send_buzzer(uint8_t on)
{
    can_tx_msg_t msg;
    msg.id = 0x105;
    msg.len = 4;
    msg.data[0] = on;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    xQueueSend(xCanTxQueue, &msg, 0);
}

void can_diag_send_vehicle_cmd(uint8_t cmd)
{
    can_tx_msg_t msg;
    msg.id = 0x104;
    msg.len = 4;
    msg.data[0] = cmd;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    xQueueSend(xCanTxQueue, &msg, 0);
}

/* ====================================================================
 *  UWB 任务 — 最高优先级
 * ====================================================================*/
void vTaskUWB(void *pvParameters)
{
    uint32_t id;
    double dist;

    printf("[TASK] UWB task started\r\n");

    /* UWB 初始化 */
    UWB_Hardware_Init();
    reset_DWIC(); vTaskDelay(pdMS_TO_TICKS(100));
    while (!dwt_checkidlerc());

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("[UWB] INIT ERR\r\n");
        vTaskDelete(NULL);
    }

    dwt_config_t config = {
        5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
        DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
        (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
    };
    dwt_configure(&config);

    { dwt_txconfig_t tp = {0x34, 0xfdfdfdfd, 0x0}; dwt_configuretxrf(&tp); }
    dwt_setrxantennadelay(16385);
    dwt_settxantennadelay(16385);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    id = dwt_readdevid();
    printf("[UWB] ID: %08lX\r\n", (unsigned long)id);

    /* DS-TWR Responder 初始化 */
    if (uwb_ds_responder_init() != UWB_OK) {
        printf("[UWB] DS-TWR init FAIL\r\n");
    }

    while (1)
    {
        /* ── BLE 连接检查 ── */
        if (!BLE_IsConnected()) {
            dwt_entersleep(0);
            printf("[UWB] Sleep (BLE disconnected)\r\n");
            while (!BLE_IsConnected()) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            /* 重连后重新初始化 */
            UWB_Hardware_Init();
            reset_DWIC(); vTaskDelay(pdMS_TO_TICKS(100));
            while (!dwt_checkidlerc());
            dwt_initialise(DWT_DW_INIT);
            dwt_configure(&config);
            { dwt_txconfig_t tp = {0x34, 0xfdfdfdfd, 0x0}; dwt_configuretxrf(&tp); }
            dwt_setrxantennadelay(16385);
            dwt_settxantennadelay(16385);
            dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
            printf("[UWB] Wakeup (BLE connected)\r\n");
        }

        /* ── DS-TWR 测距 ── */
        if (uwb_ds_responder_poll(&dist) == 1) {
            g_key_distance = (float)dist;
            can_diag_send_ranging((float)dist);

            if (g_key_distance < 2.5f && !g_car_lock) {
                g_car_lock = 1;
                printf("[LOGIC] Auto UNLOCK\r\n");
            }
            if (g_parking_brake && g_key_distance > 5.0f) {
                can_diag_send_vehicle_cmd(0x02);
                printf("[LOGIC] STOP\r\n");
            }
        } else {
            /* 超时，让出 CPU */
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/* ====================================================================
 *  WiFi 任务 — 带超时的队列等待
 * ====================================================================*/
void vTaskWiFi(void *pvParameters)
{
    uint32_t tick_5s = 0;

    printf("[TASK] WiFi task started\r\n");

    while (1)
    {
        /* ── WiFi 状态机 ── */
        wifi_sm_tick();

        if (wifi_sm_ready()) {
            /* ── OneNET 上报 (每5秒) ── */
            if (++tick_5s >= 5) {
                tick_5s = 0;
                OneNet_SendData();
                ESP8266_Clear();
            }

            /* ── 云指令接收 (带超时) ── */
            unsigned char *p = ESP8266_GetIPD(0);
            if (p) OneNet_RevPro(p);
        }

        /* ── BLE 状态同步 ── */
        g_ble_connected = ble_is_connected();

        /* ── 蜂鸣器状态同步 ── */
        static uint8_t last_buzzer = 0xFF;
        if (BUZZER_Status != last_buzzer) {
            can_diag_send_buzzer(BUZZER_Status);
            last_buzzer = BUZZER_Status;
        }

        /* ── 阻塞等待 1 秒 ── */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ====================================================================
 *  按键 任务 — 中断触发
 * ====================================================================*/
void vTaskKey(void *pvParameters)
{
    printf("[TASK] Key task started\r\n");

    while (1)
    {
        /* ── 等待按键中断通知 ── */
        if (xSemaphoreTake(xKey_Semaphore, portMAX_DELAY) == pdTRUE)
        {
            /* 消抖延时 */
            vTaskDelay(pdMS_TO_TICKS(50));

            /* KEY1 — 蜂鸣器 */
            if (gpio_input_bit_get(GPIOC, GPIO_PIN_13) == RESET) {
                BUZZER_Set(BUZZER_Status == BUZZER_ON ? BUZZER_OFF : BUZZER_ON);
                printf("[KEY] Buzzer toggle\r\n");
            }

            /* KEY2 — 模式切换 (长按检测) */
            static uint32_t key2_press_ms = 0;
            if (gpio_input_bit_get(GPIOL, GPIO_PIN_3) == RESET) {
                if (key2_press_ms == 0) key2_press_ms = xTaskGetTickCount();
            } else {
                if (key2_press_ms != 0) {
                    uint32_t dur = xTaskGetTickCount() - key2_press_ms;
                    if (dur >= 500) {
                        extern sys_mode_t g_mode;
                        g_mode = (g_mode == MODE_RADAR) ? MODE_RANGING : MODE_RADAR;
                        printf("[KEY] Mode switch\r\n");
                    }
                    key2_press_ms = 0;
                }
            }

            /* KEY3 — 刹车 */
            if (gpio_input_bit_get(GPIOL, GPIO_PIN_4) == RESET) {
                g_brake = 1;
                printf("[KEY] Brake ON\r\n");
            } else {
                g_brake = 0;
            }

            /* KEY4 — 手刹 */
            if (gpio_input_bit_get(GPIOL, GPIO_PIN_5) == RESET) {
                g_parking_brake = 1;
                printf("[KEY] Parking brake ON\r\n");
            } else {
                g_parking_brake = 0;
            }

            /* 车锁+刹车 → 启动 */
            if (g_car_lock && g_brake) {
                can_diag_send_vehicle_cmd(0x01);
                printf("[LOGIC] START\r\n");
            }
        }
    }
}

/* ====================================================================
 *  任务初始化
 * ====================================================================*/
void app_tasks_init(void)
{
    /* 创建 CAN TX 队列 */
    xCanTxQueue = xQueueCreate(CAN_TX_QUEUE_LEN, sizeof(can_tx_msg_t));

    /* 创建信号量 */
    xUWB_Semaphore = xSemaphoreCreateBinary();
    xKey_Semaphore = xSemaphoreCreateBinary();

    /* 初始化按键 EXTI 中断 */
    key_exti_init();

    /* 创建 CAN TX 任务 (独立发送) */
    xTaskCreate(vTaskCanTX, "CANTX", 256, NULL, tskIDLE_PRIORITY + 4, NULL);

    /* 创建 UWB 任务 (最高优先级) */
    xTaskCreate(vTaskUWB, "UWB", TASK_STACK_UWB, NULL, TASK_PRIO_UWB, &xTaskUWB_Handle);

    /* 创建 WiFi 任务 */
    xTaskCreate(vTaskWiFi, "WiFi", TASK_STACK_WIFI, NULL, TASK_PRIO_WIFI, &xTaskWiFi_Handle);

    /* 创建 Key 任务 */
    xTaskCreate(vTaskKey, "Key", TASK_STACK_KEY, NULL, TASK_PRIO_KEY, &xTaskKey_Handle);
}
