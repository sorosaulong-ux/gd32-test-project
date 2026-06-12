/*!
 *  \file    main.c
 *  \brief   GD32A7 Vehicle Terminal — FreeRTOS version
 *
 *  Tasks: Init → creates Radar/Ranging task, WiFi task
 *  KEY1: buzzer | KEY2: mode switch
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "systick.h"
#include "uart.h"
#include "delay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "esp8266.h"
#include "onenet.h"
#include "buzzer.h"
#include "key.h"
#include "can_diag.h"
#include "uwb.h"
#include "uwb_port.h"
#include "ml_predict.h"

#include <stdio.h>

/* ── 调试开关 ── */
//#define UWB_CSV_OUTPUT

/* ── Task priorities ── */
#define INIT_PRIO     ( tskIDLE_PRIORITY + 3 )
#define RADAR_PRIO    ( tskIDLE_PRIORITY + 2 )
#define WIFI_PRIO     ( tskIDLE_PRIORITY + 1 )

/* ── Mode ── */
typedef enum { MODE_RANGING, MODE_RADAR } sys_mode_t;
static sys_mode_t g_mode = MODE_RADAR;

/* ── Queue handle — mode change from KEY2 ISR → radar task ── */
QueueHandle_t g_mode_queue;

/* ====================================================================
 *  KEY2 (GPIOL.3) — 模式切换
 * ====================================================================*/
static void key2_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOL);
    prpu_periph_unlock(PRPU_GPIOL);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_3);
}

static void key2_poll(void)
{
    static uint32_t last_ms;
    static int8_t  last_state = -1;

    if (last_state < 0) {
        last_state = (int8_t)gpio_input_bit_get(GPIOL, GPIO_PIN_3);
        return;
    }

    uint8_t cur = gpio_input_bit_get(GPIOL, GPIO_PIN_3);
    if (last_state == 1 && cur == 0 && (xTaskGetTickCount() - last_ms) > pdMS_TO_TICKS(500)) {
        last_ms = xTaskGetTickCount();
        g_mode = (g_mode == MODE_RANGING) ? MODE_RADAR : MODE_RANGING;
        printf("\r\n=== SWITCH: %s mode ===\r\n",
               (g_mode == MODE_RADAR) ? "RADAR" : "RANGING");
    }
    last_state = cur;
}

/* ====================================================================
 *  Radar/Ranging 任务 (主业务)
 * ====================================================================*/
static void radar_task(void *pv)
{
    uint8_t  rad_inited = 0;
    uint8_t  rng_inited = 0;
    uint32_t rad_seq = 0;
    uint32_t rng_cnt = 0;

    for (;;) {
        if (g_mode == MODE_RADAR) {
            /* ── Bistatic radar ── */
            uwb_radar_result_t res;

            if (!rad_inited) {
                if (uwb_radar_init() != UWB_OK) {
                    printf("[RADAR] Init FAIL\r\n");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
                rad_inited = 1;
                rng_inited = 0;
#ifdef UWB_CSV_OUTPUT
                uwb_radar_csv(NULL);
#endif
                printf("[RADAR] Running...\r\n");
            }

            int ret = uwb_radar_scan(&res, rad_seq, "radar");
            if (ret == UWB_OK) {
                gd_eval_led_toggle(LED3);
                rad_seq++;

#ifdef UWB_CSV_OUTPUT
                uwb_radar_csv(&res);
#else
                {
                    float prob;
                    int   human = ml_predict(&res, &prob);
                    static int last_state = -1;
                    static uint32_t last_print;

                    if (human != last_state) {
                        last_state = human;
                        can_diag_send_radar(human, (uint8_t)(prob * 100.0f));
                    }

                    if ((TickType_t)(xTaskGetTickCount() - last_print) >= pdMS_TO_TICKS(4000)) {
                        last_print = xTaskGetTickCount();
                        printf("[DETECT] %s  p=%.2f\r\n",
                               human ? "HUMAN" : "EMPTY", prob);
                    }
                }
#endif
            }
        } else {
            /* ── DS-TWR Ranging ── */
            double dist;

            if (!rng_inited) {
                if (uwb_ds_responder_init() != UWB_OK) {
                    printf("[RNG] Init FAIL\r\n");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
                rng_inited = 1;
                rad_inited = 0;
                printf("[RNG] Listening for STM32...\r\n");
            }

            int ret = uwb_ds_responder_poll(&dist);
            if (ret == 1) {
                rng_cnt++;
                gd_eval_led_toggle(LED2);
                printf("[%lu] dist=%.3f m\r\n", (unsigned long)rng_cnt, dist);
                can_diag_send_ranging((float)dist);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ====================================================================
 *  WiFi + CAN + Keys 任务
 * ====================================================================*/
static void wifi_task(void *pv)
{
    unsigned short tick_5s = 0;

    for (;;) {
        /* WiFi 状态机 */
        wifi_sm_tick();

        /* OneNET (仅 WiFi OK) */
        if (wifi_sm_ready()) {
            if (++tick_5s >= 100) { tick_5s = 0;
                OneNet_SendData(); ESP8266_Clear();
            }
            { unsigned char *p = ESP8266_GetIPD(0); if (p) OneNet_RevPro(p); }
        }

        /* 按键 */
        key1_poll();
        key2_poll();

        /* 健康检测 (30s UWB ID) */
        {
            static uint16_t hc_cnt;
            static uint8_t  last_uwb_ok = 1;
            if (++hc_cnt >= 600) { hc_cnt = 0;
                uint32_t id = uwb_check_id();
                uint8_t  ok = (id == 0xDECA0302U || id == 0xDECA0312U);
                if (!ok && last_uwb_ok) {
                    can_diag_send_error(CAN_ERR_UWB, CAN_ERR_UWB_ID);
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
 *  Init 任务 — 上电初始化后删除自身
 * ====================================================================*/
static void init_task(void *pv)
{
    SCB_EnableICache();
    SCB_EnableDCache();

    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);

    uart_init();
    BUZZER_Init();
    key1_init();
    key2_init();

    printf("\r\n=== GD32A7 Vehicle Terminal (FreeRTOS) ===\r\n\r\n");

    can_diag_init();
    printf("[SYS] CAN+Buzzer ready, WiFi in background...\r\n\r\n");
    wifi_sm_start();  /* 非阻塞 WiFi */
    can_diag_send_error(CAN_ERR_SYSTEM, CAN_ERR_SYS_BOOT);

    gd_eval_led_on(LED1);

    /* 创建主业务任务 */
    xTaskCreate(radar_task, "Radar", configMINIMAL_STACK_SIZE * 4, NULL, RADAR_PRIO, NULL);
    xTaskCreate(wifi_task, "WiFi",  configMINIMAL_STACK_SIZE * 4, NULL, WIFI_PRIO, NULL);

    /* 删除自身 */
    vTaskDelete(NULL);
}

/* ====================================================================
 *  main — FreeRTOS 启动
 * ====================================================================*/
int main(void)
{
    /* 配置 4 bits 抢占优先级 (FreeRTOS 要求) */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    xTaskCreate(init_task, "Init", configMINIMAL_STACK_SIZE * 4, NULL, INIT_PRIO, NULL);

    vTaskStartScheduler();

    while (1) {}
}
