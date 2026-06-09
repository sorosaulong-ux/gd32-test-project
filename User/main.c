/*!
 *  \file    main.c
 *  \brief   GD32A7 整车诊断终端 — 测距/雷达 + WiFi + CAN
 *
 *  KEY1 (PC13): 蜂鸣器  |  KEY2 (GPIOL.3): 测距/雷达切换
 *  ESP8266 → OneNET 云  |  CAN2 0x103 → GD32F3 中控
 *  UWB1(TX) + UWB2(RX) 双模块雷达 (bistatic)
 *  UWB2 单模块 DS-TWR 测距 (与 STM32 配合)
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

#include <stdio.h>

#define ESP8266_ONENET_INFO  "AT+CIPSTART=\"TCP\",\"mqtts.heclouds.com\",1883\r\n"

/* ── 调试开关 ── */
//#define UWB_CSV_OUTPUT      /* 取消注释 = 串口输出 CIR CSV (Python 采集用) */

/* ====================================================================
 *  Mode switch
 * ====================================================================*/
typedef enum { MODE_RANGING, MODE_RADAR } sys_mode_t;
static sys_mode_t g_mode = MODE_RADAR;   /* 默认雷达(ML检测) */

/* ====================================================================
 *  KEY2 (GPIOL.3) — 模式切换 (长按500ms)
 * ====================================================================*/
static void key2_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOL);
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_3);
}

static void key2_poll(void)
{
    static uint32_t last_ms;
    static uint8_t  last_state = 1;
    uint32_t now = uwb_tick_get();
    uint8_t  cur = gpio_input_bit_get(GPIOL, GPIO_PIN_3);

    if (last_state == 1 && cur == 0 && (now - last_ms) > 500) {
        last_ms = now;
        if (g_mode == MODE_RANGING) {
            g_mode = MODE_RADAR;
            printf("\r\n=== SWITCH: RADAR mode ===\r\n");
        } else {
            g_mode = MODE_RANGING;
            printf("\r\n=== SWITCH: RANGING mode ===\r\n");
        }
    }
    last_state = cur;
}

/* ====================================================================
 *  system_init
 * ====================================================================*/
static void system_init(void)
{
    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);

    uart_init();
    key1_init();
    key2_init();

    printf("\r\n=== GD32A7 Vehicle Terminal ===\r\n\r\n");
}

/* ====================================================================
 *  network_init — WiFi → OneNET → CAN → Buzzer
 * ====================================================================*/
static void network_init(void)
{
    printf("[SYS] ESP8266 WiFi...\r\n");
    ESP8266_Init();

    printf("[SYS] MQTT connect...\r\n");
    while (ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
        delay_ms(500);

    printf("[SYS] Device login...\r\n");
    while (OneNet_DevLink()) {
        ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT");
        delay_ms(500);
    }
    OneNET_Subscribe();

    can_diag_init();
    BUZZER_Init();

    printf("[SYS] Ready — WiFi+OneNET+CAN OK\r\n\r\n");
    can_diag_send_error(CAN_ERR_SYSTEM, CAN_ERR_SYS_BOOT);
}

/* ====================================================================
 *  health_check — 每 30s 自检 WiFi + UWB2 ID
 * ====================================================================*/
static void health_check(void)
{
    static uint16_t cnt;
    static uint8_t  last_wifi_ok = 1;
    static uint8_t  last_uwb_ok  = 1;

    if (++cnt < 600) return; cnt = 0;

    uint8_t wifi_ok = (ESP8266_SendCmd("AT\r\n", "OK") == 0);
    if (!wifi_ok && last_wifi_ok) {
        can_diag_send_error(CAN_ERR_ESP8266, CAN_ERR_ESP_WIFI);
        printf("[SYS] !! WiFi lost\r\n");
    }
    if (wifi_ok && !last_wifi_ok) {
        can_diag_send_error(0, 0);
        printf("[SYS] OK WiFi restored\r\n");
    }
    last_wifi_ok = wifi_ok;

    uint32_t id = uwb_check_id();
    uint8_t  uwb_ok = (id == 0xDECA0302U || id == 0xDECA0312U);
    if (!uwb_ok && last_uwb_ok) {
        can_diag_send_error(CAN_ERR_UWB, CAN_ERR_UWB_ID);
        printf("[SYS] !! UWB2 ID: 0x%08lX\r\n", (unsigned long)id);
    }
    if (uwb_ok && !last_uwb_ok) {
        can_diag_send_error(0, 0);
        printf("[SYS] OK UWB2 restored\r\n");
    }
    last_uwb_ok = uwb_ok;
}

/* ====================================================================
 *  mode_ranging — DS-TWR Responder (UWB2 + STM32)
 * ====================================================================*/
static uint8_t  rng_inited;
static uint32_t rng_cnt;
static void mode_ranging(void)
{
    double dist;

    if (!rng_inited) {
        if (uwb_ds_responder_init() != UWB_OK) { printf("[RNG] Init FAIL\r\n"); return; }
        rng_inited = 1;
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

/* ====================================================================
 *  mode_radar — Bistatic radar (UWB1 TX + UWB2 RX)
 * ====================================================================*/
static uint8_t  rad_inited;
static uint32_t rad_seq;
static void mode_radar(void)
{
    uwb_radar_result_t res;

    if (!rad_inited) {
        if (uwb_radar_init() != UWB_OK) { printf("[RADAR] Init FAIL\r\n"); return; }
        rad_inited = 1;
#ifdef UWB_CSV_OUTPUT
        uwb_radar_csv(NULL);
#endif
        printf("[RADAR] Bistatic radar running...\r\n");
    }

    int ret = uwb_radar_scan(&res, rad_seq, "radar");
    if (ret == UWB_OK) {
        gd_eval_led_toggle(LED3);
        rad_seq++;

#ifdef UWB_CSV_OUTPUT
        uwb_radar_csv(&res);
#endif

        /* ── ML 推理: 空车(0) vs 有人(1) ── */
        float prob;
        int   human = ml_predict(&res, &prob);

        {
            static int last_state = -1;
            static uint16_t tick;

            /* 状态变化 → CAN 上报 */
            if (human != last_state) {
                last_state = human;
                can_diag_send_radar(human, (uint8_t)(prob * 100.0f));
            }

            /* 每 1 秒打印一次当前状态 (4 帧 × 250ms) */
            if (++tick >= 4) {
                tick = 0;
                printf("[DETECT] %s  p=%.2f\r\n",
                       human ? "HUMAN" : "EMPTY", prob);
            }
        }
    } else {
#ifndef UWB_CSV_OUTPUT
        /* 非 CSV 模式不打印超时点 — 保持终端干净 */
#endif
        ;
    }
}

/* ====================================================================
 *  main_loop
 * ====================================================================*/
static void main_loop(void)
{
    unsigned short tick_5s = 0;

    while (1) {
        /* OneNET 上报 5s */
        if (++tick_5s >= 100) { tick_5s = 0;
            OneNet_SendData(); ESP8266_Clear();
        }

        /* 云平台指令 */
        { unsigned char *p = ESP8266_GetIPD(0); if (p) OneNet_RevPro(p); }

        /* 按键 */
        key1_poll();
        key2_poll();

        /* 当前模式 */
        if (g_mode == MODE_RANGING) mode_ranging(); else mode_radar();

        /* 健康检测 */
        health_check();

        delay_ms(50);
    }
}

/* ====================================================================
 *  main
 * ====================================================================*/
int main(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
    systick_config();

    system_init();
    network_init();

    gd_eval_led_on(LED1);
    main_loop();
}
