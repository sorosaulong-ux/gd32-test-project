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

///* ── 调试开关 ── */
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
    prpu_periph_unlock(PRPU_GPIOL);  /* GPIOL 需要解锁 */
    gpio_mode_set(GPIOL, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_3);
}

static void key2_poll(void)
{
    static uint32_t last_ms;
    static int8_t  last_state = -1;  /* -1 = 首次, 从实际引脚读取 */
    uint32_t now = uwb_tick_get();
    uint8_t  cur = gpio_input_bit_get(GPIOL, GPIO_PIN_3);

    /* 首次调用 — 用实际引脚电平初始化, 避免浮空误触发 */
    if (last_state < 0) { last_state = (int8_t)cur; return; }

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
 *  network_init — CAN + Buzzer (立即完成), WiFi 后台状态机启动
 * ====================================================================*/
static void network_init(void)
{
    can_diag_init();
    BUZZER_Init();

    printf("[SYS] CAN+Buzzer ready, WiFi connecting in background...\r\n\r\n");
    wifi_sm_start();  /* 非阻塞状态机 — main_loop 驱动 */
    can_diag_send_error(CAN_ERR_SYSTEM, CAN_ERR_SYS_BOOT);
}

/* ====================================================================
 *  health_check — 每 30s 自检 WiFi + UWB2 ID
 * ====================================================================*/
static void health_check(void)
{
    static uint16_t cnt;
    static uint8_t  last_uwb_ok  = 1;

    if (++cnt < 600) return; cnt = 0;

    /* ── UWB2 ID 检测 ── */
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

    /* WiFi 错误由 esp8266.c 状态机自动上报 (WIFI_SM_DEAD → CAN_ERR_ESP_WIFI) */
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
        uwb_radar_csv(NULL);               /* CSV 模式: 打印 header */
#endif
        printf("[RADAR] Bistatic radar running...\r\n");
    }

    int ret = uwb_radar_scan(&res, rad_seq, "radar");
    if (ret == UWB_OK) {
        gd_eval_led_toggle(LED3);
        rad_seq++;

#ifdef UWB_CSV_OUTPUT
        /* ── CSV 采集模式: 只输出 CIR 原始数据, 不做 ML ── */
        uwb_radar_csv(&res);
#else
        /* ── 检测模式: ML 推理 + 串口打印, 不输出 CSV ── */
        {
            float prob;
            int   human = ml_predict(&res, &prob);
            static int last_state = -1;
            static uint16_t tick;

            if (human != last_state) {
                last_state = human;
                can_diag_send_radar(human, (uint8_t)(prob * 100.0f));
            }

            /* 16 帧(4s)刷新一次 — 和模型窗口对齐 */
            if (++tick >= 16) {
                tick = 0;
                printf("[DETECT] %s  p=%.2f\r\n",
                       human ? "HUMAN" : "EMPTY", prob);
            }
        }
#endif
    }
}

/* ====================================================================
 *  main_loop
 * ====================================================================*/
static void main_loop(void)
{
    unsigned short tick_5s = 0;

    while (1) {
        /* ── WiFi 状态机 (非阻塞, <1ms) ── */
        wifi_sm_tick();

        /* ── OneNET 上报 + 云指令 (仅 WiFi OK 时) ── */
        if (wifi_sm_ready()) {
            if (++tick_5s >= 100) { tick_5s = 0;
                OneNet_SendData(); ESP8266_Clear();
            }
            { unsigned char *p = ESP8266_GetIPD(0); if (p) OneNet_RevPro(p); }
        }

        /* ── 按键 ── */
        key1_poll();
        key2_poll();

        /* ── 当前模式 ── */
        if (g_mode == MODE_RANGING) mode_ranging(); else mode_radar();

        /* ── 健康检测 ── */
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
