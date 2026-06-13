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
//#include "ble.h"  /* 暂未加入 Keil 工程 */

#include <stdio.h>

/* ── 调试开关 ── */
//#define UWB_CSV_OUTPUT      /* 取消注释 = 串口输出 CIR CSV (Python 采集用) */

/* ====================================================================
 *  Mode switch
 * ====================================================================*/
typedef enum { MODE_RANGING, MODE_RADAR } sys_mode_t;
static sys_mode_t g_mode = MODE_RADAR;

/* ── 车辆控制状态 ── */
uint8_t  g_car_lock = 0;        /* 车锁: 0=关, 1=开 */
uint8_t  g_brake = 0;           /* 刹车: 0=松开, 1=踩下 */
uint8_t  g_parking_brake = 0;   /* 手刹: 0=放下, 1=拉上 */
uint8_t  g_system_status = 0;   /* 系统状态: 0=正常 */
float    g_key_distance = 99.0f;/* 车钥匙距离(m) */
uint8_t  g_ble_connected = 0;   /* BLE连接: 0=断开, 1=连接 */

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
    key3_init();  /* 刹车按键 PL4 */
    key4_init();  /* 手刹按键 PL5 */
    /* ble_init(); — BLE 暂未加入 master 工程 */

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

    /* 每30秒检查一次 */
    if (++cnt < 600) return; cnt = 0;

    /* ── UWB2 ID 检测 ── */
    uint32_t id = uwb_check_id();
    uint8_t  uwb_ok = (id == 0xDECA0302U || id == 0xDECA0312U);
    if (!uwb_ok && last_uwb_ok) {
        can_diag_send_error(CAN_ERR_UWB, CAN_ERR_UWB_ID);
        g_system_status = 1;  /* UWB异常 */
        printf("[SYS] !! UWB2 ID: 0x%08lX\r\n", (unsigned long)id);
    }
    if (uwb_ok && !last_uwb_ok) {
        can_diag_send_error(0, 0);
        g_system_status = 0;  /* 恢复正常 */
        printf("[SYS] OK UWB2 restored\r\n");
    }
    last_uwb_ok = uwb_ok;

    /* ── WiFi 状态同步 ── */
    if (!wifi_sm_ready()) {
        g_system_status = 2;  /* WiFi断连 */
    } else if (g_system_status == 2) {
        g_system_status = 0;  /* WiFi恢复 */
    }
}

/* ====================================================================
 *  quick_uwb_check — 快速检测UWB状态 (每次主循环都执行)
 * ====================================================================*/
static void quick_uwb_check(void)
{
    static uint32_t last_check_ms;
    static uint8_t  last_uwb_ok = 1;
    uint32_t now = uwb_tick_get();

    /* 每2秒快速检测一次 */
    if ((now - last_check_ms) < 2000) return;
    last_check_ms = now;

    uint32_t id = uwb_check_id();
    uint8_t  uwb_ok = (id == 0xDECA0302U || id == 0xDECA0312U);

    if (!uwb_ok && last_uwb_ok) {
        can_diag_send_error(CAN_ERR_UWB, CAN_ERR_UWB_ID);
        g_system_status = 1;
        printf("[SYS] !! UWB2 ID: 0x%08lX (quick)\r\n", (unsigned long)id);
    }
    if (uwb_ok && !last_uwb_ok) {
        can_diag_send_error(0, 0);
        g_system_status = 0;
        printf("[SYS] OK UWB2 restored (quick)\r\n");
    }
    last_uwb_ok = uwb_ok;
}

/* ====================================================================
 *  mode_ranging — DS-TWR Responder (UWB2 + STM32)
 * ====================================================================*/
static uint8_t  rng_inited;
static uint32_t rng_cnt;
static uint32_t rng_fail_cnt;
static void mode_ranging(void)
{
    double dist;
    int ret;

    if (!rng_inited) {
        if (uwb_ds_responder_init() != UWB_OK) {
            printf("[RNG] Init FAIL\r\n");
            can_diag_send_error(CAN_ERR_UWB, CAN_ERR_UWB_ID);
            return;
        }
        rng_inited = 1;
        printf("[RNG] Listening for STM32...\r\n");
    }

    ret = uwb_ds_responder_poll(&dist);
    if (ret == 1) {
        rng_cnt++;
        rng_fail_cnt = 0;  /* 成功则清零失败计数 */
        gd_eval_led_toggle(LED2);
        printf("[%lu] dist=%.3f m\r\n", (unsigned long)rng_cnt, dist);
        can_diag_send_ranging((float)dist);

        /* ── 更新车钥匙距离 ── */
        g_key_distance = (float)dist;

        /* ── 距离<2.5m 自动开锁 ── */
        if (g_key_distance < 2.5f && !g_car_lock) {
            g_car_lock = 1;
            printf("[LOGIC] Auto UNLOCK (dist=%.1f < 2.5m)\r\n", (double)g_key_distance);
        }

        /* ── 手刹+距离>5m → 熄灭 ── */
        if (g_parking_brake && g_key_distance > 5.0f) {
            can_diag_send_vehicle_cmd(VEHICLE_CMD_STOP);
            printf("[LOGIC] STOP: parking brake + dist>5m\r\n");
        }
    } else {
        /* UWB 通信失败 */
        rng_fail_cnt++;
        /* 连续失败10次 → 报告错误 */
        if (rng_fail_cnt == 10) {
            can_diag_send_error(CAN_ERR_UWB, CAN_ERR_UWB_ID);
            g_system_status = 1;
            printf("[RNG] !! UWB comm fail, reported to CAN\r\n");
        }
        /* 连续失败30次 → 尝试重新初始化 */
        if (rng_fail_cnt >= 30) {
            printf("[RNG] Reinit UWB...\r\n");
            rng_inited = 0;
            rng_fail_cnt = 0;
        }
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
            static uint32_t last_print_ms;

            if (human != last_state) {
                last_state = human;
                can_diag_send_radar(human, (uint8_t)(prob * 100.0f));
            }

            /* 状态变化或每 4s 打印一次 */
            uint32_t now = uwb_tick_get();
            if ((int32_t)(now - last_print_ms) >= 4000) {
                last_print_ms = now;
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
    static uint8_t last_buzzer = 0xFF;

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

        /* ── 按键 (每次循环都检测) ── */
        key1_poll();
        key2_poll();
        key3_poll();
        key4_poll();

        /* ── BLE 连接状态 ── */
        g_ble_connected = 1;  /* ble_is_connected() 暂未加入 */

        /* ── 蜂鸣器状态变化 → CAN 同步到 GD32F3 ── */
        if (BUZZER_Status != last_buzzer) {
            can_diag_send_buzzer(BUZZER_Status);
            last_buzzer = BUZZER_Status;
        }

        /* ── 车锁+刹车 → 启动 ── */
        if (g_car_lock && key3_pressed()) {
            can_diag_send_vehicle_cmd(VEHICLE_CMD_START);
            printf("[LOGIC] START: car lock + brake\r\n");
        }

        /* ── 快速UWB检测 (每2秒) ── */
        quick_uwb_check();

        /* ── 当前模式 ── */
        if (g_mode == MODE_RANGING) mode_ranging(); else mode_radar();

        /* ── 健康检测 (每30秒) ── */
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
