/*!
 *  \file    main.c
 *  \brief   GD32A7 — WiFi+OneNET+CAN+Buzzer 整车诊断终端
 *
 *  KEY1 本地控制蜂鸣器 | OneNET 远程控制蜂鸣器
 *  CAN2 0x103 系统错误上报 | UWB 测距 (后续加入)
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
#include <stdio.h>

#define ESP8266_ONENET_INFO  "AT+CIPSTART=\"TCP\",\"mqtts.heclouds.com\",1883\r\n"

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

    printf("\r\n=== GD32A7 Vehicle Terminal ===\r\n\r\n");
}

/* ====================================================================
 *  network_init — WiFi → OneNET → CAN → Buzzer
 * ====================================================================*/
static void network_init(void)
{
    printf("[SYS] ESP8266 WiFi connecting...\r\n");
    ESP8266_Init();

    printf("[SYS] MQTT connecting...\r\n");
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

    printf("[SYS] Ready — WiFi OK | OneNET OK | CAN 500kbps\r\n\r\n");

    /* 通知 GD32F3: 启动完成 */
    can_diag_send_error(CAN_ERR_SYSTEM, CAN_ERR_SYS_BOOT);
}

/* ====================================================================
 *  sys_health_check — 每 30s AT ping 检测 WiFi
 * ====================================================================*/
static void sys_health_check(void)
{
    static uint16_t cnt;
    static uint8_t  last_wifi_ok = 1;
    static uint8_t  last_uwb_ok = 1;

    if (++cnt >= 600) { cnt = 0;   /* 600 × 50ms = 30s */

        /* ── WiFi 检测 ── */
        uint8_t wifi_ok = (ESP8266_SendCmd("AT\r\n", "OK") == 0);

        if (!wifi_ok && last_wifi_ok) {
            can_diag_send_error(CAN_ERR_ESP8266, CAN_ERR_ESP_WIFI);
            printf("[SYS] !! ESP8266 WiFi lost\r\n");
        }
        if (wifi_ok && !last_wifi_ok) {
            can_diag_send_error(0, 0);
            printf("[SYS] OK ESP8266 WiFi restored\r\n");
        }
        last_wifi_ok = wifi_ok;

        /* ── UWB2 检测 ── */
        uint32_t uwb_id = uwb_check_id();
        uint8_t  uwb_ok = (uwb_id == 0xDECA0302U || uwb_id == 0xDECA0312U);

        if (!uwb_ok && last_uwb_ok) {
            can_diag_send_error(CAN_ERR_UWB, CAN_ERR_UWB_ID);
            printf("[SYS] !! UWB2 ID error: 0x%08lX\r\n", (unsigned long)uwb_id);
        }
        if (uwb_ok && !last_uwb_ok) {
            can_diag_send_error(0, 0);
            printf("[SYS] OK UWB2 restored (0x%08lX)\r\n", (unsigned long)uwb_id);
        }
        last_uwb_ok = uwb_ok;
    }
}

/* ====================================================================
 *  main_loop
 * ====================================================================*/
static void main_loop(void)
{
    unsigned short tick_5s = 0;

    while (1) {
        /* ── OneNET 上报 5s ── */
        if (++tick_5s >= 100) { tick_5s = 0;
            OneNet_SendData();
            ESP8266_Clear();
        }

        /* ── 云平台指令 ── */
        {
            unsigned char *p = ESP8266_GetIPD(0);
            if (p) OneNet_RevPro(p);
        }

        /* ── 本地按键 ── */
        key1_poll();

        /* ── 系统健康检测 ── */
        sys_health_check();

        /* ── UWB (后续加入) ── */

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
