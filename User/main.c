/*!
 *  \file    main.c
 *  \brief   GD32A7 — WiFi+OneNET+CAN+Buzzer 整车诊断终端
 *
 *  KEY1 本地控制蜂鸣器 | OneNET 远程控制蜂鸣器
 *  CAN2 心跳上报 | UWB 测距 (后续加入)
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

#include <stdio.h>

/* ====================================================================
 *  常量
 * ====================================================================*/
#define ESP8266_ONENET_INFO  "AT+CIPSTART=\"TCP\",\"mqtts.heclouds.com\",1883\r\n"

/* ====================================================================
 *  system_init — 所有硬件一次性初始化
 * ====================================================================*/
static void system_init(void)
{
    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);

    uart_init();             /* debug printf → PH7/PH8 */
    key1_init();             /* PC13 按键 */
    /* UWB + Buzzer 后续加入 */

    printf("\r\n=== GD32A7 Vehicle Terminal ===\r\n\r\n");
}

/* ====================================================================
 *  network_init — WiFi 连接 + OneNET 登录 + CAN 启动
 * ====================================================================*/
static void network_init(void)
{
    /* 1. ESP8266 + WiFi */
    printf("[SYS] ESP8266 WiFi connecting...\r\n");
    ESP8266_Init();

    /* 2. MQTT 服务器 */
    printf("[SYS] MQTT connecting...\r\n");
    while (ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
        delay_ms(500);

    printf("[SYS] Device login...\r\n");
    while (OneNet_DevLink()) {
        ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT");
        delay_ms(500);
    }
    OneNET_Subscribe();

    /* 3. CAN */
    can_diag_init();

    /* 4. 蜂鸣器 — 最后初始化，确保不被其他模块干扰 */
    BUZZER_Init();

    printf("[SYS] Ready — WiFi OK | OneNET OK | CAN 500kbps\r\n\r\n");
}

/* ====================================================================
 *  main_loop — 5 任务轮询, 50ms 周期
 * ====================================================================*/
static void main_loop(void)
{
    unsigned short tick_5s = 0;

    while (1) {
        /* ── CAN 心跳 1Hz ── */
        {
            static uint16_t cnt;
            if (++cnt >= 20) { cnt = 0;
                can_diag_send_heartbeat(0, 0);
                gd_eval_led_toggle(LED2);
            }
        }

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

        /* ── CAN 状态 (每 10s 打印一次) ── */
        {
            static uint16_t cnt;
            if (++cnt >= 200) { cnt = 0;
                can_diag_print_status();
            }
        }

        /* ── UWB 测距 (后续) ── */

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

    gd_eval_led_on(LED1);    /* 系统就绪 */
    main_loop();
}
