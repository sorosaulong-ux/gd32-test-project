/*!
 *  \file    main.c
 *  \brief   GD32A7 — WiFi + OneNET MQTT + Buzzer + CAN
 *
 *  UWB 暂时注释，后续加入。
 *  KEY1 (PC13) 本地控制蜂鸣器。
 *  ESP8266 (UART3 PJ7/PJ8) 连 OneNET 云，远程控制蜂鸣器。
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "systick.h"
#include "uart.h"

/* ── 协议/设备 ── */
#include "onenet.h"
#include "esp8266.h"

/* ── 硬件 ── */
#include "delay.h"
#include "usart_esp.h"
#include "buzzer.h"
#include "can.h"
#include "can_diag.h"
#include "uwb_port.h"       /* uwb_tick_get */

#include <stdio.h>
#include <string.h>

/* ====================================================================
 *  ESP8266 → OneNET MQTT 服务器
 * ====================================================================*/
#define ESP8266_ONENET_INFO  "AT+CIPSTART=\"TCP\",\"mqtts.heclouds.com\",1883\r\n"

static void cache_enable(void);

/* ====================================================================
 *  KEY1 (PC13) — 本地按键控制蜂鸣器 (轮询 + 去抖)
 * ====================================================================*/
static void key1_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_13);
}

static void key1_poll(void)
{
    static uint32_t stable_ms;   /* LOW 持续时长 */
    static uint8_t  triggered;   /* 已触发标志 (防重复) */
    static uint8_t  last_state = 1;
    uint32_t now = uwb_tick_get();

    uint8_t cur = gpio_input_bit_get(GPIOC, GPIO_PIN_13);  /* LOW=pressed */

    if (cur == 0) {
        /* 按下 — 累计 LOW 持续时间 */
        if (last_state == 1) stable_ms = 0;  /* 刚按下，清零 */
        last_state = 0;
        if (!triggered && (now - stable_ms) > 500) {  /* ★ 500ms 长按才触发 */
            triggered = 1;
            if (BUZZER_Status == BUZZER_ON)
                BUZZER_Set(BUZZER_OFF);
            else
                BUZZER_Set(BUZZER_ON);
        }
    } else {
        /* 松开 — 恢复 */
        last_state = 1;
        triggered = 0;
        stable_ms = now;
    }
}

/* ====================================================================
 *  main
 * ====================================================================*/
int main(void)
{
    unsigned short timeCount = 0;
    unsigned char *dataPtr;

    cache_enable();
    systick_config();

    /* ── 硬件初始化 ── */
    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);

    uart_init();             /* debug printf → PH7/PH8 */
    key1_init();             /* PC13 按键轮询 */
    /* UWB 暂时注释 — 后续加入 */
    /* uwb_ds_responder_init(); */

    printf("\r\n=== GD32A7 WiFi+Buzzer+CAN ===\r\n\r\n");

    /* ── Phase 1: ESP8266 初始化 + 连 WiFi ── */
    printf("[MAIN] ESP8266 Init...\r\n");
    ESP8266_Init();

    /* ── Phase 2: 连 OneNET MQTT ── */
    printf("[MAIN] Connect MQTTs Server...\r\n");
    while (ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
        delay_ms(500);

    printf("[MAIN] Device login...\r\n");
    while (OneNet_DevLink()) {
        ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT");
        delay_ms(500);
    }

    OneNET_Subscribe();

    /* ── Phase 3: CAN init (500 kbps) ── */
    can_diag_init();

    /* ★ BUZZER_Init 最后调用 — 确保其他模块 (can_gpio_config 会开 RCU_GPIOB)
     * 不会在蜂鸣器初始化之后干扰 PB13 输出状态 */
    BUZZER_Init();           /* PB13 → ODR预装HIGH → OFF */

    printf("[MAIN] System ready!\r\n");
    printf("  - WiFi: connected\r\n");
    printf("  - OneNET: online\r\n");
    printf("  - CAN2: 500 kbps\r\n");
    printf("  - KEY1 (PC13): press to toggle buzzer\r\n\r\n");

    gd_eval_led_on(LED1);  /* LED1 常亮 = 系统就绪 */

    /* ================================================================
     *  主循环
     * ================================================================*/
    while (1) {
        /* ── 1. CAN 心跳 (每 1 秒) ── */
        static uint16_t can_hb_tick;
        if (++can_hb_tick >= 20) {   /* 20 × 50ms = 1s */
            can_hb_tick = 0;
            can_diag_send_heartbeat(0, 0);
            gd_eval_led_toggle(LED2);
        }

        /* ── 2. OneNET 数据上报 (每 5 秒) ── */
        if (++timeCount >= 100) {    /* 100 × 50ms = 5s */
            OneNet_SendData();
            timeCount = 0;
            ESP8266_Clear();
        }

        /* ── 3. 轮询云平台控制指令 → 远程控制蜂鸣器 ── */
        dataPtr = ESP8266_GetIPD(0);
        if (dataPtr != NULL) {
            OneNet_RevPro(dataPtr);
        }

        /* ── 4. KEY1 本地按键轮询 ── */
        key1_poll();

        /* ── 5. UWB 测距 (后续加入) ── */
        /* uwb_ds_responder_poll(&dist); */

        delay_ms(50);
    }
}

static void cache_enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}
