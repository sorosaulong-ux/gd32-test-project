/*!
 *  \file    main.c
 *  \brief   GD32A7 — 编译开关切换
 *           MODE_RANGING  = DS-TWR Responder + CAN
 *           MODE_RADAR_RX = 雷达接收 + CAN
 *           MODE_CAN_TEST = CAN 自测（心跳1Hz，PCAN-USB验证）
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "systick.h"
#include "uart.h"
#include "uwb.h"
#include "can_diag.h"
#include <stdio.h>

/* ====================================================================
 *  ★ 编译开关 — 改这里切换模式 ★
 * ====================================================================*/
//#define MODE_RANGING         /* 测距 + CAN */
//#define MODE_RADAR_RX        /* 雷达 + CAN */
#define MODE_CAN_TEST          /* CAN 自测 */

static void cache_enable(void);

/* ====================================================================
 *  CAN 心跳定时器 — 各模式共用
 *  每 1000ms 发一次心跳到 CAN 总线
 * ====================================================================*/
static void can_tick(uint8_t mode, uint16_t flags)
{
    static uint32_t last_ms;
    uint32_t now = uwb_tick_get();  /* ms tick from SysTick */
    if (now - last_ms >= 1000U) {
        last_ms = now;
        can_diag_send_heartbeat(mode, flags);
    }
}

/* ====================================================================
 *  MODE_RANGING — DS-TWR Responder + CAN
 * ====================================================================*/
#ifdef MODE_RANGING
static void mode_ranging(void)
{
    int      ret;
    uint32_t cnt = 0;
    double   dist;

    printf("\r\n=== GD32A7 DS-TWR Responder + CAN ===\r\n\r\n");

    ret = uwb_ds_responder_init();
    if (ret != UWB_OK) {
        printf("[FAIL] uwb_ds_responder_init error %d\r\n", ret);
        while (1) { gd_eval_led_toggle(LED1); delay_1ms(200); }
    }

    can_diag_init();

    printf("Waiting for STM32 Initiator...\r\n\r\n");

    while (1) {
        cnt++;
        ret = uwb_ds_responder_poll(&dist);
        if (ret == 1) {
            gd_eval_led_toggle(LED2);
            printf("[%lu] dist=%.3f m\r\n", (unsigned long)cnt, dist);
            can_diag_send_ranging((float)dist);
        }

        can_tick(CAN_HB_MODE_RANGING, 0);
        delay_1ms(50);
    }
}
#endif /* MODE_RANGING */

/* ====================================================================
 *  MODE_RADAR_RX — 雷达接收 + CAN
 * ====================================================================*/
#ifdef MODE_RADAR_RX
static void mode_radar_rx(void)
{
    uint32_t          seq = 0;
    uwb_radar_result_t res;

    printf("\r\n=== GD32A7 Radar RX + CAN ===\r\n\r\n");

    if (uwb_radar_rx_init() != UWB_OK) {
        printf("[FAIL] uwb_radar_rx_init\r\n");
        while (1) { gd_eval_led_toggle(LED1); delay_1ms(200); }
    }

    can_diag_init();

    printf("Waiting for STM32 RADAR_TX pulses...\r\n");
    printf("(Output: CSV format for Python analysis)\r\n\r\n");

    uwb_radar_csv(NULL);  /* print CSV header */

    while (1) {
        int ret = uwb_radar_rx_scan(&res, seq, "rx");
        if (ret == UWB_OK) {
            gd_eval_led_toggle(LED2);
            uwb_radar_csv(&res);
            seq++;
        } else {
            printf(".\r\n");
        }

        can_tick(CAN_HB_MODE_RADAR, 0);
    }
}
#endif /* MODE_RADAR_RX */

/* ====================================================================
 *  MODE_CAN_TEST — CAN 自测 (PCAN-USB 验证接线)
 *
 *  接线: GD32A7 CN4 → MCP2562FD CAN收发器 → PCAN-USB
 *        CAN_H → CAN_H, CAN_L → CAN_L, GND → GND
 *
 *  预期: PCAN-USB 收到 0x100 心跳帧，每 1s 一帧
 *        串口打印 [CAN] TX=1,2,3...
 *        如果 TEC 值上升 → 波特率不匹配或接线错误
 * ====================================================================*/
#ifdef MODE_CAN_TEST
static void mode_can_test(void)
{
    uint32_t cnt = 0;

    printf("\r\n=== GD32A7 CAN Test ===\r\n\r\n");

    can_diag_init();

    /* 发几条测试帧确认总线正常 */
    uint8_t test_data[8] = {0xCA, 0xFE, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    printf("Sending test frames...\r\n");
    for (int i = 0; i < 5; i++) {
        if (SUCCESS == can_send_std_frame(DTM_CAN2, 0x100, test_data, 8)) {
            printf("  [%d] OK\r\n", i + 1);
        } else {
            printf("  [%d] FAIL — check wiring! PM2=TX, PM3=RX\r\n", i + 1);
        }
        delay_1ms(200);
    }

    printf("\r\n=== CAN heartbeat loop (1 Hz) ===\r\n");
    printf("Connect PCAN-USB, set 500 kbps, listen for ID 0x100\r\n\r\n");

    while (1) {
        cnt++;
        can_tick(CAN_HB_MODE_IDLE, 0);

        /* Print status every 5 seconds */
        if ((cnt % 100) == 0) {
            can_diag_print_status();
        }

        /* Check for CAN commands from master */
        {
            uint8_t cmd = 0, param = 0;
            if (can_diag_poll_command(&cmd, &param)) {
                printf("[CAN] CMD rx: cmd=%u param=%u\r\n", cmd, param);
                gd_eval_led_toggle(LED2);
            }
        }

        delay_1ms(50);
    }
}
#endif /* MODE_CAN_TEST */

/* ====================================================================
 *  main
 * ====================================================================*/
int main(void)
{
    cache_enable();
    systick_config();

    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);

    uart_init();

#if defined(MODE_RANGING)
    mode_ranging();
#elif defined(MODE_RADAR_RX)
    mode_radar_rx();
#elif defined(MODE_CAN_TEST)
    mode_can_test();
#else
#error "Please define MODE_RANGING, MODE_RADAR_RX, or MODE_CAN_TEST"
#endif
}

static void cache_enable(void) {
    SCB_EnableICache();
    SCB_EnableDCache();
}
