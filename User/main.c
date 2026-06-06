/*!
 *  \file    main.c
 *  \brief   GD32A7 — 编译开关切换
 *           MODE_RANGING  = DS-TWR Responder (与 STM32 KEY 配套测距)
 *           MODE_RADAR_RX = 雷达接收器 (与 STM32 RADAR_TX 配套车内检测)
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "systick.h"
#include "uart.h"
#include "uwb.h"
#include <stdio.h>

/* ====================================================================
 *  ★ 编译开关 — 改这里切换模式 ★
 * ====================================================================*/
//#define MODE_RANGING         /* 测距 */
#define MODE_RADAR_RX      /* 雷达接收 */

static void cache_enable(void);

/* ====================================================================
 *  MODE_RANGING — DS-TWR Responder
 * ====================================================================*/
#ifdef MODE_RANGING
static void mode_ranging(void)
{
    int      ret;
    uint32_t cnt = 0;
    double   dist;

    printf("\r\n=== GD32A7 DS-TWR Responder ===\r\n\r\n");

    ret = uwb_ds_responder_init();
    if (ret != UWB_OK) {
        printf("[FAIL] uwb_ds_responder_init error %d\r\n", ret);
        while (1) { gd_eval_led_toggle(LED1); delay_1ms(200); }
    }

    printf("Waiting for STM32 Initiator...\r\n\r\n");

    while (1) {
        cnt++;
        ret = uwb_ds_responder_poll(&dist);
        if (ret == 1) {
            gd_eval_led_toggle(LED2);
            printf("[%lu] dist=%.3f m\r\n", (unsigned long)cnt, dist);
        }
        delay_1ms(50);
    }
}
#endif /* MODE_RANGING */

/* ====================================================================
 *  MODE_RADAR_RX — 雷达接收 (接收 STM32 的雷达脉冲 → CIR → CSV)
 * ====================================================================*/
#ifdef MODE_RADAR_RX
static void mode_radar_rx(void)
{
    uint32_t          seq = 0;
    uwb_radar_result_t res;

    printf("\r\n=== GD32A7 Radar RX ===\r\n\r\n");

    if (uwb_radar_rx_init() != UWB_OK) {
        printf("[FAIL] uwb_radar_rx_init\r\n");
        while (1) { gd_eval_led_toggle(LED1); delay_1ms(200); }
    }

    printf("Waiting for STM32 RADAR_TX pulses...\r\n");
    printf("(Output: CSV format for Python analysis)\r\n\r\n");

    /* Print CSV header */
    uwb_radar_csv(NULL);

    while (1) {
        int ret = uwb_radar_rx_scan(&res, seq, "rx");
        if (ret == UWB_OK) {
            gd_eval_led_toggle(LED2);
            uwb_radar_csv(&res);
            seq++;
        } else {
            /* Timeout — print dot on its own line */
            printf(".\r\n");
        }
    }
}
#endif /* MODE_RADAR_RX */

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
#else
#error "Please define MODE_RANGING or MODE_RADAR_RX"
#endif
}

static void cache_enable(void) {
    SCB_EnableICache();
    SCB_EnableDCache();
}
