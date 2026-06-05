/*!
 *  \file    main.c
 *  \brief   UWB Radar CIR — CH5 PLEN=128, change LABEL below per scene
 *
 *  Edit LABEL, recompile, record 100+ frames.
 *  Then run: python cir_diff.py empty.csv book.csv
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "systick.h"
#include "uart.h"
#include "uwb.h"
#include "uwb_port.h"
#include <stdio.h>

/* ─────── CHANGE THIS PER SCENE ─────── */
#define LABEL  "human"
/* ───────────────────────────────────── */

static void cache_enable(void);

int main(void)
{
    int      ret;
    uint32_t seq    = 0;
    uint32_t t_next;
    uwb_radar_result_t res;

    cache_enable();
    systick_config();

    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);

    uart_init();
    printf("\r\n============================================\r\n");
    printf(" UWB Radar CIR  CH5 PLEN=128\r\n");
    printf(" Label: %s\r\n", LABEL);
    printf("============================================\r\n\r\n");

    ret = uwb_radar_init();
    if (ret != UWB_OK) {
        printf("[FATAL] init failed code=%d\r\n", ret);
        while (1) { gd_eval_led_toggle(LED1); delay_1ms(200); }
    }
    printf("[INIT] OK.\r\n\r\n");

    uwb_radar_csv(NULL);
    t_next = uwb_tick_get();

    while (1) {
        while (uwb_tick_get() < t_next) { /* spin */ }
        t_next += UWB_RADAR_TX_INTERVAL_MS;

        ret = uwb_radar_scan(&res, seq, LABEL);
        seq++;

        if (ret == UWB_OK) gd_eval_led_toggle(LED2);
        else               gd_eval_led_toggle(LED3);

        uwb_radar_csv(&res);
    }
}

static void cache_enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}
