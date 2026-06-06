/*!
 *  \file    main.c
 *  \brief   GD32A7 — DS-TWR Responder (ex_05b official pattern)
 *
 *  Uses single DW3000 (UWB2) as DS-TWR Responder.
 *  STM32F103 runs DS-TWR Initiator.
 *  Distance computed and displayed on GD32A7 serial.
 */

#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "systick.h"
#include "uart.h"
#include "uwb.h"
#include <stdio.h>

static void cache_enable(void);

int main(void)
{
    int      ret;
    uint32_t cnt = 0;
    double   dist;

    cache_enable();
    systick_config();

    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);

    uart_init();
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

static void cache_enable(void) {
    SCB_EnableICache();
    SCB_EnableDCache();
}
