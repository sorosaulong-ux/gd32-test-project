/*!
 *  \file    main.c
 *  \brief   DW3000 UWB test — initialise dual modules, run ranging
 *
 *  SPI:  PE6(SCK) PE5(MISO) PH14(MOSI)   [software bit‑bang]
 *  UWB1: PL2(CS) PI8(IRQ) PL0(RST)
 *  UWB2: PB11(CS) PI10(IRQ) PI11(RST)
 *  WAKEUP: PB10
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
    cache_enable();
    systick_config();

    /* LEDs */
    gd_eval_led_init(LED1); gd_eval_led_off(LED1);
    gd_eval_led_init(LED2); gd_eval_led_off(LED2);
    gd_eval_led_init(LED3); gd_eval_led_off(LED3);
    gd_eval_led_init(LED4); gd_eval_led_off(LED4);

    /* UART */
    uart_init();
    printf("\r\n=== DW3000 UWB Test ===\r\n");
    printf(" Initialising both UWB modules...\r\n\r\n");

    /* UWB init */
    if (uwb_init() != UWB_OK) {
        printf("[FAIL] No DW3000 devices found!\r\n");
        printf(" Check wiring: SPI pins, CS, RST, power (3.3V).\r\n");
        while (1) { gd_eval_led_toggle(LED1); delay_1ms(200); }
    }

    printf("[OK] Both modules ready.\r\n");
    printf(" Running ranging every 2 seconds...\r\n\r\n");

    while (1) {
        double dist;
        int    ret = uwb_ranging(&dist);

        if (ret == UWB_OK) {
            printf("[RNG] Distance = %.3f m\r\n", dist);
            gd_eval_led_toggle(LED2);
        } else if (ret == UWB_ERR_TIMEOUT) {
            printf("[RNG] Timeout — check antenna alignment\r\n");
            gd_eval_led_toggle(LED3);
        } else {
            printf("[RNG] Error code %d\r\n", ret);
            gd_eval_led_toggle(LED4);
        }

        delay_1ms(2000);
    }
}

static void cache_enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}
