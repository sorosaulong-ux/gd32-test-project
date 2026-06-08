/*!
 *  \file    esp8266.c
 *  \brief   ESP8266 AT-command driver — GD32A7 port
 *
 *  UART3 (PJ7/PJ8) @ 115200 → ESP8266
 *  Uses interrupt RX (LINFlexD3_IRQHandler in usart_esp.c)
 */

#include "gd32a7xx.h"
#include "esp8266.h"
#include "delay.h"
#include "usart_esp.h"
#include <string.h>
#include <stdio.h>

/* ─── WiFi credentials — CHANGE THESE ─── */
#define ESP8266_WIFI_INFO   "AT+CWJAP=\"aaa\",\"88888888\"\r\n"

void ESP8266_Clear(void)
{
    memset(esp8266_buf, 0, sizeof(esp8266_buf));
    esp8266_cnt = 0;
}

static int ESP8266_WaitRecive(void)
{
    if (esp8266_cnt == 0)       return REV_WAIT;
    if (esp8266_cnt == esp8266_cntPre) {
        esp8266_cnt = 0;
        return REV_OK;
    }
    esp8266_cntPre = esp8266_cnt;
    return REV_WAIT;
}

/* ─── Send AT command, wait for expected response ─── */
int ESP8266_SendCmd(char *cmd, char *res)
{
    unsigned char timeOut = 200;

    Usart_SendString(0, (unsigned char *)cmd, strlen(cmd));

    while (timeOut--) {
        if (ESP8266_WaitRecive() == REV_OK) {
            if (strstr((const char *)esp8266_buf, res) != NULL) {
                ESP8266_Clear();
                return 0;  /* success */
            }
        }
        delay_ms(10);
    }
    return 1;  /* timeout */
}

/* ─── Send data via CIPSEND ─── */
void ESP8266_SendData(unsigned char *data, unsigned short len)
{
    char cmdBuf[32];

    ESP8266_Clear();
    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CIPSEND=%d\r\n", (int)len);
    if (!ESP8266_SendCmd(cmdBuf, ">")) {
        Usart_SendString(0, data, len);
    }
}

/* ─── Extract payload from +IPD,N:data ─── */
unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{
    char *ptrIPD = NULL;

    do {
        if (ESP8266_WaitRecive() == REV_OK) {
            ptrIPD = strstr((char *)esp8266_buf, "IPD,");
            if (ptrIPD != NULL) {
                ptrIPD = strchr(ptrIPD, ':');
                if (ptrIPD != NULL) {
                    ptrIPD++;
                    return (unsigned char *)ptrIPD;
                }
                return NULL;
            }
        }
        delay_ms(5);
    } while (timeOut--);

    return NULL;
}

/* ─── ESP8266 AT init sequence ─── */
void ESP8266_Init(void)
{
    usart3_esp_init(115200UL);

    ESP8266_Clear();

    printf("[ESP] 1. AT\r\n");
    while (ESP8266_SendCmd("AT\r\n", "OK")) delay_ms(500);

    printf("[ESP] 2. CWMODE=1\r\n");
    while (ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK")) delay_ms(500);

    printf("[ESP] 3. CWDHCP\r\n");
    while (ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK")) delay_ms(500);

    printf("[ESP] 4. CWJAP (connecting WiFi...)\r\n");
    while (ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP")) delay_ms(500);

    printf("[ESP] 5. Init OK\r\n");
}
