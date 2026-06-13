/*!
 *  \file    esp8266.c
 *  \brief   ESP8266 AT-command driver — GD32A7 (blocking + non-blocking)
 */

#include "gd32a7xx.h"
#include "esp8266.h"
#include "delay.h"
#include "usart_esp.h"
#include "uwb_port.h"
#include "onenet.h"
#include "can_diag.h"
#include "app_tasks.h"
#include <string.h>
#include <stdio.h>

/* ─── WiFi credentials ─── */
#define ESP8266_WIFI_INFO   "AT+CWJAP=\"aaa\",\"88888888\"\r\n"
#define ESP8266_ONENET_TCP  "AT+CIPSTART=\"TCP\",\"mqtts.heclouds.com\",1883\r\n"

/* ─── Non-blocking command state ─── */
static const char   *cmd_expect;
static uint32_t      cmd_sent_ms;
static uint8_t       cmd_active;
#define CMD_TIMEOUT_MS  8000

/* ─── WiFi state machine ─── */
static wifi_sm_state_t sm_state = WIFI_SM_IDLE;
static uint32_t        sm_step_ms;
static uint8_t         sm_retry;
#define SM_RETRY_MAX     3
#define SM_STEP_TIMEOUT  10000
#define SM_DEAD_WAIT     60000

/* ====================================================================
 *  Blocking helpers (master-verified)
 * ====================================================================*/
void ESP8266_Clear(void)
{
    memset(esp8266_buf, 0, sizeof(esp8266_buf));
    esp8266_cnt = 0;
}

static int ESP8266_WaitRecive(void)
{
    static uint32_t stable_ms;

    if (esp8266_cnt == 0)       { stable_ms = 0; return REV_WAIT; }
    if (esp8266_cnt == esp8266_cntPre) {
        /* 2ms 稳定防抖 — FreeRTOS 多任务抢占导致 cnt 持续变化 */
        if (stable_ms == 0) stable_ms = uwb_tick_get();
        if (uwb_tick_get() - stable_ms >= 2U) {
            stable_ms = 0;
            esp8266_cnt = 0;
            return REV_OK;
        }
        return REV_WAIT;
    }
    stable_ms = 0;
    esp8266_cntPre = esp8266_cnt;
    return REV_WAIT;
}

int ESP8266_SendCmd(char *cmd, char *res)
{
    unsigned char timeOut = 200;
    Usart_SendString(0, (unsigned char *)cmd, strlen(cmd));
    while (timeOut--) {
        if (ESP8266_WaitRecive() == REV_OK) {
            if (strstr((const char *)esp8266_buf, res) != NULL) {
                ESP8266_Clear();
                return 0;
            }
        }
        delay_ms(10);
    }
    return 1;
}

void ESP8266_SendData(unsigned char *data, unsigned short len)
{
    char cmdBuf[32];
    ESP8266_Clear();
    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CIPSEND=%d\r\n", (int)len);
    if (!ESP8266_SendCmd(cmdBuf, ">")) {
        Usart_SendString(0, data, len);
    }
}

unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{
    char *ptrIPD = NULL;
    do {
        if (ESP8266_WaitRecive() == REV_OK) {
            ptrIPD = strstr((char *)esp8266_buf, "IPD,");
            if (ptrIPD != NULL) {
                ptrIPD = strchr(ptrIPD, ':');
                return (ptrIPD != NULL) ? (unsigned char *)(ptrIPD + 1) : NULL;
            }
        }
        delay_ms(5);
    } while (timeOut--);
    return NULL;
}

/* ====================================================================
 *  ESP8266_Init — blocking version (kept for reference)
 * ====================================================================*/
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

/* ====================================================================
 *  Non-blocking command
 * ====================================================================*/
void esp8266_cmd_send(const char *cmd, const char *expect)
{
    ESP8266_Clear();
    Usart_SendString(0, (unsigned char *)cmd, strlen(cmd));
    cmd_expect  = expect;
    cmd_sent_ms = uwb_tick_get();
    cmd_active  = 1;
}

int esp8266_cmd_done(void)
{
    if (!cmd_active) return 1;

    if (ESP8266_WaitRecive() == REV_OK) {
        cmd_active = 0;
        if (strstr((const char *)esp8266_buf, cmd_expect) != NULL) {
            ESP8266_Clear();
            return 1;
        }
        return -1;
    }

    if (uwb_tick_get() - cmd_sent_ms > CMD_TIMEOUT_MS) {
        cmd_active = 0;
        return -1;
    }
    return 0;
}

/* ====================================================================
 *  WiFi state machine
 * ====================================================================*/

static void sm_next(wifi_sm_state_t s)
{
    sm_state   = s;
    sm_step_ms = uwb_tick_get();
    sm_retry   = 0;
}

void wifi_sm_start(void)
{
    sm_next(WIFI_SM_INIT_UART);
}

void wifi_sm_tick(void)
{
    uint32_t now = uwb_tick_get();

    switch (sm_state) {

    case WIFI_SM_IDLE:
    case WIFI_SM_DEAD:
        if ((int32_t)(now - sm_step_ms) > SM_DEAD_WAIT) {
            printf("[WiFi] Retrying...\r\n");
            sm_next(WIFI_SM_INIT_UART);
        }
        break;

    case WIFI_SM_INIT_UART:
        usart3_esp_init(115200UL);
        sm_next(WIFI_SM_AT);
        break;

    case WIFI_SM_AT:
        ESP8266_Clear();
        esp8266_cmd_send("AT\r\n", "OK");
        sm_next(WIFI_SM_CWMODE);
        break;

    case WIFI_SM_CWMODE:
    case WIFI_SM_CWDHCP:
    case WIFI_SM_CWJAP:
        {
            int rc = esp8266_cmd_done();
            if (rc == 0) return;
            if (rc == -1) {
                if (++sm_retry > SM_RETRY_MAX) {
                    sm_next(WIFI_SM_DEAD);
                    can_diag_send_error(CAN_ERR_ESP8266, CAN_ERR_ESP_WIFI);
                    return;
                }
                goto resend;
            }
            sm_retry = 0;
        }

        ESP8266_Clear();
        switch (sm_state) {
        case WIFI_SM_AT:
            esp8266_cmd_send("AT+CWMODE=1\r\n", "OK");
            sm_next(WIFI_SM_CWDHCP);
            break;
        case WIFI_SM_CWMODE:
            esp8266_cmd_send("AT+CWDHCP=1,1\r\n", "OK");
            sm_next(WIFI_SM_CWJAP);
            break;
        case WIFI_SM_CWDHCP:
            esp8266_cmd_send(ESP8266_WIFI_INFO, "GOT IP");
            sm_next(WIFI_SM_CWJAP);
            break;
        case WIFI_SM_CWJAP:
            sm_next(WIFI_SM_TCP);
            break;
        default: break;
        }
        break;

    /* ── WIFI_SM_TCP: 提权→阻塞 TCP+MQTT(防UWB抢占)→降权 ── */
    case WIFI_SM_TCP:
    {
        extern TaskHandle_t xTaskUWB_Handle;

        /* 临时提权: 确保 TCP+MQTT 阻塞期间不被 UWB 抢占 */
        if (xTaskUWB_Handle) vTaskPrioritySet(xTaskUWB_Handle, tskIDLE_PRIORITY + 1);

        printf("[WiFi] TCP connecting...\r\n");
        while (ESP8266_SendCmd(ESP8266_ONENET_TCP, "CONNECT"))
            delay_ms(500);

        printf("[WiFi] Device login...\r\n");
        while (OneNet_DevLink()) {
            ESP8266_SendCmd(ESP8266_ONENET_TCP, "CONNECT");
            delay_ms(500);
        }

        OneNET_Subscribe();

        /* 恢复 UWB 优先级 */
        if (xTaskUWB_Handle) vTaskPrioritySet(xTaskUWB_Handle, TASK_PRIO_UWB);

        sm_next(WIFI_SM_OK);
        printf("[WiFi] ONLINE\r\n");
        can_diag_send_error(0, 0);
        break;
    }

    case WIFI_SM_OK:
        break;

    default:
        sm_next(WIFI_SM_IDLE);
        break;
    }

    return;

resend:
    ESP8266_Clear();
    switch (sm_state) {
    case WIFI_SM_CWMODE:  esp8266_cmd_send("AT+CWMODE=1\r\n", "OK"); break;
    case WIFI_SM_CWDHCP: esp8266_cmd_send("AT+CWDHCP=1,1\r\n", "OK"); break;
    case WIFI_SM_CWJAP:  esp8266_cmd_send(ESP8266_WIFI_INFO, "GOT IP"); break;
    default: break;
    }
}

int wifi_sm_ready(void)
{
    return (sm_state == WIFI_SM_OK);
}

void wifi_sm_force_restart(void)
{
    printf("[WiFi] Force restart\r\n");
    sm_next(WIFI_SM_DEAD);
}
