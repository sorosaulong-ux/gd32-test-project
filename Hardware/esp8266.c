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
#include <string.h>
#include <stdio.h>

/* ─── WiFi credentials ─── */
#define ESP8266_WIFI_INFO   "AT+CWJAP=\"aaa\",\"88888888\"\r\n"
#define ESP8266_ONENET_TCP  "AT+CIPSTART=\"TCP\",\"mqtts.heclouds.com\",1883\r\n"

/* ─── Non‑blocking command state ─── */
static const char   *cmd_expect;
static uint32_t      cmd_sent_ms;
static uint8_t       cmd_active;
#define CMD_TIMEOUT_MS  8000

/* ─── WiFi state machine ─── */
static wifi_sm_state_t sm_state = WIFI_SM_IDLE;
static uint32_t        sm_step_ms;
static uint8_t         sm_retry;
#define SM_RETRY_MAX     3
#define SM_STEP_TIMEOUT  10000    /* ms — per‑step timeout */
#define SM_DEAD_WAIT     60000    /* ms — retry interval */

/* ====================================================================
 *  Blocking helpers (unchanged — used by ESP8266_SendData etc.)
 * ====================================================================*/
void ESP8266_Clear(void)
{
    memset(esp8266_buf, 0, sizeof(esp8266_buf));
    esp8266_cnt = 0;
}

static int ESP8266_WaitRecive(void)
{
    if (esp8266_cnt == 0)       return REV_WAIT;
    if (esp8266_cnt == esp8266_cntPre) { esp8266_cnt = 0; return REV_OK; }
    esp8266_cntPre = esp8266_cnt;
    return REV_WAIT;
}

int ESP8266_SendCmd(char *cmd, char *res)
{
    unsigned char timeOut = 200;
    printf("[ESP] TX: %.20s...\r\n", cmd);
    Usart_SendString(0, (unsigned char *)cmd, strlen(cmd));
    while (timeOut--) {
        if (ESP8266_WaitRecive() == REV_OK) {
            printf("[ESP] RX: %s\r\n", esp8266_buf);
            if (strstr((const char *)esp8266_buf, res) != NULL) {
                ESP8266_Clear();
                printf("[ESP] OK (expect: %s)\r\n", res);
                return 0;
            }
        }
        delay_ms(10);
    }
    printf("[ESP] TIMEOUT (expect: %s)\r\n", res);
    return 1;
}

void ESP8266_SendData(unsigned char *data, unsigned short len)
{
    char cmdBuf[32];
    ESP8266_Clear();
    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CIPSEND=%d\r\n", (int)len);
    printf("[ESP] SendData: len=%d\r\n", len);
    if (!ESP8266_SendCmd(cmdBuf, ">")) {
        printf("[ESP] SendData: sending %d bytes...\r\n", len);
        Usart_SendString(0, data, len);
        printf("[ESP] SendData: sent OK\r\n");
    } else {
        printf("[ESP] SendData: AT+CIPSEND failed\r\n");
    }
}

unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{
    char *ptrIPD = NULL;
    printf("[ESP] GetIPD: waiting (timeout=%d)...\r\n", timeOut);
    do {
        if (ESP8266_WaitRecive() == REV_OK) {
            ptrIPD = strstr((char *)esp8266_buf, "IPD,");
            if (ptrIPD != NULL) {
                printf("[ESP] IPD received\r\n");
                ptrIPD = strchr(ptrIPD, ':');
                return (ptrIPD != NULL) ? (unsigned char *)(ptrIPD + 1) : NULL;
            }
        }
        delay_ms(5);
    } while (timeOut--);
    printf("[ESP] GetIPD TIMEOUT\r\n");
    return NULL;
}

/* ====================================================================
 *  ESP8266_Init — 保留原有阻塞版本（非阻塞状态机复用它但分步走）
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
 *  Non‑blocking command
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

    /* Check RX buffer */
    if (ESP8266_WaitRecive() == REV_OK) {
        cmd_active = 0;
        printf("[ESP] cmd_done RX: %s\r\n", esp8266_buf);
        if (strstr((const char *)esp8266_buf, cmd_expect) != NULL) {
            ESP8266_Clear();
            printf("[ESP] cmd_done OK (expect: %s)\r\n", cmd_expect);
            return 1;  /* success */
        }
        printf("[ESP] cmd_done MISS (expect: %s)\r\n", cmd_expect);
        return -1; /* wrong response */
    }

    if (uwb_tick_get() - cmd_sent_ms > CMD_TIMEOUT_MS) {
        cmd_active = 0;
        printf("[ESP] cmd_done TIMEOUT (expect: %s)\r\n", cmd_expect);
        return -1; /* timeout */
    }
    return 0;  /* still waiting */
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
    /* UART already initialised by ESP8266_Init() or will be by first step */
    sm_next(WIFI_SM_INIT_UART);
}

void wifi_sm_tick(void)
{
    uint32_t now = uwb_tick_get();

    switch (sm_state) {

    case WIFI_SM_IDLE:
    case WIFI_SM_DEAD:
        /* 等待 DEAD_WAIT 后自动重试 */
        if ((int32_t)(now - sm_step_ms) > SM_DEAD_WAIT) {
            printf("[WiFi] Retrying...\r\n");
            sm_next(WIFI_SM_INIT_UART);
        }
        break;

    case WIFI_SM_INIT_UART:
        /* 确保 UART3 已初始化 */
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
        /* ── 等当前 AT 命令完成 ── */
        {
            int rc = esp8266_cmd_done();
            if (rc == 0) return;

            if (rc == -1) {
                if (++sm_retry > SM_RETRY_MAX) {
                    printf("[WiFi] Step %d failed, DEAD\r\n", (int)sm_state);
                    sm_next(WIFI_SM_DEAD);
                    can_diag_send_error(CAN_ERR_ESP8266, CAN_ERR_ESP_WIFI);
                    return;
                }
                goto resend;
            }
            sm_retry = 0;
        }

        /* ── 成功后 → 发下一步命令 ── */
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
            printf("[WiFi] CWJAP OK, DHCP cooldown 3s\r\n");
            sm_next(WIFI_SM_TCP);   /* 进 TCP 冷却 */
            break;
        default: break;
        }
        break;

    /* ── WIFI_SM_TCP: 完全复制 master 裸机的阻塞 TCP+MQTT ── */
    case WIFI_SM_TCP:
    {
        printf("[WiFi] TCP connecting (master-verified)...\r\n");
        while (ESP8266_SendCmd(ESP8266_ONENET_TCP, "CONNECT"))
            delay_ms(500);

        printf("[WiFi] Device login...\r\n");
        while (OneNet_DevLink()) {
            ESP8266_SendCmd(ESP8266_ONENET_TCP, "CONNECT");
            delay_ms(500);
        }

        OneNET_Subscribe();
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
