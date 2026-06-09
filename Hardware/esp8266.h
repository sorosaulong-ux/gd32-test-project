/*!
 *  \file    esp8266.h
 *  \brief   ESP8266 AT-command WiFi module — GD32A7 non-blocking
 *
 *  Connected to LINFLEXD_UART3 (EVAL_COMB: PJ7=TX, PJ8=RX) @ 115200
 */

#ifndef _ESP8266_H_
#define _ESP8266_H_

#include <stdint.h>

#define REV_OK      0
#define REV_WAIT    1

extern unsigned char  esp8266_buf[512];
extern unsigned short esp8266_cnt;
extern unsigned short esp8266_cntPre;

/* ====================================================================
 *  Blocking API (legacy — ESP8266_Init + initial setup)
 * ====================================================================*/
void          ESP8266_Init(void);          /* blocking — 跑一次即可 */
void          ESP8266_Clear(void);
int           ESP8266_SendCmd(char *cmd, char *res);
void          ESP8266_SendData(unsigned char *data, unsigned short len);
unsigned char *ESP8266_GetIPD(unsigned short timeOut);

/* ====================================================================
 *  Non-blocking API — 状态机驱动, 每帧 tick (< 1ms)
 * ====================================================================*/

/* 非阻塞命令 */
void esp8266_cmd_send(const char *cmd, const char *expect);
int  esp8266_cmd_done(void);              /* 1=OK  0=waiting  -1=timeout */

/* WiFi 连接状态机 */
typedef enum {
    WIFI_SM_IDLE = 0,      /* 空闲 / 未启动 */
    WIFI_SM_INIT_UART,     /* 1: UART3 init */
    WIFI_SM_AT,            /* 2: AT test */
    WIFI_SM_CWMODE,        /* 3: AT+CWMODE=1 */
    WIFI_SM_CWDHCP,        /* 4: AT+CWDHCP */
    WIFI_SM_CWJAP,         /* 5: AT+CWJAP (WiFi connect) */
    WIFI_SM_TCP,           /* 6: AT+CIPSTART TCP → OneNET */
    WIFI_SM_MQTT_CONNECT,  /* 7: MQTT CONNECT */
    WIFI_SM_MQTT_SUBSCRIBE,/* 8: MQTT Subscribe */
    WIFI_SM_OK,            /* 9: 连接正常 */
    WIFI_SM_DEAD,          /* 10: 放弃, 等重试 */
} wifi_sm_state_t;

void     wifi_sm_start(void);              /* 启动状态机 */
void     wifi_sm_tick(void);               /* 每 50ms 调一次 */
int      wifi_sm_ready(void);              /* 1 = 可收发数据 */
void     wifi_sm_force_restart(void);      /* 强制重连 */

#endif
