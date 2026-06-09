/*!
 *  \file    esp8266.h
 *  \brief   ESP8266 AT-command WiFi module — GD32A7 port
 *
 *  Connected to LINFLEXD_UART3 (EVAL_COMB: PJ7=TX, PJ8=RX) @ 115200
 */

#ifndef _ESP8266_H_
#define _ESP8266_H_

#include <stdint.h>

#define REV_OK      0   /* receive done */
#define REV_WAIT    1   /* receive not done */

extern unsigned char  esp8266_buf[512];
extern unsigned short esp8266_cnt;
extern unsigned short esp8266_cntPre;

void          ESP8266_Init(void);
void          ESP8266_Clear(void);
int           ESP8266_SendCmd(char *cmd, char *res);
void          ESP8266_SendData(unsigned char *data, unsigned short len);
unsigned char *ESP8266_GetIPD(unsigned short timeOut);

#endif
