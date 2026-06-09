//单片机头文件
#include "gd32f30x.h"

//网络设备驱动
#include "esp8266.h"

//硬件驱动
#include "delay.h"
#include "usart.h"

//C库
#include <string.h>
#include <stdio.h>


#define ESP8266_WIFI_INFO		"AT+CWJAP=\"abc\",\"abc123456\"\r\n"

unsigned char esp8266_buf[512];
unsigned short esp8266_cnt = 0, esp8266_cntPre = 0;

void ESP8266_Clear(void)
{
	memset(esp8266_buf, 0, sizeof(esp8266_buf));
	esp8266_cnt = 0;
}

_Bool ESP8266_WaitRecive(void)
{
	if(esp8266_cnt == 0) return REV_WAIT;
	if(esp8266_cnt == esp8266_cntPre)
	{
		esp8266_cnt = 0;
		return REV_OK;
	}
	esp8266_cntPre = esp8266_cnt;
	return REV_WAIT;
}

_Bool ESP8266_SendCmd(char *cmd, char *res)
{
	unsigned char timeOut = 200;

	Usart_SendString(USART1, (unsigned char *)cmd, strlen((const char *)cmd));
	while(timeOut--)
	{
		if(ESP8266_WaitRecive() == REV_OK)
		{
			if(strstr((const char *)esp8266_buf, res) != NULL)
			{
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
	sprintf(cmdBuf, "AT+CIPSEND=%d\r\n", len);
	if(!ESP8266_SendCmd(cmdBuf, ">"))
	{

		Usart_SendString(USART1, data, len);
	}
}

unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{
	char *ptrIPD = NULL;
	do
	{
		if(ESP8266_WaitRecive() == REV_OK)
		{
			ptrIPD = strstr((char *)esp8266_buf, "IPD,");
			if(ptrIPD != NULL)
			{
				ptrIPD = strchr(ptrIPD, ':');
				if(ptrIPD != NULL)
				{
					ptrIPD++;
					return (unsigned char *)(ptrIPD);
				}
				else return NULL;
			}
		}
		delay_ms(5);
	} while(timeOut--);
	return NULL;
}

void ESP8266_Init(void)
{
	ESP8266_Clear();
	
	UsartPrintf(USART_DEBUG, "1. AT\r\n");
	while(ESP8266_SendCmd("AT\r\n", "OK")) delay_ms(500);
	
	UsartPrintf(USART_DEBUG, "2. CWMODE\r\n");
	while(ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK")) delay_ms(500);
	
	UsartPrintf(USART_DEBUG, "3. AT+CWDHCP\r\n");
	while(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK")) delay_ms(500);
	
	UsartPrintf(USART_DEBUG, "4. CWJAP (Connecting to WiFi...)\r\n");
	while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP")) delay_ms(500);
	
	UsartPrintf(USART_DEBUG, "5. ESP8266 Init OK\r\n");
}


void USART1_IRQHandler(void)
{
    // 判断接收数据寄存器非空标志位 
	if(usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE) != RESET)
	{
		if(esp8266_cnt >= sizeof(esp8266_buf))	esp8266_cnt = 0;
        
        // 读取数据
		esp8266_buf[esp8266_cnt++] = (unsigned char)usart_data_receive(USART1);
        
        // 清除中断标志位
		usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);
	}
}
