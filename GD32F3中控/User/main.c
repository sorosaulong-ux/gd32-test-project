//单片机头文件
#include "gd32f30x.h"

//网络协议层与设备
#include "onenet.h"
#include "esp8266.h"

//硬件驱动
#include "delay.h"
#include "usart.h"
#include "buzzer.h"  
#include "key.h"
#include "oled.h"
#include "can_drive.h"  

#include <string.h>

#define ESP8266_ONENET_INFO		"AT+CIPSTART=\"TCP\",\"mqtts.heclouds.com\",1883\r\n"

void Hardware_Init(void);

int main(void)
{
    unsigned short timeCount = 0;	
    unsigned short canCount = 0;   
    unsigned char *dataPtr = NULL;
    
    uint8_t last_buzzer_status = 255; 
    
    // CAN 通信变量
    uint8_t tx_data[4] = {0xCA, 0xFE, 0x12, 0x34};
    uint8_t rx_data[8] = {0};
    uint32_t rx_id;
	
    Hardware_Init();				
    
    // OLED 开机提示
    OLED_Clear();
    OLED_ShowString(1, 1, "WiFi Conn...");

    // 联网登录
    ESP8266_Init();					
    UsartPrintf(USART_DEBUG, "Connect MQTTs Server...\r\n");
    OLED_ShowString(2, 1, "MQTT Conn...");
    while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT")) DelayXms(500);
    
    UsartPrintf(USART_DEBUG, "Device login ...\r\n");
    OLED_ShowString(3, 1, "Logging in...");
    while(OneNet_DevLink())			
    {
        ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT");
        DelayXms(500);
    }
		
    OneNET_Subscribe();             
	
    // 登录成功，绘制主界面 UI
    OLED_Clear();
    OLED_ShowString(1, 1, "OneNET Online!");
    OLED_ShowString(3, 1, "BUZZER:");
    
    while(1)
    {
        // ================= 1. OLED 实时刷新蜂鸣器状态 =================
        if(BUZZER_Status != last_buzzer_status)
        {
            if(BUZZER_Status == BUZZER_ON) OLED_ShowString(3, 9, "ON "); 
            else OLED_ShowString(3, 9, "OFF");
            last_buzzer_status = BUZZER_Status;
        }

        // ================= 2. CAN 总线心跳包 (每 1 秒发送一次) =================
        if(++canCount >= 20) 
        {
            canCount = 0;
            // 带有真实状态反馈的发送
            if(CAN_Send_Msg(0x11, tx_data, 4) == 0)
            {
                UsartPrintf(USART_DEBUG, "[CAN] MCU -> PCAN: Send OK\r\n");
            }
            else
            {
                UsartPrintf(USART_DEBUG, "[CAN] MCU -> PCAN: Send TIMEOUT / FAILED (Check cable!)\r\n");
            }
        }

        // ================= 3. CAN 实时接收 (零延迟轮询) =================
        if(CAN_Receive_Msg(&rx_id, rx_data) == 0)
        {
            UsartPrintf(USART_DEBUG, "[CAN] PCAN -> MCU! ID:0x%X, Data:%02X %02X %02X %02X\r\n", 
                        rx_id, rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
            
        }

        // ================= 4. OneNET 云端通信定时任务 =================
        if(++timeCount >= 100)		
        {
            OneNet_SendData();									
            timeCount = 0;
            ESP8266_Clear();
        }
        
        // 接收云端控制指令
        dataPtr = ESP8266_GetIPD(0);
        if(dataPtr != NULL) OneNet_RevPro(dataPtr); 
		
        DelayMs(50);
    }
}

void Hardware_Init(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    Delay_Init();									
    Usart0_Init(115200);   		
    Usart1_Init(115200);   			
    BUZZER_Init();      
    Key_Init();         
    OLED_Init();        
    CAN_Init_Config();  
	
    UsartPrintf(USART_DEBUG, "Hardware init OK\r\n");
		UsartPrintf(USART_DEBUG, "\r\n---> System APB1 Clock: %d Hz <---\r\n", rcu_clock_freq_get(CK_APB1));
}