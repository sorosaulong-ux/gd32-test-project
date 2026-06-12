#include "gd32f30x.h"

#include "delay.h"
#include "usart.h"
#include "buzzer.h"  
#include "key.h"
#include "oled.h"
#include "can_drive.h"  

#include <string.h>
#include <stdio.h>

#define CAN_ID_ERROR    0x103U
#define CAN_ID_BUZZER   0x105U

#define ERR_UWB         0x01
#define ERR_ESP         0x02
#define ERR_CAN         0x03
#define ERR_SYSTEM      0x04
#define ERR_ONENET      0x05

static uint8_t g_err_code = 0;
static uint8_t g_err_sub = 0;
static uint8_t g_fault = 0;
static uint8_t g_can_ok = 0;

static const char* get_error_text(uint8_t err, uint8_t sub)
{
    switch(err)
    {
        case 0:       return "NONE";
        case ERR_UWB: return "UWB Error";
        case ERR_ESP: return "WiFi Error";
        case ERR_CAN: return "CAN Error";
        case ERR_SYSTEM: return "Sys Error";
        case ERR_ONENET: return "MQTT Error";
        default:      return "Unknown";
    }
}

static void Hardware_Init(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    Delay_Init();									
    Usart0_Init(115200);   		
    BUZZER_Init();      
    Key_Init();         
    OLED_Init();        
    CAN_Init_Config();  
	
    UsartPrintf(USART_DEBUG, "Hardware init OK\r\n");
}

int main(void)
{
    uint8_t rx_data[8] = {0};
    uint32_t rx_id;
    uint8_t last_fault = 255;
    uint8_t last_buzzer = 255;
    uint16_t can_timeout = 0;
    uint8_t oled_inited = 0;
    
    Hardware_Init();
    
    OLED_Clear();
    OLED_ShowString(1, 1, "CAN: Waiting");
    OLED_ShowString(2, 1, "System: NORMAL");
    OLED_ShowString(3, 1, "Error:  NONE");
    OLED_ShowString(4, 1, "BUZZER: OFF");
    
    UsartPrintf(USART_DEBUG, "Entering main loop...\r\n");
    
    while(1)
    {
        if(CAN_Receive_Msg(&rx_id, rx_data) == 0)
        {
            can_timeout = 0;
            
            if(!g_can_ok)
            {
                g_can_ok = 1;
                OLED_ShowString(1, 1, "CAN: OK     ");
            }
            
            if(rx_id == CAN_ID_ERROR)
            {
                uint8_t err = rx_data[0];
                uint8_t sub = rx_data[1];
                uint8_t sig = rx_data[3];
                
                if(sig == 0xAA)
                {
                    g_err_code = err;
                    g_err_sub = sub;
                    g_fault = 1;
                    BUZZER_Set(BUZZER_ON);
                }
                else if(sig == 0x55)
                {
                    g_err_code = err;
                    g_err_sub = sub;
                    g_fault = 0;
                    BUZZER_Set(BUZZER_OFF);
                }
                
                UsartPrintf(USART_DEBUG, "[CAN] 0x103 err=%02X sub=%02X %s\r\n",
                            err, sub, sig == 0xAA ? "FAULT" : "CLEAR");
            }
            else if(rx_id == CAN_ID_BUZZER)
            {
                BUZZER_Set(rx_data[0] ? BUZZER_ON : BUZZER_OFF);
                UsartPrintf(USART_DEBUG, "[CAN] 0x105 Buzzer=%d\r\n", rx_data[0]);
            }
        }
        
        if(++can_timeout >= 100)
        {
            if(g_can_ok)
            {
                g_can_ok = 0;
                OLED_ShowString(1, 1, "CAN: Waiting");
            }
            can_timeout = 100;
        }
        
        {
            char buf[20];
            
            if(g_fault != last_fault)
            {
                OLED_ShowString(2, 1, g_fault ? "System: FAULT!" : "System: NORMAL");
                last_fault = g_fault;
            }
            
            if(g_fault || oled_inited == 0)
            {
                snprintf(buf, sizeof(buf), "Error: %-10s", get_error_text(g_err_code, g_err_sub));
                OLED_ShowString(3, 1, buf);
            }
            
            if(BUZZER_Status != last_buzzer)
            {
                snprintf(buf, sizeof(buf), "BUZZER: %s", BUZZER_Status == BUZZER_ON ? "ON " : "OFF");
                OLED_ShowString(4, 1, buf);
                last_buzzer = BUZZER_Status;
            }
            
            oled_inited = 1;
        }
        
        {
            static uint16_t hb_cnt;
            if(++hb_cnt >= 40)
            {
                hb_cnt = 0;
                UsartPrintf(USART_DEBUG, "[HB] alive, CAN=%d fault=%d\r\n", g_can_ok, g_fault);
            }
        }
        
        DelayMs(50);
    }
}
