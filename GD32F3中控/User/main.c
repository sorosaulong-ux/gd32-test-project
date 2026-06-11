//��Ƭ��ͷ�ļ�
#include "gd32f30x.h"

//Ӳ������
#include "delay.h"
#include "usart.h"
#include "buzzer.h"  
#include "key.h"
#include "oled.h"
#include "can_drive.h"  

#include <string.h>

/* ── CAN ID (与 GD32A7 一致) ── */
#define CAN_ID_ERROR    0x103U

/* ── 错误码 (与 GD32A7 一致) ── */
#define ERR_UWB         0x01
#define ERR_ESP         0x02
#define ERR_CAN         0x03
#define ERR_SYSTEM      0x04
#define ERR_ONENET      0x05

/* ── 系统状态 ── */
static uint8_t g_err_code = 0;
static uint8_t g_err_sub = 0;
static uint8_t g_fault = 0;  /* 0=正常, 1=故障 */

void Hardware_Init(void);
static void oled_update(void);
static const char* get_error_text(uint8_t err, uint8_t sub);

int main(void)
{
    uint8_t rx_data[8] = {0};
    uint32_t rx_id;
    uint8_t last_fault = 255;
    
    Hardware_Init();
    
    OLED_Clear();
    OLED_ShowString(1, 1, "System: NORMAL");
    OLED_ShowString(2, 1, "Error:  NONE");
    OLED_ShowString(3, 1, "BUZZER: OFF");
    OLED_ShowString(4, 1, "Waiting CAN...");
    
    while(1)
    {
        /* ── CAN 接收 ── */
        if(CAN_Receive_Msg(&rx_id, rx_data) == 0)
        {
            if(rx_id == CAN_ID_ERROR)
            {
                uint8_t err = rx_data[0];
                uint8_t sub = rx_data[1];
                uint8_t sig = rx_data[3];
                
                if(sig == 0xAA)
                {
                    /* 故障 */
                    g_err_code = err;
                    g_err_sub = sub;
                    g_fault = 1;
                    BUZZER_Set(BUZZER_ON);
                }
                else if(sig == 0x55)
                {
                    /* 正常/清除 */
                    g_err_code = err;
                    g_err_sub = sub;
                    g_fault = 0;
                    BUZZER_Set(BUZZER_OFF);
                }
                
                UsartPrintf(USART_DEBUG, "[CAN] 0x103 err=%02X sub=%02X %s\r\n",
                            err, sub, sig == 0xAA ? "FAULT" : "CLEAR");
            }
        }
        
        /* ── OLED 更新 (状态变化时) ── */
        if(g_fault != last_fault)
        {
            oled_update();
            last_fault = g_fault;
        }
    }
}

static void oled_update(void)
{
    char buf[20];
    
    OLED_Clear();
    
    /* 第1行: 系统状态 */
    if(g_fault)
        OLED_ShowString(1, 1, "System: FAULT!");
    else
        OLED_ShowString(1, 1, "System: NORMAL");
    
    /* 第2行: 错误类型 */
    snprintf(buf, sizeof(buf), "Error: %s", get_error_text(g_err_code, g_err_sub));
    OLED_ShowString(2, 1, buf);
    
    /* 第3行: 蜂鸣器状态 */
    snprintf(buf, sizeof(buf), "BUZZER: %s", BUZZER_Status == BUZZER_ON ? "ON " : "OFF");
    OLED_ShowString(3, 1, buf);
}

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
}