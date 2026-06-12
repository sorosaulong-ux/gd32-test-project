/*!
 *  \file    main.c
 *  \brief   STM32F103 — EWM550 UWB 钥匙 (标签/Responder)
 *           BLE 连接后启用 UWB，断开后休眠
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "ewm550.h"
#include "ble.h"
#include <string.h>
#include <stdio.h>

/* ─── USART1 调试串口 (PA9, 115200) ─── */
static void debug_init(void)
{
    GPIO_InitTypeDef g; USART_InitTypeDef u;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    g.GPIO_Mode = GPIO_Mode_AF_PP; g.GPIO_Pin = GPIO_Pin_9;
    g.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOA, &g);
    u.USART_BaudRate = 115200; u.USART_WordLength = USART_WordLength_8b;
    u.USART_StopBits = USART_StopBits_1; u.USART_Parity = USART_Parity_No;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    u.USART_Mode = USART_Mode_Tx; USART_Init(USART1, &u); USART_Cmd(USART1, ENABLE);
}
static void up(const char *s) { while (*s) { while (!(USART1->SR & 0x80)); USART1->DR = *s++; } }

/* ─── 解析距离数据 ─── */
/* 格式: P,1111,10cm\r\n 或 P0,AA00,10cm,20dB\r\n */
static float parse_distance(const char *buf)
{
    float dist = -1.0f;
    char *p;

    /* 查找 "cm" 前面的数字 */
    p = strstr(buf, "cm");
    if (p)
    {
        /* 回退找到数字开始位置 */
        char *num_end = p;
        while (num_end > buf && *(num_end-1) != ',') num_end--;

        dist = atof(num_end);
    }

    return dist;
}

/* ─── 解析 SNR ─── */
static int parse_snr(const char *buf)
{
    int snr = -1;
    char *p;

    p = strstr(buf, "dB");
    if (p)
    {
        char *num_end = p;
        while (num_end > buf && *(num_end-1) != ',') num_end--;
        snr = atoi(num_end);
    }

    return snr;
}

/* ====================================================================
 *  main
 * ====================================================================*/
int main(void)
{
    char ewm_buf[128];
    char oled_line1[20], oled_line2[20];
    float distance = -1.0f;
    int snr = -1;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    debug_init();
    up("\r\nSTM32 EWM550 KEY v1.0\r\n");

    OLED_Init();
    OLED_ShowString(1, 1, "UWB: Init...");
    OLED_ShowString(3, 1, "BLE: Init...");

    /* BLE 初始化 */
    BLE_Init();
    up("BLE init OK\r\n");
    OLED_ShowString(3, 1, "BLE: Waiting..");
    while (!BLE_IsConnected()) { Delay_ms(500); }
    OLED_ShowString(3, 1, "BLE: Connected ");

    /* EWM550 初始化 */
    EWM550_Init(921600);
    up("EWM550 UART3 init OK\r\n");

    /* 配置为标签模式 (Responder) */
    EWM550_Reset();
    up("EWM550 reset OK\r\n");

    EWM550_SetRole(0);  /* 0=Responder (标签) */
    up("EWM550 role=Responder\r\n");

    EWM550_SetAddress("1111");  /* 标签地址 */
    up("EWM550 addr=1111\r\n");

    OLED_ShowString(1, 1, "UWB: Ready     ");
    up("EWM550 running\r\n");

    while (1)
    {
        /* BLE 连接检查 */
        if (!BLE_IsConnected())
        {
            OLED_ShowString(3, 1, "BLE: Disconn.  ");
            OLED_ShowString(1, 1, "UWB: Stopped   ");
            OLED_ShowString(2, 1, "                ");

            /* 进入低功耗 */
            EWM550_SendCmd("+++");
            Delay_ms(100);
            EWM550_SendCmd("AT+SLEEP=1\r\n");  /* 深度休眠 */
            Delay_ms(100);

            while (!BLE_IsConnected()) { Delay_ms(500); }

            /* 唤醒 */
            EWM550_Reset();
            EWM550_SetRole(0);
            EWM550_SetAddress("1111");

            OLED_ShowString(3, 1, "BLE: Connected ");
            OLED_ShowString(1, 1, "UWB: Ready     ");
        }

        /* 读取 EWM550 数据 */
        memset(ewm_buf, 0, sizeof(ewm_buf));
        if (EWM550_WaitResponse(ewm_buf, sizeof(ewm_buf), 100))
        {
            /* 解析距离 */
            distance = parse_distance(ewm_buf);
            snr = parse_snr(ewm_buf);

            if (distance >= 0)
            {
                snprintf(oled_line1, sizeof(oled_line1), "UWB: %.0f cm   ", distance);
                OLED_ShowString(1, 1, oled_line1);

                if (snr >= 0)
                {
                    snprintf(oled_line2, sizeof(oled_line2), "SNR: %d dB     ", snr);
                    OLED_ShowString(2, 1, oled_line2);
                }

                up(ewm_buf);
            }
        }
        else
        {
            /* 无数据 */
            OLED_ShowString(2, 1, "SNR: -- dB     ");
        }

        Delay_ms(100);
    }
}
