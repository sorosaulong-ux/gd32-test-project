/*!
 *  \file    ewm550.c
 *  \brief   EWM550 UWB 模块驱动 (UART + AT指令)
 *
 *  USART3: PB10(TX) PB11(RX) @ 921600
 *  RST: PB12 (低电平复位)
 */

#include "ewm550.h"
#include "Delay.h"
#include <string.h>
#include <stdio.h>

#define EWM550_USART       USART3
#define EWM550_RST_PORT    GPIOB
#define EWM550_RST_PIN     GPIO_Pin_12

static volatile uint8_t rx_buf[256];
static volatile uint16_t rx_cnt = 0;

/* USART3 中断接收 */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(EWM550_USART, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = USART_ReceiveData(EWM550_USART);
        if (rx_cnt < sizeof(rx_buf))
        {
            rx_buf[rx_cnt++] = ch;
        }
    }
}

void EWM550_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* 2. PB10 - USART3 TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 3. PB11 - USART3 RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 4. PB12 - RST (输出推挽) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(EWM550_RST_PORT, EWM550_RST_PIN);  /* RST 拉高，正常工作 */

    /* 5. USART3 配置 */
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(EWM550_USART, &USART_InitStructure);

    /* 6. 中断配置 */
    USART_ITConfig(EWM550_USART, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(EWM550_USART, ENABLE);
}

void EWM550_SendCmd(const char *cmd)
{
    while (*cmd)
    {
        while (USART_GetFlagStatus(EWM550_USART, USART_FLAG_TXE) == RESET);
        USART_SendData(EWM550_USART, *cmd++);
    }
    while (USART_GetFlagStatus(EWM550_USART, USART_FLAG_TC) == RESET);
}

uint8_t EWM550_WaitResponse(char *buf, uint16_t len, uint16_t timeout_ms)
{
    uint16_t i = 0;
    uint16_t start = 0;

    rx_cnt = 0;
    memset((void*)rx_buf, 0, sizeof(rx_buf));

    start = 0;
    while (i < timeout_ms)
    {
        Delay_ms(1);
        i++;

        /* 检查是否收到完整响应 (以 \r\n 结尾) */
        if (rx_cnt >= 2 && rx_buf[rx_cnt-1] == '\n' && rx_buf[rx_cnt-2] == '\r')
        {
            break;
        }
    }

    /* 拷贝到输出缓冲区 */
    if (rx_cnt > 0 && rx_cnt < len)
    {
        memcpy(buf, (const void*)rx_buf, rx_cnt);
        buf[rx_cnt] = '\0';
        return 1;  /* 成功 */
    }

    return 0;  /* 超时或无数据 */
}

void EWM550_SetRole(uint8_t role)
{
    char cmd[32];
    char resp[64];

    /* 进入 AT 模式 */
    EWM550_SendCmd("+++");
    Delay_ms(100);
    EWM550_WaitResponse(resp, sizeof(resp), 500);

    /* 设置角色 */
    snprintf(cmd, sizeof(cmd), "AT+ROLE=%d\r\n", role);
    EWM550_SendCmd(cmd);
    Delay_ms(100);
    EWM550_WaitResponse(resp, sizeof(resp), 500);

    /* 退出 AT 模式 */
    EWM550_SendCmd("AT+EXIT\r\n");
    Delay_ms(100);
}

void EWM550_SetAddress(const char *addr)
{
    char cmd[64];
    char resp[64];

    EWM550_SendCmd("+++");
    Delay_ms(100);
    EWM550_WaitResponse(resp, sizeof(resp), 500);

    snprintf(cmd, sizeof(cmd), "AT+SRCADDR=%s\r\n", addr);
    EWM550_SendCmd(cmd);
    Delay_ms(100);
    EWM550_WaitResponse(resp, sizeof(resp), 500);

    EWM550_SendCmd("AT+EXIT\r\n");
    Delay_ms(100);
}

void EWM550_Reset(void)
{
    GPIO_ResetBits(EWM550_RST_PORT, EWM550_RST_PIN);  /* RST 拉低 */
    Delay_ms(100);
    GPIO_SetBits(EWM550_RST_PORT, EWM550_RST_PIN);    /* RST 拉高 */
    Delay_ms(500);  /* 等待模块启动 */
}
