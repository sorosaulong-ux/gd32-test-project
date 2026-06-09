#include "usart.h"
#include "delay.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

/*
************************************************************
*	函数名称：	Usart0_Init (原Usart1_Init)
*	说明：		TX-PA9		RX-PA10 (调试打印口)
************************************************************
*/
void Usart0_Init(uint32_t baudrate)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_USART0);
	
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
	
	usart_deinit(USART0);
	usart_baudrate_set(USART0, baudrate);
	usart_word_length_set(USART0, USART_WL_8BIT);						
	usart_stop_bit_set(USART0, USART_STB_1BIT);							
	usart_parity_config(USART0, USART_PM_NONE);							
	usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);			
	usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
	
	usart_receive_config(USART0, USART_RECEIVE_ENABLE);					
	usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);				
	usart_enable(USART0);												
	
	usart_interrupt_enable(USART0, USART_INT_RBNE);						
	nvic_irq_enable(USART0_IRQn, 0, 2);
}

/*
************************************************************
*	函数名称：	Usart1_Init (原Usart2_Init)
*	说明：		TX-PA2		RX-PA3 (ESP8266通信口)
************************************************************
*/
void Usart1_Init(uint32_t baudrate)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_USART1);
	
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
	
	usart_deinit(USART1);
	usart_baudrate_set(USART1, baudrate);
	usart_word_length_set(USART1, USART_WL_8BIT);
	usart_stop_bit_set(USART1, USART_STB_1BIT);
	usart_parity_config(USART1, USART_PM_NONE);
	usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
	usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
	
	usart_receive_config(USART1, USART_RECEIVE_ENABLE);
	usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
	usart_enable(USART1);
	
	usart_interrupt_enable(USART1, USART_INT_RBNE);
	nvic_irq_enable(USART1_IRQn, 0, 0);
}

void Usart_SendString(uint32_t usart_periph, unsigned char *str, unsigned short len)
{
	unsigned short count = 0;
	for(; count < len; count++)
	{
		usart_data_transmit(usart_periph, *str++);									
		while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));				
	}
}

void UsartPrintf(uint32_t usart_periph, char *fmt,...)
{
	unsigned char UsartPrintfBuf[296];
	va_list ap;
	unsigned char *pStr = UsartPrintfBuf;
	
	va_start(ap, fmt);
	vsnprintf((char *)UsartPrintfBuf, sizeof(UsartPrintfBuf), fmt, ap);				
	va_end(ap);
	
	while(*pStr != 0)
	{
		usart_data_transmit(usart_periph, *pStr++);
		while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
	}
}

void USART0_IRQHandler(void)
{
	if(usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE) != RESET) 
	{
		usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);
	}
}