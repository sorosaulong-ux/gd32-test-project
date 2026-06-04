/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability (Modified for GD32)
 */

#include "port.h"
#include "gd32f30x.h"
#include "delay.h" 
#include "deca_device_api.h"

// 定义全局激活标志位，默认 1 号机
uint8_t Active_UWB = 1;

/* ---------------- 1. 智能片选底层函数 ---------------- */
void UWB_CS_Low(void) {
    if(Active_UWB == 1) gpio_bit_reset(UWB1_CS_PORT, UWB1_CS_PIN);
    else if(Active_UWB == 2) gpio_bit_reset(UWB2_CS_PORT, UWB2_CS_PIN);
}

void UWB_CS_High(void) {
    if(Active_UWB == 1) gpio_bit_set(UWB1_CS_PORT, UWB1_CS_PIN);
    else if(Active_UWB == 2) gpio_bit_set(UWB2_CS_PORT, UWB2_CS_PIN);
}

/* ---------------- 2. SPI 与引脚初始化 ---------------- */
void UWB_SPI_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI0);

    // 共享 SPI 总线 (PA5:SCK, PA6:MISO, PA7:MOSI)
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    // 【绝密防打架顺序】先将两个 CS 的输出寄存器拉高（取消选中），然后再把引脚配置为输出！
    gpio_bit_set(UWB1_CS_PORT, UWB1_CS_PIN);
    gpio_bit_set(UWB2_CS_PORT, UWB2_CS_PIN);
    gpio_init(UWB1_CS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, UWB1_CS_PIN);
    gpio_init(UWB2_CS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, UWB2_CS_PIN);

    // SPI0 参数初始化
    spi_parameter_struct spi_init_struct;
    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX; 
    spi_init_struct.device_mode          = SPI_MASTER;               
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;       
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;   
    spi_init_struct.nss                  = SPI_NSS_SOFT;             
    spi_init_struct.prescale             = SPI_PSC_64;               
    spi_init_struct.endian               = SPI_ENDIAN_MSB;           
    spi_init(SPI0, &spi_init_struct);

    // 强制拉高内部 NSS，防止死机
    spi_nss_internal_high(SPI0);
    spi_enable(SPI0);
}

static port_dwic_isr_t port_dwic_isr = NULL;
unsigned long portGetTickCnt(void) { return 0; }

void delay_1us(uint32_t count) {
    volatile uint32_t i;
    for(i = 0; i < count * 15; i++) { __NOP(); }
}

int usleep(uint32_t usec) { delay_1us(usec); return 0; }
void Sleep(uint32_t x) { DelayMs(x); }

/* ---------------- 3. 智能复位函数 ---------------- */
void reset_DWIC(void)
{
    if(Active_UWB == 1) {
        gpio_init(DW1_RSTn_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, DW1_RSTn);
        gpio_bit_reset(DW1_RSTn_GPIO, DW1_RSTn);
        Sleep(2);
        gpio_init(DW1_RSTn_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, DW1_RSTn);
    } else {
        gpio_init(DW2_RSTn_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, DW2_RSTn);
        gpio_bit_reset(DW2_RSTn_GPIO, DW2_RSTn);
        Sleep(2);
        gpio_init(DW2_RSTn_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, DW2_RSTn);
    }
    Sleep(5); 
}

void wakeup_device_with_io(void) { SET_WAKEUP_PIN_IO_HIGH; WAIT_500uSEC; SET_WAKEUP_PIN_IO_LOW; }

void make_very_short_wakeup_io(void) {
    uint8_t cnt; SET_WAKEUP_PIN_IO_HIGH;
    for (cnt=0; cnt<10; cnt++) { __NOP(); }
    SET_WAKEUP_PIN_IO_LOW;
}

void port_set_dw_ic_spi_slowrate(void) {
    spi_disable(SPI0);                           
    SPI_CTL0(SPI0) &= ~((uint32_t)SPI_CTL0_PSC); 
    SPI_CTL0(SPI0) |= SPI_PSC_64;                
    spi_nss_internal_high(SPI0);             
    spi_enable(SPI0);                            
}

void port_set_dw_ic_spi_fastrate(void) {
    spi_disable(SPI0);                           
    SPI_CTL0(SPI0) &= ~((uint32_t)SPI_CTL0_PSC); 
    SPI_CTL0(SPI0) |= SPI_PSC_8;                 
    spi_nss_internal_high(SPI0);             
    spi_enable(SPI0);                            
}

void process_deca_irq(void) {
    while(port_CheckEXT_IRQ() != 0) {
        if(port_dwic_isr) port_dwic_isr();
    }
}

void port_DisableEXT_IRQ(void) { nvic_irq_disable(EXTI5_9_IRQn); }
void port_EnableEXT_IRQ(void) { nvic_irq_enable(EXTI5_9_IRQn, 0, 0); }
uint32_t port_GetEXT_IRQStatus(void) { return 1; }

uint32_t port_CheckEXT_IRQ(void) { 
    if(Active_UWB == 1) return gpio_input_bit_get(DECA1_IRQ_GPIO, DECA1_IRQ);
    else return gpio_input_bit_get(DECA2_IRQ_GPIO, DECA2_IRQ);
}

void port_set_dwic_isr(port_dwic_isr_t dwic_isr) {
    port_DisableEXT_IRQ();
    port_dwic_isr = dwic_isr;
    port_EnableEXT_IRQ();
}

void deca_sleep(unsigned int time_ms) { Sleep(time_ms); }
void deca_usleep(unsigned long time_us) { usleep(time_us); }
decaIrqStatus_t decamutexon(void) { __disable_irq(); return 0; }
void decamutexoff(decaIrqStatus_t s) { __enable_irq(); }

/* ---------------- 4. 智能读写底层 (带超时防卡死保护) ---------------- */
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer)
{
    int i; uint32_t timeout;
    UWB_CS_Low(); // 智能拉低当前激活芯片的片选

    for(i = 0; i < headerLength; i++) {
        timeout = 0xFFFF;
        while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) && timeout--);
        spi_i2s_data_transmit(SPI0, headerBuffer[i]);
        timeout = 0xFFFF;
        while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) && timeout--);
        spi_i2s_data_receive(SPI0); 
    }

    for(i = 0; i < bodyLength; i++) {
        timeout = 0xFFFF;
        while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) && timeout--);
        spi_i2s_data_transmit(SPI0, bodyBuffer[i]);
        timeout = 0xFFFF;
        while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) && timeout--);
        spi_i2s_data_receive(SPI0);
    }

    UWB_CS_High(); // 智能拉高片选
    return 0;
}

int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8)
{
    int i; uint32_t timeout;
    UWB_CS_Low(); 

    for(i = 0; i < headerLength; i++) {
        timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) && timeout--);
        spi_i2s_data_transmit(SPI0, headerBuffer[i]);
        timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) && timeout--);
        spi_i2s_data_receive(SPI0);
    }
    for(i = 0; i < bodyLength; i++) {
        timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) && timeout--);
        spi_i2s_data_transmit(SPI0, bodyBuffer[i]);
        timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) && timeout--);
        spi_i2s_data_receive(SPI0);
    }
    timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) && timeout--);
    spi_i2s_data_transmit(SPI0, crc8);
    timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) && timeout--);
    spi_i2s_data_receive(SPI0);

    UWB_CS_High(); 
    return 0;
}

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer)
{
    int i; uint32_t timeout;
    UWB_CS_Low(); 

    for(i = 0; i < headerLength; i++) {
        timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) && timeout--);
        spi_i2s_data_transmit(SPI0, headerBuffer[i]);
        timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) && timeout--);
        spi_i2s_data_receive(SPI0);
    }
    for(i = 0; i < readLength; i++) {
        timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) && timeout--);
        spi_i2s_data_transmit(SPI0, 0xFF); 
        timeout = 0xFFFF; while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) && timeout--);
        readBuffer[i] = spi_i2s_data_receive(SPI0); 
    }

    UWB_CS_High();
    return 0;
}