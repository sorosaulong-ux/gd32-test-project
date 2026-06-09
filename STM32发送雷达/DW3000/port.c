#include "port.h"
#include "MySPI.h"
#include "deca_device_api.h"
#include "Delay.h"  // 这里使用你工程里现有的延时函数 (江协科技的Delay模块)

/* ---------------- 1. 延时函数封装 ---------------- */
void Sleep(uint32_t Delay) {
    Delay_ms(Delay); 
}
void deca_sleep(unsigned int time_ms) { 
    Sleep(time_ms); 
}
void deca_usleep(unsigned long time_us) {
    Delay_us(time_us);
}

/* ---------------- 2. 硬件引脚整体初始化 ---------------- */
void UWB_Hardware_Init(void)
{
    /* 1. 初始化你的自定义 SPI (PA4~PA7) */
    MySPI_Init(); 

    /* 2. 初始化复位引脚(PB0) 和 中断引脚(PB1) */
    RCC_APB2PeriphClockCmd(DW_RSTn_CLK | DECA_IRQ_CLK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // RST (PB0) 初始化为推挽输出，默认拉高
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = DW_RSTn_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW_RSTn_PORT, &GPIO_InitStructure);
    GPIO_SetBits(DW_RSTn_PORT, DW_RSTn_PIN);

    // IRQ (PB1) 初始化为下拉输入 (DW3000 中断是高电平触发)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Pin = DECA_IRQ_PIN;
    GPIO_Init(DECA_IRQ_PORT, &GPIO_InitStructure);
}

/* ---------------- 3. UWB 芯片复位逻辑 ---------------- */
void reset_DWIC(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 配置 PB0 为推挽输出，并强制拉低 2 毫秒
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = DW_RSTn_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW_RSTn_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(DW_RSTn_PORT, DW_RSTn_PIN);
    
    Sleep(2);
    
    // 配置 PB0 为浮空输入，释放总线，让 DW3000 内部的上拉电阻自己拉高
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = DW_RSTn_PIN;
    GPIO_Init(DW_RSTn_PORT, &GPIO_InitStructure);
    
    Sleep(5);
}

/* ---------------- 4. SPI 速率切换 ---------------- */
void port_set_dw_ic_spi_slowrate(void) {
    // 降速：通过修改 CR1 寄存器将分频器设为 128 (72MHz / 128 = 562.5kHz)
    SPI_Cmd(SPI1, DISABLE);
    SPI1->CR1 = (SPI1->CR1 & ~SPI_BaudRatePrescaler_256) | SPI_BaudRatePrescaler_128;
    SPI_Cmd(SPI1, ENABLE);
}

void port_set_dw_ic_spi_fastrate(void) {
    // 提速：将分频器设为 8 (72MHz / 8 = 9MHz)
    SPI_Cmd(SPI1, DISABLE);
    SPI1->CR1 = (SPI1->CR1 & ~SPI_BaudRatePrescaler_256) | SPI_BaudRatePrescaler_8;
    SPI_Cmd(SPI1, ENABLE);
}

uint32_t port_CheckEXT_IRQ(void) { 
    return GPIO_ReadInputDataBit(DECA_IRQ_PORT, DECA_IRQ_PIN);
}

/* ---------------- 5. 核心：完美融合 MySPI 的读写函数 ---------------- */
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer)
{
    MySPI_Start();  // 软拉低片选 (PA4 = 0)
    for(int i = 0; i < headerLength; i++) {
        MySPI_SwapByte(headerBuffer[i]);
    }
    for(int i = 0; i < bodyLength; i++) {
        MySPI_SwapByte(bodyBuffer[i]);
    }
    MySPI_Stop();   // 软拉高片选 (PA4 = 1)
    return 0;
}

int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8)
{
    MySPI_Start();
    for(int i = 0; i < headerLength; i++) MySPI_SwapByte(headerBuffer[i]);
    for(int i = 0; i < bodyLength; i++)   MySPI_SwapByte(bodyBuffer[i]);
    MySPI_SwapByte(crc8);
    MySPI_Stop();
    return 0;
}

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer)
{
    MySPI_Start();
    for(int i = 0; i < headerLength; i++) {
        MySPI_SwapByte(headerBuffer[i]);
    }
    for(int i = 0; i < readLength; i++) {
        readBuffer[i] = MySPI_SwapByte(0xFF); // 发送 Dummy 字节获取数据
    }
    MySPI_Stop();
    return 0;
}

decaIrqStatus_t decamutexon(void) { __disable_irq(); return 0; }
void decamutexoff(decaIrqStatus_t s) { __enable_irq(); }
/* ==========================================================
 * 补充缺失的唤醒函数以通过 deca_device.c 的编译检查
 * 目前不使用睡眠模式，因此保留为空函数即可
 * ========================================================== */
void wakeup_device_with_io(void) 
{
    // Dummy function
}

void make_very_short_wakeup_io(void) 
{
    // Dummy function
}
