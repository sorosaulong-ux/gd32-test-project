#include "port.h"
#include "MySPI.h"
#include "deca_device_api.h"
#include "Delay.h"  // ����ʹ���㹤�������е���ʱ���� (��Э�Ƽ���Delayģ��)

/* ---------------- 1. ��ʱ������װ ---------------- */
void Sleep(uint32_t Delay) {
    Delay_ms(Delay); 
}
void deca_sleep(unsigned int time_ms) { 
    Sleep(time_ms); 
}
void deca_usleep(unsigned long time_us) {
    Delay_us(time_us);
}

/* ---------------- 2. Ӳ�����������ʼ�� ---------------- */
void UWB_Hardware_Init(void)
{
    /* 1. ��ʼ������Զ��� SPI (PA4~PA7) */
    MySPI_Init(); 

    /* 2. ��ʼ����λ����(PB0) �� �ж�����(PB1) �� WAKEUP(PB10) */
    RCC_APB2PeriphClockCmd(DW_RSTn_CLK | DECA_IRQ_CLK | DW_WKUP_CLK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // RST (PB0) ��ʼ��Ϊ���������Ĭ������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = DW_RSTn_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW_RSTn_PORT, &GPIO_InitStructure);
    GPIO_SetBits(DW_RSTn_PORT, DW_RSTn_PIN);

    // IRQ (PB1) ��ʼ��Ϊ�������� (DW3000 �ж��Ǹߵ�ƽ����)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_InitStructure.GPIO_Pin = DECA_IRQ_PIN;
    GPIO_Init(DECA_IRQ_PORT, &GPIO_InitStructure);

    // WAKEUP (PB10) ��ʼ��Ϊ����������Ĭ������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = DW_WKUP_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW_WKUP_PORT, &GPIO_InitStructure);
    GPIO_SetBits(DW_WKUP_PORT, DW_WKUP_PIN);
}

/* ---------------- 3. UWB оƬ��λ�߼� ---------------- */
void reset_DWIC(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // ���� PB0 Ϊ�����������ǿ������ 2 ����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = DW_RSTn_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW_RSTn_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(DW_RSTn_PORT, DW_RSTn_PIN);
    
    Sleep(2);
    
    // ���� PB0 Ϊ�������룬�ͷ����ߣ��� DW3000 �ڲ������������Լ�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = DW_RSTn_PIN;
    GPIO_Init(DW_RSTn_PORT, &GPIO_InitStructure);
    
    Sleep(5);
}

/* ---------------- 4. SPI �����л� ---------------- */
void port_set_dw_ic_spi_slowrate(void) {
    // ���٣�ͨ���޸� CR1 �Ĵ�������Ƶ����Ϊ 128 (72MHz / 128 = 562.5kHz)
    SPI_Cmd(SPI1, DISABLE);
    SPI1->CR1 = (SPI1->CR1 & ~SPI_BaudRatePrescaler_256) | SPI_BaudRatePrescaler_128;
    SPI_Cmd(SPI1, ENABLE);
}

void port_set_dw_ic_spi_fastrate(void) {
    // ���٣�����Ƶ����Ϊ 8 (72MHz / 8 = 9MHz)
    SPI_Cmd(SPI1, DISABLE);
    SPI1->CR1 = (SPI1->CR1 & ~SPI_BaudRatePrescaler_256) | SPI_BaudRatePrescaler_8;
    SPI_Cmd(SPI1, ENABLE);
}

uint32_t port_CheckEXT_IRQ(void) { 
    return GPIO_ReadInputDataBit(DECA_IRQ_PORT, DECA_IRQ_PIN);
}

/* ---------------- 5. ���ģ������ں� MySPI �Ķ�д���� ---------------- */
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer)
{
    MySPI_Start();  // ������Ƭѡ (PA4 = 0)
    for(int i = 0; i < headerLength; i++) {
        MySPI_SwapByte(headerBuffer[i]);
    }
    for(int i = 0; i < bodyLength; i++) {
        MySPI_SwapByte(bodyBuffer[i]);
    }
    MySPI_Stop();   // ������Ƭѡ (PA4 = 1)
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
        readBuffer[i] = MySPI_SwapByte(0xFF); // ���� Dummy �ֽڻ�ȡ����
    }
    MySPI_Stop();
    return 0;
}

decaIrqStatus_t decamutexon(void) { __disable_irq(); return 0; }
void decamutexoff(decaIrqStatus_t s) { __enable_irq(); }
/* ==========================================================
 * WAKEUP 引脚唤醒 (PB10)
 * DW3000 休眠后，通过 WAKEUP 引脚拉低 500us 唤醒
 * ========================================================== */
void wakeup_device_with_io(void) 
{
    GPIO_ResetBits(DW_WKUP_PORT, DW_WKUP_PIN);  // WAKEUP 拉低
    Delay_us(500);
    GPIO_SetBits(DW_WKUP_PORT, DW_WKUP_PIN);    // WAKEUP 拉高
}

void make_very_short_wakeup_io(void) 
{
    GPIO_ResetBits(DW_WKUP_PORT, DW_WKUP_PIN);  // WAKEUP 拉低
    Delay_us(10);
    GPIO_SetBits(DW_WKUP_PORT, DW_WKUP_PIN);    // WAKEUP 拉高
}
