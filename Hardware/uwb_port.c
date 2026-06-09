/*!
 *  \file    uwb_port.c
 *  \brief   DW3000 port layer — GD32A7xx (software SPI)
 */

#include "uwb_port.h"
#include "systick.h"
#include <stdio.h>

uint8_t Active_UWB = 1;
static port_dwic_isr_t s_dwic_isr = NULL;
static uint32_t bb_nop = 2;   /* fast=2 (~13MHz), slow=200 (~130kHz) */

/* ------------------------------------------------------------------ */
static void bb_delay(void)
{
    volatile uint32_t n = bb_nop;
    while (n--) { __NOP(); }
}

/* ------------------------------------------------------------------ */
static void sck_l(void)  { GPIO_BC(UWB_SPI_SCL_PORT)  = UWB_SPI_SCL_PIN; }
static void sck_h(void)  { GPIO_BOP(UWB_SPI_SCL_PORT) = UWB_SPI_SCL_PIN; }
static void mosi_l(void) { GPIO_BC(UWB_SPI_MOSI_PORT) = UWB_SPI_MOSI_PIN; }
static void mosi_h(void) { GPIO_BOP(UWB_SPI_MOSI_PORT) = UWB_SPI_MOSI_PIN; }
static uint8_t miso_r(void) { return gpio_input_bit_get(UWB_SPI_MISO_PORT, UWB_SPI_MISO_PIN); }

/* ------------------------------------------------------------------ */
static void cs_low(void)
{
    if (Active_UWB == 1) GPIO_BC(UWB1_CS_PORT) = UWB1_CS_PIN;
    else                  GPIO_BC(UWB2_CS_PORT) = UWB2_CS_PIN;
}

static void cs_high(void)
{
    if (Active_UWB == 1) GPIO_BOP(UWB1_CS_PORT) = UWB1_CS_PIN;
    else                  GPIO_BOP(UWB2_CS_PORT) = UWB2_CS_PIN;
}

/* ====================================================================
 *  UWB_SPI_Init()
 * ====================================================================*/
void UWB_SPI_Init(void)
{
    prpu_periph_unlock(PRPU_GPIOL);
    prpu_periph_unlock(PRPU_GPIOI);

    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOL);
    rcu_periph_clock_enable(RCU_GPIOI);

    /* SCK */
    GPIO_BC(UWB_SPI_SCL_PORT) = UWB_SPI_SCL_PIN;
    gpio_mode_set(UWB_SPI_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, UWB_SPI_SCL_PIN);
    gpio_output_options_set(UWB_SPI_SCL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, UWB_SPI_SCL_PIN);
    /* MOSI */
    GPIO_BC(UWB_SPI_MOSI_PORT) = UWB_SPI_MOSI_PIN;
    gpio_mode_set(UWB_SPI_MOSI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, UWB_SPI_MOSI_PIN);
    gpio_output_options_set(UWB_SPI_MOSI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, UWB_SPI_MOSI_PIN);
    /* MISO */
    gpio_mode_set(UWB_SPI_MISO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, UWB_SPI_MISO_PIN);

    /* CS1 */
    GPIO_BOP(UWB1_CS_PORT) = UWB1_CS_PIN;
    gpio_mode_set(UWB1_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, UWB1_CS_PIN);
    gpio_output_options_set(UWB1_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, UWB1_CS_PIN);
    /* CS2 */
    GPIO_BOP(UWB2_CS_PORT) = UWB2_CS_PIN;
    gpio_mode_set(UWB2_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, UWB2_CS_PIN);
    gpio_output_options_set(UWB2_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, UWB2_CS_PIN);

    /* IRQ */
    gpio_mode_set(UWB1_IRQ_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, UWB1_IRQ_PIN);
    gpio_mode_set(UWB2_IRQ_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, UWB2_IRQ_PIN);

    /* WAKEUP */
    GPIO_BC(UWB_WAKE_PORT) = UWB_WAKE_PIN;
    gpio_mode_set(UWB_WAKE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, UWB_WAKE_PIN);
    gpio_output_options_set(UWB_WAKE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, UWB_WAKE_PIN);
}

/* ====================================================================
 *  SPI speed
 * ====================================================================*/
void port_set_dw_ic_spi_slowrate(void) { bb_nop = 12; }
void port_set_dw_ic_spi_fastrate(void) { bb_nop = 2;  }

/* ====================================================================
 *  reset_DWIC()
 * ====================================================================*/
void reset_DWIC(void)
{
    uint32_t rst_port, rst_pin;
    if (Active_UWB == 1) { rst_port = UWB1_RST_PORT; rst_pin = UWB1_RST_PIN; }
    else                 { rst_port = UWB2_RST_PORT; rst_pin = UWB2_RST_PIN; }

    gpio_mode_set(rst_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, rst_pin);
    gpio_output_options_set(rst_port, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, rst_pin);
    GPIO_BC(rst_port) = rst_pin;
    Sleep(2);

    gpio_mode_set(rst_port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, rst_pin);
    Sleep(5);
}

/* ====================================================================
 *  WAKEUP
 * ====================================================================*/
void wakeup_device_with_io(void)  { SET_WAKEUP_PIN_IO_HIGH; usleep(500); SET_WAKEUP_PIN_IO_LOW; }

void make_very_short_wakeup_io(void)
{
    SET_WAKEUP_PIN_IO_HIGH;
    for (volatile int i = 0; i < 10; i++) { __NOP(); }
    SET_WAKEUP_PIN_IO_LOW;
}

/* ====================================================================
 *  Bit‑bang byte
 * ====================================================================*/
static uint8_t bb_xfer(uint8_t tx)
{
    uint8_t rx = 0;
    for (int b = 7; b >= 0; b--) {
        if (tx & (1U << b)) mosi_h(); else mosi_l();
        bb_delay();
        sck_h();
        bb_delay();
        if (miso_r()) rx |= (1U << b);
        sck_l();
        bb_delay();
    }
    return rx;
}

/* ====================================================================
 *  SPI I/O
 * ====================================================================*/
int writetospi(uint16_t hdr_len, const uint8_t *hdr,
               uint16_t body_len, const uint8_t *body)
{
    cs_low();
    for (int i = 0; i < hdr_len;  i++) bb_xfer(hdr[i]);
    for (int i = 0; i < body_len; i++) bb_xfer(body[i]);
    cs_high();
    return 0;
}

int writetospiwithcrc(uint16_t hdr_len, const uint8_t *hdr,
                       uint16_t body_len, const uint8_t *body, uint8_t crc)
{
    cs_low();
    for (int i = 0; i < hdr_len;  i++) bb_xfer(hdr[i]);
    for (int i = 0; i < body_len; i++) bb_xfer(body[i]);
    bb_xfer(crc);
    cs_high();
    return 0;
}

int readfromspi(uint16_t hdr_len, uint8_t *hdr,
                 uint16_t rd_len, uint8_t *rd_buf)
{
    cs_low();
    for (int i = 0; i < hdr_len; i++) bb_xfer(hdr[i]);
    for (int i = 0; i < rd_len;  i++) rd_buf[i] = bb_xfer(0xFF);
    cs_high();
    return 0;
}

/* ====================================================================
 *  IRQ
 * ====================================================================*/
uint32_t port_CheckEXT_IRQ(void)
{
    if (Active_UWB == 1)
        return gpio_input_bit_get(UWB1_IRQ_PORT, UWB1_IRQ_PIN);
    return gpio_input_bit_get(UWB2_IRQ_PORT, UWB2_IRQ_PIN);
}

void port_DisableEXT_IRQ(void)
{
    nvic_irq_disable(EXTI5_9_IRQn);
    nvic_irq_disable(EXTI10_15_IRQn);
}

void port_EnableEXT_IRQ(void)
{
    rcu_periph_clock_enable(RCU_SYSCFG);
    /* UWB1: PI.8 */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOI, EXTI_SOURCE_PIN8);
    exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_8);
    nvic_irq_enable(EXTI5_9_IRQn, 1, 0);
    /* UWB2: PI.10 */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOI, EXTI_SOURCE_PIN10);
    exti_init(EXTI_10, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_10);
    nvic_irq_enable(EXTI10_15_IRQn, 1, 0);
}

uint32_t port_GetEXT_IRQStatus(void) { return 1; }

void port_set_dwic_isr(port_dwic_isr_t isr)
{
    port_DisableEXT_IRQ();
    s_dwic_isr = isr;
    port_EnableEXT_IRQ();
}

void process_deca_irq(void)
{
    while (port_CheckEXT_IRQ() != 0)
        if (s_dwic_isr) s_dwic_isr();
}

void EXTI5_9_IRQHandler(void)
{
    if (SET == exti_interrupt_flag_get(EXTI_8)) {
        exti_interrupt_flag_clear(EXTI_8);
        if (s_dwic_isr) s_dwic_isr();
    }
}

void EXTI10_15_IRQHandler(void)
{
    if (SET == exti_interrupt_flag_get(EXTI_10)) {
        exti_interrupt_flag_clear(EXTI_10);
        if (s_dwic_isr) s_dwic_isr();
    }
}

/* ====================================================================
 *  Timing
 * ====================================================================*/
void Sleep(uint32_t x)       { delay_1ms(x); }
void deca_sleep(unsigned int ms)  { delay_1ms(ms); }

void delay_1us(uint32_t count)
{
    volatile uint32_t i = count * 40;
    while (i--) { __NOP(); }
}

int  usleep(uint32_t usec)     { delay_1us(usec); return 0; }
void deca_usleep(unsigned long us) { delay_1us((uint32_t)us); }

decaIrqStatus_t decamutexon(void)  { __disable_irq(); return 0; }
void decamutexoff(decaIrqStatus_t s) { __enable_irq(); (void)s; }

/* ARMCLANG __aeabi_assert stub (deca_device.c uses assert()) */
void __aeabi_assert(const char *expr, const char *file, int line)
{
    (void)expr; (void)file; (void)line;
    while (1);
}

/* ---- ms tick counter ---- */
static volatile uint32_t s_tick_ms = 0;
void uwb_tick_inc(void)  { s_tick_ms++; }
uint32_t uwb_tick_get(void) { return s_tick_ms; }
unsigned long portGetTickCnt(void) { return s_tick_ms; }
