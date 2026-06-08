/*!
 *  \file    delay.h
 *  \brief   Delay compatibility wrapper — maps GD32F3 names → GD32A7
 *
 *  GD32F3:  delay_ms(), delay_us(), DelayMs(), DelayXms()
 *  GD32A7:  delay_1ms(), delay_1us()  (from systick.h / uwb_port.h)
 */

#ifndef __DELAY_H
#define __DELAY_H

#include "systick.h"      /* delay_1ms() */
#include "uwb_port.h"     /* delay_1us() */

/* ---- map old GD32F3 names to GD32A7 ---- */
#define delay_init()      /* no-op on GD32A7 — SysTick already configured */
#define delay_ms(x)       delay_1ms(x)
#define delay_us(x)       delay_1us(x)
#define DelayMs(x)        delay_1ms(x)
#define DelayXms(x)       delay_1ms(x)
#define DelayUs(x)        delay_1us(x)

#endif /* __DELAY_H */
