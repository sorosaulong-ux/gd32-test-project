/*!
 *  \file    key.h
 *  \brief   KEY1 (PC13) + KEY3 (PL4) + KEY4 (PL5) 按键驱动
 */

#ifndef __KEY_H
#define __KEY_H

#include "gd32a7xx.h"

void key1_init(void);
void key1_poll(void);

void key3_init(void);
void key3_poll(void);
uint8_t key3_pressed(void);

void key4_init(void);
void key4_poll(void);
uint8_t key4_pressed(void);

#endif
