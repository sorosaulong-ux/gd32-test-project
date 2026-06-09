/*!
 *  \file    key.h
 *  \brief   KEY1 (PC13) 按键驱动 — 轮询 + 长按去抖
 */

#ifndef __KEY_H
#define __KEY_H

#include "gd32a7xx.h"

void key1_init(void);
void key1_poll(void);

#endif
