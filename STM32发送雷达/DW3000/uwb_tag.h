#ifndef __UWB_TAG_H
#define __UWB_TAG_H

#include "stm32f10x.h"

uint8_t UWB_Tag_Init(void);
uint8_t UWB_Tag_Execute_Ranging(double *distance_out);

#endif
