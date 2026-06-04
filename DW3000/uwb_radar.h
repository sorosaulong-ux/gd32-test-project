#ifndef __UWB_RADAR_H
#define __UWB_RADAR_H

#include "gd32f30x.h"

/* 雷达接口声明 */
uint8_t UWB_Radar_Init(void);
void UWB_Radar_Scan_And_Send(void);

#endif /* __UWB_RADAR_H */