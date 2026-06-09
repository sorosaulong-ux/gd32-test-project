/*
*********************************************************************************************************
* UWB RANGING MODULE
*
* 双 UWB 模块 SS-TWR 测距封装组件 (GD32F30x)
*
* 文件名 : uwb_ranging.h
* 描  述 : 提供双节点初始化、单边双向测距的状态机调度接口
*********************************************************************************************************
*/
#ifndef __UWB_RANGING_H
#define __UWB_RANGING_H

#include "gd32f30x.h"

/* 测距状态返回值定义 */
#define UWB_RANGING_OK       0   // 测距成功，数据有效
#define UWB_ERR_UWB2_TIMEOUT 1   // 错误：UWB2(基站)未接收到POLL包 (可能拔线或距离过远)
#define UWB_ERR_UWB1_TIMEOUT 2   // 错误：UWB1(标签)未接收到RESP回信包

/* 外部公用接口声明 */
uint8_t UWB_System_Init(void);
uint8_t UWB_Execute_Ranging(double *distance_out);

#endif /* __UWB_RANGING_H */