/**
  ******************************************************************************
  * @file    Delay_Config.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.20
  * @brief   两个不精确延时的函数，主要是为了初始化的时候使用
             不允许在任务开始跑（FreeRTOS开启）之后调用这里的函数！只能在初始化
             BSP_All_Init的时候使用！
  ******************************************************************************
*/
#include <stdint.h>

#ifndef __DELAY_CONFIG_H
#define __DELAY_CONFIG_H

/****************************函数声明****************************/
void Delay_ms_Unexact(uint32_t nms);
void Delay_us_Unexact(uint32_t nus);

#endif
