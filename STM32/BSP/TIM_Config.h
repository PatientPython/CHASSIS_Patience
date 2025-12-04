/**
  ******************************************************************************
  * @file   TIM_Config.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.11
  * @brief   定时器的初始化函数声明
  ******************************************************************************
*/
#ifndef __TIM_CONFIG_H
#define __TIM_CONFIG_H

void My_TIM5_Init(void);

void RunTimeReset(void);
uint32_t RunTimeGet(void);

#endif
