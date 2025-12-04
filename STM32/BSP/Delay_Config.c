/**
  ******************************************************************************
  * @file    Delay_Config.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.20
  * @brief   两个不精确延时的函数，主要是为了初始化的时候使用
             不允许在任务开始跑之后调用这里的函数！只能在初始化
             BSP_All_Init的时候使用！
  ******************************************************************************
*/

#include "stm32f4xx.h"

/**
  * @brief  不精确延时毫秒级时间的函数
  * @note   只能在初始化时调用！任务中间请勿调用！
  * @param  nms：需要延时的毫秒数
  * @retval 无
  */
void Delay_ms_Unexact(uint32_t nms)
{
    uint32_t i = 0;
    for(i = 0; i < nms; i++)
    {
        uint32_t Temp = 42000;
        while(Temp--);
    }
}

/**
  * @brief  不精确延时微秒级时间的函数
  * @note   只能在初始化时调用！任务中间请勿调用！
  * @param  nus：需要延时的微秒数
  * @retval 无
  */
void Delay_us_Unexact(uint32_t nus)
{
    uint32_t i=0;
    for(i = 0; i < nus; i++)
    {
        uint32_t Temp = 40;
        while(Temp--);
    }
}
