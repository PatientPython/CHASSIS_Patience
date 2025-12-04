/**
  ******************************************************************************
  * @file   TIM_Config.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.11
  * @brief   定时器的初始化
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include "stm32f4xx.h"

/****************************常量、宏定义定义（不需要修改）****************************/
#define TIM5_Prescaler  8400 - 1
#define TIM5_Period     4294967296 - 1

/**
  * @brief  TIM5的初始化函数
  * @note   TIM5挂载在APB1上，APB1 = 84MHz，Pre = 8400，相当于10KHz，也就是周期100us递增一次
            TIM5的ARR和CNT是32bit的，溢出时间429496729600us，约为119.3个小时，时间够用了，不对溢出做处理（如果后面RM发展成了核动力电池可能需要修改）
            把定时器5用来当机器人运行的计时器
  * @param  无
  * @retval 无
  */
void My_TIM5_Init(void)
{
    TIM_TimeBaseInitTypeDef	TIM_TimeBaseInitStruct;

    /**************************** 开启TIM5时钟 ****************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    /**************************** 配置TIM5时基单元 ****************************/
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_Prescaler         = TIM5_Prescaler;
    TIM_TimeBaseInitStruct.TIM_Period            = TIM5_Period;
    TIM_TimeBaseInitStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;           //高级定时器才有的玩意，通用直接0就行
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);

    /**************************** 使能TIM5 ****************************/
    TIM_Cmd(TIM5, ENABLE);
}

/**
  * @brief  重置系统运行时间
  * @note   把运行时间重置为0
  * @param  无
  * @retval 无
  */
void RunTimeReset(void)
{
    TIM_SetCounter(TIM5,0);
}

/**
  * @brief  获取系统运行时间
  * @note   获取系统的运行时间，单位为毫秒。注意在开启系统的时候执行一下RunTimeReset()函数
  * @param  无
  * @retval uint32_t的一个数，表示系统从RunTimeReset()之后的运行时间，单位为ms
  */
uint32_t RunTimeGet(void)
{
    uint32_t TempTime = TIM_GetCounter(TIM5);
    TempTime = TempTime / 10;                   //换算为ms
    return TempTime;
}
