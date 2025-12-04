/**
  ******************************************************************************
  * @file    WWDG_Config.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.19
  * @brief   窗口看门狗初始化
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include "stm32f4xx.h"

/****************************常量、宏定义定义（不需要修改）****************************/
const uint8_t WWDG_CounterValue = (0x40 | 0x2F); //WWDG喂狗值 = 0x6F
const uint8_t WWDG_WindowValue  = (0x40 | 0x3F); //WWDG上窗口值 = 0x7F

/**
  * @brief  窗口看门狗初始化的函数
  * @note   关于喂狗时间的计算：APB1时钟频率42MHz
            WWDG单次递减时间(单位ms)：1/(APB1频率/4096/分频系数)*1000 = 0.195047619
            喂狗最短时间：(喂狗值-上窗口值)*单次递减时间 = 负数（喂狗的时候必定小于上窗口值）
            喂狗最长时间：(喂狗值-0x40)*单次递减时间 = 9.167238ms
  * @param  无
  * @retval 无
  */
void My_WWDG_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    
    /**************************** 开启WWDG时钟 ****************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,ENABLE);
    
    /**************************** 配置WWDG ****************************/
    WWDG_SetPrescaler(WWDG_Prescaler_2);//2分频
    WWDG_SetWindowValue(WWDG_WindowValue);//配置上窗口值

    /**************************** 配置WWDG中断 ****************************/
    NVIC_InitStruct.NVIC_IRQChannel     = WWDG_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd  = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStruct);
    WWDG_ClearFlag();//清除中断标志位
    WWDG_EnableIT(); //使能中断
    
    WWDG_Enable(WWDG_CounterValue);//启用WWDG时要顺带喂狗
}
