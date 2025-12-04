/**
  ******************************************************************************
  * @file    IRQ_Handler_Config.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.10
  * @brief   各个STM32中断服务函数的配置，主要是CAN、USART、WWDG的中断服务函数
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdio.h>
#include "USART_Communication.h"
#include "CAN_Communication.h"
#include "WWDG_Config.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Chassis.h"

/**
  * @brief  CAN1发送中断服务函数
  * @param  无
  * @retval 无
  */
void CAN1_TX_IRQHandler(void)
{
    /****************检测发送邮箱空****************/
    if(CAN_GetITStatus(CAN1,CAN_IT_TME) == SET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
    }
}

/**
  * @brief  CAN1接收中断服务函数
  * @note   利用不同ID号来处理数据
  * @param  无
  * @retval 无
  */
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg CAN_RxMsg;

    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &CAN_RxMsg);

        /****************根据不同ID号来处理数据****************/
        //Lucky:下面这个switch可以试着封装一下
        switch(CAN_RxMsg.StdId)
        {
            //case:
        }
        GST_SystemMonitor.CAN1Rx_cnt++;
    }
}

/**
  * @brief  CAN2发送中断服务函数
  * @param  无
  * @retval 无
  */
void CAN2_TX_IRQHandler(void)
{
    /****************检测发送邮箱空****************/
    if(CAN_GetITStatus(CAN2, CAN_IT_TME) == SET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
    }
}

/**
  * @brief  CAN2接收中断服务函数
  * @note   处理轮毂电机的CAN数据
  * @param  无
  * @retval 无
  */
void CAN2_RX0_IRQHandler(void)
{
    /****************如果不是CAN2接收中断，直接返回****************/
    if(CAN_GetITStatus(CAN2, CAN_IT_FMP0) != SET)
    {return;}

    CanRxMsg CAN_RxMsg;

    /****************清除中断标志位、接收数据****************/
    CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
    CAN_Receive(CAN2, CAN_FIFO0, &CAN_RxMsg);

    /****************根据不同ID号来处理数据****************/
    switch(CAN_RxMsg.StdId)
    {
        /****************右轮毂电机****************/
        case CANID_HM2:
            CANRx_C620DataParse(&GstCH_HM2RxC620Data, &CAN_RxMsg); //解析CAN数据到电调反馈结构体中
            GST_SystemMonitor.HubMotor2Rx_cnt++;
            break;
        
        /****************左轮毂电机****************/
        case CANID_HM1:
            CANRx_C620DataParse(&GstCH_HM1RxC620Data, &CAN_RxMsg); //解析CAN数据到电调反馈结构体中
            GST_SystemMonitor.HubMotor1Rx_cnt++;
            break;
    }
    GST_SystemMonitor.CAN2Rx_cnt++;
}

/**
  * @brief  判断串口1单次接收是否完成的函数
  * @note   判断原理就是DMA每传输一个数据，它的数据传输计数器就会减1
  *         USART1Rx使用的DMA配置为循环模式，减到0后会自动变回设定的重装值UA1RxDMAbuf_LEN，此时一次接收结束
  *         所以只需要看DMA的DataCounter就知道单次接收是否结束了
  * @param  无
  * @retval true：单次接收结束  false：单次接收未结束
*/
bool __IsUSART1SingleRecOK(void)
{
    if(DMA_GetCurrDataCounter(USART1_RX_STREAM) == UA1RxDMAbuf_LEN)
    {return true;}

    else
    {return false;}
}

/**
  * @brief  USART1接收中断服务函数，对遥控器传输的数据进行解析，从而控制机器人
  * @note   串口1，连接遥控接收机DR16。
  *         中断进入条件：空闲中断（IDLE中断）
  *         中断产生机制：DR16每隔7ms通过DBus发送一帧数据（18字节），一帧数据发送完成后，IDLE中断就会被触发
  * @param  无
  * @retval 无
*/
void USART1_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != SET)
    {return;}
    
    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    USART1->SR;
    USART1->DR;	

    /****************************串口1接收数据****************************/
    if(__IsUSART1SingleRecOK())
    {
        GF_USART1_RxDone = true;        //串口1接收完成标志位，数据在ReceiverTask里面被处理
        GST_SystemMonitor.USART1Rx_cnt++;
    }
    else
    {
        DMA_Cmd(USART1_RX_STREAM, DISABLE);                         //设置当前计数值前先禁用DMA
        DMA_SetCurrDataCounter(USART1_RX_STREAM, UA1RxDMAbuf_LEN);  //设置当前待发的数据的数量
        DMA_Cmd(USART1_RX_STREAM, ENABLE);                          //启用串口DMA接收
    }
}

/**
  * @brief  判断串口2单次接收是否完成的函数
  * @note   判断原理就是DMA每传输一个数据，它的数据传输计数器就会减1
  *         USART2Rx使用的DMA配置为循环模式，减到0后会自动变回设定的重装值UA2RxDMAbuf_LEN，此时一次接收结束
  *         所以只需要看DMA的DataCounter就知道单次接收是否结束了
  * @param  无
  * @retval true：单次接收结束  false：单次接收未结束
*/
bool __IsUSART2SingleRecOK(void)
{
    if(DMA_GetCurrDataCounter(USART2_RX_STREAM) == UA2RxDMAbuf_LEN)
    {return true;}

    else
    {return false;}
}

/**
  * @brief  串口2的中断服务函数，与云台云控通讯
  * @note   
  *         中断进入条件：空闲中断（IDLE中断）
  * @param  无
  * @retval 无
*/
void USART2_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != SET)
    {return;}

    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    USART2->SR;
    USART2->DR;		
        
    /****************************串口2接收数据以及数据处理****************************/
    if(__IsUSART2SingleRecOK())
    {
        UA2Rx_IMU1DataProcess();//IMU1，云台云控数据的接收、处理
        GST_SystemMonitor.USART2Rx_cnt++;
    }
    else
    {
        DMA_Cmd(USART2_RX_STREAM, DISABLE);                         //设置当前计数值前先禁用DMA
        DMA_SetCurrDataCounter(USART2_RX_STREAM, UA2RxDMAbuf_LEN);  //设置当前待发的数据的数量
        DMA_Cmd(USART2_RX_STREAM, ENABLE);                          //启用串口DMA接收
    }
}

/**
  * @brief  串口3的中断服务函数，作为备用（串口发送时会进入，但是没有对数据进行处理）
  * @note   中断进入条件：空闲中断（IDLE中断），用电脑给主控发消息的时候进入。
  *         这个函数暂时没有什么作用，如果后续有用可以自行修改内部内容
  * @param  无
  * @retval 无
*/
void USART3_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != SET)
    {return;}

    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    USART3->SR;
    USART3->DR;

    /****************************串口3数据接收****************************/
    GST_SystemMonitor.USART3Rx_cnt++;

    DMA_Cmd(USART3_RX_STREAM, DISABLE);         //设置当前计数值前先禁用DMA
    USART3_RX_STREAM->NDTR = UA3RxDMAbuf_LEN;   //设置当前待发的数据的数量:Number of Data units to be TRansferred
    DMA_Cmd(USART3_RX_STREAM, ENABLE);          //启用串口DMA接收
}

/**
  * @brief  判断串口4单次接收是否完成的函数
  * @note   判断原理就是DMA每传输一个数据，它的数据传输计数器就会减1
  *         UART4Rx使用的DMA配置为循环模式，减到0后会自动变回设定的重装值UA4RxDMAbuf_LEN，此时一次接收结束
  *         所以只需要看DMA的DataCounter就知道单次接收是否结束了
  * @param  无
  * @retval true：单次接收结束  false：单次接收未结束
*/
bool __IsUART4SingleRecOK(void)
{
    
    if(DMA_GetCurrDataCounter(UART4_RX_STREAM) == UA4RxDMAbuf_LEN)
    {return true;}

    else
    {return false;}
}

/**
  * @brief  串口4的中断服务函数，作为和IMU2（底盘云控）的通讯
  * @note   主要是关节电机的反馈数据、底盘的IMU数据
  * @param  无
  * @retval 无
*/
void UART4_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != SET)
    {return;}

    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    UART4->SR;
    UART4->DR;

    /****************************串口4数据接收、解析****************************/
    if(__IsUART4SingleRecOK())
    {
        UA4Rx_IMU2DataProcess();        //IMU2底盘云控数据的接收、处理
        GST_SystemMonitor.UART4Rx_cnt++;
    }
    else
    {
        DMA_Cmd(UART4_RX_STREAM, DISABLE);         //设置当前计数值前先禁用DMA
        UART4_RX_STREAM->NDTR = UA4RxDMAbuf_LEN;   //设置当前待发的数据的数量:Number of Data units to be TRansferred
        DMA_Cmd(UART4_RX_STREAM, ENABLE);          //启用串口DMA接收
    }
}

/**
  * @brief  窗口看门狗中断服务函数
  * @note   该函数用于Debug的时候打印相关数据
  * @param  无
  * @retval 无
*/
void WWDG_IRQHandler()
{
    WWDG_SetCounter(WWDG_CounterValue);  //喂狗，值在0x40-0x7F之间
    WWDG_ClearFlag();
}

/**
  * @brief  硬件错误中断服务函数
  * @note   当出现堆栈错误、数据类型错误、Flash被覆盖等等问题的时候会跳转到这里
  * @param  无
  * @retval 无
*/
// void HardFault_Handler(void)
// {
//     while(1)
//     {
//         printf("HardFault!");
//     }
// }
