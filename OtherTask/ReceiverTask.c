/**
  ******************************************************************************
  * @file    ReceiverTask.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.13
  * @brief   遥控接收机的数据解析、处理任务
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "USART_Communication.h"
#include "GlobalDeclare_General.h"
#include "ReceiverTask.h"

const static TickType_t S_ReceiveTaskPeriod = 1; //任务周期，单位ms

/**
  * @brief  这是FreeRTOS中的一个任务函数，用来做接收机接收后的操作
  * @note   while(1)是因为FreeRTOS的任务函数必须是一个死循环，因为没有特殊含义所以不给缩进了
  * @param  无
  * @retval 无
  */
void ReceiverTask(void* arg)
{
while(1)
{
    if(GF_USART1_RxDone == true)    //串口1接收完成，则进行接收任务。也是用了DMA接收方式
    {
        GF_USART1_RxDone = false;   //清除串口1接收完成标志位
        UA1Rx_ReceiverDataProcess();//串口1接收数据处理，解析到GST_Receiver结构体中
    }
    vTaskDelay(S_ReceiveTaskPeriod);
}
}

