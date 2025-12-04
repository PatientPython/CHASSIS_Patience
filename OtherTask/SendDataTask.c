/**
  ******************************************************************************
  * @file    SendDataTask.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.19
  * @brief   主控发送数据的任务，包括IMU1、IMU2、电机等等
  ******************************************************************************
*/
/****************************头文件引用****************************/
#include "FreeRTOS.h"
#include "task.h"
#include "GlobalDeclare_General.h"
#include "CAN_Communication.h"
#include "USART_Communication.h"

/****************************常量、宏定义定义（不需要修改）****************************/
const static TickType_t S_SendDataTaskPeriod = 1;     //任务周期，单位ms

/**************************************函数开始**************************************/
/**
  * @brief  SendDataTask，主控发送数据的任务函数
  * @note   这是FreeRTOS中的一个任务函数，用来发送数据给IMU1、IMU2、电机等
  *         while(1)是因为FreeRTOS的任务函数必须是一个死循环，没有特殊含义，就不给缩进了
  * @param  无
  * @retval 无
*/
void SendDataTask(void* arg)
{
while(1)
{
    /****************任务内容****************/
    UA2Tx_SendDataToIMU1();     //向云台云控IMU1发送数据
    UA4Tx_SendDataToIMU2();     //向底盘云控IMU2发送数据
    CANTx_SendCurrentToMotor(); //向除了关节电机外的各个电机发送电流
    
    /***********任务帧率计数器自增************/
    GST_SystemMonitor.SendDataTask_cnt++;

    /****************任务延时****************/
    vTaskDelay(S_SendDataTaskPeriod);
}
}
