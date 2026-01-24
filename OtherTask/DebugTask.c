/**
  ******************************************************************************
  * @file    DebugTask.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.11
  * @brief   DebugTask任务函数。
  ******************************************************************************
*/

/****************************头文件引用****************************/
/*非临时添加头文件，不要修改*/
#include "FreeRTOS.h"
#include "task.h"
#include "USART_Communication.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Chassis.h"
#include "GlobalDeclare_Gimbal.h"
#include "DebugTask.h"

/*临时添加的头文件，用完就可以删掉或者注释掉*/
#include "CAN_Communication.h"
#include "General_AuxiliaryFunc.h"
#include "Chassis_APIFunction.h"
#include "Algorithm.h"
extern uint32_t DebugTaskHighWaterMark   ;
extern uint32_t ReceiverTaskHighWaterMark;
extern uint32_t ChassisTaskHighWaterMark ;
extern uint32_t SendDataTaskHighWaterMark;
extern TaskHandle_t DebugTaskHandle;
extern TaskHandle_t ReceiverTaskHandle;
extern TaskHandle_t ChassisTaskHandle;
extern TaskHandle_t SendDataTaskHandle;

/****************************常量、宏定义定义（不需要修改）****************************/
const static TickType_t S_DebugTaskPeriod = 10;     //任务周期，单位ms
const static uint16_t S_AllFpsCountPeriod = 1000;   //函数执行周期，单位ms

/**************************************函数开始**************************************/
/**
  * @brief  DebugTask
  * @note   这是FreeRTOS中的一个任务函数，用来调试和打印数据，然后就可以在vofa中查看了
  *         while(1)是因为FreeRTOS的任务函数必须是一个死循环
  * @param  无
  * @retval 无
*/
void DebugTask(void* arg)
{
    /****************************任务开始****************************/
    while(1)
    {        
        /****************测试时使用**************/
        DebugTaskHighWaterMark      = uxTaskGetStackHighWaterMark(DebugTaskHandle);
        ReceiverTaskHighWaterMark   = uxTaskGetStackHighWaterMark(ReceiverTaskHandle);
        ChassisTaskHighWaterMark    = uxTaskGetStackHighWaterMark(ChassisTaskHandle);
        SendDataTaskHighWaterMark   = uxTaskGetStackHighWaterMark(SendDataTaskHandle);
        /****************测试时使用**************/

        /****************任务内容****************/
        VofaPrint();        //把数据打印到Vofa，打印的数据可以进入函数中自定义
        AllTaskFpsCount();  //统计各项任务的帧率

        /***********任务帧率计数器自增************/
        GST_SystemMonitor.DebugTask_cnt++;

        /****************任务延时****************/
        vTaskDelay(S_DebugTaskPeriod);

    }
}

/**
  * @brief  把数据打印到Vofa的函数
  * @note   底层实际上是调用串口3来打印，经过远程调试器（或Jlink）把数据传输到电脑，然后就可以用Vofa来查看相关数据了
  * @param  无
  * @retval 无
*/
//TODO 后续可以给这个函数加上一些参数，比如传入WatchFps就打印fps。传入自定义就打印自定义函数
void VofaPrint(void)
{
    USART3_DMA_printf
    ("C:%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n",
/*0*/   (float)GSTCH_Data.Leg1F_N,
/*1*/   (float)GSTCH_Data.Leg2F_N,
/*2*/   (float)GstCH_Leg1F_N_LPF.Input,
/*3*/   (float)GstCH_Leg2F_N_LPF.Input,
/*4*/   (float)GstCH_LegLen1PID.Des,
/*5*/   (float)GstCH_LegLen1PID.FB,
/*6*/   (float)GstCH_LegLen1PID.U,
/*7*/   (float)GSTCH_Data.VelBody_HM_Obs,
/*8*/   (float)GSTCH_Data.AccXFB,
/*9*/   (float)LegFFForce_Inertial_1
    );
}

// /*0*/   (float)GSTCH_Data.Leg1F_N,
// /*1*/   (float)GSTCH_Data.Leg2F_N,
// /*7*/   (float)G_u8Test,
// /*8*/   (float)G_u16Test,
// /*9*/   (float)G_u32Test,


/**
  * @brief  统计所有任务帧率的函数
  * @note   关于各个任务的正常帧率请见ReadMe.txt
  * @param  无
  * @retval 无
*/
void AllTaskFpsCount(void)
{
    static uint16_t S_FpsCountCnt = 0;//帧率统计计数器，保证帧率统计的时间是1s左右

    /**************** 如果时间小于执行周期直接返回 ****************/
    if(S_FpsCountCnt < (S_AllFpsCountPeriod/S_DebugTaskPeriod - 1)) //函数执行周期是S_DebugTaskPeriod，所以除上这个才能保证延时时间为S_AllFpsCountPeriod
    {
        S_FpsCountCnt++;
        return;
    }

    /**************** 如果时间等于S_AllFpsCountPeriod开始统计 ****************/
    /**********清空本函数计数器**********/
    S_FpsCountCnt = 0;

    /************CAN帧率统计************/
    GST_SystemMonitor.CAN1Rx_fps = GST_SystemMonitor.CAN1Rx_cnt;
    GST_SystemMonitor.CAN1Rx_cnt = 0;
    GST_SystemMonitor.CAN2Rx_fps = GST_SystemMonitor.CAN2Rx_cnt;
    GST_SystemMonitor.CAN2Rx_cnt = 0;

    GST_SystemMonitor.HubMotor1Rx_fps = GST_SystemMonitor.HubMotor1Rx_cnt;
    GST_SystemMonitor.HubMotor1Rx_cnt = 0;
    GST_SystemMonitor.HubMotor2Rx_fps = GST_SystemMonitor.HubMotor2Rx_cnt;
    GST_SystemMonitor.HubMotor2Rx_cnt = 0;

    /************串口帧率统计************/
    GST_SystemMonitor.USART1Rx_fps = GST_SystemMonitor.USART1Rx_cnt;
    GST_SystemMonitor.USART1Rx_cnt = 0;
    GST_SystemMonitor.USART2Rx_fps = GST_SystemMonitor.USART2Rx_cnt;
    GST_SystemMonitor.USART2Rx_cnt = 0;
    GST_SystemMonitor.USART3Rx_fps = GST_SystemMonitor.USART3Rx_cnt;
    GST_SystemMonitor.USART3Rx_cnt = 0;
    GST_SystemMonitor.UART4Rx_fps = GST_SystemMonitor.UART4Rx_cnt;
    GST_SystemMonitor.UART4Rx_cnt = 0;
    GST_SystemMonitor.UART5Rx_fps = GST_SystemMonitor.UART5Rx_cnt;
    GST_SystemMonitor.UART5Rx_cnt = 0;
    GST_SystemMonitor.USART6Rx_fps = GST_SystemMonitor.USART6Rx_cnt;
    GST_SystemMonitor.USART6Rx_cnt = 0;

    /************其他任务帧率统计************/
    GST_SystemMonitor.ChassisTask_fps = GST_SystemMonitor.ChassisTask_cnt;
    GST_SystemMonitor.ChassisTask_cnt = 0;
    GST_SystemMonitor.DebugTask_fps = GST_SystemMonitor.DebugTask_cnt;
    GST_SystemMonitor.DebugTask_cnt = 0;
    GST_SystemMonitor.SendDataTask_fps = GST_SystemMonitor.SendDataTask_cnt;
    GST_SystemMonitor.SendDataTask_cnt = 0;
    GST_SystemMonitor.GimbalTask_fps = GST_SystemMonitor.GimbalTask_cnt;
    GST_SystemMonitor.GimbalTask_cnt = 0;
    GST_SystemMonitor.ShootTask_fps = GST_SystemMonitor.ShootTask_cnt;
    GST_SystemMonitor.ShootTask_cnt = 0;
}
