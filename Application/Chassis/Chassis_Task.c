/**
  ******************************************************************************
  * @file    Chassis_Task.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.19
  * @brief   底盘控制任务
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include <FreeRTOS.h>
#include <task.h>
#include <stdbool.h>
#include "Chassis_Task.h"
#include "Algorithm.h"
#include "Chassis_APIFunction.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Chassis.h"
#include "Chassis_Stratgy.h"

//待优化：到时候删除这个引用的头文件
#include "General_AuxiliaryFunc.h"

static TickType_t S_lastWakeTimeChassisTask = 0; //上次唤醒的时间，DelayUntil绝对延时函数的参数

/**
  * @brief  ChassisTask底盘控制任务
  * @note   底盘的控制任务函数
  *         while(1)是因为FreeRTOS的任务函数必须是一个死循环
  * @param  无
  * @retval 无
  */
void ChassisTask(void* arg)
{
    while(1)
    {
        ChassisControl();       //底盘的总控制   //待完成

        //CAP_Control()         //待完成：超电的控制、通讯等等

        GST_SystemMonitor.ChassisTask_cnt++;		//底盘帧率统计：cnt计数
        vTaskDelayUntil(&S_lastWakeTimeChassisTask, GCH_TaskPeriod); //绝对延时
    }
}


//待修改
/*测试时的分割线---------------------------------------------------------------------------------------------*/
//待修改


/**
  * @brief  底盘控制函数
  * @param  无
  * @retval 无
*/
void ChassisControl(void)
{
    Chassis_AllFBDataUpdate();  //更新底盘相关数据的反馈值
    CH_LegLinkageCal_Process(); //腿部五连杆解算处理
    CH_OffGroundCal_Process();  //离地检测计算处理

    /*********************测试时使用1上边界***********************************************************************************************/
    /*********************测试时使用1下边界***********************************************************************************************/
    
    //待优化：现在的代码如果在起立的时候切换到Follow模式，会导致出事，后面记得改
    //待优化：Sitting模式还有一个打蛋需要写，写到Booster里面

    
    /*********************测试时使用2上边界***********************************************************************************************/
    //待优化：下面这一段可以适当封装一下，不然看起来有点丑陋
    /************底盘模式选择及对应控制************/
    /*记录上次底盘模式，选择当前底盘模式*/
    GEMCH_ModePre = GEMCH_Mode;

    Chassis_ModeChooseParameter_StructTypeDef tmp_ModeChoosePara;   //临时底盘模式选择参数结构体（没必要用全局变量，直接用局部变量即可）
    CH_ModeChooseParaStructUpdate(&tmp_ModeChoosePara);             //更新底盘模式选择参数
    GEMCH_Mode = ChassisModeChoose_RCControl(tmp_ModeChoosePara);   //根据不同情况，选择底盘模式

    /*记录模式开始时间*/
    ChassisMode_GetModeStartTime(&GSTCH_Data, GEMCH_Mode, GEMCH_ModePre);

    /*不同底盘模式对应控制*/
    ChassisModeControl_RCControl(GEMCH_Mode);

    /*********************测试时使用2下边界***********************************************************************************************/

}


