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
//测试25.12.2
OffGround_StructTypeDef GSTCH_OffGround1 = {2.4, 0, 10, 0.001};
OffGround_StructTypeDef GSTCH_OffGround2 = {2.4, 0, 10, 0.001};
void ChassisControl(void)
{
    /*********************测试时使用1上边界***********************************************************************************************/
    OffGround_BodyZAccUpdate(&GSTCH_OffGround1, GSTCH_Data.AccZFB);
    OffGround_PitchAngleUpdate(&GSTCH_OffGround1, GSTCH_Data.PitchAngleFB);
    OffGround_PitchAngleVelUpdate(&GSTCH_OffGround1, GSTCH_Data.PitchAngleVelFB);
    OffGround_LegLinkRelateDataUpdate(GstCH_LegLinkCal1, &GSTCH_OffGround1, LeftSide);
    OffGround_TorqueDataUpdate(&GSTCH_OffGround1, GSTCH_JM3.TorqueFB, GSTCH_JM1.TorqueFB);
    OffGround_GetRealFAndTp(&GSTCH_OffGround1);
    G_fTest1 = OffGround_GetSupportForce(&GSTCH_OffGround1);

    OffGround_BodyZAccUpdate(&GSTCH_OffGround2, GSTCH_Data.AccZFB);
    OffGround_PitchAngleUpdate(&GSTCH_OffGround2, GSTCH_Data.PitchAngleFB);
    OffGround_PitchAngleVelUpdate(&GSTCH_OffGround2, GSTCH_Data.PitchAngleVelFB);
    OffGround_LegLinkRelateDataUpdate(GstCH_LegLinkCal2, &GSTCH_OffGround2, RightSide);
    OffGround_TorqueDataUpdate(&GSTCH_OffGround2, GSTCH_JM2.TorqueFB, GSTCH_JM4.TorqueFB);
    OffGround_GetRealFAndTp(&GSTCH_OffGround2);
    G_fTest2 = OffGround_GetSupportForce(&GSTCH_OffGround2);

    /*********************测试时使用1下边界***********************************************************************************************/
    

    Chassis_AllFBDataUpdate();  //更新底盘相关数据的反馈值
    CH_LegLinkageCal_Process(); //腿部五连杆解算处理

    /*FB数据在ChassisControl之前就更新*/
    /*Des数据在每一个Stratgy里面更新*/
    //腿长计算无论什么时候都可以，所以放在这里计算
    //LQR和VMC计算必须在允许运动的时候计算（也就是在Strategy控制里计算）
    
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


