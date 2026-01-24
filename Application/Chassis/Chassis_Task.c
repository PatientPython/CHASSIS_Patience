/**
 ******************************************************************************
 * @file    Chassis_Task.c
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.9.19
 * @brief   底盘控制任务
 ******************************************************************************
 */

/****************************头文件引用****************************/
#include "Chassis_Task.h"

#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>

#include "Algorithm.h"
#include "Chassis_APIFunction.h"
#include "Chassis_Stratgy.h"
#include "GlobalDeclare_Chassis.h"
#include "GlobalDeclare_General.h"
#include "TIM_Config.h"  // RunTimeGet

// TODO 到时候删除这个引用的头文件
#include "General_AuxiliaryFunc.h"

static TickType_t S_lastWakeTimeChassisTask =
    0;  // 上次唤醒的时间，DelayUntil绝对延时函数的参数

/**
 * @brief  ChassisTask底盘控制任务
 * @note   底盘的控制任务函数
 *         while(1)是因为FreeRTOS的任务函数必须是一个死循环
 * @param  无
 * @retval 无
 */
void ChassisTask(void* arg) {
    Chassis_AllParaInit(); //底盘所有参数初始化

    while (1) {
        ChassisControl();  // 底盘的总控制

        // CAP_Control()         // TODO 超电的控制、通讯等等

        GST_SystemMonitor.ChassisTask_cnt++;  // 底盘帧率统计：cnt计数
        vTaskDelayUntil(&S_lastWakeTimeChassisTask, GCH_TaskPeriod);  // 绝对延时
    }
}

/**
 * @brief  底盘控制函数
 * @param  无
 * @retval 无
 */
void ChassisControl(void) {
    //! 做了一下处理，不再按照结构体数据封装，而是按照数据处理逻辑封装。
    //! 再进行控制之前对GSTCH_Data里面的数据有更新和处理的写在这里面
    CH_FBData_Parse();           // 解析传感器的原始反馈数据
    CH_LegKinematics_Process();  // 腿部五连杆的正运动学运算
    CH_OffGround_Process();      // 离地检测地面支持力计算
    CH_VelKF_Process();          // 卡尔曼滤波器速度读取和预测自适应控制
    CH_HMTorqueComp_Process();   // 轮毂电机补偿力矩控制：左右轮轮毂电机打滑和受阻检测，正常抓地力补偿
    CH_LegLenAdjust_Process();   // 底盘手动腿长三档控制处理函数
    //* 更新底盘模式选择相关变量、读取当前模式
    CH_ChassisModeUpdate();
    // 检测到安全模式之后会在这里执行清零操作
    ChassisModeControl_RCControl(GEMCH_Mode);
}
