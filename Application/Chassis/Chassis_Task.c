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

// 待优化：到时候删除这个引用的头文件
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
    while (1) {
        ChassisControl();  // 底盘的总控制   // TODO 待完成

        // CAP_Control()         //待完成：超电的控制、通讯等等

        GST_SystemMonitor.ChassisTask_cnt++;  // 底盘帧率统计：cnt计数
        vTaskDelayUntil(&S_lastWakeTimeChassisTask,
                        GCH_TaskPeriod);  // 绝对延时
    }
}

/**
 * @brief  底盘控制函数
 * @param  无
 * @retval 无
 */
void ChassisControl(void) {
    /* 1. 底盘数据更新（必做：获取传感器数据、解算姿态） */
    Chassis_AllFBDataUpdate();   // 更新底盘相关数据的反馈值
    CH_LegLinkageCal_Process();  // 腿部五连杆解算处理
    CH_OffGroundCal_Process();   // 离地检测计算处理
    CH_FFInertialForceCal_Process();     // 更新侧向惯性前馈力计算

    //* 更新底盘模式选择相关变量、读取当前模式
    // 也就是说这里面包括检查是否需要进入安全模式的逻辑
    CH_ChassisModeUpdate();
    // 检测到安全模式之后会在这里执行清零操作
    ChassisModeControl_RCControl(GEMCH_Mode);
}
