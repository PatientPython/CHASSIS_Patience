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
    //! CH_FBData_Parse 只进行对传感器原始数据的解析
    //! CH_LegKinematics_Process 只进行腿部运动、运动学解算与分发
    //! CH_SupportForce_Process 只进行地面支持力计算、离地检测
    //! CH_VelocityObs_Process 可能观测器要单独写个函数放在这里，先预处理在进行运算
    CH_FBData_Parse();   // 解析传感器的原始反馈数据
    CH_LegKinematics_Process();  // 腿部五连杆的正运动学运算
    CH_SupportForce_Process();   // 离地检测地面支持力计算
    CH_VelKF_Process();    // 卡尔曼滤波器速度读取和预测自适应控制（要求先有离地标志位，写在离地检测之后；要求先有yaw角速度，写在原始数据反馈之后）
    CH_InertialFF_Process();     // 侧向惯性前馈力计算（要求速度信息，写在卡尔曼滤波器之后）

    // TODO: 后续可添加 模型自适应控制函数
    //* 更新底盘模式选择相关变量、读取当前模式
    // 也就是说这里面包括检查是否需要进入安全模式的逻辑
    CH_ChassisModeUpdate();
    // 检测到安全模式之后会在这里执行清零操作
    ChassisModeControl_RCControl(GEMCH_Mode);
}
