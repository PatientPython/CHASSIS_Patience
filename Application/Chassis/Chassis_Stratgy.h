/**
 ******************************************************************************
 * @file    Chassis_Stratgy.h
 * @author  26赛�?�，平衡步兵电控，林宸曳
 * @date    2025.11.2
 * @brief   底盘控制策略相关函数头文�?
 ******************************************************************************
 */

#ifndef __CHASSIS_STRATGY_H
#define __CHASSIS_STRATGY_H

#include "GlobalDeclare_Chassis.h"

//* 检测当前条件是否满足进入某模式的函�?
bool _ChIsEnter_ManualSafeMode_RCControl(void);
bool _ChIsEnter_AutoSafeMode_RCControl(void);
bool _ChIsEnter_StandUpMode_RCControl(void);
// bool _ChIsEnter_SittingMode_RCControl(void);
// bool _ChIsEnter_SlowSitDownMode_RCControl(void);
// bool _ChIsEnter_FreeMode_RCControl(void);
// bool _ChIsEnter_FollowMode_RCControl(void);
// bool _ChIsEnter_OffGroundMode_RCControl(void);

//* 底盘模式更新函数
ChassisMode_EnumTypeDef ChassisModeChoose_RCControl(
    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);

//* 获取底盘各模式开始时间函�?
void ChassisStratgy_ModeStartTimeUpdate(CHData_StructTypeDef* pCHData,
                                        ChassisMode_EnumTypeDef Mode,
                                        ChassisMode_EnumTypeDef ModePre);

//* 模式具体功能实现函数
void ChModeControl_AutoSafeMode_RCControl(void);
void ChModeControl_ManualSafeMode_RCControl(void);
void ChModeControl_StandUpMode_RCControl(void);
// void ChModeControl_SittingMode_RCControl(void);
// void ChModeControl_SlowSitDownMode_RCControl(void);
// void ChModeControl_FreeMode_RCControl(void);
// void ChModeControl_FollowMode_RCControl(void);
// void ChModeControl_OffGroundMode_RCControl(void);

//* 模式控制最终执行函�?
void ChassisModeControl_RCControl(ChassisMode_EnumTypeDef ModeNow);
#endif
