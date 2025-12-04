/**
  ******************************************************************************
  * @file    Chassis_Stratgy.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.11.2
  * @brief   底盘控制策略相关函数头文件
  ******************************************************************************
*/

#ifndef __CHASSIS_STRATGY_H
#define __CHASSIS_STRATGY_H

#include "GlobalDeclare_Chassis.h"

ChassisMode_EnumTypeDef ChassisModeChoose_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);
void ChassisModeControl_RCControl(ChassisMode_EnumTypeDef ModeNow);
void ChassisMode_GetModeStartTime(CHData_StructTypeDef* pCHData, ChassisMode_EnumTypeDef Mode, ChassisMode_EnumTypeDef ModePre);

#endif
