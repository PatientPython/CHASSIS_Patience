/**
  ******************************************************************************
  * @file    General_AuxiliaryFunc.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.22
  * @brief   通用的辅助功能函数
  ******************************************************************************
*/
#ifndef __GENERAL_AUXILIARYFUNC_H
#define __GENERAL_AUXILIARYFUNC_H

#include <stdbool.h>
#include "GlobalDeclare_General.h"
//#region /**** 遥控器相关函数 ***************************/
/**************遥控器是否连接**************/

bool IsRCConnected(void);

/**************左拨杆相关**************/

bool IsLeftLevelUp(void);
bool IsLeftLevelMid(void);
bool IsLeftLevelDown(void);

/**************右拨杆相关**************/

bool IsRightLevelUp(void);
bool IsRightLevelMid(void);
bool IsRightLevelDown(void);

/**************左摇杆相关**************/

bool IsLeftJoyStickUp(void);
bool IsLeftJoyStickDown(void);
bool IsLeftJoyStickLeft(void);
bool IsLeftJoyStickRight(void);
bool IsLeftJoyStickBeyondDeadZoneUp(void);
bool IsLeftJoyStickBeyondDeadZoneDown(void);

/**************右摇杆相关**************/

bool IsRightJoyStickUp(void);
bool IsRightJoyStickDown(void);
bool IsRightJoyStickLeft(void);
bool IsRightJoyStickRight(void);

/**************拨轮相关**************/

bool IsRollerUp(void);
bool IsRollerDown(void);

/**************双摇杆相关**************/

bool IsInsideEightGesture(void);
//#endregion

//#region /**** 步进改变数值函数 *************************/
float StepChangeValue(float ValueNow, float ValueDes, float Step);
//#endregion

/************限幅函数************/

float Limit(float RawData, float Min, float Max);


#endif
