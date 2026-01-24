/**
 ******************************************************************************
 * @file    Chassis_APIFunction.h
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.10.25
 * @brief   底盘功能函数头文件
 ******************************************************************************
 */
#ifndef __CHASSIS_APIFUNCTION_H
#define __CHASSIS_APIFUNCTION_H

#include "GlobalDeclare_Chassis.h"





/*************************底盘正式结构体的数据修改、处理、更新相关函数************************************/
/****各种FB反馈数据的修改、处理、更新相关函数*****/

void CH_FBData_Parse(void);

/****各种Des目标数据的修改、处理、更新相关函数****/

void CH_LegLenDes_Update(RobotControl_StructTypeDef RMCtrl);
void CH_LQR_DesDataUpdate(RobotControl_StructTypeDef RMCtrl);
void CH_VMC_DesDataUpdate(RobotControl_StructTypeDef RMCtrl);
void HM_DesDataUpdate(RobotControl_StructTypeDef RMCtrl);
void JM_DesDataUpdate(RobotControl_StructTypeDef RMCtrl);

/**********************************底盘的解算相关函数******************************************/
//? 包含底盘运动学解算、支撑力解算、速度卡尔曼滤波处理、惯性前馈处理、LQR计算、VMC计算等
//? 卡尔曼滤波处理函数会用到离地标志位，写在支撑力解算函数之后
//? 卡尔曼滤波处理函数会用到yaw角速度，写在原始数据反馈函数之后
//? 模型自适应控制函数会用到观测到的速度信息，写在卡尔曼滤波处理函数之后
void CH_LegKinematics_Process(void);
void CH_OffGround_Process(void);
void CH_VelKF_Process(void);
void CH_HMTorqueComp_Process(void);
// 底盘姿态控制核心函数
void CH_LQRCal_Process(void);
void CH_VMCCal_Process(void);

/************各种数据的重置、清零函数**********/

void Chassis_AllDesDataReset(void);

void Chassis_DisFBClear(void);

void Chassis_RobotCtrlDefaultConfigDataReset(void);
void Chassis_RobotCtrlForceConfigDataReset(void);
void Chassis_RobotCtrlDataReset(void);

/**********************************底盘其他相关函数********************************************/
// TODO 底盘数据低通滤波处理函数（考虑放在哪里）
float _Ch_FBData_LPF(float RawData, LPF_StructTypeDef* pLPF);

uint8_t IsEnterManualCalibration(void);

void CH_MotionUpdateAndProcess(RobotControl_StructTypeDef RMCtrl);

void ChModeControl_FreeMode_RCControl_MoveHandler(CHData_StructTypeDef* CHData, RobotControl_StructTypeDef* RMCtrl);
bool ChModeControl_FreeMode_RCControl_IsEnterTopMode(CHData_StructTypeDef CHData);
void ChModeControl_FreeMode_RCControl_TopHandler(CHData_StructTypeDef* CHData, RobotControl_StructTypeDef* RMCtrl);

#endif
