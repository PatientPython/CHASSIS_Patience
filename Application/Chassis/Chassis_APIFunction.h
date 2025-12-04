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

//#region /**** 结构体声明************************************************************************/
/*底盘模式选择相关参数结构体，存放模式选择需要用到的参数变量*/
typedef struct
{
    //注意变量命名规则：所有变量名前面都加上MC_前缀，表示ModeChoose
    /*在这里面添加变量后，需要去CH_ModeChooseParaStructUpdate中*/
    /*添加该变量的更新语句，最后在ChassisModeChoose调用*/

    ChassisMode_EnumTypeDef MC_ModePre;                     //上次模式，MC_表示ModeChoose
    _CH_ModeStartTime_StructTypeDef MC_ST_ModeStartTime;    //各模式开始时间结构体（模式选择的时候作为辅助使用），MC_表示ModeChoose
    uint32_t MC_TimeNow;                                    //当前时间，单位毫秒（模式选择的时候作为辅助使用），MC_表示ModeChoose

    float MC_LegLenAvgFB;           //腿长平均值，单位mm，MC_表示ModeChoose
    float MC_VelFB;                 //底盘速度反馈值，向前为正，单位m/s，MC_表示ModeChoose

    /*AutoSafe模式需要用到的参数*/
    uint16_t MC_HubMotor1Rx_fps;    //轮毂电机1通讯帧率，MC_表示ModeChoose
    uint16_t MC_HubMotor2Rx_fps;    //轮毂电机2通讯帧率，MC_表示ModeChoose
    uint16_t MC_UART4Rx_fps;        //串口4，即IMU2通讯帧率，MC_表示ModeChoose
}Chassis_ModeChooseParameter_StructTypeDef;
//#endregion

/*************************底盘正式结构体的数据修改、处理、更新相关函数************************************/
/****各种FB反馈数据的修改、处理、更新相关函数*****/

void Chassis_AllFBDataUpdate(void);

/****各种Des目标数据的修改、处理、更新相关函数****/

void CH_LegLenDes_Update(RobotControl_StructTypeDef RMCtrl);
void CH_LQR_DesDataUpdate(RobotControl_StructTypeDef RMCtrl);
void CH_VMC_DesDataUpdate(RobotControl_StructTypeDef RMCtrl);
void HM_DesDataUpdate(RobotControl_StructTypeDef RMCtrl);
void JM_DesDataUpdate(RobotControl_StructTypeDef RMCtrl);

/************各种数据的重置、清零函数**********/

void Chassis_AllDesDataReset(void);

void Chassis_DisFBClear(void);

void Chassis_RobotCtrlDefaultConfigDataReset(void);
void Chassis_RobotCtrlForceConfigDataReset(void);
void Chassis_RobotCtrlDataReset(void);

/**********************************底盘的解算相关函数******************************************/

void CH_LegLinkageCal_Process(void);
void CH_LQRCal_Process(void);
void CH_VMCCal_Process(void);

/**********************************底盘其他相关函数********************************************/

uint8_t IsEnterManualCalibration(void);

void CH_MotionUpdateAndProcess(RobotControl_StructTypeDef RMCtrl);
void CH_ModeChooseParaStructUpdate(Chassis_ModeChooseParameter_StructTypeDef *pModeChoosePara);

#endif
