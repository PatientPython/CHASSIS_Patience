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

/*底盘模式选择相关参数结构体，存放模式选择需要用到的参数变量*/
typedef struct {
    // 注意变量命名规则：所有变量名前面都加上MC_前缀，表示ModeChoose
    /*在这里面添加变量后，需要去CH_ModeChooseParaStructUpdate中*/
    /*添加该变量的更新语句，最后在ChassisModeChoose调用*/

    ChassisMode_EnumTypeDef MC_ModePre;  // 上次模式，MC_表示ModeChoose
    _CH_ModeStartTime_StructTypeDef MC_ST_ModeStartTime;  // 各模式开始时间结构体（模式选择的时候作为辅助使用），MC_表示ModeChoose
    uint32_t MC_TimeNow;  // 当前时间，单位毫秒（模式选择的时候作为辅助使用），MC_表示ModeChoose

    float MC_LegLenAvgFB;  // 腿长平均值，单位mm，MC_表示ModeChoose
    float MC_VelFB;  // 底盘速度反馈值，向前为正，单位m/s，MC_表示ModeChoose

    float MC_F_OffGround;           //离地状态标志，true表示离地，false表示触地，只要有任何一侧离地都算，MC_表示ModeChoose

    /*AutoSafe模式需要用到的参数*/
    uint16_t MC_HubMotor1Rx_fps;  // 轮毂电机1通讯帧率，MC_表示ModeChoose
    uint16_t MC_HubMotor2Rx_fps;  // 轮毂电机2通讯帧率，MC_表示ModeChoose
    uint16_t MC_UART4Rx_fps;      // 串口4，即IMU2通讯帧率，MC_表示ModeChoose
} Chassis_ModeChooseParameter_StructTypeDef;

//* 底盘模式更新函数
void ChassisStrategy_ModeChooseParaStructUpdate(Chassis_ModeChooseParameter_StructTypeDef* pModeChoosePara);
ChassisMode_EnumTypeDef ChassisStrategy_ModeChoose_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);
void CH_ChassisModeUpdate(void);

//* 获取底盘各模式开始时间函数
void ChassisStrategy_ModeStartTimeUpdate(CHData_StructTypeDef* pCHData, ChassisMode_EnumTypeDef Mode, ChassisMode_EnumTypeDef ModePre);

//* 检测当前条件是否满足进入某模式的函数
bool _ChIsEnter_ManualSafeMode_RCControl(void);
bool _ChIsEnter_AutoSafeMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);
bool _ChIsEnter_StandUpMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);
bool _ChIsEnter_SittingMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);
bool _ChIsEnter_SlowSitDownMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);
bool _ChIsEnter_FreeMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);
bool _ChIsEnter_FollowMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);
bool _ChIsEnter_OffGroundMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara);

//* 模式具体功能实现函数
void CH_RCInputPre_Process(void);
void ChModeControl_AutoSafeMode_RCControl(void);
void ChModeControl_ManualSafeMode_RCControl(void);
void ChModeControl_StandUpMode_RCControl(void);
void ChModeControl_StandbyMode_RCControl(void);
void ChModeControl_SitDownMode_RCControl(void);
void ChModeControl_FreeMode_RCControl(void);
void ChModeControl_FollowMode_RCControl(void);
void ChModeControl_OffGroundMode_RCControl(void);

//* 模式控制最终执行函数
void ChassisModeControl_RCControl(ChassisMode_EnumTypeDef ModeNow);
#endif
