/**
 ******************************************************************************
 * @file    GlobalDeclare_Chassis.h
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.9.19
 * @brief   用来存放底盘相关的全局变量
 ******************************************************************************
 */

#ifndef __GLOBALDECLARE_CHASSIS_H
#define __GLOBALDECLARE_CHASSIS_H

#include <stdbool.h>

#include "Algorithm.h"
#include "FreeRTOS.h"
#include "GlobalDeclare_General.h"

// #pragma region
// /****枚举声明************************************************************************/
/*底盘模式相关枚举*/
typedef enum {
    /*每增加一个模式需要做的事情*/
    // 1、在这里增加一个枚举值
    // 2、在_CH_ModeStartTime_StructTypeDef结构体中增加一个对应的开始时间
    // 3、在Chassis_Stratgy.c的ChassisModeStartTimeUpdate函数中增加一个对应的case，记录模式开始时间
    // 4、在Chassis_Stratgy.c中的ChassisModeChoose_RCControl函数中增加对应模式的选择条件
    // 5、在Chassis_Stratgy.c的ChassisControl函数中增加对应模式的控制策略实现
    // 6、如果需要，在Chassis_ModeChooseParameter_StructTypeDef结构体中增加对应的参数变量

    CHMode_RC_ManualSafe,  //遥控器底盘模式：手动安全（遥控器拨杆左下右上）
    CHMode_RC_AutoSafe,    //遥控器底盘模式：自动安全
    CHMode_RC_Sitting,     //遥控器底盘模式：坐下
    CHMode_RC_StandUp,     //遥控器底盘模式：起立
    CHMode_RC_Free,        //遥控器底盘模式：自由（非跟随模式）
    CHMode_RC_SlowSitDown, //遥控器底盘模式：缓慢坐下
    CHMode_RC_Follow,      //遥控器底盘模式：跟随（随云台转动）
    CHMode_RC_OffGround,   //遥控器底盘模式：离地
    CHMode_RC_TouchGround, //遥控器底盘模式：触地（特指离地后触地）
    CHMode_RC_Struggle,    //遥控器底盘模式：脱困（指轮打滑和受阻两种情况）
    // 待补充
} ChassisMode_EnumTypeDef;

/*底盘运动方向相关枚举*/
typedef enum {
    MoveDirection_Brake,   // 停止
    MoveDirection_Forward, // 前进
    MoveDirection_Backward // 后退
} Chassis_MoveDirection_EnumTypeDef;
// #pragma endregion

// #pragma region
// /****结构体声明************************************************************************/
/*模式开始时间结构体：实际在CHData_StructTypeDef中使用*/
typedef struct {
    uint32_t RC_ManualSafe;   // RC遥控器控制下，手动安全模式开始时间，单位毫秒
    uint32_t RC_AutoSafe;     // RC遥控器控制下，自动安全模式开始时间，单位毫秒
    uint32_t RC_Sitting;      // RC遥控器控制下，坐下模式开始时间，单位毫秒
    uint32_t RC_StandUp;      // RC遥控器控制下，起立模式开始时间，单位毫秒
    uint32_t RC_Free;         // RC遥控器控制下，自由模式开始时间，单位毫秒
    uint32_t RC_SlowSitDown;  // RC遥控器控制下，缓慢坐下模式开始时间，单位毫秒
    uint32_t RC_Follow;       // RC遥控器控制下，跟随模式开始时间，单位毫秒
    uint32_t RC_OffGround;    // RC遥控器控制下，离地模式开始时间，单位毫秒
    uint32_t RC_TouchGround;  // RC遥控器控制下，触地模式开始时间，单位毫秒
    uint32_t RC_Struggle;     // RC遥控器控制下，脱困模式开始时间，单位毫秒
} _CH_ModeStartTime_StructTypeDef;

/*IMU2底盘云控数据处理结构体类型定义，包括发送和接收(注意4字节对齐)(32位单片机默认)*/
// TODO 这份数据里面好像有一些乱七八糟的、没有用到的东西，后面重构完代码了可以修改，注意连着IMU2一起改
// TODO 这份数据结构体里面的顺序很乱，后面也可以改，比如把yaw的放一起，pitch的放一起
// TODO ST_Rx、ST_Tx的结构体顺序有点乱，是因为老代码IMU2的数据通讯顺序就是这样的，后面可以考虑改成更合理的顺序
//        但是注意要把底盘云控IMU2的代码一起改
typedef struct {
    /*主控接收IMU2发来的数据结构体*/
    struct _IMU2Rx_StructTypeDef {
        /*帧头*/
        uint8_t head[2];

        /*//TODO 不知道干嘛的*/
        uint8_t JM_2StatusFB;  // TODO 并未使用的变量，不知道干嘛用的
        float leg_fps;         // TODO 并未使用的变量，不知道干嘛用的

        /*IMU的数据，各种速度、位置、加速度*/
        float YawAngle;
        float PitchAngle;
        float PitchAngleVel;
        float AccX;
        float YawAngleVel;
        float RollAngle;
        float AccZ;

        /*右前关节电机*/
        float JM2_TorqueFB;    // 力矩，单位：Nm
        float JM2_AngleVelFB;  // 角速度，单位：度/s
        float JM2_AngleFB;     // 位置，单位：度

        /*右后关节电机*/
        float JM4_TorqueFB;    // 力矩，单位：Nm
        float JM4_AngleVelFB;  // 角速度，单位：度/s
        float JM4_AngleFB;     // 位置，单位：度

        /*左前关节电机*/
        float JM1_TorqueFB;    // 力矩，单位：Nm
        float JM1_AngleVelFB;  // 角速度，单位：度/s
        float JM1_AngleFB;     // 位置，单位：度

        /*左后关节电机*/
        float JM3_TorqueFB;    // 力矩，单位：Nm
        float JM3_AngleVelFB;  // 角速度，单位：度/s
        float JM3_AngleFB;     // 位置，单位：度

        /*//TODO 不知道干嘛的*/
        int GF_fps;  // TODO 并未使用的变量，不知道干嘛用的
        int RB_fps;  // TODO 并未使用的变量，不知道干嘛用的
        int LF_fps;  // TODO 并未使用的变量，不知道干嘛用的
        int LB_fps;  // TODO 并未使用的变量，不知道干嘛用的

        /*//TODO 原来接测距用的，但是暂时也没有使用*/
        float MeasuredDistance;

        /*帧尾*/
        uint8_t tail[2];
    } ST_Rx;

    /*主控发送给IMU2的数据结构体*/
    struct {
        /*帧头*/
        uint8_t head[2];

        /*一些标志位*/
        uint8_t
            ReloadStatus;  // TODO 原本是之前赛季的补弹标志位，由于规则改动，无需补弹功能，直接发false就行，后续可以删掉，记得IMU2里面也要一起删
        uint8_t RestartFlag;         // IMU2底盘云控重启标志位
        uint8_t JM2StatusDes;        // TODO 并未使用的变量，不知道干嘛用的
        uint8_t LegCalibrationFlag;  // 腿部校准标志位

        /*右前关节电机*/
        float JM2_TorqueDes;    // 力矩目标值，单位：Nm
        float JM2_AngleVelDes;  // 角速度目标值，单位：度/s
        float JM2_AngleDes;     // 位置目标值，单位：度

        /*右后关节电机*/
        float JM4_TorqueDes;    // 力矩目标值，单位：Nm
        float JM4_AngleVelDes;  // 角速度目标值，单位：度/s
        float JM4_AngleDes;     // 位置目标值，单位：度

        /*左前关节电机*/
        float JM1_TorqueDes;    // 力矩目标值，单位：Nm
        float JM1_AngleVelDes;  // 角速度目标值，单位：度/s
        float JM1_AngleDes;     // 位置目标值，单位：度

        /*左后关节电机*/
        float JM3_TorqueDes;    // 力矩目标值，单位：Nm
        float JM3_AngleVelDes;  // 角速度目标值，单位：度/s
        float JM3_AngleDes;     // 位置目标值，单位：度

        /*MIT控制相关参数*/
        float MITKp;  // 位置比例系数Kp
        float MITKd;  // 速度比例系数Kd

        /*帧尾*/
        uint8_t tail[2];
    } ST_Tx;
} IMU2Data_StructTypeDef;

/*关节电机MIT控制协议数据结构体*/
// TODO 这份结构体的成员有些是没有用的，可以考虑删掉，同时注意把底盘云控IMU2的数据结构体也一起改
typedef struct {
    /*需要初始化赋值的成员*/
    uint8_t ID;   // 电机的ID
    float MITKp;  // 位置比例系数Kp
    float MITKd;  // 速度比例系数Kd

    /*不需要初始化赋值的成员*/
    uint8_t StatusDes;  // 电机状态控制指令码

    float AngleDes;     // 位置目标值，单位：度
    float AngleVelDes;  // 角速度目标值，单位：度/s
    float TorqueDes;    // 目标力矩，单位：Nm

    float AngleFB;     // 位置反馈值，单位：度
    float AngleVelFB;  // 角速度反馈值，单位：度/s
    float TorqueFB;    // 扭矩反馈值，单位：Nm
} JMData_StructTypeDef;

/*轮毂电机数据结构体*/
typedef struct {
    /*需要初始化赋值的成员*/
    float ReductionRatio;  // 减速比

    /*不需要初始化赋值的成员*/
    float AngleVelFB;   // 电机减速箱输出的角速度反馈值，向前为正，单位：rad/s
    int16_t CurrentFB;  // 电机电流反馈值
    int8_t TempFB;      // 电机温度反馈值，单位：°C

    float TorqueDes;  // 电机力矩目标值，向前转为正，单位：Nm
    float TorqueAdapt; // 电机力矩补偿值，向前转为正，单位：Nm
    int16_t CurrentDes;  // 电机电流目标值，注意单位不是mA也不是A，是直接给电调的电流值
                     // TODO
    // 电调手册上写了，这个值的范围是-16384到16384，代表-20A到20A的电流输出。
} HMData_StructTypeDef;

/*底盘数据结构体*/
typedef struct {
    /*腿长相关*/
    float LegLen1Des;  // 左腿目标长度（经过TD处理），单位m
    float LegLen2Des;  // 右腿目标长度（经过TD处理），单位m
    float LegLen1FB;   // 左腿实际长度，单位m
    float LegLen2FB;   // 右腿实际长度，单位m

    /*LQR相关*/
    float DisDes;  // 底盘位移目标值，向前为正，单位m
    float DisFB;   // 底盘位移反馈值，向前为正，单位m
    float VelDes;  // 底盘速度目标值，向前为正，单位m/s
    float VelFB;   // 底盘速度反馈值，向前为正，单位m/s

    float YawDeltaDes;     // 底盘偏转角目标增量，上往下看逆时针为正，单位度
    float YawAngleVelDes;  // 底盘偏转速度目标值，上往下看逆时针为正，单位度/s
    float YawAngleVelFB;   // 底盘偏转速度反馈值，上往下看逆时针为正，单位度/s

    float Theta1Des;  // 左腿与绝对垂直方向夹角目标值，向后摆为正，单位度
    float Theta2Des;  // 右腿与绝对垂直方向夹角目标值，向后摆为正，单位度
    float Theta1FB;   // 左腿相对机体垂直方向夹角，向后摆为正，单位度
                      // //老代码说明：老代码叫AlphaLeftReal
    float Theta2FB;   // 右腿相对机体垂直方向夹角，向后摆为正，单位度
                      // //老代码说明：老代码叫AlphaRightReal

    float Theta1AngleVelDes;  // 左腿摆角速度反馈值，向后摆为正，单位度/s
    float Theta2AngleVelDes;  // 右腿摆角速度反馈值，向后摆为正，单位度/s
    float Theta1AngleVelFB;   // 左腿摆角速度反馈值，向后摆为正，单位度/s
    float Theta2AngleVelFB;   // 右腿摆角速度反馈值，向后摆为正，单位度/s

    float PitchAngleDes;     // 底盘俯仰角目标值，抬头为正，单位度
    float PitchAngleFB;      // 底盘俯仰角反馈值，抬头为正，单位度
    float PitchAngleVelDes;  // 底盘俯仰角速度目标值，抬头为正，单位度/s
    float PitchAngleVelFB;   // 底盘俯仰角速度反馈值，抬头为正，单位度/s

    /*VMC相关*/
    float Leg1ForceDes;   // 左腿VMC腿部作用力目标值，向下为正，单位N
    float Leg2ForceDes;   // 右腿VMC腿部作用力目标值，向下为正，单位N
    float Leg1TorqueDes;  // 左腿VMC腿部作用力矩目标值，向后摆为正，单位N·m
    float Leg2TorqueDes;  // 右腿VMC腿部作用力矩目标值，向后摆为正，单位N·m

    /*运动姿态解算相关*/
    float xC1_dot;      // 左边五连杆解算C点x坐标的微分，向前为正，单位m/s
    float xC2_dot;      // 右边五连杆解算C点x坐标的微分，向前为正，单位m/s
    float RollAngleFB;  // 底盘横滚角反馈值，左侧高为正，单位度
    float AccXFB;       // 底盘加速度反馈值，向前为正，单位m/s²
    float AccZFB;       // 底盘垂直加速度反馈值，向上为正，单位m/s²

    /*底盘控制策略相关*/
    _CH_ModeStartTime_StructTypeDef ST_ModeStartTime;  // 底盘各模式开始时间结构体

    bool F_DirectionInvert;  // XXX 赛场上快速转向 底盘前进方向反转标志位，true表示前进方向反转，false表示前进方向不反转
    Chassis_MoveDirection_EnumTypeDef EM_MoveDirection;  // 底盘运动方向枚举变量，表示当前底盘是刹车、前进还是后退

    /*离地检测相关*/
    float Leg1F_N;  // 左腿腿部支持力反馈值，单位N
    float Leg2F_N;  // 右腿腿部支持力反馈值，单位N

    /*特殊工况相关*/
    float HM1_VelErr;  // 左轮毂电机速度误差，单位：rad/s
    float HM2_VelErr;  // 右轮毂电机速度误差，单位：rad/s

    /*标志位相关*/
    bool F_OffGround1;  // 左腿离地状态标志位，true表示离地，false表示未离地
    bool F_OffGround2;  // 右腿离地状态标志位，true表示离地，false表示未离地

    bool F_SlipHM1;     // 左轮打滑状态标志位，true表示打滑，false表示未打滑
    bool F_SlipHM2;     // 右轮打滑状态标志位，true表示打滑，false表示未打滑

    bool F_BlockHM1;    // 左轮受阻状态标志位，true表示受阻，false表示未受阻
    bool F_BlockHM2;    // 右轮受阻状态标志位，true表示受阻，false表示未受阻

    /*腿长变化相关*/
    float LegLenManualDes; // 手动控制的目标腿长（三档：Min, Mid, High）
    bool F_JoyUpLatched; // 摇杆上抬锁存标志
    bool F_JoyDownLatched; // 摇杆下拨锁存标志
} CHData_StructTypeDef;
// #pragma endregion

// #pragma region
// /****变量引出extern声明****************************************************************/
// #pragma region
// /****************************时间相关********************************/
/*ChassisTask的任务周期、任务时间*/

extern const TickType_t GCH_TaskPeriod;
extern const float GCH_TaskTime;

/*ChassisStrategy里的各个模式相关时间*/
extern uint16_t CHMode_AllMode_PreProcessTime;
extern uint16_t CHMode_RC_StandUp_TotalTime;

// #pragma endregion

// #pragma region
// /****************************通讯相关*********************************/

extern IMU2Data_StructTypeDef GstCH_IMU2;
extern uint8_t GFCH_IMU2Restart;
extern uint8_t GFCH_LegCalibration;

extern C620FeedBackData_StructTypeDef GstCH_HM1RxC620Data;
extern C620FeedBackData_StructTypeDef GstCH_HM2RxC620Data;
// #pragma endregion

// #pragma region
// /*************************低通滤波器、卡尔曼滤波器相关***********************/

extern LPF_StructTypeDef GstCH_HM1_AngleVelLPF;
extern LPF_StructTypeDef GstCH_HM2_AngleVelLPF;

extern LPF_StructTypeDef GstCH_TheoryVelLPF;
extern LPF_StructTypeDef GstCH_VelCompLPF;

extern LPF_StructTypeDef GstCH_xC1dotLPF;
extern LPF_StructTypeDef GstCH_xC2dotLPF;

extern LPF_StructTypeDef GstCH_Theta1dotLPF;
extern LPF_StructTypeDef GstCH_Theta2dotLPF;

extern LPF_StructTypeDef GstCH_YawAngleVelLPF;
extern LPF_StructTypeDef GstCH_PitchAngleVelLPF;

extern LPF_StructTypeDef GstCH_Leg1F_N_LPF;
extern LPF_StructTypeDef GstCH_Leg2F_N_LPF;

extern KF_StructTypeDef GstCH_VelKF;

extern HM_TorqueComp_StructTypeDef GSTCH_HMTorqueComp;
// #pragma endregion

// #pragma region
// /************************TD算法相关**********************************/

extern TD_StructTypeDef GstCH_LegLen1TD;
extern TD_StructTypeDef GstCH_LegLen2TD;
extern TD_StructTypeDef GstCH_YawAngleTD;
extern TD_StructTypeDef GstCH_DisTD;

extern float TD_LegLen_rStandUp;
extern float TD_LegLen_rNorm;
extern float TD_LegLen_rSlowSitDown;

// #pragma endregion

// #pragma region
// /************************PID算法相关*********************************/

extern PID_StructTypeDef GstCH_LegLen1PID;
extern PID_StructTypeDef GstCH_LegLen2PID;
extern PID_StructTypeDef GstCH_RollCompPID;

extern float PID_LegLen_KpStandUp;
extern float PID_LegLen_KdStandUp;
extern float PID_LegLen_KpNorm;
extern float PID_LegLen_KdNorm;

// #pragma endregion

// #pragma region /****底盘平移、旋转控制相关*****************************/

extern float ChMove_StillVelTH;
extern float ChMove_VelDesMax;
extern float ChMove_Acc_Moving;
extern float ChMove_Acc_Brake;
extern float ChMove_VelDesMin;
extern float ChMove_VelMovingChangeRateMin;
extern float ChMove_VelMovingChangeRateMax;
extern float ChMove_VelBrakingChangeRateMax;
extern float ChMove_BrakeVelLimitTH;

extern float ChMove_YawAngleVelMaxTH;
extern float ChMove_TurnYawVel_Normal;
extern float ChMove_YawAngleVelAddStep;
// #pragma endregion

// #pragma region /****底盘小陀螺相关*****************************/

extern float RCTopMode_EnterVelMinTH;
extern float RCTopMode_EnterDelayTime;
extern float RCTopMode_TopAngleVelDesMax;
extern float RCTopMode_TopAngleVelAddStep;
extern float RCTopMode_TopAngleVelBrakeStep;
extern float RCTopMode_ExitAngleVelTH;
// #pragma endregion

// #pragma region /****SlowSitDown相关*****************************/

extern float SlowSitDown_YawAngleVelBrakeStep;
extern float SlowSitDown_LegFFForceDecStep;
// #pragma endregion

// #pragma region
// /******************底盘其他相关参数***********************************/

extern const float JointMotorMAXTorque;

extern float ChassisPitchAngleZP;
extern float ChassisRollAngleZP;

extern float LegLenMin;
extern float LegLenMinTH;
extern float LegLenLow;
extern float LegLenMid;
extern float LegLenHigh;
extern float LegLenOffGround;

extern const float m_w;
extern const float R_l;
extern const float R_w;

//* 用于前馈力计算 
extern const float m_total;
extern const float CH_Phys_EffMass;
extern const float CH_Phys_InertialCoeff;

extern const float LegFFForce_Gravity_1;
extern const float LegFFForce_Gravity_2;
extern float LegFFForce_Inertial_1;
extern float LegFFForce_Inertial_2;

//* 用于地面支持力估计、离地检测 
extern const float CH_Phys_OffGrd_CorCoeff;
extern const float CH_Phys_OffGrd_CplCoeff;

//* 用于缓慢坐下和离地模式的腿部前馈力计算
extern float LegFFForce_SlowSitDown;
extern float LegFFForce_OffGround;
// #pragma endregion

// #pragma region
// /*********************底盘运动控制相关-辅助变量************************/

extern LegLinkageCal_StructTypeDef GstCH_LegLinkCal1;
extern LegLinkageCal_StructTypeDef GstCH_LegLinkCal2;

extern LQR_StructTypeDef GstCH_LQRCal;

extern VMC_StructTypeDef GstCH_Leg1VMC;
extern VMC_StructTypeDef GstCH_Leg2VMC;

extern OffGround_StructTypeDef GstCH_OffGround1;
extern OffGround_StructTypeDef GstCH_OffGround2;
// #pragma endregion

// #pragma region
// /*****************其他底盘运动控制相关-正式变量************************/

extern ChassisMode_EnumTypeDef GEMCH_Mode;
extern ChassisMode_EnumTypeDef GEMCH_ModePre;

extern JMData_StructTypeDef GSTCH_JM1;
extern JMData_StructTypeDef GSTCH_JM2;
extern JMData_StructTypeDef GSTCH_JM3;
extern JMData_StructTypeDef GSTCH_JM4;

extern HMData_StructTypeDef GSTCH_HM1;
extern HMData_StructTypeDef GSTCH_HM2;

extern CHData_StructTypeDef GSTCH_Data;
// #pragma endregion

// #pragma region
// /****宏定义引出声明（一般不需要修改）****************************************************/
/**********机器人机械结构相关****************/
// 换车时需修改
// #define WheelRadius (72.0f * MM2M)  // 轮子的半径，单位米
// XXX 换车时需修改(写在了全局变量里面)
// #define m_w 2.4f  // 单个轮子的质量，单位kg

/*************电机、电调相关****************/
#define HM_ReductionRatio Motor_3508GearboxReductionRatio  // 轮毂电机减速比
#define HM_MaxCurrent Motor_3508MaxCurrent                 // 轮毂电机最大电流值
#define HM_MinCurrent Motor_3508MinCurrent                 // 轮毂电机最小电流值
#define HM_Kt Motor_3508Kt  // 轮毂电机转矩常数，单位：Nm/A
#define HM_AmpereToCurrent Motor_3508AmpereToCurrent  // 轮毂电机电流转换系数，单位：映射电流值/A

/**************CAN通讯相关******************/
/*轮毂电机ID*/
// 换车时需要修改
#define CANID_HM1 0x202  // 左轮毂电机，主控接收的CANID
#define CANID_HM2 0x201  // 右轮毂电机，主控接收的CANID

/**********GFCH_IMU2Restart的取值***********/
#define IMU2RestartYES 0xF  // IMU2要重启
#define IMU2RestartNO 0x0   // IMU2不重启
// #pragma endregion

#define LegLen_StandUp_Practice 0.30f  // 单位：m，自己的设定的站立目标腿长

/********************************************************** 函数声明 **********************************************************/

void Chassis_AllParaInit(void);

bool GSTCH_DataGet_F_OffGround1(CHData_StructTypeDef CHData);
bool GSTCH_DataGet_F_OffGround2(CHData_StructTypeDef CHData);



#endif
