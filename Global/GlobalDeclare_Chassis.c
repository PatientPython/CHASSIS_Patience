/**
  ******************************************************************************
  * @file    GlobalDeclare_Chassis.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.19
  * @brief   用来存放底盘相关的全局变量
  ******************************************************************************
*/

/*************************************************头文件引用*************************************************/
#include "GlobalDeclare_Chassis.h"
#include "GlobalDeclare_General.h"
#include "Algorithm.h"
#include "FreeRTOS.h"

/****************************************宏定义、常量定义（不需要修改）****************************************/
/*FreeRTOS任务相关*/
const TickType_t GCH_TaskPeriod = 1;      //ChassisTask的任务周期，单位为FreeRTOS的系统节拍。默认是ms（取决于configTICK_RATE_HZ）
const float GCH_TaskTime = (float)GCH_TaskPeriod/(float)configTICK_RATE_HZ; //任务运行周期，单位为秒

/*一些默认定义*/
#define SampleTime_Default GCH_TaskTime   //默认采样时间，单位秒

//#region /****关节电机相关*****************************************/
/*关节电机ID*/
//待修改：这里的ID号在搞新车的时候重新设置，按照左前、右前、左后、右后的顺序设置
//换车时需要修改
#define JM1ID   0x03    //关节电机1的ID
#define JM2ID   0x01    //关节电机2的ID
#define JM3ID   0x04    //关节电机3的ID
#define JM4ID   0x02    //关节电机4的ID

//换车时需修改
const float JointMotorMAXTorque = Motor_MG8016Ei6MaxTorque; //关节电机最大力矩，单位Nm
//#endregion

//#region /****腿部五连杆解算****************************************/
/*五连杆结构体初始化宏定义*/
#define LegLinkageCal_StructINIT(ThighLen, CalfLen, l5, phi1ZP, phi2ZP, SampleTime)  {ThighLen, CalfLen, CalfLen, ThighLen, l5, phi1ZP, phi2ZP, SampleTime}
/*各杆长度*/
//换车时需要修改
#define LeftThighLen        150.0f       //左腿大腿长度，单位mm，五连杆解算里面的l1
#define LeftCalfLen         270.0f       //左腿小腿长度，单位mm，五连杆解算里面的l2
#define RightThighLen       150.0f       //右腿大腿长度，单位mm，五连杆解算里面的l4
#define RightCalfLen        270.0f       //右腿小腿长度，单位mm，五连杆解算里面的l3
#define SameSideJMDistance  150.0f       //同侧腿的关节电机间距，单位mm，五连杆解算里面的l5
/*关节电机编码器零点（相对于五连杆解算的坐标系）*/
//换车时需要修改
#define JM1LinkageCalZP     -22.0f       //左侧phi4，关节电机1（左前）五连杆解算坐标系的零点，单位度
#define JM3LinkageCalZP     180.0f+22.0f //左侧phi1，关节电机3（左后）五连杆解算坐标系的零点，单位度
#define JM2LinkageCalZP     180.0f+22.0f //右侧phi1，关节电机2（右前）五连杆解算坐标系的零点，单位度
#define JM4LinkageCalZP     -22.0f       //右侧phi4，关节电机4（右后）五连杆解算坐标系的零点，单位度
//#endregion

//#region /****低通滤波器滤波系数*********************************/
//换车时需修改
#define LPF_Alpha_HM_AngleVel    0.091f     //轮毂电机速度低通滤波器系数
#define LPF_Alpha_xCdot          0.091f     //底盘速度补偿低通滤波器系数
#define LPF_Alpha_Thetadot       0.167f     //腿部Theta_dot低通滤波器系数
#define LPF_Alpha_YawAngleVel    0.09f      //底盘Yaw轴角速度低通滤波器系数
#define LPF_Alpha_PitchAngleVel  0.06f      //底盘Pitch轴角速度低通滤波器系数
#define LPF_Alpha_AccXFB         0.6f       //底盘加速度AccX低通滤波器系数
#define LPF_Alpha_VelTheory      0.1f       //底盘理论质心水平速度低通滤波器系数
#define LPF_Alpha_VelComp        0.1f       //底盘速度补偿低通滤波器系数
//#endregion


/****************************************宏定义、常量定义（可能需要修改）***************************************/
//#region /****TD相关系数*********************************/
#define TD_SampleTime   SampleTime_Default  //TD采样时间，单位秒

#define TD_LegLen_r    0.0f                 //腿长TD：速度因子，越大跟踪越快，但微分信号的噪声也会越大
#define TD_LegLen_h0   1*TD_SampleTime      //腿长TD：滤波因子，越大滤波效果越好，通常取采样时间的整数倍

float TD_LegLen_rStandUp = 4000.0f;    //腿长TD：起立模式下的速度因子
float TD_LegLen_rNorm    = 4000.0f;    //腿长TD：正常模式下的速度因子
float TD_LegLen_rSlowSitDown = 70.0f;  //腿长TD：缓慢坐下模式下的速度因子
//#endregion

//#region /****PID相关参数********************************/
/*腿长PID相关*/
#define PID_LegLen_Kp     0.0f              //腿长PID：比例系数Kp，取0表示在外部根据不同模式赋值
#define PID_LegLen_Ki     0.0f              //腿长PID：积分系数Ki，取0表示不使用积分
#define PID_LegLen_Kd     0.0f              //腿长PID：微分系数Kd，取0表示在外部根据不同模式赋值
#define PID_LegLen_UMax   400.0f            //腿长PID：总输出最大值
#define PID_LegLen_UpMax  PID_LegLen_UMax   //腿长PID：Kp项输出最大值
#define PID_LegLen_UiMax  0.0f              //腿长PID：Ki项输出最大值
#define PID_LegLen_UdMax  PID_LegLen_UMax   //腿长PID：Kd项输出最大值
#define PID_LegLen_AddMax 0.01f             //腿长PID：误差单次累加最大值

float PID_LegLen_KpStandUp = 0.8f;          //腿长PID：起立状态下Kp值
float PID_LegLen_KdStandUp = 20.0f;         //腿长PID：起立状态下Kd值
float PID_LegLen_KpNorm    = 0.8f;          //腿长PID：正常时的Kp值
float PID_LegLen_KdNorm    = 20.0f;         //腿长PID：正常时的Kd值

/*Roll轴补偿相关*/
//待优化：可以试试给小陀螺单独一套PID参数
#define PID_RollComp_Kp     500.0f              //Roll轴补偿PID：比例系数Kp
#define PID_RollComp_Ki     0.0f                //Roll轴补偿PID：积分系数Ki，取0表示不使用积分
#define PID_RollComp_Kd     16000.0f            //Roll轴补偿PID：微分系数Kd
#define PID_RollComp_UMax   400.0f              //Roll轴补偿PID：总输出最大值
#define PID_RollComp_UpMax  PID_RollComp_UMax   //Roll轴补偿PID：Kp项输出最大值
#define PID_RollComp_UiMax  0.0f                //Roll轴补偿PID：Ki项输出最大值
#define PID_RollComp_UdMax  50.0f               //Roll轴补偿PID：Kd项输出最大值
#define PID_RollComp_AddMax 0.01f               //Roll轴补偿PID：误差单次累加最大值

//老代码说明：
//ST_PID_INIT(fpKp,fpKi,fpKd,fpUMax,fpUiMax,fpUdMax,AddMax)
//        {0,0,fpKp,fpKi,fpKd,0,0,0,0,0,0,0,fpUMax,fpUMax,fpUiMax,fpUdMax,AddMax}

//#endregion

//#region /****零点补偿、前馈力补偿、腿长相关**************************/
/*腿长相关*/
float LegLenMin   = 108.0f;   //腿长最小值，单位mm
float LegLenMinTH = 6.0f;     //腿长最小值阈值，单位mm，腿长距离LegLenMin在该阈值内时，认为到达最小腿长位置
float LegLenLow  = 140.0f;    //低腿长，单位mm
float LegLenMid  = 200.0f;    //中腿长，单位mm
float LegLenHigh = 290.0f;    //高腿长，单位mm

/*底盘零点补偿相关*/
//换车时需要修改
float ChassisPitchAngleZP = 0.0f;  //底盘Pitch轴零点补偿值，单位度，正值表示实际需要抬头才能平衡
float ChassisRollAngleZP  = 0.95f; //底盘Roll轴零点补偿值，单位度

/*腿部前馈力补偿相关*/
float LegFFForce_Norm            = 80.0f;   //正常时的腿部前馈力，单位N，用于补偿重力对腿部的影响

float LegFFForce_SlowSitDown     = 5.0f;    //缓慢坐下模式的腿部前馈力，单位N，用于补偿重力对腿部的影响
float LegFFForce_SlowSitDownStep = 0.05f;   //缓慢坐下模式的腿部前馈力的步进增加值
//#endregion

//#region /****底盘模式控制策略相关**************************/
uint16_t CHMode_AllMode_PreProcessTime  = 4;      //各个模式的前置处理时间，单位毫秒
uint16_t CHMode_RC_StandUp_TotalTime    = 800;    //起立模式的总持续时间，单位毫秒

//#endregion

/********************************************变量定义(不需要修改)********************************************/
//#region /****通讯相关************************************************/
/*IMU2通讯相关*/
IMU2Data_StructTypeDef GstCH_IMU2;           //底盘云控IMU2的通讯数据结构体，包括接收和发送
uint8_t GFCH_IMU2Restart    = IMU2RestartNO; //底盘云控IMU2重启标志位，默认不重启
uint8_t GFCH_LegCalibration = 0;             //腿部校准标志位，1：校准，0：不校准

/*电机/电调通讯相关*/
C620FeedBackData_StructTypeDef GstCH_HM1RxC620Data;    //左轮毂电机电调反馈数据结构体
C620FeedBackData_StructTypeDef GstCH_HM2RxC620Data;    //右轮毂电机电调反馈数据结构体
//#endregion

//#region /****滤波器、观测器相关***************************************/
/*轮毂电机速度低通滤波器*/
LPF_StructTypeDef GstCH_HM1_AngleVelLPF = {LPF_Alpha_HM_AngleVel}; //左轮毂电机速度低通滤波器
LPF_StructTypeDef GstCH_HM2_AngleVelLPF = {LPF_Alpha_HM_AngleVel}; //右轮毂电机速度低通滤波器

/*xC_dot低通滤波器*/
LPF_StructTypeDef GstCH_xC1dotLPF = {LPF_Alpha_xCdot}; //xC_dot低通滤波器，左腿
LPF_StructTypeDef GstCH_xC2dotLPF = {LPF_Alpha_xCdot}; //xC_dot低通滤波器，右腿

/*Theta_dot低通滤波器*/
LPF_StructTypeDef GstCH_Theta1dotLPF = {LPF_Alpha_Thetadot}; //Theta1_dot低通滤波器，左腿
LPF_StructTypeDef GstCH_Theta2dotLPF = {LPF_Alpha_Thetadot}; //Theta2_dot低通滤波器，右腿

/*底盘加速度AccX低通滤波器*/
LPF_StructTypeDef GstCH_AccX_LPF   = {LPF_Alpha_AccXFB};   //底盘加速度AccX低通滤波器结构体

/*底盘理论质心水平速度低通滤波器*/
LPF_StructTypeDef GstCH_TheoryVelLPF = {LPF_Alpha_VelTheory};

/*底盘速度补偿低通滤波器*/
LPF_StructTypeDef GstCH_VelCompLPF = {LPF_Alpha_VelComp};

/*底盘速度龙伯格观测器*/
LuenbergerObserver_StructTypeDef GstCH_VelObserver;

/*底盘Yaw、Pitch角速度低通滤波器*/
LPF_StructTypeDef GstCH_YawAngleVelLPF   = {LPF_Alpha_YawAngleVel};   //底盘Yaw轴角速度低通滤波器结构体
LPF_StructTypeDef GstCH_PitchAngleVelLPF = {LPF_Alpha_PitchAngleVel}; //底盘Pitch轴角速度低通滤波器结构体
//#endregion

//#region /****TD算法相关*****************************/
/*腿长TD跟踪结构体，初始化顺序：r, h0, SampleTime（速度因子、滤波因子、采样时间）*/
//待考虑：这里的TD要不要后面改成用mm为单位的，然后下面的PID也改成mm为单位的（注意修改参数、使用的地方不要*MM2M）
TD_StructTypeDef GstCH_LegLen1TD = {TD_LegLen_r, TD_LegLen_h0, TD_SampleTime}; //左腿长度TD结构体，以m米为单位
TD_StructTypeDef GstCH_LegLen2TD = {TD_LegLen_r, TD_LegLen_h0, TD_SampleTime}; //右腿长度TD结构体，以m米为单位
//#endregion

//#region /****PID控制相关*****************************/
/*腿长PID控制结构体，初始化顺序：Kp, Ki, Kd, UMax, UpMax, UiMax, UdMax, AddMax：比例系数、积分系数、微分系数、总输出最大值、Kp项输出最大值、Ki项输出最大值、Kd项输出最大值、SumE单次累加的最大值*/
PID_StructTypeDef GstCH_LegLen1PID = {PID_LegLen_Kp, PID_LegLen_Ki, PID_LegLen_Kd, PID_LegLen_UMax, PID_LegLen_UpMax, PID_LegLen_UiMax, PID_LegLen_UdMax, PID_LegLen_AddMax}; //左腿长度PID结构体，以m米为单位
PID_StructTypeDef GstCH_LegLen2PID = {PID_LegLen_Kp, PID_LegLen_Ki, PID_LegLen_Kd, PID_LegLen_UMax, PID_LegLen_UpMax, PID_LegLen_UiMax, PID_LegLen_UdMax, PID_LegLen_AddMax}; //右腿长度PID结构体，以m米为单位

/*Roll轴补偿PID结构体，初始化顺序：Kp, Ki, Kd, UMax, UpMax, UiMax, UdMax, AddMax：比例系数、积分系数、微分系数、总输出最大值、Kp项输出最大值、Ki项输出最大值、Kd项输出最大值、SumE单次累加的最大值*/
PID_StructTypeDef GstCH_RollCompPID = {PID_RollComp_Kp,PID_RollComp_Ki,PID_RollComp_Kd,PID_RollComp_UMax,PID_RollComp_UpMax,PID_RollComp_UiMax,PID_RollComp_UdMax,PID_RollComp_AddMax}; //底盘Roll轴补偿PID结构体

//#endregion

//#region /****其他底盘运动控制相关-辅助变量*****************************/
/* 腿部五连杆解算结构体，INIT顺序为ThighLen, CalfLen, l5, phi1ZP, phi2ZP, SampleTime */
/* 大腿长度、小腿长度、关节电机间距、phi1零点、phi2零点、采样时间 */
LegLinkageCal_StructTypeDef GstCH_LegLinkCal1 = LegLinkageCal_StructINIT \
(LeftThighLen,  LeftCalfLen,  SameSideJMDistance, JM3LinkageCalZP, JM1LinkageCalZP, SampleTime_Default);  //左腿五连杆解算结构体

LegLinkageCal_StructTypeDef GstCH_LegLinkCal2 = LegLinkageCal_StructINIT \
(RightThighLen, RightCalfLen, SameSideJMDistance, JM2LinkageCalZP, JM4LinkageCalZP, SampleTime_Default);  //右腿五连杆解算结构体

/*LQR解算结构体*/
LQR_StructTypeDef GstCH_LQRCal; //底盘LQR计算结构体

/*VMC计算结构体*/
VMC_StructTypeDef GstCH_Leg1VMC; //左腿VMC计算结构体
VMC_StructTypeDef GstCH_Leg2VMC; //右腿VMC计算结构体
//#endregion

//#region /****其他底盘运动控制相关-正式变量*****************************/
/*底盘状态枚举*/
ChassisMode_EnumTypeDef GEMCH_Mode    = CHMode_RC_ManualSafe; //底盘模式，默认是手动安全模式
ChassisMode_EnumTypeDef GEMCH_ModePre = CHMode_RC_ManualSafe; //上次的底盘模式，默认是手动安全模式

/*关节电机MIT协议控制结构体。INIT顺序为：ID MITKp MITKd：电机ID  MIT协议的位置系数  MIT协议的速度系数*/
JMData_StructTypeDef GSTCH_JM1 = {JM1ID,0,0}; //左前关节电机控制结构体
JMData_StructTypeDef GSTCH_JM2 = {JM2ID,0,0}; //右前关节电机控制结构体
JMData_StructTypeDef GSTCH_JM3 = {JM3ID,0,0}; //左后关节电机控制结构体
JMData_StructTypeDef GSTCH_JM4 = {JM4ID,0,0}; //右后关节电机控制结构体

/*轮毂电机控制结构体。INIT顺序为：ReductionRatio：电机减速比*/
HMData_StructTypeDef GSTCH_HM1 = {HM_ReductionRatio}; //左轮毂电机控制结构体
HMData_StructTypeDef GSTCH_HM2 = {HM_ReductionRatio}; //右轮毂电机控制结构体

/*底盘数据结构体*/
CHData_StructTypeDef GSTCH_Data;       //底盘正式数据结构体，存放和底盘相关的几乎所有数据
//#endregion
