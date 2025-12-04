/**
  ******************************************************************************
  * @file    GlobalDeclare_General.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.11
  * @brief   extern需要给外部调用的全局变量
  ******************************************************************************
*/
#ifndef __GLOBALDECLARE_GENERAL_H
#define __GLOBALDECLARE_GENERAL_H

#include <stdint.h>
#include <stdbool.h>

/****************************************结构体声明****************************************/
/*辅助结构体：机器人底盘默认可配置的控制变量结构体：无需解除标志位就允许配置*/
typedef struct
{
    /*增加一个成员就需要在Chassis_RobotCtrlDefaultConfigDataReset里增加对应的清零代码*/
    float LegLen1Des;       //左腿腿长目标值（未经过TD处理），单位mm
    float LegLen2Des;       //右腿腿长目标值（未经过TD处理），单位mm

    float DisDes;           //底盘位移目标值，向前为正，单位m
    float VelDes;           //底盘速度目标值，向前为正，单位m/s
    float YawDeltaDes;      //底盘Yaw轴偏转角度目标值，逆时针为正，单位度/s
    float YawAngleVelDes;   //偏转角速度目标值，逆时针为正，单位度/s

    float Leg1FFForce;      //左腿前馈力，单位N
    float Leg2FFForce;      //右腿前馈力，单位N
}_ChassisCtrlData_DefaultConfig_StructTypeDef;

/*辅助结构体：机器人底盘强制可配置的控制变量结构体：必须解除标志位才可以配置*/
typedef struct
{
    /*增加一个成员就需要在Chassis_RobotCtrlForceConfigDataReset里增加对应的清零代码*/
    /*LQR用户自定义配置*/
    bool  F_LQR_UserSetEnable;  //LQR自定义配置标志位。true：所有目标值均可由用户自定义。false：除了前四项外，其他目标值保持默认0值

    float Theta1Des;          //左腿摆角目标值，后摆为正，单位度
    float Theta1AngleVelDes;  //左腿摆角速度目标值，后摆为正，单位度/s
    float Theta2Des;          //右腿摆角目标值，后摆为正，单位度
    float Theta2AngleVelDes;  //右腿摆角速度目标值，后摆为正，单位度/s
    float PitchAngleDes;      //俯仰角目标值，抬头为正，单位度
    float PitchAngleVelDes;   //俯仰角速度目标值，抬头为正，单位度/s

    /*VMC用户自定义配置*/
    bool  F_VMC_UserSetEnable;  //VMC自定义配置标志位。true：VMC的力、力矩由用户自定义，Leg1F、Leg2F、Leg1T、Leg2T自定义变量生效。false：VMC自动计算力和力矩

    float Leg1FDes;      //左腿沿杆力的目标值，单位N
    float Leg2FDes;      //右腿沿杆力的目标值，单位N
    float Leg1TDes;      //左腿力矩目标值，后摆为正，单位N·m
    float Leg2TDes;      //右腿力矩目标值，后摆为正，单位N·m

    /*轮毂电机扭矩用户自定义配置*/
    bool F_HMTorque_UserSetEnable;  //轮毂电机扭矩自定义配置标志位。true：轮毂电机扭矩由用户自定义，HM1T、HM2T生效。false：轮毂电机扭矩由底盘控制策略计算得出
    
    float HM1TDes;  //左轮毂电机扭矩目标值，正值为前进，单位N·m
    float HM2TDes;  //右轮毂电机扭矩目标值，正值为前进，单位N·m

    /*关节电机扭矩用户自定义配置*/
    bool F_JMTorque_UserSetEnable;  //关节电机扭矩自定义配置标志位。true：关节电机扭矩由用户自定义，JM1T~JM4T生效。false：关节电机扭矩由底盘控制策略计算得出

    float JM1TDes;      //关节电机1力矩目标值，单位N·m
    float JM2TDes;      //关节电机2力矩目标值，单位N·m
    float JM3TDes;      //关节电机3力矩目标值，单位N·m
    float JM4TDes;      //关节电机4力矩目标值，单位N·m
}_ChassisCtrlData_ForceConfig_StructTypeDef;

/*机器人整体控制结构体*/
typedef struct
{
    /*底盘相关*/
    _ChassisCtrlData_DefaultConfig_StructTypeDef STCH_Default;   //底盘默认可配置控制变量结构体
    _ChassisCtrlData_ForceConfig_StructTypeDef   STCH_Force;     //底盘强制可配置控制变量结构体

    ////待补充 /*云台相关数据结构体*/
}RobotControl_StructTypeDef;

/*系统监控结构体，用来查看各个任务的帧率*/
typedef struct
{
    /************************计数器************************/
    uint16_t CAN1Rx_cnt;        //CAN1Rx计数器
    uint16_t CAN2Rx_cnt;        //CAN2Rx计数器
    uint16_t HubMotor1Rx_cnt;   //左轮毂电机接收计数器
    uint16_t HubMotor2Rx_cnt;   //右轮毂电机接收计数器

    uint16_t USART1Rx_cnt;      //USART1Rx计数器
    uint16_t USART2Rx_cnt;      //USART2Rx计数器
    uint16_t USART3Rx_cnt;      //USART3Rx计数器
    uint16_t UART4Rx_cnt;       //UART4Rx计数器
    uint16_t UART5Rx_cnt;       //UART5Rx计数器
    uint16_t USART6Rx_cnt;      //USART6Rx计数器

    uint16_t SendDataTask_cnt;  //SendDataTask计数器
    uint16_t ChassisTask_cnt;   //ChassisTask计数器
    uint16_t GimbalTask_cnt;    //GimbalTask计数器
    uint16_t ShootTask_cnt;     //ShootTask计数器
    uint16_t DebugTask_cnt;     //DebugTask计数器

    /************************帧率************************/
    uint16_t CAN1Rx_fps;        //CAN1Rx帧率
    uint16_t CAN2Rx_fps;        //CAN2Rx帧率
    uint16_t HubMotor1Rx_fps;   //左轮毂电机帧率
    uint16_t HubMotor2Rx_fps;   //右轮毂电机帧率

    uint16_t USART1Rx_fps;      //USART1Rx帧率
	uint16_t USART2Rx_fps;      //USART2Rx帧率
	uint16_t USART3Rx_fps;      //USART3Rx帧率
	uint16_t UART4Rx_fps;       //UART4Rx帧率
	uint16_t UART5Rx_fps;       //UART5Rx帧率
	uint16_t USART6Rx_fps;      //USART6Rx帧率

    uint16_t SendDataTask_fps;  //SendDataTask帧率
    uint16_t ChassisTask_fps;   //ChassisTask帧率
    uint16_t GimbalTask_fps;    //GimbalTask帧率
    uint16_t ShootTask_fps;     //ShootTask帧率
    uint16_t DebugTask_fps;     //DebugTask帧率
}SystemMonitor_StructTypeDef;

/*遥控器接收机 接收DR16 18字节数据的结构体*/
typedef struct
{
    /*辅助结构体1：遥控器数据结构体*/
    struct _RCControlData_StructTypeDef
    {
        uint16_t JoyStickR_X; //右摇杆X轴，通道0
        uint16_t JoyStickR_Y; //右摇杆Y轴，通道1
        uint16_t JoyStickL_X; //左摇杆X轴，通道2
        uint16_t JoyStickL_Y; //左摇杆Y轴，通道3
        uint16_t Roller;      //拨轮，通道4
        uint8_t  Level_L;     //左拨杆
        uint8_t  Level_R;     //右拨杆
    }ST_RC;

    /*辅助结构体2：鼠标数据结构体*/
    //待优化：这里的变量命名不是很清楚（比如这个Z是干嘛的，Left表示左键有点奇怪）
    struct _KeyMouseControlData_StructTypeDef
    {
        int16_t X;
        int16_t Y;
        int16_t Z;
        uint8_t Left;   //左键是否按下
        uint8_t Right;  //右键是否按下
    }ST_Mouse;
    
    uint16_t usKeyboard;
}ReceiverData_StructTypeDef;

/*C620反馈数据结构体*/
typedef struct
{
    int16_t EncoderValue; //编码器反馈值0-8191（0x1FFF），对应电机的0-360度
    int16_t AngleVelFB;      //速度反馈，单位：r/min（注意是电机不是减速箱输出轴）
    int16_t CurrentFB;    //电流反馈，单位：mA
    int8_t  TempFB;       //温度反馈，单位：°C
}C620FeedBackData_StructTypeDef;

/*编码器相关数据结构体*/
//待考虑：要不要加入差值、零点之类的变量
typedef struct
{
    /*需要初始化赋值的成员*/
    const int32_t PPR;   //转一圈产生的编码器脉冲数

    /*不需要初始化赋值的成员*/
    int32_t Value;       //编码器值
    int32_t ValuePre;    //编码器上次值
    int32_t ValueSum;    //编码器值累计和
}EncoderData_StructTypeDef;

//#region /****枚举声明*********************************************************************************/
/*机器人左右侧枚举类型*/
typedef enum
{
    LeftSide,   //左侧
    RightSide   //右侧
}RobotSide_EnumTypeDef;
//#endregion

/********************************不需要修改的变量引出extern声明********************************/
/*************机器人整体控制结构体*************/

extern RobotControl_StructTypeDef GST_RMCtrl;

/***************test标志位、变量***************/
extern uint8_t  G_TestFlag;
extern uint8_t  G_u8Test;
extern uint16_t G_u16Test;
extern uint32_t G_u32Test;
extern int16_t  G_s16Test;
extern int32_t  G_s32Test;
extern float    G_fTest;
extern float    G_fTest1;
extern float    G_fTest2;
extern float    G_fTest3;

/********Debug使用，系统监控结构体********/
extern SystemMonitor_StructTypeDef GST_SystemMonitor;

/*****************通讯相关****************/
extern bool GF_USART1_RxDone;
extern ReceiverData_StructTypeDef GST_Receiver;


/****************************************宏定义引出声明（一般不需要修改）****************************************/
/************************遥控器相关************************/
/*拨杆相关*/
#define RCLevel_Up       ((uint8_t)1)   //遥控器拨杆在上
#define RCLevel_Mid      ((uint8_t)3)   //遥控器拨杆在中
#define RCLevel_Down     ((uint8_t)2)   //遥控器拨杆在下
/*通道（拨轮、摇杆）相关*/
#define RCChannelValue_Min          ((uint16_t)364)  //遥控器通道最小值
#define RCChannelValue_Mid          ((uint16_t)1024) //遥控器通道中间值
#define RCChannelValue_Max          ((uint16_t)1684) //遥控器通道最大值
#define RCJoyStick_UpTH             ((uint16_t)RCChannelValue_Max - 100)   //遥控器摇杆上阈值，Y轴超过该值认为摇杆在上
#define RCJoyStick_DownTH           ((uint16_t)RCChannelValue_Min + 100)   //遥控器摇杆下阈值，Y轴低于该值认为摇杆在下
#define RCJotStick_LeftTH           ((uint16_t)RCChannelValue_Min + 100)   //遥控器摇杆左阈值，X轴低于该值认为摇杆在左
#define RCJoyStick_RightTH          ((uint16_t)RCChannelValue_Max - 100)   //遥控器摇杆右阈值，X轴超过该值认为摇杆在右
#define RCRoller_UpTH               ((uint16_t)RCChannelValue_Min + 100)   //遥控器拨轮上阈值，拨轮低于该值认为拨轮在上
#define RCRoller_DownTH             ((uint16_t)RCChannelValue_Max - 100)   //遥控器拨轮下阈值，拨轮超过该值认为拨轮在下
#define RCChannel_DeadZone          ((uint16_t)340)    //遥控器通道死区范围，摇杆在中间位置时，偏差小于该值则认为是0输入
//待优化：下面的宏定义暂时没什么用，可以考虑删掉。如果有用到的话改成一个更合适的名字
// #define RC_CH_VALUE_GIMBAL_DEAD     ((u16)20)
// #define RC_CH_VALUE_RANGE           ((u16)660)


/********SystemMonitor相关，最低帧率阈值取正常值的70%左右********/
#define USART1Rx_fpsMinTH       50      //串口1接收帧率最小阈值，低于该值可能是遥控器断开
#define UART4Rx_fpsMinTH        800     //串口4接收帧率最小阈值，低于该值可能是IMU2断开
#define HubMotorRx_fpsMinTH     700     //轮毂电机接收帧率最小阈值，低于该值可能是电机断开

/********电机、电调相关********/
#define ESC_C620Period              0.001f                   //C620电调反馈周期，单位：秒
#define ESC_C620PPR                 8192                     //C620电调的编码器每圈脉冲数
#define ESC_C620MaxCurrent          16384                    //C620电调的最大电流值，对应20A
#define ESC_C620MinCurrent          -16384                   //C620电调的最小电流值，对应-20A
#define ESC_C620AmpereToCurrent     (1/20.0f*16384.0f)       //C620电调电流转换系数，单位：映射电流值/A

// #define Motor_3508ReductionRatio        3591.0f/187.0f          //3508电机减速比(本体的减速箱)
#define Motor_3508GearboxReductionRatio 268.0f/17.0f            //3508电机外接齿轮箱减速比
#define Motor_3508MaxCurrent            ESC_C620MaxCurrent      //3508电机（连在C620电调上）的最大电流值
#define Motor_3508MinCurrent            ESC_C620MinCurrent      //3508电机（连在C620电调上）的最小电流值
#define Motor_3508AmpereToCurrent       ESC_C620AmpereToCurrent //3508电机（连在C620电调上）电流转换系数，单位：映射电流值/A
#define Motor_3508Kt                    0.3f                    //3508电机转矩常数，单位：Nm/A

#define Motor_MG8016Ei6MaxTorque        37.0f	                //瓴控的MG8016E-i6电机最大转矩，单位：Nm


/********计算相关********/
/*角弧度制互换*/
#define A2R     PI/180.0f    //角度制转弧度制，AngleToRadian
#define R2A     180.0f/PI    //弧度制转角度制，RadianToAngle
/*其他单位互换*/
#define MM2M    0.001f       //毫米转米，MillimeterToMeter
#define M2MM    1000.0f      //米转毫米，MeterToMillimeter
/*其他常数*/
#define GravityAcc  9.80f    //标准重力加速度，单位：m/s²
#define GravityAcc_Harbin 9.806639f //哈尔滨地区重力加速度，单位：m/s²
#define GravityAcc_ChangSha 9.7915f //长沙地区重力加速度，单位：m/s²
#define GravityAcc_ShenZhen 9.7803f //深圳地区重力加速度，单位：m/s²

#endif
