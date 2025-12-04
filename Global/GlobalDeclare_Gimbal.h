/**
  ******************************************************************************
  * @file    GlobalDeclare_Gimbal.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.19
  * @brief   用来声明、引出与云台有关的全局变量
  ******************************************************************************
*/

#include "stdint.h"
#include "Algorithm.h"
#include "FreeRTOS.h"

/*************************************结构体声明**************************************/
/*IMU1云台云控数据处理结构体，包括发送和接收(注意4字节对齐)(32位单片机默认)*/
//待优化：云控现在不需要Reload了，可以后面看看删掉（注意云控那边也要修改）
typedef struct
{
    struct
    {
        uint8_t head[2];        //帧头
        float PitchAngle;       
		float PitchAngleBuff;   
        float PitchAngleVel;       
        float YawAngle;         
        float YawSpeed;         
		float RollAngle;        
		float RollAngleBuff;    //待优化：这个变量没有用到
        float Distance_Z;       
        uint8_t tail[2];             
    } ST_Rx;

    struct
    {
        uint8_t head[2];
        uint8_t ReloadStatus;   //好像是装弹标志位，但是现在没用了，可以考虑删掉（但是不能直接无脑删啊，在数据收发那边会有问题的）
        uint8_t ReStart;        //IMU1云台云控重启标志位
        uint8_t tail[2];
    } ST_Tx;                         
}IMU1Data_StructTypeDef;


//#region /**** 变量引出extern声明****************************************************************/
/*FreeRTOS任务相关*/
extern const TickType_t GGB_TaskPeriod;
extern const float GGB_TaskTime;

/****************************通讯相关****************************/
extern IMU1Data_StructTypeDef GstGB_IMU1; 
extern uint8_t GFGB_IMU1Restart;


/************************PID、位姿相关************************/
extern PID_StructTypeDef GstGB_PitchPosPID;
extern PID_StructTypeDef GstGB_PitchSpeedPID;
extern PID_StructTypeDef GstGB_YawPosPID;
extern PID_StructTypeDef GstGB_YawSpeedPID;
extern float GGB_RollAngle;
extern float GGB_RollAngleBuff;

//#endregion

/************************************宏定义引出声明************************************/
/********************通讯相关********************/
/*GFGB_IMU1Restart的取值*/
#define IMU1RestartYES  0xF  //IMU1确定重启
#define IMU1RestartNO   0x0  //IMU1不重启
