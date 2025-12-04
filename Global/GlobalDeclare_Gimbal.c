/**
  ******************************************************************************
  * @file    GlobalDeclare_Gimbal.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.19
  * @brief   用来存放与云台有关的全局变量
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include "stm32f4xx.h"
#include "GlobalDeclare_Gimbal.h"
#include "Algorithm.h"
#include "FreeRTOS.h"

/****************************************宏定义、常量定义（不需要修改）****************************************/
/*FreeRTOS任务相关*/
const TickType_t GGB_TaskPeriod = 1;      //Gimbal的任务周期，单位为FreeRTOS的系统节拍。默认是ms（取决于configTICK_RATE_HZ）
const float GGB_TaskTime = (float)GGB_TaskPeriod/(float)configTICK_RATE_HZ; //任务运行周期，单位为秒

/*一些默认定义*/
#define SampleTime_Default GGB_TaskTime   //默认采样时间，单位秒

/********************************************变量定义(不需要修改)********************************************/
/****************************通讯相关****************************/
IMU1Data_StructTypeDef GstGB_IMU1;          //云台云控IMU1的通讯数据结构体，包括接收和发送
uint8_t GFGB_IMU1Restart = IMU1RestartNO;   //云台云控IMU1重启标志位，默认不重启

/************************位姿、电机PID相关************************/
PID_StructTypeDef GstGB_PitchPosPID;    //云台Pitch位置环PID结构体
PID_StructTypeDef GstGB_PitchSpeedPID;  //云台Pitch速度环PID结构体
PID_StructTypeDef GstGB_YawPosPID;      //云台Yaw位置环PID结构体
PID_StructTypeDef GstGB_YawSpeedPID;    //云台Yaw速度环PID结构体
float GGB_RollAngle = 0;                //云台Roll轴角度            //待优化：只在发给视觉的时候用到
