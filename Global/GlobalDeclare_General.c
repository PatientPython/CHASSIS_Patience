/**
  ******************************************************************************
  * @file    GlobalDeclare_General.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.19
  * @brief   用来存放通用的全局变量，如果是与底盘或者云台有关的全局变量，
             请前往GlobalDeclare_Chassis或GlobalDeclare_Gimbal
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include <stdbool.h>
#include "GlobalDeclare_General.h"
#include <arm_math.h>

/******************************************不需要修改的变量定义******************************************/
/********test标志位、变量********/

uint8_t  G_TestFlag = 0;     //test用标志位，可以随便放在一些你想放的位置方便Debug，但是注意Debug后要删掉
uint8_t  G_u8Test;           //test用的，uint8_t变量
uint16_t G_u16Test;          //test用的，uint16_t变量
uint32_t G_u32Test;          //test用的，uint32_t变量
int16_t  G_s16Test;          //test用的，int16_t变量
int32_t  G_s32Test;          //test用的，int32_t变量
float    G_fTest;            //test用的，float变量
float    G_fTest1;           //test用的，float变量1
float    G_fTest2;           //test用的，float变量2
float    G_fTest3;           //test用的，float变量3

/********Debug使用，系统监控结构体********/

SystemMonitor_StructTypeDef GST_SystemMonitor;  //系统监控结构体，帮助统计各个任务的帧率

/********通讯相关********/

bool GF_USART1_RxDone = false;           //串口1接收完成标志位
ReceiverData_StructTypeDef GST_Receiver;    //遥控器接收机接收数据结构体

/*机器人整体控制结构体*/
RobotControl_StructTypeDef GST_RMCtrl; //机器人整体控制结构体，这个结构体只输入，不输出。如果要获取对应变量的输出，请前往对应的数据结构体，比如GSTCH_Data等。

