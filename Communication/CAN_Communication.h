/**
  ******************************************************************************
  * @file    CAN_Communication.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.13
  * @brief   存放CAN的通讯相关函数等等
  ******************************************************************************
*/
#include "stm32f4xx.h"
#include "GlobalDeclare_Chassis.h"

uint8_t CAN_Send(CAN_TypeDef *CANx,uint32_t ID,int16_t Data1,int16_t Data2,int16_t Data3,int16_t Data4);

void CANTx_SendCurrentToMotor(void);

void CANRx_C620DataParse(C620FeedBackData_StructTypeDef* pESC, CanRxMsg* RxMsg);

