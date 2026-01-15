/**
  ******************************************************************************
  * @file    CAN_Communication.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.13
  * @brief   存放CAN的通讯相关函数等等
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Chassis.h"

/**
  * @brief  CAN的发送函数,CAN1和CAN2都可以用这个函数发送
  * @note   只支持标准帧，如果后面要拓展帧需要重写。固定发送8个Byte
  * @param  CANx：CAN1或者CAN2，指定用哪个CAN发送
  * @param  ID：CAN的标准数据帧ID号
  * @param  Datax：int16_t类型，要发送的数据
  * @retval 和CAN_Transmit的一样
*/
uint8_t CAN_Send(CAN_TypeDef *CANx,uint32_t ID,int16_t Data1,int16_t Data2,int16_t Data3,int16_t Data4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = ID;               //ID号
    TxMessage.IDE = CAN_Id_Standard;    //标准帧
    TxMessage.RTR = CAN_RTR_Data;       //数据帧
    TxMessage.DLC = 0x08;               //8个Byte
	
    TxMessage.Data[0] = (uint8_t)(Data1 >> 8);
    TxMessage.Data[1] = (uint8_t)Data1;
    TxMessage.Data[2] = (uint8_t)(Data2 >> 8);
    TxMessage.Data[3] = (uint8_t)Data2;
    TxMessage.Data[4] = (uint8_t)(Data3 >> 8);
    TxMessage.Data[5] = (uint8_t)Data3;
    TxMessage.Data[6] = (uint8_t)(Data4 >> 8);
    TxMessage.Data[7] = (uint8_t)Data4;
    return CAN_Transmit(CANx,&TxMessage);
}

/**
  * @brief  CAN向除了关节电机外的各个电机发送电流的函数
  * @note   关节电机的发送是在IMU2中，主控先把目标值发给IMU2，再由IMU2发给关节电机
  * @param  无
  * @retval 无
*/
void CANTx_SendCurrentToMotor(void)
{
	/*底盘：发送轮毂电机电流*/
    CAN_Send(CAN2,0x200,GSTCH_HM2.CurrentDes,GSTCH_HM1.CurrentDes,0,0);
}

/**
  * @brief  用来解析C620电调CAN反馈的数据
  * @note   
  * @param  pESC：C620电调反馈数据结构体指针
  * @param  RxMsg：CAN接收消息结构体指针
  * @retval 无
*/
void CANRx_C620DataParse(C620FeedBackData_StructTypeDef* pESC, CanRxMsg* RxMsg)
{
    /*CAN接收的数据，解析到C620反馈数据结构体*/
    pESC->EncoderValue = (RxMsg->Data[0]<<8 | RxMsg->Data[1]);  //老代码说明：原来是个函数叫Get_Encoder_Number()
    pESC->AngleVelFB   = (RxMsg->Data[2]<<8 | RxMsg->Data[3]);  //老代码说明：原来是个函数叫Get_Speed()
    pESC->CurrentFB    = (RxMsg->Data[4]<<8 | RxMsg->Data[5]);  
    pESC->TempFB       =  RxMsg->Data[6];                       //老代码说明：原来是个函数叫Get_Temperature()
}

/**
  * @brief  用来处理C620电调CAN反馈的编码器值
  * @note   更新的变量为EncoderValue和EncoderValueSum，即编码器值和编码器值累计和
  * @param  pHM：轮毂电机控制结构体指针
  * @param  EncoderNowValue：电调反馈的当前编码器值
  * @retval 无
*/
//老代码说明：原来的函数名Abs_Encoder_Process()
void CANRx_EncoderValueProcess(EncoderData_StructTypeDef* pEncoder, int32_t ValueNow)
{
    /********************获取数据********************/
    pEncoder->ValuePre = pEncoder->Value;   //更新上次编码器值
    pEncoder->Value    = ValueNow;          //更新当前编码器值

    /********************计算编码器值变化量********************/
    int32_t Delta = pEncoder->Value - pEncoder->ValuePre;
    /*如果两次编码器的反馈值差别太大*/
    /*表示编码器圈数发生了改变*/
    /*需要加上或减去一圈产生的值*/
    if(Delta < -4000)
    {
        Delta += (pEncoder->PPR);
    }
    else if(Delta > 4000)
    {
        Delta -= (pEncoder->PPR);
    }

    /********************更新数据到结构体中********************/
    pEncoder->ValueSum += Delta;  //计算编码器值累计和
}
