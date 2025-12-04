/**
  ******************************************************************************
  * @file    USART_Communication.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.13
  * @brief   串口的通讯相关函数、缓冲区等等
  ******************************************************************************
*/
#ifndef __USART_COMMUNICATION_H
#define __USART_COMMUNICATION_H

#include "stdint.h"

/****************************变量引出extern声明****************************/
/********串口发送/接收缓冲区长度********/
extern const uint8_t UA1RxDMAbuf_LEN;
extern const uint8_t UA2TxDMAbuf_LEN;
extern const uint8_t UA2RxDMAbuf_LEN;
extern const uint8_t UA3TxDMAbuf_LEN;
extern const uint8_t UA3RxDMAbuf_LEN;
extern const uint8_t UA4TxDMAbuf_LEN;
extern const uint8_t UA4RxDMAbuf_LEN;
extern const uint8_t UA5TxDMAbuf_LEN;
extern const uint16_t UA5RxDMAbuf_LEN;
extern const uint8_t UA6TxDMAbuf_LEN;
extern const uint8_t UA6RxDMAbuf_LEN;
/********串口缓冲区数组********/
extern uint8_t UA1RxDMAbuf[];
extern uint8_t UA2RxDMAbuf[];
extern uint8_t UA3RxDMAbuf[];
extern uint8_t UA4RxDMAbuf[];
extern uint8_t UA5TxDMAbuf[];
extern uint8_t UA5RxDMAbuf[];
extern uint8_t UA6RxDMAbuf[];

/****************************宏定义引出声明****************************/
/******** DMA数据流的宏定义 ********/
#define USART1_RX_STREAM        DMA2_Stream2 //DMA2_Channel4
#define USART2_TX_STREAM        DMA1_Stream6 //DMA1_Channel4
#define USART2_RX_STREAM        DMA1_Stream5 //DMA1_Channel4
#define USART3_TX_STREAM        DMA1_Stream3 //DMA1_Channel4
#define USART3_RX_STREAM        DMA1_Stream1 //DMA1_Channel4
#define UART4_TX_STREAM         DMA1_Stream4 //DMA1_Channel4
#define UART4_RX_STREAM         DMA1_Stream2 //DMA1_Channel4
#define UART5_TX_STREAM         DMA1_Stream7 //DMA1_Channel4
#define UART5_RX_STREAM         DMA1_Stream0 //DMA1_Channel4
#define USART6_RX_STREAM        DMA2_Stream1 //DMA2_Channel5
#define USART6_TX_STREAM        DMA2_Stream6 //DMA2_Channel5


/****************************函数声明****************************/
/********串口3、6的DMA打印函数********/
void USART3_DMA_printf(const char* fmt, ...);
void USART6_DMA_printf(const char* fmt, ...);

/********串口收发、数据解析函数********/
void UA1Rx_ReceiverDataProcess(void);
void UA2Rx_IMU1DataProcess(void);
void UA2Tx_SendDataToIMU1(void);
void UA4Rx_IMU2DataProcess(void);
void UA4Tx_SendDataToIMU2(void);

#endif
