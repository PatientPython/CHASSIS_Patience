/**
  ******************************************************************************
  * @file    CAN_Config.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.21
  * @brief   CAN1、CAN2的初始化配置函数，包括波特率、中断优先级等等
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include "stm32f4xx.h"

/**
  * @brief  CAN1的初始化函数
  * @note   占用PB8、9复用为CAN1引脚；
            配置CAN为1M波特率（42MHz，分频系数3，SS = 1Tq，BS1 = 9Tq，BS2 = 4Tq）。
            配置过滤器为全部通过状态（也就是不过滤）。
            开启CAN1 FIFO 0有报文中断，抢占优先级0。
            开启CAN1发送邮箱空中断，抢占优先级1。
  * @param  无
  * @retval 无
  */
void My_CAN1_Init(void)
{
    GPIO_InitTypeDef       GPIO_InitStruct;
    NVIC_InitTypeDef       NVIC_InitStruct;
    CAN_InitTypeDef        CAN_InitStruct;
    CAN_FilterInitTypeDef  CAN_FilterInitStruct;
    
    /**************************************** 开启有关时钟 *****************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //使能GPIO端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);   //使能CAN1时钟,APB1，42MHz
    
    /**************************************** CAN相关GPIO配置 *****************************************/
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1); //PB8为CAN1_Rx
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1); //PB9为CAN1_Tx
    
    /**************************************** CAN单元初始化 *****************************************/
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStruct);
    
    CAN_InitStruct.CAN_NART = DISABLE;        //开启自动重传（注意这里没有写错，DISABLE是开启自动重传）
    CAN_InitStruct.CAN_TXFP = ENABLE;         //发送顺序为先来后到，先请求先发送
    CAN_InitStruct.CAN_RFLM = DISABLE;        //关闭接收FIFO锁定模式
    CAN_InitStruct.CAN_ABOM = ENABLE;         //开启离线（总线关闭状态）自动恢复
    CAN_InitStruct.CAN_TTCM = DISABLE;        //关闭时间触发通信模式
    CAN_InitStruct.CAN_AWUM = DISABLE;        //关闭自动唤醒模式

    /*CAN1波特率=42MHz/3/(9+4+1)=1MHz*/
    CAN_InitStruct.CAN_Mode      = CAN_Mode_Normal; //普通模式
    CAN_InitStruct.CAN_Prescaler = 3;               //分频系数
    CAN_InitStruct.CAN_SJW       = CAN_SJW_2tq;     //再同步补偿最大宽度
    CAN_InitStruct.CAN_BS1       = CAN_BS1_9tq;     //BS1长度
    CAN_InitStruct.CAN_BS2       = CAN_BS2_4tq;     //BS2长度
    CAN_Init(CAN1, &CAN_InitStruct);                //根据指定的参数初始化CAN寄存器
    
    /**************************** CAN过滤器初始化 ****************************/
    CAN_FilterInitStruct.CAN_FilterNumber         = 0;                       //配置过滤器0
    CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;   //标识符过滤器为屏蔽模式（或者叫掩码模式）
    CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;   //配置为32位标识符过滤器
    CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000;                  //32位标识符过滤器的高16位
    CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000;                  //32位标识符过滤器的低16位
    CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000;                  //过滤器掩码高16位
    CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000;                  //过滤器掩码低16位
    CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //过滤器0关联到FIFO0
    CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;                  //激活过滤器
    CAN_FilterInit(&CAN_FilterInitStruct);             //根据指定的参数初始化CAN_Filter寄存器，配置为全部可以通过模式
        
    /****************************************CAN中断配置****************************************/
    NVIC_InitStruct.NVIC_IRQChannel                    = CAN1_RX0_IRQn;//接收FIFO 0的中断
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 0;        //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;        //子优先级（无效）
    NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE;   //IRQ通道使能
    NVIC_Init(&NVIC_InitStruct);   //根据指定的参数初始化NVIC寄存器

    NVIC_InitStruct.NVIC_IRQChannel                    = CAN1_TX_IRQn;//CAN1的发送中断
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 1;        //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;        //子优先级（无效）
    NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE;   //IRQ通道使能
    NVIC_Init(&NVIC_InitStruct);   //根据指定的参数初始化NVIC寄存器

    /**************************************** CAN1中断使能 *****************************************/
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  //开启中断：CAN1的FIFO 0有报文
    CAN_ITConfig(CAN1, CAN_IT_TME,  ENABLE);  //开启中断：CAN1发送邮箱空
}

/**
  * @brief  CAN2的初始化函数
  * @note   占用PB5、6复用为CAN2引脚；
            配置CAN为1M波特率（42MHz，分频系数3，SS = 1Tq，BS1 = 9Tq，BS2 = 4Tq）。
            配置过滤器为全部通过状态（也就是不过滤）。
            开启CAN2 FIFO 0有报文中断，抢占优先级0。
            开启CAN2发送邮箱空中断，抢占优先级1。
  * @param  无
  * @retval 无
  */
void My_CAN2_Init(void)
{
    GPIO_InitTypeDef       GPIO_InitStruct;
    NVIC_InitTypeDef       NVIC_InitStruct;
    CAN_InitTypeDef        CAN_InitStruct;
    CAN_FilterInitTypeDef  CAN_FilterInitStruct;
    
    /**************************************** 开启有关时钟 *****************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //使能GPIO端口时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);   //使用CAN2,需先使能CAN1的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,  ENABLE);   //使能CAN2时钟,42MHz
    
    /**************************************** CAN相关GPIO配置 *****************************************/
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);//PB5，为CAN2_RX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);//PB6，为CAN2_TX
    
    /**************************************** CAN单元初始化 *****************************************/
    CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStruct);
    
    CAN_InitStruct.CAN_NART = DISABLE;        //开启自动重传（注意这里没有写错，DISABLE是开启自动重传）
    CAN_InitStruct.CAN_TXFP = ENABLE;         //发送顺序为先来后到，先请求先发送
    CAN_InitStruct.CAN_RFLM = DISABLE;        //关闭接收FIFO锁定模式
    CAN_InitStruct.CAN_ABOM = ENABLE;         //开启离线（总线关闭状态）自动恢复
    CAN_InitStruct.CAN_TTCM = DISABLE;        //关闭时间触发通信模式
    CAN_InitStruct.CAN_AWUM = DISABLE;        //关闭自动唤醒模式

    /*CAN1波特率=42MHz/3/(9+4+1)=1MHz*/
    CAN_InitStruct.CAN_Mode      = CAN_Mode_Normal; //普通模式
    CAN_InitStruct.CAN_Prescaler = 3;               //分频系数
    CAN_InitStruct.CAN_SJW       = CAN_SJW_2tq;     //再同步补偿最大宽度
    CAN_InitStruct.CAN_BS1       = CAN_BS1_9tq;     //BS1长度
    CAN_InitStruct.CAN_BS2       = CAN_BS2_4tq;     //BS2长度
    CAN_Init(CAN2, &CAN_InitStruct);                //根据指定的参数初始化CAN寄存器
    
    /**************************** CAN过滤器初始化 ****************************/
    CAN_FilterInitStruct.CAN_FilterNumber         = 14;                       //配置过滤器0
    CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;   //标识符过滤器为屏蔽模式（或者叫掩码模式）
    CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;   //配置为32位标识符过滤器
    CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000;                  //32位标识符过滤器的高16位
    CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000;                  //32位标识符过滤器的低16位
    CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000;                  //过滤器掩码高16位
    CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000;                  //过滤器掩码低16位
    CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //过滤器0关联到FIFO0
    CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;                  //激活过滤器
    CAN_FilterInit(&CAN_FilterInitStruct);             //根据指定的参数初始化CAN_Filter寄存器，配置为全部可以通过模式

    /****************************************CAN2中断配置****************************************/
    NVIC_InitStruct.NVIC_IRQChannel                    = CAN2_RX0_IRQn;//接收FIFO 0的中断
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 0;        //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;        //子优先级（无效）
    NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE;   //IRQ通道使能
    NVIC_Init(&NVIC_InitStruct);   //根据指定的参数初始化NVIC寄存器

    NVIC_InitStruct.NVIC_IRQChannel                    = CAN2_TX_IRQn;//CAN1的发送中断
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 1;        //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;        //子优先级（无效）
    NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE;   //IRQ通道使能
    NVIC_Init(&NVIC_InitStruct);   //根据指定的参数初始化NVIC寄存器

    /****************************************CAN2中断使能****************************************/
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);  //开启中断：CAN2的FIFO 0有报文
    CAN_ITConfig(CAN2, CAN_IT_TME,  ENABLE);  //开启中断：CAN2发送邮箱空
}
