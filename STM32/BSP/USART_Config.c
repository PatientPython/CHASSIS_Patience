/**
  ******************************************************************************
  * @file    USART_Config.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.21
  * @brief   串口功能的初始化、prinft的重定向
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include "stm32f4xx.h"
#include "USART_Communication.h"
#include "stdio.h"

/**
  * @brief  USART1的初始化函数
  * @note   串口1，波特率100k,用于遥控的接收机DR16通信
  * @param  无
  * @retval 无
  */
void My_USART1_Init(void)
{
    GPIO_InitTypeDef	GPIO_InitStruct;
    USART_InitTypeDef	USART_InitStruct;
    NVIC_InitTypeDef 	NVIC_InitStruct;
    DMA_InitTypeDef		DMA_InitStruct;

    /****************************打开相关时钟****************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

    /****************************USART1的GPIO配置****************************/
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);//PA10为USART的RX
    
    /****************************USART1配置****************************/
    USART_DeInit(USART1);
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate     = 100000;	        //SBUS波特率是100k
    USART_InitStruct.USART_WordLength   = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits	    = USART_StopBits_1;
    USART_InitStruct.USART_Parity	    = USART_Parity_Even;//偶校验，与接收机协议相同
    USART_InitStruct.USART_Mode		    = USART_Mode_Rx;    //仅接收
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct); 

    /****************************USART1的中断配置****************************/
    USART_ClearFlag(USART1, USART_FLAG_IDLE); //清除标志位
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	    //开启空闲中断（接收的格式固定，所以用空闲中断）
    
    NVIC_InitStruct.NVIC_IRQChannel                     = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority   = 0;//抢占优先级0
    NVIC_InitStruct.NVIC_IRQChannelSubPriority          = 1;//子优先级无效
    NVIC_InitStruct.NVIC_IRQChannelCmd                  = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /****************************USART1的DMA配置****************************/
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	    //开启串口DMA接收功能
    DMA_DeInit(USART1_RX_STREAM);//复位对应的DMA
    while(DMA_GetCmdStatus(USART1_RX_STREAM) == ENABLE); //等待DMA可配置
    DMA_StructInit(&DMA_InitStruct);

    DMA_InitStruct.DMA_Channel                 =   DMA_Channel_4;               //USART1_RX为通道4
    DMA_InitStruct.DMA_PeripheralBaseAddr      =   (uint32_t)&(USART1->DR);	//设置DMA传输外设基地址为串口1的DR寄存器
    DMA_InitStruct.DMA_PeripheralInc           =   DMA_PeripheralInc_Disable;   //设置外设地址不自增
    DMA_InitStruct.DMA_PeripheralDataSize      =   DMA_PeripheralDataSize_Byte; //设置外设的数据长度为字节（8bits）

    DMA_InitStruct.DMA_Memory0BaseAddr         =   (uint32_t)UA1RxDMAbuf;     //设置DMA传输内存基地址为串口1接收缓冲区
    DMA_InitStruct.DMA_MemoryInc               =   DMA_MemoryInc_Enable;	    //设置内存地址自增
    DMA_InitStruct.DMA_MemoryDataSize          =   DMA_MemoryDataSize_Byte;	 	//设置内存的数据长度为字节（8bits）

    DMA_InitStruct.DMA_DIR                     =   DMA_DIR_PeripheralToMemory;  //设置数据传输方向
    DMA_InitStruct.DMA_BufferSize              =   UA1RxDMAbuf_LEN;             //设置DMA一次传输数据量的大小,DR16每隔7ms通过DBus发送一帧数据（18字节）
    DMA_InitStruct.DMA_Mode                    =   DMA_Mode_Circular;		    //设置DMA模式为循环模式
    DMA_InitStruct.DMA_Priority                =   DMA_Priority_VeryHigh;	    //设置DMA通道的优先级为最高优先级

    DMA_InitStruct.DMA_FIFOMode                =   DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold           =   DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_MemoryBurst             =   DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst         =   DMA_PeripheralBurst_Single;
    DMA_Init(USART1_RX_STREAM, &DMA_InitStruct);
    DMA_Cmd(USART1_RX_STREAM, ENABLE);	//使能DMA
    
    USART_Cmd(USART1, ENABLE); //使能串口
}

/**
  * @brief  USART2的初始化函数
  * @note   串口2，波特率576000,用于和云台云控通讯
  * @param  无
  * @retval 无
  */
void My_USART2_Init(void)
{
    GPIO_InitTypeDef	GPIO_InitStruct;
    USART_InitTypeDef	USART_InitStruct;
    NVIC_InitTypeDef 	NVIC_InitStruct;
    DMA_InitTypeDef		DMA_InitStruct;

    /****************************打开相关时钟****************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);        //使能GPIOD时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);      //使能USART2时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);        //使能DMA1时钟

    /****************************USART2的GPIO配置****************************/
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//PP
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);   //TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);   //RX

    /****************************USART2配置****************************/
    USART_DeInit(USART2);
    USART_StructInit(&USART_InitStruct);
	USART_InitStruct.USART_BaudRate			= 576000;//云控波特率
    USART_InitStruct.USART_WordLength		= USART_WordLength_8b;
    USART_InitStruct.USART_StopBits			= USART_StopBits_1;
    USART_InitStruct.USART_Parity			= USART_Parity_No;  //无校验
    USART_InitStruct.USART_Mode			  	= USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);
    
    /****************************USART2的中断配置****************************/
    USART_ClearFlag(USART2, USART_FLAG_IDLE); //清除标志位
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);	//开启空闲中断

    NVIC_InitStruct.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;     //抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;     //子优先级无效
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;//IRQ通道使能
    NVIC_Init(&NVIC_InitStruct);           //根据指定的参数初始化VIC寄存器

    /****************************USART2_Tx的DMA配置****************************/
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  //使能串口发送DMA
    DMA_DeInit(USART2_TX_STREAM);
    while( DMA_GetCmdStatus(USART2_TX_STREAM) == ENABLE );			//等待DMA可配置
    DMA_StructInit(&DMA_InitStruct);

    DMA_InitStruct.DMA_Channel			    =   DMA_Channel_4;         
    DMA_InitStruct.DMA_PeripheralBaseAddr	=	(uint32_t)&(USART2->DR);
    DMA_InitStruct.DMA_Memory0BaseAddr	    =	NULL;//暂无
    DMA_InitStruct.DMA_DIR				    =	DMA_DIR_MemoryToPeripheral;	//内存到外设
    DMA_InitStruct.DMA_BufferSize		    =	NULL;//暂无
    DMA_InitStruct.DMA_PeripheralInc	    =	DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc		    =	DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize	    =	DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode				    =	DMA_Mode_Normal;			//正常发送
    DMA_InitStruct.DMA_Priority			    =	DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_FIFOMode			    =	DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold        =	DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_MemoryBurst		    =	DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst	    =	DMA_PeripheralBurst_Single;
    DMA_Init(USART2_TX_STREAM, &DMA_InitStruct);
    DMA_Cmd(USART2_TX_STREAM, DISABLE);	//失能DMA
    //提示：这里的DMA在IMU_Tx_Protocol中使能

    /****************************USART2_Rx的DMA配置****************************/
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  //使能串口接收DMA
    DMA_DeInit(USART2_RX_STREAM);
    while( DMA_GetCmdStatus(USART2_RX_STREAM) == ENABLE );			//等待DMA可配置
    DMA_StructInit(&DMA_InitStruct);

    DMA_InitStruct.DMA_Channel              =   DMA_Channel_4;				//即DMA_Channel_4
    DMA_InitStruct.DMA_PeripheralBaseAddr   =   (uint32_t)&(USART2->DR);
    DMA_InitStruct.DMA_Memory0BaseAddr      =   (uint32_t)UA2RxDMAbuf;
    DMA_InitStruct.DMA_DIR                  =   DMA_DIR_PeripheralToMemory;	//外设到内存
    DMA_InitStruct.DMA_BufferSize           =   UA2RxDMAbuf_LEN;
    DMA_InitStruct.DMA_PeripheralInc        =	DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc		    =	DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize	    =	DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode                 =   DMA_Mode_Circular;			//循环接收
    DMA_InitStruct.DMA_Priority             =   DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_FIFOMode			    =	DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold        =	DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_MemoryBurst		    =	DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst	    =	DMA_PeripheralBurst_Single;
    DMA_Init(USART2_RX_STREAM, &DMA_InitStruct);
    DMA_Cmd(USART2_RX_STREAM, ENABLE);	//使能DMA

    USART_Cmd(USART2, ENABLE);                      //使能串口
}

/**
  * @brief  USART3的初始化函数
  * @note   串口3，波特率460800,调试专用串口底层配置。注意发送DMA被SD卡占用（然而实际上没有使用SD卡）
  * @param  无
  * @retval 无
  */
void My_USART3_Init(void)
{
    GPIO_InitTypeDef	GPIO_InitStruct;
    USART_InitTypeDef	USART_InitStruct;
    NVIC_InitTypeDef 	NVIC_InitStruct;
    DMA_InitTypeDef		DMA_InitStruct;

    /****************************打开相关时钟****************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /****************************USART3的GPIO配置****************************/
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_OType	=	GPIO_OType_PP;//PP
    GPIO_InitStruct.GPIO_PuPd	=	GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Mode	=	GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin	  =	GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Speed	=	GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3); //TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); //RX

    /****************************USART3配置****************************/
    USART_DeInit(USART3);
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate			= 460800;	//此处使用高波特率，1ms内发送字节数控制在40字节内
    USART_InitStruct.USART_WordLength	    = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits			= USART_StopBits_1;
    USART_InitStruct.USART_Parity		    = USART_Parity_No;
    USART_InitStruct.USART_Mode			    = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStruct);

    /****************************USART3的中断配置****************************/
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);         //串口3接收空闲中断

    NVIC_InitStruct.NVIC_IRQChannel				        = USART3_IRQn; //串口3接收空闲中断
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority   = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority		    = 10;
    NVIC_InitStruct.NVIC_IRQChannelCmd				    = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /****************************USART3_Tx的DMA配置****************************/
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    DMA_DeInit(USART3_TX_STREAM);
    while( DMA_GetCmdStatus(USART3_TX_STREAM) == ENABLE );			//等待DMA可配置
    DMA_StructInit(&DMA_InitStruct);

    DMA_InitStruct.DMA_Channel			    =	DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr	=	(uint32_t)&(USART3->DR);
    DMA_InitStruct.DMA_Memory0BaseAddr	  	=	NULL;//暂无
    DMA_InitStruct.DMA_DIR				    =	DMA_DIR_MemoryToPeripheral;	//内存到外设
    DMA_InitStruct.DMA_BufferSize	        =	NULL;//暂无
    DMA_InitStruct.DMA_PeripheralInc	    =	DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc		    =	DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize	  	=	DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode			        =	DMA_Mode_Normal;			//正常发送
    DMA_InitStruct.DMA_Priority		      	=	DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold	    =	DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_MemoryBurst		    =	DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst	    =	DMA_PeripheralBurst_Single;
    DMA_Init(USART3_TX_STREAM, &DMA_InitStruct);
    DMA_Cmd(USART3_TX_STREAM, DISABLE);

    /****************************USART3_Rx的DMA配置****************************/
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    DMA_DeInit(USART3_RX_STREAM);
    while( DMA_GetCmdStatus(USART3_RX_STREAM) == ENABLE );//等待DMA可配置
    DMA_StructInit(&DMA_InitStruct);

    DMA_InitStruct.DMA_Channel              = 	DMA_Channel_4;			  	//即DMA_Channel_4
    DMA_InitStruct.DMA_PeripheralBaseAddr   = 	(uint32_t)&(USART3->DR);
    DMA_InitStruct.DMA_Memory0BaseAddr      = 	(uint32_t)UA3RxDMAbuf;
    DMA_InitStruct.DMA_DIR                  = 	DMA_DIR_PeripheralToMemory;	//外设到内存
    DMA_InitStruct.DMA_BufferSize           = 	UA3RxDMAbuf_LEN;
    DMA_InitStruct.DMA_PeripheralInc        =	DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc		    =	DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize	  	=	DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode                 = 	DMA_Mode_Circular;			//循环接收
    DMA_InitStruct.DMA_Priority             = 	DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold	    =	DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_MemoryBurst		    =	DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst	    =	DMA_PeripheralBurst_Single;
    DMA_Init(USART3_RX_STREAM, &DMA_InitStruct);
    DMA_Cmd(USART3_RX_STREAM, ENABLE);

    USART_Cmd(USART3, ENABLE);                             //使能串口
}

/**
  * @brief  UART4的初始化函数
  * @note   串口4，波特率2000000,用来与底盘云控通信
  * @param  无
  * @retval 无
  */
void My_UART4_Init(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1,ENABLE);	//使能PC端口时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);						//使能UART4时钟


    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);//Tx
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);//Rx


    gpio.GPIO_OType = GPIO_OType_PP;//PP
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOC, &gpio);

    nvic.NVIC_IRQChannel 					= UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority	= 0;		//抢占优先级
    nvic.NVIC_IRQChannelSubPriority			= 6;		//子优先级无效
    nvic.NVIC_IRQChannelCmd					= ENABLE;	//IRQ通道使能
    NVIC_Init(&nvic);//根据指定的参数初始化NVIC寄存器

    usart.USART_BaudRate			= 2000000;				//波特率
    usart.USART_WordLength		= USART_WordLength_8b;	//字长为8位数据格式
    usart.USART_StopBits			= USART_StopBits_1;		//一个停止位
    usart.USART_Parity				= USART_Parity_No;		//无奇偶校验位
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
    usart.USART_Mode			   	= USART_Mode_Tx|USART_Mode_Rx;		//收发模式
    USART_Init(UART4, &usart);//初始化串口

    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);	//开启空闲中断
    USART_Cmd(UART4, ENABLE);	//使能串口
		
    //UART4_Tx
    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
    DMA_DeInit(UART4_TX_STREAM);
    while( DMA_GetCmdStatus(UART4_TX_STREAM) == ENABLE );		  	//等待DMA可配置

    dma.DMA_Channel            =    DMA_Channel_4;              //外设地址
    dma.DMA_PeripheralBaseAddr =    (uint32_t)&(UART4->DR);
    dma.DMA_Memory0BaseAddr    =    NULL;
    dma.DMA_DIR                =    DMA_DIR_MemoryToPeripheral; //DMA传输为单向
    dma.DMA_BufferSize         =    UA4TxDMAbuf_LEN;                       //设置DMA在传输区的长度
    dma.DMA_PeripheralInc      =    DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          =    DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize =    DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     =    DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               =    DMA_Mode_Normal;
    dma.DMA_Priority           =    DMA_Priority_High;
    dma.DMA_FIFOMode           =    DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold      =    DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst        =    DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst    =    DMA_PeripheralBurst_Single;
    DMA_Init(UART4_TX_STREAM, &dma);
    DMA_Cmd(UART4_TX_STREAM, DISABLE);

    //UART4_Rx
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
    DMA_DeInit(UART4_RX_STREAM);
    while( DMA_GetCmdStatus(UART4_RX_STREAM) == ENABLE ); //等待DMA可配置

    dma.DMA_Channel            =  DMA_Channel_4;			  	//即DMA_Channel_5
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(UART4->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA4RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//外设到内存
    dma.DMA_BufferSize         = 	UA4RxDMAbuf_LEN;
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//循环接收
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(UART4_RX_STREAM, &dma);
    DMA_Cmd(UART4_RX_STREAM, ENABLE);	//使能DMA
}

/**
  * @brief  UART5的初始化函数
  * @note   串口5，波特率115200,用于裁判系统通讯
  * @param  无
  * @retval 无
  */
void My_UART5_Init(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); 	//TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5); 	//RX

    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd 	= GPIO_PuPd_UP;
    gpio.GPIO_Mode 	= GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin 	= GPIO_Pin_12; 	//TX
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin 	= GPIO_Pin_2;
    GPIO_Init(GPIOD, &gpio); 	    //RX

    USART_DeInit(UART5);
    usart.USART_BaudRate        = 115200;//裁判系统
    usart.USART_WordLength		= USART_WordLength_8b;
    usart.USART_StopBits	    = USART_StopBits_1;
    usart.USART_Parity		    = USART_Parity_No;
    usart.USART_Mode		    = USART_Mode_Rx | USART_Mode_Tx;//收发模式
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART5, &usart);

    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);   //开启空闲中断
    USART_Cmd(UART5, ENABLE);                       //使能串口
    USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);   //使能串口发送DMA
    USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);   //使能串口接收DMA

    nvic.NVIC_IRQChannel				    = UART5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority		    = 6;
    nvic.NVIC_IRQChannelCmd				    = ENABLE;
    NVIC_Init(&nvic);

    //UART5_Tx
    DMA_DeInit(UART5_TX_STREAM);
    while( DMA_GetCmdStatus(UART5_TX_STREAM) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel			      	=  DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr	=	(uint32_t)&(UART5->DR);
    dma.DMA_Memory0BaseAddr		  =	(uint32_t)UA5TxDMAbuf;
    dma.DMA_DIR				        	=	DMA_DIR_MemoryToPeripheral;	//内存到外设
    dma.DMA_BufferSize		    	=	UA4TxDMAbuf_LEN;
    dma.DMA_PeripheralInc	    	=	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc		      	=	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	  	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode			         	=	DMA_Mode_Normal;			//正常发送
    dma.DMA_Priority			      =	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		       	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst		    	=	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst	   	=	DMA_PeripheralBurst_Single;
    DMA_Init(UART5_TX_STREAM, &dma);
    DMA_Cmd(UART5_TX_STREAM, DISABLE);	//失能DMA

    //UART5_Rx
    DMA_DeInit(UART5_RX_STREAM);
    while( DMA_GetCmdStatus(UART5_RX_STREAM) == ENABLE );			//等待DMA可配置

    dma.DMA_Channel            =   DMA_Channel_4;				//即DMA_Channel_5
    dma.DMA_PeripheralBaseAddr = 	(uint32_t)&(UART5->DR);
    dma.DMA_Memory0BaseAddr    = 	(uint32_t)UA5RxDMAbuf;
    dma.DMA_DIR                = 	DMA_DIR_PeripheralToMemory;	//外设到内存
    dma.DMA_BufferSize         = 	UA5RxDMAbuf_LEN;
    dma.DMA_Mode               = 	DMA_Mode_Circular;			//循环接收
    dma.DMA_Priority           = 	DMA_Priority_VeryHigh;
    DMA_Init(UART5_RX_STREAM, &dma);
    DMA_Cmd(UART5_RX_STREAM, ENABLE);	//使能DMA
}

/**
  * @brief  USART6的初始化函数
  * @note   串口6，波特率921600,用于视觉通讯（即小电脑）
  * @param  无
  * @retval 无
  */
void My_USART6_Init(void)
{
    GPIO_InitTypeDef	gpio;
    USART_InitTypeDef	usart;
    NVIC_InitTypeDef 	nvic;
    DMA_InitTypeDef		dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); //TX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); //RX

    gpio.GPIO_OType = GPIO_OType_PP;//PP
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &gpio);

    USART_DeInit(USART6);
    usart.USART_BaudRate    =   921600;//视觉联调
    usart.USART_WordLength  =	USART_WordLength_8b;
    usart.USART_StopBits    =	USART_StopBits_1;
    usart.USART_Parity      =	USART_Parity_No;
    usart.USART_Mode	    =	USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
    USART_Init(USART6, &usart);

    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);	//开启空闲中断
    USART_Cmd(USART6, ENABLE);                      //使能串口
    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);  //使能串口发送DMA
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);  //使能串口接收DMA

    nvic.NVIC_IRQChannel				    = USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority  = 0;
    nvic.NVIC_IRQChannelSubPriority		    = 10;
    nvic.NVIC_IRQChannelCmd				    = ENABLE;
    NVIC_Init(&nvic);

    //USART6_Tx
    DMA_DeInit(USART6_TX_STREAM);
    while( DMA_GetCmdStatus(USART6_TX_STREAM) == ENABLE );	  //等待DMA可配置

    dma.DMA_Channel			      	= DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr	=	(uint32_t)&(USART6->DR);
    dma.DMA_Memory0BaseAddr	  	=	NULL;                       //暂无
    dma.DMA_DIR					        =	DMA_DIR_MemoryToPeripheral;	//内存到外设
    dma.DMA_BufferSize		     	=	NULL;                       //暂无
    dma.DMA_PeripheralInc	    	=	DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc			      =	DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize	=	DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize	  	=	DMA_MemoryDataSize_Byte;
    dma.DMA_Mode			        	=	DMA_Mode_Normal;			      //正常发送
    dma.DMA_Priority		      	=	DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode		      	=	DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold	    	=	DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst			    =	DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst		  =	DMA_PeripheralBurst_Single;
    DMA_Init(USART6_TX_STREAM, &dma);
    DMA_Cmd(USART6_TX_STREAM, DISABLE);	//失能DMA

    //USART6_Rx
    DMA_DeInit(USART6_RX_STREAM);
    while( DMA_GetCmdStatus(USART6_RX_STREAM) == ENABLE );	//等待DMA可配置

    dma.DMA_Channel            = DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
    dma.DMA_Memory0BaseAddr    = (uint32_t)UA6RxDMAbuf;
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;//外设到内存
    dma.DMA_BufferSize         = UA6RxDMAbuf_LEN;
    dma.DMA_Mode               = DMA_Mode_Circular;		      //循环接收
    dma.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_Init(USART6_RX_STREAM, &dma);
    DMA_Cmd(USART6_RX_STREAM, ENABLE);	//使能DMA
}

/**
  * @brief  重定义fputc函数，使其输出到串口3
  * @note   有了这个函数之后，可以直接使用printf()，
  *         输出的结果会通过串口3发送
  */
int fputc(int ch, FILE *f)
{
    while(USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);//等待传输完成(发送完成标志位)
    USART_SendData(USART3,ch);
    return ch;
}
