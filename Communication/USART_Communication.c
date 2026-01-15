/**
 ******************************************************************************
 * @file    USART_Communication.c
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.10.13
 * @brief   存放串口的通讯相关函数、缓冲区等等
 *          注意如果通讯在中断服务函数里的话，去IRQHandler里面找
 ******************************************************************************
 */
/****************************头文件包含****************************/
#include "USART_Communication.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "GlobalDeclare_Chassis.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Gimbal.h"
#include "arm_math.h"
#include "stdarg.h"
#include "stm32f4xx.h"

/****************************常量、宏定义定义（不需要修改）****************************/
/********串口发送/接收缓冲区长度********/
const uint8_t UA1RxDMAbuf_LEN =
    18;  // 串口1接收缓冲区大小。遥控接收机DR16每隔7ms通过DBus发送一帧数据（18字节）
const uint8_t UA2TxDMAbuf_LEN = 6;     // 云台云控IMU1控制板Tx
const uint8_t UA2RxDMAbuf_LEN = 38;    // 云台云控IMU1控制板Rx
const uint8_t UA3TxDMAbuf_LEN = 128;   // Debug用
const uint8_t UA3RxDMAbuf_LEN = 128;   // Debug用
const uint8_t UA4RxDMAbuf_LEN = 106;   // 底盘云控IMU2
const uint8_t UA4TxDMAbuf_LEN = 66;    // 底盘云控IMU2
const uint8_t UA5TxDMAbuf_LEN = 128;   // 裁判系统
const uint16_t UA5RxDMAbuf_LEN = 500;  // 裁判系统
const uint8_t UA6TxDMAbuf_LEN = 74;    // 视觉（小电脑）
const uint8_t UA6RxDMAbuf_LEN = 42;    // 视觉（小电脑）

/********CRC校验相关********/
const uint16_t CRC16_InitValue = 0xffff;  //.c文件私有，不extern

/****************************结构体、数组定义（不需要修改）****************************/
/********串口缓冲区数组定义********/
uint8_t UA1RxDMAbuf[UA1RxDMAbuf_LEN] = {0};  // 串口1接收缓冲区
uint8_t UA2RxDMAbuf[UA2RxDMAbuf_LEN] = {0};  // 串口2接收缓冲区
uint8_t UA3RxDMAbuf[UA3RxDMAbuf_LEN] = {0};  // 串口3接收缓冲区
uint8_t UA4RxDMAbuf[UA4RxDMAbuf_LEN] = {0};  // 串口4接收缓冲区
uint8_t UA5TxDMAbuf[UA5TxDMAbuf_LEN] = {0};  // 串口5发送变量
uint8_t UA5RxDMAbuf[UA5RxDMAbuf_LEN] = {0};  // 串口5接收变量
uint8_t UA6RxDMAbuf[UA6RxDMAbuf_LEN] = {0};  // 串口6接收变量
char USART3_DMA_Buf[255] = {0};              // 串口3发送缓冲区
char USART6_DMA_Buf[255] = {0};              // 串口6发送缓冲区

/********CRC校验相关********/
const uint16_t CRC16_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48,
    0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108,
    0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb,
    0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e,
    0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e,
    0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd,
    0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285,
    0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44,
    0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014,
    0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3,
    0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862,
    0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e,
    0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1,
    0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483,
    0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50,
    0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7,
    0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1,
    0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72,
    0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e,
    0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf,
    0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d,
    0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/****************************函数****************************/
// // #pragma region /********CRC校验相关函数********/
/**
 * @brief  CRC16校验中计算输入数据的CRC8校验字
 * @note   CRC16校验的辅助函数
 * @param  pchMessage：uint8_t类型的指针，一般传入串口缓冲区数组
 * @param  dwLength：传入数组的总长度-2
 * @param  wCRC：算法中的一个计算用的数字
 * @retval 一个uint16_t的数据
 */
uint16_t _CRC16_GetCheckSum(uint8_t* pchMessage, uint32_t dwLength,
                            uint16_t wCRC) {
    if (pchMessage == 0) {
        return 0xFFFF;
    }

    uint8_t chData;
    while (dwLength--) {
        chData = *pchMessage++;
        wCRC = ((uint16_t)(wCRC) >> 8) ^
               CRC16_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/**
 * @brief  CRC16校验函数
 * @note   在串口数据通讯时进行CRC16校验，确保数据的准确
 * @param  pchMessage：uint8_t类型的指针，一般传入串口缓冲区数组
 * @param  dwLength：传入数组的总长度
 * @retval 0：错误  1：正确
 */
uint32_t _CRC16_Verify(uint8_t* pchMessage, uint32_t dwLength) {
    if ((pchMessage == 0) || (dwLength <= 2)) {
        return 0;
    }

    uint16_t wExpected = 0;
    wExpected = _CRC16_GetCheckSum(pchMessage, dwLength - 2, CRC16_InitValue);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] &&
            ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**
 * @brief  添加CRC16校验字段的函数
 * @note   在输入的数组尾添加CRC16检验字段，用来串口数据传输间的数据检验
 * @param  pchMessage：uint8_t类型的指针，一般传入串口缓冲区数组
 * @param  dwLength：传入数组的总长度 = Data长度 + CRC校验长度
 * @retval 无
 */
void _CRC16_Append(uint8_t* pchMessage, uint32_t dwLength) {
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) {
        return;
    }

    wCRC =
        _CRC16_GetCheckSum((uint8_t*)pchMessage, dwLength - 2, CRC16_InitValue);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
// // #pragma endregion

// // #pragma region /********串口通讯相关函数********/
/**
 * @brief  串口6调用DMA发送数据函数
 * @note   该函数暂时没有用到，作为备用函数
 * @param  与printf函数相同
 * @retval 无
 */
void USART6_DMA_printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    uint8_t USART6_DMA_Len = (uint8_t)vsprintf(
        USART6_DMA_Buf, fmt,
        ap);  // 将格式化数据打到USART6_DMA_Buf字符串上，返回格式化长度

    while (DMA_GetCurrDataCounter(USART6_TX_STREAM));  // 等之前的发完
    DMA_ClearITPendingBit(
        USART6_TX_STREAM,
        DMA_IT_TCIF6);  // 开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

    DMA_Cmd(USART6_TX_STREAM, DISABLE);  // 设置当前计数值前先禁用DMA
    USART6_TX_STREAM->M0AR =
        (uint32_t)USART6_DMA_Buf;  // 设置当前待发数据基地址:Memory0Target
    USART6_TX_STREAM->NDTR =
        (uint32_t)USART6_DMA_Len;  // 设置当前待发的数据的数量:Number of Data
                                   // units to be Transferred
    DMA_Cmd(USART6_TX_STREAM, ENABLE);  // 启用串口DMA发送
}

/**
 * @brief  串口1接收数据的解析，遥控器接收机>>主控
 * @param  无
 * @retval 无
 */
void UA1Rx_ReceiverDataProcess(void) {
    /**************************************************遥控器数据解码**************************************************/
    /*右摇杆*/
    GST_Receiver.ST_RC.JoyStickR_X =
        (UA1RxDMAbuf[0] | (UA1RxDMAbuf[1] << 8)) & 0x07ff;  // Channe0——水平通道
    GST_Receiver.ST_RC.JoyStickR_Y =
        ((UA1RxDMAbuf[1] >> 3) | (UA1RxDMAbuf[2] << 5)) &
        0x07ff;  // Channe1——垂直通道
    /*左摇杆*/
    GST_Receiver.ST_RC.JoyStickL_X =
        ((UA1RxDMAbuf[2] >> 6) | (UA1RxDMAbuf[3] << 2) |
         (UA1RxDMAbuf[4] << 10)) &
        0x07ff;  // Channe2——水平通道
    GST_Receiver.ST_RC.JoyStickL_Y =
        ((UA1RxDMAbuf[4] >> 1) | (UA1RxDMAbuf[5] << 7)) &
        0x07ff;  // Channe3——垂直通道
    /*拨盘*/
    GST_Receiver.ST_RC.Roller =
        (UA1RxDMAbuf[16] | UA1RxDMAbuf[17] << 8) & 0x07ff;  // Channe4——垂直通道
    /*左3位拨杆*/
    GST_Receiver.ST_RC.Level_L =
        ((UA1RxDMAbuf[5] >> 4) & 0x000C) >> 2;  //(上——1；中——3；下——2)
    /*右3位拨杆*/
    GST_Receiver.ST_RC.Level_R =
        ((UA1RxDMAbuf[5] >> 4) & 0x0003);  //(上——1；中——3；下——2)

    /**************************************************鼠标数据解码***************************************************/
    GST_Receiver.ST_Mouse.X = UA1RxDMAbuf[6] | (UA1RxDMAbuf[7] << 8);  // X坐标
    GST_Receiver.ST_Mouse.Y = UA1RxDMAbuf[8] | (UA1RxDMAbuf[9] << 8);  // Y坐标
    GST_Receiver.ST_Mouse.Z =
        UA1RxDMAbuf[10] | (UA1RxDMAbuf[11] << 8);  // Z坐标
    GST_Receiver.ST_Mouse.Left =
        UA1RxDMAbuf[12];  // 左键状态（1：按下；0：没按下）
    GST_Receiver.ST_Mouse.Right =
        UA1RxDMAbuf[13];  // 右键状态（1：按下；0：没按下）

    /**************************************************键盘数据解码***************************************************/
    GST_Receiver.usKeyboard =
        UA1RxDMAbuf[14] | (UA1RxDMAbuf[15] << 8);  // 键盘值
}

/**
 * @brief  串口2的接收校验函数，是一个简单的辅助函数，用__开头
 * @note   先判断帧头，然后CRC校验
 * @param  无
 * @retval false：错误  true：正确
 */
bool __UA2Rx_IsVerifySuccess(void) {
    /*****************帧头校验，失败返回false*****************/
    if (UA2RxDMAbuf[0] != 0x55 || UA2RxDMAbuf[1] != 0x00) {
        return false;
    }

    /*****************CRC校验，失败返回false*****************/
    if (_CRC16_Verify(UA2RxDMAbuf, UA2RxDMAbuf_LEN) != 1) {
        return false;
    }

    /*****************两者都通过了，返回true*****************/
    return true;
}

/**
 * @brief  串口2接收数据的解析，IMU1云台云控>>主控
 * @param  无
 * @retval 无
 */
// 待优化：这里的YawSpeed那个计算，原理应该就是Pitch有角度的时候，YawSpeed其实是要除上cos（PitchAngle）才是真实的YawSpeed，注意检查一下云控里面有没有已经做过处理了
// 待优化：现在的变量是直接传给PID结构体，但我的设想是PID结构体都属于辅助结构体
//        后面要再设置一个正式结构体来存储这些变量，然后在控制任务里面把正式结构体的数据传给PID结构体计算
void UA2Rx_IMU1DataProcess(void) {
    if (__UA2Rx_IsVerifySuccess())  // 数据校验，包括帧头和CRC
    {
        /**************** 转移串口接收缓冲区的数据到GST_IMU1.ST_Rx中
         * ****************/
        memcpy(&GstGB_IMU1.ST_Rx, UA2RxDMAbuf, sizeof(GstGB_IMU1.ST_Rx));

        /**************** 把每个数据分配到对应的变量中去 ****************/
        // 待优化：把这里的数据分配放到云台任务里的DataUpdate函数里去做
        float TempCoe = arm_cos_f32(GstGB_IMU1.ST_Rx.PitchAngle *
                                    A2R);  // 处理YawSpeed时需要除的一个值

        // 待修改：下面这些变量的赋值，后面要改成赋值给正式结构体，而不是直接赋值给PID结构体
        GstGB_PitchPosPID.FB = (float)GstGB_IMU1.ST_Rx.PitchAngle;
        GstGB_YawPosPID.FB = (float)GstGB_IMU1.ST_Rx.YawAngle;
        GstGB_PitchSpeedPID.FB = (float)GstGB_IMU1.ST_Rx.PitchAngleVel;
        GstGB_YawSpeedPID.FB = (float)(GstGB_IMU1.ST_Rx.YawSpeed / TempCoe);
        GGB_RollAngle = (float)GstGB_IMU1.ST_Rx.RollAngle;
    }
}

/**
 * @brief  串口2发送数据，主控>>IMU1云台云控
 * @note   发送的有效数据为：
 *             云台云控是否重启标志位
 *             补弹标志位（已经失效）
 * @param  无
 * @retval 无
 */
void UA2Tx_SendDataToIMU1(void) {
    /****************发送数据结构体赋值、添加CRC校验字****************/
    GstGB_IMU1.ST_Tx.head[0] = 0x55;                       // 帧头1
    GstGB_IMU1.ST_Tx.head[1] = 0x00;                       // 帧头2
    GstGB_IMU1.ST_Tx.ReStart = (uint8_t)GFGB_IMU1Restart;  // 云台云控重启标志位
    GstGB_IMU1.ST_Tx.ReloadStatus =
        (uint8_t)(false);  // 原本是之前赛季的补弹标志位，由于规则改动，无需补单功能，直接发false就行

    _CRC16_Append((GstGB_IMU1.ST_Tx.head), UA2TxDMAbuf_LEN);

    /****************清除中断标志位、配置DMA开始数据传输****************/
    DMA_ClearITPendingBit(
        USART2_TX_STREAM,
        DMA_IT_TCIF6);  // 开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

    DMA_Cmd(USART2_TX_STREAM, DISABLE);  // 设置当前计数值前先禁用DMA
    USART2_TX_STREAM->M0AR =
        (uint32_t)&GstGB_IMU1.ST_Tx;  // 设置当前待发数据基地址:Memory0 tARget
    USART2_TX_STREAM->NDTR =
        (uint32_t)UA2TxDMAbuf_LEN;  // 设置当前待发的数据的数量:Number of Data
                                    // units to be TRansferred
    DMA_Cmd(USART2_TX_STREAM, ENABLE);
}

/**
 * @brief  串口3调用DMA发送数据函数
 * @note   该函数用于DebugTask的时候打印相关数据
 * @param  与printf函数相同，但一般调试的时候都发%.3f的数据
 * @retval 无
 */
void USART3_DMA_printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    uint8_t USART3_DMA_Len = (uint8_t)vsprintf(
        USART3_DMA_Buf, fmt, ap);  // 将格式化数据打到sz字符串上，返回格式化长度

    DMA_ClearITPendingBit(
        USART3_TX_STREAM,
        DMA_IT_TCIF3);  // 开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

    DMA_Cmd(USART3_TX_STREAM, DISABLE);  // 设置当前计数值前先禁用DMA
    USART3_TX_STREAM->M0AR =
        (uint32_t)USART3_DMA_Buf;  // 设置当前待发数据基地址:Memory0Target
    USART3_TX_STREAM->NDTR =
        (uint32_t)USART3_DMA_Len;  // 设置当前待发的数据的数量:Number of Data
                                   // units to be Transferred
    DMA_Cmd(USART3_TX_STREAM, ENABLE);  // 启用串口DMA发送
}

/**
 * @brief  串口4的接收校验函数，是一个简单的辅助函数，用__开头
 * @note   无CRC校验，直接判断帧头
 * @param  无
 * @retval false：错误  true：正确
 */
bool __UA4Rx_IsVerifySuccess(void) {
    /*****************先进行帧头校验，再CRC校验*****************/
    if (UA4RxDMAbuf[0] == 0x55 && UA4RxDMAbuf[1] == 0x00)  // 帧头校验
    {
        if (_CRC16_Verify(UA4RxDMAbuf, UA4RxDMAbuf_LEN) == 1) {
            return true;
        }
    }

    /*****************校验没通过，返回错误*****************/
    return false;
}

/**
 * @brief  串口4接收数据，IMU2底盘云控>>主控
 * @note   接收可以接收底盘云控的数据（主要是关节电机的反馈数据）并解析
 * @param  无
 * @retval 无
 */
void UA4Rx_IMU2DataProcess(void) {
    if (__UA4Rx_IsVerifySuccess())  // 数据校验是否通过，不通过则说明数据有误，直接丢弃
    {
        /*将串口4缓冲区的数据转移到GstCH_IMU2.ST_Rx结构体中*/
        memcpy(&GstCH_IMU2.ST_Rx, UA4RxDMAbuf, UA4RxDMAbuf_LEN);
    }
}

/**
 * @brief  串口4发送数据，主控>>IMU2底盘云控
 * @note
 * 向云控2发送数据，包括关节电机的角度、扭矩、转速，是否重启云控，是否进行腿部校准等
 * @param  无
 * @retval 无
 */
void UA4Tx_SendDataToIMU2(void) {
    /****************发送数据结构体赋值、添加CRC校验字****************/
    GstCH_IMU2.ST_Tx.head[0] = 0x55;  // 帧头1
    GstCH_IMU2.ST_Tx.head[1] = 0x00;  // 帧头2
    GstCH_IMU2.ST_Tx.RestartFlag = (uint8_t)
        GFCH_IMU2Restart;  // 底盘云控重启标志位（待优化：但是经过测试IMU2好像不会重启，可能是IMU2那边没有设置好，后面可以看看改一下）
    GstCH_IMU2.ST_Tx.ReloadStatus =
        (uint8_t)(false);  // 待优化：原本是之前赛季的补弹标志位，由于规则改动，无需补弹功能，直接发false就行，后续可以删掉，记得IMU2里面也要一起删

    _CRC16_Append((GstCH_IMU2.ST_Tx.head), UA4TxDMAbuf_LEN);

    GstCH_IMU2.ST_Tx.JM2StatusDes =
        GSTCH_JM2
            .StatusDes;  // 关节电机状态指令，以JM2为准，其他电机状态指令相同
    GstCH_IMU2.ST_Tx.LegCalibrationFlag =
        GFCH_LegCalibration;  // 是否要进行腿部校准

    /*因为关节电机控制结构体里面的MITKp、MITKd都给的是0*/
    /*所以下面的AngleDes、AngleVelDes不会起作用，只有力矩控制会起作用*/
    // 换车时需修改
    GstCH_IMU2.ST_Tx.JM2_AngleDes = GSTCH_JM2.AngleDes;  // 关节电机2目标角度
    GstCH_IMU2.ST_Tx.JM2_AngleVelDes =
        GSTCH_JM2.AngleVelDes;                             // 关节电机2目标转速
    GstCH_IMU2.ST_Tx.JM2_TorqueDes = GSTCH_JM2.TorqueDes;  // 关节电机2目标扭矩

    GstCH_IMU2.ST_Tx.JM4_AngleDes = GSTCH_JM4.AngleDes;  // 关节电机4目标角度
    GstCH_IMU2.ST_Tx.JM4_AngleVelDes =
        GSTCH_JM4.AngleVelDes;                             // 关节电机4目标转速
    GstCH_IMU2.ST_Tx.JM4_TorqueDes = GSTCH_JM4.TorqueDes;  // 关节电机4目标扭矩

    GstCH_IMU2.ST_Tx.JM1_AngleDes = GSTCH_JM1.AngleDes;  // 关节电机1目标角度
    GstCH_IMU2.ST_Tx.JM1_AngleVelDes =
        GSTCH_JM1.AngleVelDes;                             // 关节电机1目标转速
    GstCH_IMU2.ST_Tx.JM1_TorqueDes = GSTCH_JM1.TorqueDes;  // 关节电机1目标扭矩

    GstCH_IMU2.ST_Tx.JM3_AngleDes = GSTCH_JM3.AngleDes;  // 关节电机3目标角度
    GstCH_IMU2.ST_Tx.JM3_AngleVelDes =
        GSTCH_JM3.AngleVelDes;                             // 关节电机3目标转速
    GstCH_IMU2.ST_Tx.JM3_TorqueDes = GSTCH_JM3.TorqueDes;  // 关节电机3目标扭矩

    GstCH_IMU2.ST_Tx.MITKp =
        GSTCH_JM1.MITKp;  // 关节电机的MITKp，以JM1为准，其他电机相同
    GstCH_IMU2.ST_Tx.MITKd =
        GSTCH_JM1.MITKd;  // 关节电机的MITKd，以JM1为准，其他电机相同

    /****************清除中断标志位、配置DMA开始数据传输****************/
    DMA_ClearITPendingBit(
        UART4_TX_STREAM,
        DMA_IT_TCIF4);  // 开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

    DMA_Cmd(UART4_TX_STREAM, DISABLE);  // 设置当前计数值前先禁用DMA
    UART4_TX_STREAM->M0AR =
        (uint32_t)&GstCH_IMU2.ST_Tx;  // 设置当前待发数据基地址:Memory0 Target
    UART4_TX_STREAM->NDTR =
        (uint32_t)UA4TxDMAbuf_LEN;  // 设置当前待发的数据的数量:Number of Data
                                    // units to be Transferred
    DMA_Cmd(UART4_TX_STREAM, ENABLE);  // 启用串口DMA发送
}

// // #pragma endregion
