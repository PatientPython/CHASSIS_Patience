/**
  ******************************************************************************
  * @file    General_AuxiliaryFunc.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.22
  * @brief   通用的辅助功能函数
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include <stdbool.h>
#include <arm_math.h>
#include "GlobalDeclare_General.h"


// #pragma region /**** 限幅函数 ***************************/
/**
  * @brief  限幅函数Limit
  * @note   把值限制在Min~Max之间
  * @param  RawData：float类型的数，原始数据
  * @param  Min：float类型的数，最小值
  * @param  Max：float类型的数，最大值
  * @retval float类型的数，限幅之后的值
*/
float Limit(float RawData, float Min, float Max)
{
    if      (RawData < Min)     return Min;
    else if (RawData > Max)     return Max;
    else                        return RawData;
}
// #pragma endregion

// #pragma region /**** 遥控器是否连接、手势判断相关函数 ***************************/
/**
  * @brief  判断遥控器是否连接的函数
  * @param  无
  * @retval true：遥控器已连接 false：遥控器未连接
*/
bool IsRCConnected(void)
{
    /*串口1帧率过低，遥控器未连接*/
    if(GST_SystemMonitor.USART1Rx_fps < USART1Rx_fpsMinTH)
    {return false;}

    /*遥控器通道值过大/过小，认为遥控器未连接*/
    if(GST_Receiver.ST_RC.JoyStickL_X < RCChannelValue_Min || GST_Receiver.ST_RC.JoyStickL_X > RCChannelValue_Max)
    {return false;}
    if(GST_Receiver.ST_RC.JoyStickL_Y < RCChannelValue_Min || GST_Receiver.ST_RC.JoyStickL_Y > RCChannelValue_Max)
    {return false;}
    if(GST_Receiver.ST_RC.JoyStickR_X < RCChannelValue_Min || GST_Receiver.ST_RC.JoyStickR_X > RCChannelValue_Max)
    {return false;}
    if(GST_Receiver.ST_RC.JoyStickR_Y < RCChannelValue_Min || GST_Receiver.ST_RC.JoyStickR_Y > RCChannelValue_Max)
    {return false;}

    /*遥控器拨盘值异常，认为遥控器未连接*/
    if(GST_Receiver.ST_RC.Roller < RCChannelValue_Min || GST_Receiver.ST_RC.Roller > RCChannelValue_Max)
    {return false;}

    /*以上条件均通过，遥控器拨杆值正常，认为遥控器连接*/
    if(   GST_Receiver.ST_RC.Level_L == RCLevel_Up
       || GST_Receiver.ST_RC.Level_L == RCLevel_Mid
       || GST_Receiver.ST_RC.Level_L == RCLevel_Down)
    {return true;}
    
    if(   GST_Receiver.ST_RC.Level_R == RCLevel_Up
       || GST_Receiver.ST_RC.Level_R == RCLevel_Mid
       || GST_Receiver.ST_RC.Level_R == RCLevel_Down)
    {return true;}

    /*默认遥控器未连接*/
    return false;
}

/****拨杆相关*****/
/**
  * @brief  判断遥控器是否左拨杆在上的函数
  * @param  无
  * @retval true：左拨杆在上 false：左拨杆不在上
*/
bool IsLeftLevelUp(void)
{
    if(GST_Receiver.ST_RC.Level_L == RCLevel_Up)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否左拨杆在中的函数
  * @param  无
  * @retval true：左拨杆在中 false：左拨杆不在中
*/
bool IsLeftLevelMid(void)
{
    if(GST_Receiver.ST_RC.Level_L == RCLevel_Mid)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否左拨杆在下的函数
  * @param  无
  * @retval true：左拨杆在下 false：左拨杆不在下
*/
bool IsLeftLevelDown(void)
{
    if(GST_Receiver.ST_RC.Level_L == RCLevel_Down)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否右拨杆在上的函数
  * @param  无
  * @retval true：右拨杆在上 false：右拨杆不在上
*/
bool IsRightLevelUp(void)
{
    if(GST_Receiver.ST_RC.Level_R == RCLevel_Up)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否右拨杆在中的函数
  * @param  无
  * @retval true：右拨杆在中 false：右拨杆不在中
*/
bool IsRightLevelMid(void)
{
    if(GST_Receiver.ST_RC.Level_R == RCLevel_Mid)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否右拨杆在下的函数
  * @param  无
  * @retval true：右拨杆在下 false：右拨杆不在下
*/
bool IsRightLevelDown(void)
{
    if(GST_Receiver.ST_RC.Level_R == RCLevel_Down)
    {return true;}
    else
    {return false;}
}

/****摇杆相关*****/
/**
  * @brief  判断遥控器是否左摇杆在上的函数
  * @param  无
  * @retval true：左摇杆在上 false：左摇杆不在上
*/
bool IsLeftJoyStickUp(void)
{
    if(GST_Receiver.ST_RC.JoyStickL_Y >= RCJoyStick_UpTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否左摇杆在下的函数
  * @param  无
  * @retval true：左摇杆在下 false：左摇杆不在下
*/
bool IsLeftJoyStickDown(void)
{
    if(GST_Receiver.ST_RC.JoyStickL_Y <= RCJoyStick_DownTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否左摇杆在左的函数
  * @param  无
  * @retval true：左摇杆在左 false：左摇杆不在左
*/
bool IsLeftJoyStickLeft(void)
{
    if(GST_Receiver.ST_RC.JoyStickL_X <= RCJotStick_LeftTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否左摇杆在右的函数
  * @param  无
  * @retval true：左摇杆在右 false：左摇杆不在右
*/
bool IsLeftJoyStickRight(void)
{
    if(GST_Receiver.ST_RC.JoyStickL_X >= RCJoyStick_RightTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否右摇杆在上的函数
  * @param  无
  * @retval true：右摇杆在上 false：右摇杆不在上
*/
bool IsRightJoyStickUp(void)
{
    if(GST_Receiver.ST_RC.JoyStickR_Y >= RCJoyStick_UpTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否右摇杆在下的函数
  * @param  无
  * @retval true：右摇杆在下 false：右摇杆不在下
*/
bool IsRightJoyStickDown(void)
{
    if(GST_Receiver.ST_RC.JoyStickR_Y <= RCJoyStick_DownTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否右摇杆在左的函数
  * @param  无
  * @retval true：右摇杆在左 false：右摇杆不在左
*/
bool IsRightJoyStickLeft(void)
{
    if(GST_Receiver.ST_RC.JoyStickR_X <= RCJotStick_LeftTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器是否右摇杆在右的函数
  * @param  无
  * @retval true：右摇杆在右 false：右摇杆不在右
*/
bool IsRightJoyStickRight(void)
{
    if(GST_Receiver.ST_RC.JoyStickR_X >= RCJoyStick_RightTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断左摇杆是否向上超出死区
  * @param  无
  * @retval true：左摇杆向上超出死区 false：左摇杆未向上超出死区
*/
bool IsLeftJoyStickBeyondDeadZoneUp(void)
{
    if(GST_Receiver.ST_RC.JoyStickL_Y >= (RCChannelValue_Mid + RCChannel_DeadZone))
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断左摇杆是否向下超出死区
  * @param  无
  * @retval true：左摇杆向下超出死区 false：左摇杆未向下超出死区
*/
bool IsLeftJoyStickBeyondDeadZoneDown(void)
{
    if(GST_Receiver.ST_RC.JoyStickL_Y <= (RCChannelValue_Mid - RCChannel_DeadZone))
    {return true;}
    else
    {return false;}
}

/****拨轮相关****/
/**
  * @brief  判断遥控器拨轮是否在上的函数
  * @note   注意拨轮往上是数值变小
  * @param  无
  * @retval true：拨轮在上 false：拨轮不在上
*/
bool IsRollerUp(void)
{
    if(GST_Receiver.ST_RC.Roller <= RCRoller_UpTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器拨轮是否在下的函数
  * @note   注意拨轮往下是数值变大
  * @param  无
  * @retval true：拨轮在下 false：拨轮不在下
*/
bool IsRollerDown(void)
{
    if(GST_Receiver.ST_RC.Roller >= RCRoller_DownTH)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  判断遥控器拨轮是否自动回正的函数
  * @note   注意拨轮往下是数值变大
  * @param  无
  * @retval true：拨轮回正 false：拨轮未回正
*/
bool IsRollerMid(void) {
    if (GST_Receiver.ST_RC.Roller > RCRoller_UpTH && 
        GST_Receiver.ST_RC.Roller < RCRoller_DownTH) {
        return true;
    }
    return false;
}


/**
  * @brief  判断遥控器是否是内八手势的函数（左摇杆右下，右摇杆左下）
  * @note   （我很抱歉我没有找到更合适的“内八”翻译，只能写成InsideEight）
  * @param  无
  * @retval true：是内八手势 false：不是内八手势
*/
bool IsInsideEightGesture(void)
{
    /****************如果左摇杆不在右下，返回false****************/
    if(!(IsLeftJoyStickRight() && IsLeftJoyStickDown()))
    {return false;}

    /****************如果右摇杆不在左下，返回false****************/
    if(!(IsRightJoyStickLeft() && IsRightJoyStickDown()))
    {return false;}
    
    /****************两个条件都满足，内八手势，返回true****************/
    return true;
}
// #pragma endregion

// #pragma region /**** 步进改变数值函数 ***************************/

/**
  * @brief  步进改变数值的函数
  * @note   用于将一个数值按给定步长，逐步改变到目标值
  * @param  ValueNow：当前值
  * @param  ValueDes：目标值
  * @param  Step：改变的步长
  * @retval 根据给定步长变化后的下一个值
*/
float StepChangeValue(float ValueNow, float ValueDes, float Step)
{
    /*下一个值*/
    float ValueNext = ValueNow;

    /*如果当前值小于目标值*/
    if(ValueNow < ValueDes)
    {
        ValueNext += Step;          //下一个值增加给定步长
        if(ValueNext > ValueDes)    //如果下一个值超过目标值
        {ValueNext = ValueDes;}     //则下一个值等于目标值
    }

    /*如果当前值大于目标值*/
    else if(ValueNow > ValueDes)
    {
        ValueNext -= Step;          //下一个值减少给定步长
        if(ValueNext < ValueDes)    //如果下一个值超过目标值
        {ValueNext = ValueDes;}     //则下一个值等于目标值
    }

    return ValueNext;
}
// #pragma endregion



/*******遥控器通道转化为[-1, 1]的归一化数据********/

/**
 * @brief  将遥控器通道值转换为归一化值的函数
 * @param[in] raw           原始通道值（16位有符号整数）
 * @return float            归一化值（-1.0到1.0之间的浮点数）
 */
float RCChannelToNorm(int16_t raw)
{
    float diff = (float)raw - (float)RCChannelValue_Mid;
    if (fabsf(diff) < (float)RCChannel_DeadZone) {
        return 0.0f;
    }
    float maxDiff = (float)(RCChannelValue_Max - RCChannelValue_Mid);
    return Limit(diff / maxDiff, -1.0f, 1.0f);
}
