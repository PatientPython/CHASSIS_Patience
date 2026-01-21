/**
  ******************************************************************************
  * @file    Algorithm_Simple.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.15
  * @brief   写各种简单的函数、算法
  ******************************************************************************
*/

#include <arm_math.h>
#include <math.h>
#include "GlobalDeclare_General.h"

/**
  * @brief  sin计算函数
  * @note   实际上是调用了math库的sin函数，重写一下看起来更简单顺眼
  * @param  AngleInDegree：float类型的数，角度制的角度值，单位度
  * @retval 计算结果，float类型的数，范围[-1, 1]
*/
float MySin(float AngleInDegree)
{
    return arm_sin_f32(AngleInDegree * A2R);    //使用arm_math库的sin函数，更快一点，但是可能没那么准
    // return sinf(AngleInDegree * A2R);        //使用math库的sin函数，更准一点，但是可能没那么快
}

/**
  * @brief  cos计算函数
  * @note   实际上是调用了math库的cos函数，重写一下看起来更简单顺眼
  * @param  AngleInDegree：float类型的数，角度制的角度值，单位度
  * @retval 计算结果，float类型的数，范围[-1, 1]
*/
float MyCos(float AngleInDegree)
{
    return arm_cos_f32(AngleInDegree * A2R);    //使用arm_math库的cos函数，更快一点，但是可能没那么准
    // return cosf(AngleInDegree * A2R);        //使用math库的cos函数，更准一点，但是可能没那么快
}

/**
  * @brief  平方根计算函数
  * @note   实际上是调用了arm_math库的函数，重写一下看起来更简单顺眼
  * @param  Value：float类型的数，用来计算的值
  * @retval 计算结果，float类型的数
*/
float MySqrt(float Value)
{
    float result;
    arm_sqrt_f32(Value, &result);
    return result;
}

/**
  * @brief  平方计算函数
  * @note   没有什么特殊的，只是封装了看起来更顺眼，注意和平方根区分
  * @param  Value：float类型的数，用来计算的值
  * @retval 计算结果，float类型的数
*/
float MySqr(float Value)
{
    return Value * Value;
}

/**
 * @brief 立方计算函数
 * @note   没有什么特殊的，只是封装了看起来更顺眼
 * @param  Value：float类型的数，用来计算的值
 * @retval 计算结果，float类型的数
 */
float MyCube(float Value)
{
    return Value * Value * Value;
}

/**
  * @brief  archtan计算函数
  * @note   实际上是调用了math库的函数，重写一下看起来更简单顺眼
  * @param  Y：float类型的数，y坐标值
  * @param  X：float类型的数，x坐标值
  * @retval 计算结果，float类型的数，范围[-pi/2, pi/2]
*/
float MyAtan(float Value)
{
    return atanf(Value);
}

/**
  * @brief  符号判断函数
  * @note   根据输入值的正负，返回1、-1或0
  * @param  Value：float类型的数，输入值
  * @retval 1：输入值大于0；-1：输入值小于0；0：输入值等于0
*/
float MySign(float Value)
{
    if(Value > 0.0f)
    {
        return 1.0f;
    }
    else if(Value < 0.0f)
    {
        return -1.0f;
    }
    else
    {
        return 0.0f;
    }
}

/**
  * @brief  float类型的绝对值函数
  * @note   返回输入值的绝对值
  * @param  Value：float类型的数，输入值
  * @retval 输入值的绝对值，float类型的数
*/
float MyAbsf(float Value)
{
    return fabsf(Value);
}

