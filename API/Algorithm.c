/**
  ******************************************************************************
  * @file    Algorithm.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.19
  * @brief   写各种偏复杂的算法，例如PID、LQR等等，以此方便外部调用
  *          简单的算法另外写在别的文件里面（文件暂时没想好名字）
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include "Algorithm.h"
#include "Algorithm_Simple.h"
#include "General_AuxiliaryFunc.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Chassis.h"
#include <arm_math.h>

/****************************结构体、数组定义（可能需要修改）****************************/
/*默认的LQR计算K矩阵*/
float LQR_DefaultK_Matrix[4][10] =
{
    {-4.9782,   -7.4064,   -2.7824,   -1.6201,  -22.4452,   -2.2603,   -8.4339,   -1.0402,   29.7838,    1.4493},
    {-4.9782,   -7.4064,    2.7824,    1.6201,   -8.4339,   -1.0402,  -22.4452,   -2.2603,   29.7838,    1.4493},
    { 4.7969,    7.0784,   -6.8980,   -4.0543,   48.4559,    4.1136,  -20.6028,   -1.1189,  274.6596,    7.8673},
    { 4.7969,    7.0784,    6.8980,    4.0543,  -20.6028,   -1.1189,   48.4559,    4.1136,  274.6596,    7.8673}
};


//#region PID相关函数全家桶
/**
  * @brief  PID的目标值设置函数
  * @note   用来设置PID结构体中的目标值Des
  * @param  PIDptr：PID_StructTypeDef类型的指针，要设置的PID结构体指针
  * @param  NewDes：要设置的目标值
  * @retval 无
*/
void PID_SetDes(PID_StructTypeDef* PIDptr, float NewDes)
{
    PIDptr->Des = NewDes;
}

/**
  * @brief  PID的反馈值设置函数
  * @note   用来设置PID结构体中的反馈值FB
  * @param  PIDptr：PID_StructTypeDef类型的指针，要设置的PID结构体指针
  * @param  NewFB：要设置的反馈值
  * @retval 无
*/
void PID_SetFB(PID_StructTypeDef* PIDptr, float NewFB)
{
    PIDptr->FB  = NewFB;
}

/**
  * @brief  PID的Kp、Ki、Kd设置函数
  * @note   用来设置PID结构体中的Kp、Ki、Kd参数
  * @param  PIDptr：PID_StructTypeDef类型的指针，要设置的PID结构体指针
  * @param  NewKp：要设置的Kp参数值
  * @param  NewKi：要设置的Ki参数值
  * @param  NewKd：要设置的Kd参数值
  * @retval 无
*/
void PID_SetKpKiKd(PID_StructTypeDef* PIDptr, float NewKp, float NewKi, float NewKd)
{
    PIDptr->Kp = NewKp;
    PIDptr->Ki = NewKi;
    PIDptr->Kd = NewKd;
}

/**
  * @brief  PID的计算函数
  * @note   最普通的PID基础上，增加了对SumE增幅的限幅，以及对PID输出值的限幅
  * @param  PIDptr：PID_StructTypeDef类型的指针，要计算的PID结构体指针
  * @retval 无
*/
void PID_Cal(PID_StructTypeDef *PIDptr)
{
    /**************************** 更新上次误差、误差计算、误差累计 ****************************/
    /* 更新上次误差 */
    PIDptr->PreErr = PIDptr->Err;

    /* 误差计算 */
    PIDptr->Err = PIDptr->Des - PIDptr->FB; //误差 = 目标 - 真实值

    /* SumErr的计算，即误差累计 */
    if(PIDptr->Ki == 0)
    {
        PIDptr->SumErr = 0; //如果Ki = 0，不累计误差，防止SumErr一直累加，造成数值过大
    }
    else
    {
        float AddValueTemp = Limit(PIDptr->Err, -PIDptr->AddMax, PIDptr->AddMax);
        PIDptr->SumErr = PIDptr->SumErr + AddValueTemp; //SumErr单次加的值不超过AddMax
    }

    /**************************** PID的计算、限幅 ****************************/
    /* PID单项的计算、限幅 */
    PIDptr->Up = PIDptr->Kp *  PIDptr->Err;
    PIDptr->Ui = PIDptr->Ki *  PIDptr->SumErr;
    PIDptr->Ud = PIDptr->Kd * (PIDptr->Err - PIDptr->PreErr);
    PIDptr->Up = Limit(PIDptr->Up, -PIDptr->UpMax, PIDptr->UpMax);
    PIDptr->Ui = Limit(PIDptr->Ui, -PIDptr->UiMax, PIDptr->UiMax);
    PIDptr->Ud = Limit(PIDptr->Ud, -PIDptr->UdMax, PIDptr->UdMax);

    /* PID总输出的计算、限幅 */
    PIDptr->U = PIDptr->Up + PIDptr->Ui + PIDptr->Ud;
    PIDptr->U = Limit(PIDptr->U, -PIDptr->UMax, PIDptr->UMax);
}

/**
  * @brief  获取PID的输出值函数
  * @note   用来获取PID结构体中的输出值U
  * @param  PIDptr：PID_StructTypeDef类型的指针，要获取的PID结构体指针
  * @retval PID的输出值U
*/
float PID_GetOutput(PID_StructTypeDef* PIDptr)
{
    return PIDptr->U;
}

/**
  * @brief  PID的重置函数
  * @note   用来重置PID结构体中的各项数据（不包括参数）
  * @param  PIDptr：PID_StructTypeDef类型的指针，要重置的PID结构体指针
  * @param  FBValue：真实值，用来重置FB和Dis
  * @retval 无
*/
void PID_Reset(PID_StructTypeDef* PIDptr, float FBValue)
{
    PIDptr->Des    = FBValue;
    PIDptr->FB     = FBValue;

    PIDptr->Err    = 0.0f;
    PIDptr->PreErr = 0.0f;
    PIDptr->SumErr = 0.0f;

    PIDptr->Up     = 0.0f;
    PIDptr->Ui     = 0.0f;
    PIDptr->Ud     = 0.0f;
    PIDptr->U      = 0.0f;
}
//#endregion

//#region TD相关函数全家桶
/**
  * @brief  TD的输入值设置函数
  * @note   用来设置TD结构体中的输入值v
  * @param  TDptr：TD_StructTypeDef类型的指针，要设置的TD结构体指针
  * @param  Input：要设置的输入原始值
  * @retval 无
*/
void TD_SetInput(TD_StructTypeDef* TDptr, float Input)
{
    TDptr->v = Input;
}

/**
  * @brief  TD的r参数设置函数
  * @note   用来设置TD结构体中的r参数值，即速度因子
  * @param  TDptr：TD_StructTypeDef类型的指针，要设置的TD结构体指针
  * @param  New_r：要设置的r参数值
  * @retval 无
*/
void TD_Setr(TD_StructTypeDef* TDptr, float New_r)
{
    TDptr->r = New_r;
}

/**
  * @brief  计算TD的fst函数
  * @note   TD算法中的最速控制综合函数，也是TD算法的核心
  * @param  TDptr：TD_StructTypeDef类型的指针，要计算的TD结构体指针
  * @retval fst参数值
*/
float _TD_fst(TD_StructTypeDef* TDptr)
{
    float d  = TDptr->r * TDptr->h0;                     //d  = r * h0
    float d0 = TDptr->h0 * d;                          //d0 = h0 * d = h0 * r * h0
    float y  = TDptr->x1 - TDptr->v + TDptr->h0 * TDptr->x2; //y  = (x1 - v) + h0 * x2
    float a0 = MySqrt(d*d + 8*TDptr->r*MyAbsf(y));     //a0 = sqrt(d*d + 8*r*|y|)
    float a;
    if(MyAbsf(y) > d0)
    {                                                //如果|y| > d0
        a = TDptr->x2 + MySign(y)*(a0 - d) / 2;        //a = x2 + sign(y)*(a0 - d)/2
    }
    else if(MyAbsf(y) <= d0)
    {                                                //如果|y| <= d0
        a = TDptr->x2 + y / TDptr->h0;                   //a = x2 + y/h0
    }

    float fst;
    if(MyAbsf(a) > d)
    {                                                //如果|a| > d
        fst = -TDptr->r * MySign(a);                   //fst = -r * sign(a)
    }
    else if(MyAbsf(a) <= d)
    {                                                //如果|a| <= d
        fst = -TDptr->r * a / d;                       //fst = -r * a/d
    }

    return fst;
}

/**
  * @brief  TD的计算函数
  * @note   ADRC自抗扰算法中的TD算法
  *         TD算法的介绍见ReadMe中的算法说明
  * @param  TDptr：TD_StructTypeDef类型的指针，要计算的TD结构体指针
  * @retval 无
*/
void TD_Cal(TD_StructTypeDef* TDptr)
{
    float fst = _TD_fst(TDptr); //计算fst参数

    /* TD状态更新 */
    TDptr->x2 = TDptr->x2 + fst * TDptr->SampleTime;      //x2(k+1) = x2(k) + fst * Ts
    TDptr->x1 = TDptr->x1 + TDptr->x2 * TDptr->SampleTime;  //x1(k+1) = x1(k) + x2 * Ts
}

/**
  * @brief  获取TD的输出值函数
  * @note   用来获取TD结构体中的输出值x1
  * @param  TDptr：TD_StructTypeDef类型的指针，要获取的TD结构体指针
  * @retval TD的输出值x1，即平滑跟踪值
*/
float TD_GetOutput(TD_StructTypeDef* TDptr)
{
    return TDptr->x1;
}

/**
  * @brief  TD的重置函数
  * @note   用来重置TD结构体中的各项数据（不包括参数）
  * @param  TDptr：TD_StructTypeDef类型的指针，要重置的TD结构体指针
  * @param  FBValue：反馈值，用来重置v、x1
  * @retval 无
*/
void TD_Reset(TD_StructTypeDef* TDptr, float FBValue)
{
    TDptr->v  = FBValue;
    TDptr->x1 = FBValue;
    TDptr->x2 = 0.0f;
}

//#endregion

//#region 五连杆解算相关函数全家桶
/**
  * @brief  腿部五连杆解算：phi1、phi4角度数据更新函数
  * @note   用来更新腿部五连杆解算结构体中的phi1、phi4，即腿部角度数据（具体定义见ReadMe中的算法说明）
  *         把关节电机的原始角度数据加上对应的坐标系零点，得到五连杆解算的phi1、phi4数据
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @param  JMAngleVel1：phi1对应的关节电机的原始角度，单位：度。取GSTCH_JMx.AngleFB
  * @param  JMAngleVel2：phi4对应的关节电机的原始角度，单位：度。取GSTCH_JMx.AngleFB
  * @retval 无
*/
void LegLinkage_AngleDataUpdate(LegLinkageCal_StructTypeDef* LegPtr, float JMAngle1, float JMAngle2)
{
    LegPtr->phi1 = JMAngle1 + LegPtr->phi1ZP;
    LegPtr->phi4 = JMAngle2 + LegPtr->phi4ZP;
}

/**
  * @brief  腿部五连杆解算：phi1、phi4角速度数据更新函数
  * @note   用来更新腿部五连杆解算结构体中的phi1_dot、phi4_dot，即腿部角速度数据（具体定义见ReadMe中的算法说明）
  *         直接把关节电机的角速度数据赋值给phi1_dot、phi4_dot
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @param  JMAngleVel1：phi1对应的关节电机的角速度值，单位：度/s。取GSTCH_JMx.AngleVelFB
  * @param  JMVelValue2：phi4对应的关节电机的角速度值，单位：度/s。取GSTCH_JMx.AngleVelFB
  * @retval 无
*/
void LegLinkage_AngleVelDataUpdate(LegLinkageCal_StructTypeDef* LegPtr, float JMAngleVel1, float JMVelValue2)
{
    LegPtr->phi1_dot = JMAngleVel1;
    LegPtr->phi4_dot = JMVelValue2;
}

/**
  * @brief  腿部五连杆解算：正向运动学计算函数
  * @note   根据关节电机的角度，计算腿部等效连杆的长度L0、角度phi0
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @retval 无
*/
void LegLinkage_ForwardKinematicsCal(LegLinkageCal_StructTypeDef* LegPtr)
{
    /******************************计算各点坐标*****************************/
    LegPtr->xA = 0;
    LegPtr->yA = 0;
    LegPtr->xE = LegPtr->l5;
    LegPtr->yE = 0;
    LegPtr->xB = LegPtr->xA + LegPtr->l1 * MyCos(LegPtr->phi1); //xB = xA + l1*cos(phi1)
    LegPtr->yB = LegPtr->yA + LegPtr->l1 * MySin(LegPtr->phi1); //yB = yA + l1*sin(phi1)
    LegPtr->xD = LegPtr->xE + LegPtr->l4 * MyCos(LegPtr->phi4); //xD = xE + l4*cos(phi4)
    LegPtr->yD = LegPtr->yE + LegPtr->l4 * MySin(LegPtr->phi4); //yD = yE + l4*sin(phi4)    

    /******************************phi2的计算*****************************/
    float BD = MySqrt(MySqr(LegPtr->xB - LegPtr->xD) + MySqr(LegPtr->yB - LegPtr->yD));//BD = sqrt((xB - xD)^2 + (yB - yD)^2)

    float A1 = 2 * LegPtr->l2 * (LegPtr->xD - LegPtr->xB);       //A1 = 2*l2*(xD - xB)
    float B1 = 2 * LegPtr->l2 * (LegPtr->yD - LegPtr->yB);       //B1 = 2*l2*(yD - yB)
    float C1 = MySqr(BD);                                        //C1 = BD^2

    float Temp1 = MySqrt(MySqr(A1) + MySqr(B1) - MySqr(C1));
    LegPtr->phi2 = 2*R2A* MyAtan((B1 + Temp1) / (A1 + C1));      //phi2 = 2*atan((B1 + sqrt(A1*A1 + B1*B1 - C1*C1)) / (A1 + C1))

    /******************************phi3的计算*****************************/
    float A2 = 2 * LegPtr->l3 * (LegPtr->xB - LegPtr->xD);       //A2 = 2*l3*(xB - xD)
    float B2 = 2 * LegPtr->l3 * (LegPtr->yB - LegPtr->yD);       //B2 = 2*l3*(yB - yD)
    float C2 = MySqr(BD);                                        //C2 = BD^2

    float Temp2 = MySqrt(MySqr(A2) + MySqr(B2) - MySqr(C2));
    LegPtr->phi3 = 2*R2A* MyAtan((B2 - Temp2) / (A2 + C2));      //phi3 = 2*atan((B2 - sqrt(A2*A2 + B2*B2 - C2*C2)) / (A2 + C2))

    /******************************C点直角坐标计算*****************************/
    LegPtr->xC = LegPtr->xB + LegPtr->l2 * MyCos(LegPtr->phi2); //xC = xB + l2*cos(phi2)
    LegPtr->yC = LegPtr->yB + LegPtr->l2 * MySin(LegPtr->phi2); //yC = yB + l2*sin(phi2)

    /******************************等效摆杆长度、角度计算*****************************/
    float Half_l5 = LegPtr->l5 / 2.0f;
    LegPtr->L0 = MySqrt(MySqr(LegPtr->xC - Half_l5) + MySqr(LegPtr->yC));   //L0 = sqrt((xC - l5/2)^2 + yC^2)
    LegPtr->phi0 = R2A * MyAtan((LegPtr->yC) / (LegPtr->xC - Half_l5));     //phi0 = atan(yC / (xC - l5/2))

	if(LegPtr->phi0 < 0) 
    {LegPtr->phi0 = 180 + LegPtr->phi0;} //保证phi0在[0, 180]范围内

    /******************************计算几个变量的微分，保存它们上次值*****************************/
    /*xC，注意这里的单位还是mm*/
    LegPtr->xC_dot = (LegPtr->xC - LegPtr->xC_Pre) / LegPtr->SampleTime; //xC_dot = (xC - xC_Pre) / SampleTime
    LegPtr->xC_Pre = LegPtr->xC;
}

/**
  * @brief  腿部五连杆解算：获取腿长的函数（不是等效连杆L0）
  * @note   注意：这里的腿长起始是yC，即关节电机到地面的竖直距离，而不是L0的长度
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @retval 腿的实际长度，单位：mm
*/
float LegLinkage_GetLegLength(LegLinkageCal_StructTypeDef* LegPtr)
{
    return LegPtr->yC;
}

/**
  * @brief  腿部五连杆解算：获取等效连杆L0长度的函数
  * @note   注意与腿长LegLength区别
  *         注意单位问题：返回值单位为mm
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @retval 等效连杆L0的长度，单位：mm
*/
float LegLinkage_GetL0Length(LegLinkageCal_StructTypeDef* LegPtr)
{
    return LegPtr->L0;
}

/**
  * @brief  腿部五连杆解算：获取等效连杆与竖直方向的夹角
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @param  LegSide：RobotSide_EnumTypeDef类型的枚举值，表示左腿还是右腿
  *             @arg LeftSide：左腿
  *             @arg RightSide：右腿
  * @retval 五连杆坐标系下，等效连杆与竖直方向的夹角（按照向后摆为正方向），单位：度
*/
float LegLinkage_GetTheta(LegLinkageCal_StructTypeDef* LegPtr, RobotSide_EnumTypeDef LegSide)
{
    float ThetaTemp = 0.0f;
    switch (LegSide)
    {
        case  LeftSide:
            ThetaTemp = -MyAtan((LegPtr->xC-LegPtr->l5/2.0f)/LegPtr->yC)*R2A;
            break;

        case  RightSide:
            ThetaTemp = MyAtan((LegPtr->xC-LegPtr->l5/2.0f)/LegPtr->yC)*R2A;
            break;
    }
    return ThetaTemp;
}

/**
  * @brief  获取C点x坐标的微分值
  * @note   注意单位问题：返回值单位为m/s，向前为正方向
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @param  LegSide：RobotSide_EnumTypeDef类型的枚举值，表示左腿还是右腿
  *             @arg LeftSide：左腿
  *             @arg RightSide：右腿
  * @retval xC_dot（C点x坐标的微分，向前为正，单位m/s）
*/
float LegLinkage_GetxCdot(LegLinkageCal_StructTypeDef* LegPtr, RobotSide_EnumTypeDef LegSide)
{
    float xC_dot_Temp = 0.0f;
    if(LegSide == LeftSide)
    {
        xC_dot_Temp = LegPtr->xC_dot * MM2M;
    }
    else if(LegSide == RightSide)
    {
        xC_dot_Temp = -LegPtr->xC_dot * MM2M;
    }
    return xC_dot_Temp;
}

/**
  * @brief  获取等效连杆L0的微分值
  * @note   注意单位问题：返回值单位为m/s，变长为正方向
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @retval L0_dot（等效连杆长度的微分，变长为正，单位m/s）
*/
float LegLinkage_GetL0dot(LegLinkageCal_StructTypeDef* LegPtr)
{
    float L0_dot_Result = 0.0f;

    /*获取转换矩阵的两个元素（其实就是VMC的转换矩阵中的元素）*/
    float T_Matrix11 = LegPtr->l1*MM2M * MySin((LegPtr->phi0-LegPtr->phi3)) * MySin((LegPtr->phi1-LegPtr->phi2)) / MySin((LegPtr->phi3-LegPtr->phi2));//T11 = L1*sin(phi0-phi3)*sin(phi1-phi2) / sin(phi3-phi2)
    float T_Matrix21 = LegPtr->l4*MM2M * MySin((LegPtr->phi0-LegPtr->phi2)) * MySin((LegPtr->phi3-LegPtr->phi4)) / MySin((LegPtr->phi3-LegPtr->phi2));//T21 = L4*sin(phi0-phi2)*sin(phi3-phi4) / sin(phi3-phi2)

    //防止出现NaN，也就是分母为0的情况
    if(isnan(T_Matrix11) == true)
    {T_Matrix11 = 0.0f;}
    if(isnan(T_Matrix21) == true)
    {T_Matrix21 = 0.0f;}

    /*获取L0_dot*/
	L0_dot_Result = T_Matrix11 * LegPtr->phi1_dot + T_Matrix21 * LegPtr->phi4_dot;
    return L0_dot_Result * MM2M;
}

/**
  * @brief  获取Theta（与竖直方向夹角）的微分
  * @note   注意单位问题：返回值单位为度/s，向后为正方向
  *         这里不用两次差值除以时间（即对Theta进行数值微分），是因为这样的话噪声会被放大，且可能出现毛刺
  *         采用运动学雅可比矩阵的方法，直接由关节角速度计算得到，更加平滑可靠
  *         聪明的你也许会发现，这里的雅各比矩阵计算其实和VMC的矩阵是一样的（因为公式推导就是如此）
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @param  LegSide：RobotSide_EnumTypeDef类型的枚举值，表示左腿还是右腿
  *             @arg LeftSide：左腿
  *             @arg RightSide：右腿
  * @retval Theta_dot（等效摆杆与竖直夹角的微分，向后为正，单位度/s）
*/
float LegLinkage_GetThetadot(LegLinkageCal_StructTypeDef* LegPtr, RobotSide_EnumTypeDef LegSide)
{
    float Theta_dot_Temp = 0.0f;

    /*获取转换矩阵的两个元素（其实就是VMC的转换矩阵中的元素）*/
    float T_Matrix12 = LegPtr->l1*MM2M * MyCos(LegPtr->phi0-LegPtr->phi3) * MySin(LegPtr->phi1-LegPtr->phi2) / (LegPtr->L0*MM2M * MySin(LegPtr->phi3-LegPtr->phi2));//T12 = L1*cos(phi0-phi3)*sin(phi1-phi2) / [L0*sin(phi3-phi2)]   
    float T_Matrix22 = LegPtr->l4*MM2M * MyCos(LegPtr->phi0-LegPtr->phi2) * MySin(LegPtr->phi3-LegPtr->phi4) / (LegPtr->L0*MM2M * MySin(LegPtr->phi3-LegPtr->phi2));//T22 = L4*cos(phi0-phi2)*sin(phi3-phi4) / [L0*sin(phi3-phi2)]

    //防止出现NaN，也就是分母为0的情况
    if(isnan(T_Matrix12) == true) 
    {T_Matrix12 = 0.0f;} 
    if(isnan(T_Matrix22) == true) 
    {T_Matrix22 = 0.0f;}

    /*获取Theta_dot（未考虑方向），Theta_dot = T12 * phi1_dot + T22 * phi4_dot*/
    Theta_dot_Temp = T_Matrix12 * LegPtr->phi1_dot + T_Matrix22 * LegPtr->phi4_dot;

    /*根据左右侧，确定Theta_dot的正方向*/
    if(LegSide == LeftSide)
    {
        Theta_dot_Temp = +(Theta_dot_Temp);
    }
    else if(LegSide == RightSide)
    {
        Theta_dot_Temp = -Theta_dot_Temp;
    }
    return Theta_dot_Temp;
}
//#endregion

//#region 低通滤波器LPF相关函数全家桶
/**
  * @brief  一阶低通滤波器的输入值设置函数
  * @note   用来设置一阶低通滤波器结构体中的输入值Input
  * @param  LPFptr：LPF_StructTypeDef类型的指针，一阶低通滤波器结构体指针
  * @param  NewInput：要设置的输入值
  * @retval 无
*/
void LPF_SetInput(LPF_StructTypeDef* LPFptr, float NewInput)
{
    LPFptr->Input = NewInput;
}

/**
  * @brief  一阶低通滤波器计算函数
  * @note   根据一阶低通滤波器公式进行计算，能够将原始数据进行滤波处理，使得数据更加平滑
  *         如果结构体中的Alpha有赋值，就直接用Alpha进行滤波计算，否则根据CutOffFreq和SampleTime计算Alpha
  * @param  LPFptr：LPF_StructTypeDef类型的指针，一阶低通滤波器结构体指针
  * @retval 无
*/
void LPF_Cal(LPF_StructTypeDef* LPFptr)
{
    if(LPFptr->Alpha > 0) //如果滤波系数Alpha有赋值，就直接用Alpha进行滤波
    {
        LPFptr->Out = (1.0f - LPFptr->Alpha)*LPFptr->Out + LPFptr->Alpha*LPFptr->Input; //Out[n] = (1 - Alpha)*Out[n-1] + Alpha*Input[n]
        return;
    }

    /*辅助变量的计算*/
    float RC = 1.0f / (2.0f * PI * LPFptr->CutOffFreq); //RC = 1 / (2*pi*CutOffFreq)

    /*滤波系数的计算*/
    float Alpha = LPFptr->SampleTime / (RC + LPFptr->SampleTime); //Alpha = SampleTime / (RC + SampleTime)

    /*滤波计算*/
    LPFptr->Out = (1.0f - Alpha)*LPFptr->Out + Alpha*LPFptr->Input; //Out[n] = (1 - Alpha)*Out[n-1] + Alpha*Input[n]
}

/**
  * @brief  获取一阶低通滤波器的输出值函数
  * @note   用来获取一阶低通滤波器结构体中的输出值Out
  * @param  LPFptr：LPF_StructTypeDef类型的指针，一阶低通滤波器结构体指针
  * @retval 一阶低通滤波器的输出值Out
*/
float LPF_GetOutput(LPF_StructTypeDef* LPFptr)
{
    return LPFptr->Out;
}
//#endregion

//#region LQR相关函数全家桶
/**
  * @brief  LQR的K矩阵更新函数
  * @note   如果传入的腿长有一条为0，则不拟合K矩阵，采用默认值
  *         如果传入的腿长不为0，则根据腿长对K矩阵进行拟合更新
  * @param  LQRptr：LQR_StructTypeDef类型的指针，LQR计算结构体指针
  * @param  LegLen1：左腿腿长，单位：m
  * @param  LegLen2：右腿腿长，单位：m
  * @retval 无
*/
void LQR_K_MatrixUpdate(LQR_StructTypeDef* LQRptr, float LegLen1, float LegLen2)
{
    /***************腿长为0，采用默认K矩阵***************/
    if((LegLen1 == 0.0f) || (LegLen2 == 0.0f))
    {
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 10; j++)
            {
                LQRptr->K_Matrix[i][j] = LQR_DefaultK_Matrix[i][j];
            }
        }
        return;
    }

    /***************腿长不为0，拟合K矩阵***************/
    //待补充：根据腿长拟合K矩阵，其实也可以根据不同腿长直接给3个K矩阵，省去拟合的麻烦
    //暂时先用默认值凑合着
    //下面是老代码
/*
void Get_K_Length_WholeBody(float L_Right,float L_Left)
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<10;j++)
        {
            k_matrix_wholebody[i][j] = k_matrix_coeff_wholebody[i][j][0]+k_matrix_coeff_wholebody[i][j][5]*L_Right*L_Left+
                                        +k_matrix_coeff_wholebody[i][j][1]*L_Left+k_matrix_coeff_wholebody[i][j][2]*L_Left*L_Left+
                                        +k_matrix_coeff_wholebody[i][j][3]*L_Right+k_matrix_coeff_wholebody[i][j][4]*L_Right*L_Right;

        }
    }
}

    void CalLQR_WholeBody(ST_Whole_Body_LQR *pstLqr,float L_Right,float L_Left)
{
    //获得K矩阵（虽然写了这个拟合，但是下面计算并没有用到，白给了）
    Get_K_Length_WholeBody(L_Right,L_Left);
    
    // 正常情况下，没有离地的时候
    if(OffGround_Flag == 0 || OS_TIME()-Start_Time_Cnt<800000)
    {
    }
    else
    {                            
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<10;j++)
            {
                pstLqr->LQR_K_Matrix[i][j] = 0;
            }
        }
        if(g_chassis_stair_mode == Stair_Up_Mode)
        {
            pstLqr->LQR_K_Matrix[2][4] = 40;
            pstLqr->LQR_K_Matrix[2][5] = 3; 
            pstLqr->LQR_K_Matrix[3][6] = 40;
            pstLqr->LQR_K_Matrix[3][7] = 3; 
        }
        else
        {
            pstLqr->LQR_K_Matrix[2][4] = 70;
            pstLqr->LQR_K_Matrix[2][5] = 3; 
            pstLqr->LQR_K_Matrix[3][6] = 70;
            pstLqr->LQR_K_Matrix[3][7] = 3; 
        }
    }
}
**/
}

/**
  * @brief  LQR的状态向量x更新函数
  * @note   根据传入的误差值，更新LQR的状态向量x
  *         误差值为：目标值 - 反馈值，Des - FB
  * @param  LQRptr：LQR_StructTypeDef类型的指针，LQR计算结构体指针
  * @param  DisErr：位移误差，单位：m
  * @param  VelErr：速度误差，单位：m/s
  * @param  YawAngleErr：偏转角度误差，单位：Rad
  * @param  YawAngleVelErr：偏转角速度误差，单位：Rad/s
  * @param  Theta1AngleErr：左腿摆角误差，单位：Rad
  * @param  Theta1AngleVelErr：左腿摆角速度误差，单位：Rad/s
  * @param  Theta2AngleErr：右腿摆角误差，单位：Rad
  * @param  Theta2AngleVelErr：右腿摆角速度误差，单位：Rad/s
  * @param  PitchAngleErr：俯仰角误差，单位：Rad
  * @param  PitchAngleVelErr：俯仰角速度误差，单位：Rad/s
  * @retval 无
*/
void LQR_xVector_DataUpdate(LQR_StructTypeDef* LQRptr, 
                            float DisErr,           float VelErr, 
                            float YawAngleErr,      float YawAngleVelErr,
                            float Theta1AngleErr,   float Theta1AngleVelErr, 
                            float Theta2AngleErr,   float Theta2AngleVelErr, 
                            float PitchAngleErr,    float PitchAngleVelErr)
{
    LQRptr->x_Vector[0] = DisErr;
    LQRptr->x_Vector[1] = VelErr;
    LQRptr->x_Vector[2] = YawAngleErr;
    LQRptr->x_Vector[3] = YawAngleVelErr;
    LQRptr->x_Vector[4] = Theta1AngleErr;
    LQRptr->x_Vector[5] = Theta1AngleVelErr;
    LQRptr->x_Vector[6] = Theta2AngleErr;
    LQRptr->x_Vector[7] = Theta2AngleVelErr;
    LQRptr->x_Vector[8] = PitchAngleErr;
    LQRptr->x_Vector[9] = PitchAngleVelErr;
}

/**
  * @brief  LQR计算函数
  * @note   根据LQR控制算法公式u = K * (X_Des - X_FB)进行计算，计算控制向量u
  * @param  LQRptr：LQR_StructTypeDef类型的指针，LQR计算结构体指针
  * @retval 无
*/
void LQR_Cal(LQR_StructTypeDef* LQRptr)
{
    for(int i = 0; i < 4; i++)
    {
        LQRptr->u_Vector[i] = 0.0f;
        for(int j = 0; j < 10; j++)
        {
            LQRptr->u_Vector[i] += LQRptr->K_Matrix[i][j] * LQRptr->x_Vector[j];
        }
    }
}

/**
  * @brief  获取LQR控制向量u的某个分量值
  * @note   根据传入的索引值，获取LQR控制向量u的对应分量值。注意索引值从0开始。
  *         直接返回原始值，正方向需要自行在外部考虑
  * @param  LQRptr：LQR_StructTypeDef类型的指针，LQR计算结构体指针
  * @param  index：索引值，表示u的第几个元素，在本机器人中取值范围为0~3
  * @retval LQR控制向量uVector的对应位置元素值
  *     @arg 0：T_wl，左轮扭矩
  *     @arg 1：T_wr，右轮扭矩
  *     @arg 2：T_bl，左摆杆扭矩
  *     @arg 3：T_br，右摆杆扭矩
*/
float LQR_Get_uVector(LQR_StructTypeDef* LQRptr, int index)
{
    if((index < 0) || (index > 3))
    {
        return 0.0f; //索引越界，返回0
    }

    return LQRptr->u_Vector[index];
}
//#endregion

//#region VMC相关函数全家桶
/**
  * @brief  VMC的F矩阵更新函数
  * @note   根据传入的力和力矩值，更新VMC的F矩阵，注意传入的力矩的正方向
  * @param  VMCptr：VMC_StructTypeDef类型的指针，VMC计算结构体指针
  * @param  Force：沿杆方向的力，单位：N
  * @param  Torque：力矩，单位：N·m，向后为正方向
  * @param  LegSide：RobotSide_EnumTypeDef类型的枚举值，表示左腿还是右腿
  *             @arg LeftSide：左腿
  *             @arg RightSide：右腿
  * @retval 无
*/
void VMC_FMatrixUpdate(VMC_StructTypeDef* VMCptr,float Force, float Torque, RobotSide_EnumTypeDef LegSide)
{
    /*腿部沿杆方向的力F*/
    VMCptr->F_Matrix[0] = Force;

    /*腿部力矩T*/
    if(LegSide == LeftSide)
    {VMCptr->F_Matrix[1] = Torque;}

    else if(LegSide == RightSide)
    {VMCptr->F_Matrix[1] = -Torque;}    
}

/**
  * @brief  VMC计算函数
  * @note   把简化建模得到的等效摆杆的力和力矩转换为关节电机的力和力矩
  * @param  VMCptr：VMC_StructTypeDef类型的指针，VMC计算结构体指针
  * @param  LegPtr：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @retval 无
*/
void VMC_Cal(VMC_StructTypeDef* VMCptr, LegLinkageCal_StructTypeDef* LegPtr)
{
    /*计算转化矩阵*/
    // J[0][0] = L1*sin(phi0-phi3)*sin(phi1-phi2) / sin(phi3-phi2)
    VMCptr->J_Matrix[0][0] = LegPtr->l1*MM2M * MySin(LegPtr->phi0 - LegPtr->phi3) * MySin(LegPtr->phi1 - LegPtr->phi2) / MySin(LegPtr->phi3 - LegPtr->phi2);
    // J[0][1] = L1*cos(phi0-phi3)*sin(phi1-phi2) / (L0*sin(phi3-phi2))
    VMCptr->J_Matrix[0][1] = LegPtr->l1*MM2M * MyCos(LegPtr->phi0 - LegPtr->phi3) * MySin(LegPtr->phi1 - LegPtr->phi2) / (LegPtr->L0*MM2M * MySin(LegPtr->phi3 - LegPtr->phi2));
    // J[1][0] = L4*sin(phi0-phi2)*sin(phi3-phi4) / sin(phi3-phi2)
    VMCptr->J_Matrix[1][0] = LegPtr->l4*MM2M * MySin(LegPtr->phi0 - LegPtr->phi2) * MySin(LegPtr->phi3 - LegPtr->phi4) / MySin(LegPtr->phi3 - LegPtr->phi2);
    // J[1][1] = L4*cos(phi0-phi2)*sin(phi3-phi4) / (L0*sin(phi3-phi2))
    VMCptr->J_Matrix[1][1] = LegPtr->l4*MM2M * MyCos(LegPtr->phi0 - LegPtr->phi2) * MySin(LegPtr->phi3 - LegPtr->phi4) / (LegPtr->L0*MM2M * MySin(LegPtr->phi3 - LegPtr->phi2));

    /*将等效力、力矩映射到关节电机的力矩上*/
    VMCptr->T_Matrix[0] = VMCptr->J_Matrix[0][0]*VMCptr->F_Matrix[0] + VMCptr->J_Matrix[0][1]*VMCptr->F_Matrix[1];
    VMCptr->T_Matrix[1] = VMCptr->J_Matrix[1][0]*VMCptr->F_Matrix[0] + VMCptr->J_Matrix[1][1]*VMCptr->F_Matrix[1];
}

/**
  * @brief  获取VMC计算得到的关节电机力矩
  * @note   根据传入的索引值，获取VMC计算得到的关节电机力矩。注意索引值从0开始。
  *         直接返回原始值，正方向需要自行在外部考虑
  * @param  VMCptr：VMC_StructTypeDef类型的指针，VMC计算结构体指针
  * @param  index：索引值，表示T_Matrix的第几个元素，在本机器人中取值范围为0~1
  *     @arg 0：T1，对应五连杆中的phi1关节电机力矩
  *     @arg 1：T4，对应五连杆中的phi4关节电机力矩
  * @retval VMC计算得到的对应关节电机力矩值
*/
float VMC_Get_TMatrix(VMC_StructTypeDef* VMCptr, int index)
{
    if((index < 0) || (index > 1))
    {
        return 0.0f; //索引越界，返回0
    }

    return VMCptr->T_Matrix[index];
}

//#endregion

//#region 离地OffGround检测相关函数全家桶
//待测试25.11.28,下面的所有离地检测相关
/**
  * @brief  离地检测的机体Z轴加速度更新函数
  * @note   根据传入的Z轴加速度值，更新OffGround结构体中的机体Z轴加速度
  * @param  pOffGrd：OffGround_StructTypeDef类型的指针，离地检测结构体指针
  * @param  AccZ_Body：机体Z轴加速度值，单位：m/s²
  * @retval 无
*/
void OffGround_BodyZAccUpdate(OffGround_StructTypeDef *pOffGrd, float AccZ_Body)
{
    pOffGrd->ZAcc_Body = AccZ_Body;
}

/**
  * @brief  离地检测的俯仰角更新函数
  * @note   根据传入的俯仰角值，更新OffGround结构体中的俯仰角
  * @param  pOffGrd：OffGround_StructTypeDef类型的指针，离地检测结构体指针
  * @param  PitchAngle：俯仰角值，单位：度
  * @retval 无
*/
void OffGround_PitchAngleUpdate(OffGround_StructTypeDef *pOffGrd, float PitchAngle)
{
    pOffGrd->PitchAngle = PitchAngle;
}

/**
  * @brief  离地检测的俯仰角速度更新函数
  * @note   根据传入的俯仰角速度值，更新OffGround结构体中的俯仰角速度
  * @param  pOffGrd：OffGround_StructTypeDef类型的指针，离地检测结构体指针
  * @param  PitchAngleVel：俯仰角速度值，单位：度/s
  * @retval 无
*/
void OffGround_PitchAngleVelUpdate(OffGround_StructTypeDef *pOffGrd, float PitchAngleVel)
{
    pOffGrd->PitchAngleVel = PitchAngleVel;
}

/**
  * @brief  离地检测中，五连杆相关的数据更新函数
  * @note   根据传入的五连杆计算参数结构体，更新离地检测结构体中需要的五连杆相关数据
  * @param  LegPtr[in]：LegLinkageCal_StructTypeDef类型的指针，五连杆计算参数结构体指针
  * @param  LegSide：RobotSide_EnumTypeDef类型的枚举值，表示左腿还是右腿
  * @param  pOffGrd[out]：OffGround_StructTypeDef类型的指针，离地检测结构体指针
  * @retval 无
*/
void OffGround_LegLinkRelateDataUpdate(LegLinkageCal_StructTypeDef LegPtr, OffGround_StructTypeDef *pOffGrd, RobotSide_EnumTypeDef LegSide)
{
    /*把上次的值保存下来*/
    pOffGrd->L0_dot_pre = pOffGrd->L0_dot;
    pOffGrd->Phi_dot_pre = pOffGrd->Phi_dot;

    /*把离地检测中需要的、五连杆相关的数据写入*/
    pOffGrd->L1 = LegPtr.l1 * MM2M;
    pOffGrd->L4 = LegPtr.l4 * MM2M;

    pOffGrd->Phi1 = LegPtr.phi1;
    pOffGrd->Phi2 = LegPtr.phi2;
    pOffGrd->Phi3 = LegPtr.phi3;
    pOffGrd->Phi4 = LegPtr.phi4;

    pOffGrd->Phi0 = LegPtr.phi0;
    
    
    //待优化：下面这些数据未经过滤波处理，可能会有噪声，后续可以考虑加个滤波器
    pOffGrd->L0      = LegLinkage_GetL0Length(&LegPtr) * MM2M;
    pOffGrd->L0_dot  = LegLinkage_GetL0dot(&LegPtr);
    pOffGrd->L0_ddot = (pOffGrd->L0_dot - pOffGrd->L0_dot_pre) / pOffGrd->SampleTime;

    pOffGrd->Theta     = LegLinkage_GetTheta(&LegPtr, LegSide);
    pOffGrd->Theta_dot = LegLinkage_GetThetadot(&LegPtr, LegSide);

    pOffGrd->Phi      = pOffGrd->Theta - pOffGrd->PitchAngle;        //地面坐标系下，摆杆与竖直方向夹角（注意需要减去机体俯仰角）
    pOffGrd->Phi_dot  = pOffGrd->Theta_dot - pOffGrd->PitchAngleVel; //地面坐标系下，摆杆与竖直方向夹角的微分（注意需要减去机体俯仰角速度）
    pOffGrd->Phi_ddot = (pOffGrd->Phi_dot - pOffGrd->Phi_dot_pre) / pOffGrd->SampleTime;    //地面坐标系下，摆杆与竖直方向夹角的二阶微分
}

/**
  * @brief  离地检测中，关节电机力矩数据更新函数
  * @note   根据传入的关节电机力矩值，更新离地检测结构体中需要的关节电机力矩数据
  * @param  pOffGrd：OffGround_StructTypeDef类型的指针，离地检测结构体指针
  * @param  T1：五连杆中phi1关节电机力矩，单位：N·m
  * @param  T4：五连杆中phi4关节电机力矩，单位：N·m
  * @retval 无
*/
void OffGround_TorqueDataUpdate(OffGround_StructTypeDef *pOffGrd, float T1, float T4)
{
    pOffGrd->T1 = T1;
    pOffGrd->T4 = T4;
}

/**
  * @brief  离地检测中，获取F和Tp的真实值
  * @note   根据传入的关节电机力矩，计算得到真实的F和Tp值
  * @param  pOffGrd[out]：OffGround_StructTypeDef类型的指针，离地检测结构体指针
  * @retval 无
*/
void OffGround_GetRealFAndTp(OffGround_StructTypeDef *pOffGrd)
{
    float J_Matrix_Inverse[2][2]; //这个矩阵是VMC转化矩阵的逆矩阵

    /*将离地检测需要使用的数值写入临时变量*/
    float L1 = pOffGrd->L1;
    float L4 = pOffGrd->L4;
    float L0 = pOffGrd->L0;

    float Phi1 = pOffGrd->Phi1;
    float Phi2 = pOffGrd->Phi2;
    float Phi3 = pOffGrd->Phi3;
    float Phi4 = pOffGrd->Phi4;
    float Phi0 = pOffGrd->Phi0;

    /*计算逆矩阵的各个元素*/
    float Sigma1 = L4 * (MyCos(Phi0-Phi2) * MySin(Phi0-Phi3) * MySin(Phi3-Phi4) - MyCos(Phi0-Phi3) * MySin(Phi0-Phi2) * MySin(Phi3-Phi4));
    float Sigma2 = L1 * (MyCos(Phi0-Phi2) * MySin(Phi0-Phi3) * MySin(Phi1-Phi2) - MyCos(Phi0-Phi3) * MySin(Phi0-Phi2) * MySin(Phi1-Phi2));
    float Sigma3 = MySin(Phi2 - Phi3);

    J_Matrix_Inverse[0][0] =      -MyCos(Phi0 - Phi2) * Sigma3 / Sigma2;
    J_Matrix_Inverse[0][1] =       MyCos(Phi0 - Phi3) * Sigma3 / Sigma1;
    J_Matrix_Inverse[1][0] =  L0 * MySin(Phi0 - Phi2) * Sigma3 / Sigma2;
    J_Matrix_Inverse[1][1] = -L0 * MySin(Phi0 - Phi3) * Sigma3 / Sigma1;

    /*计算真实的F和Tp值，传给输出变量*/
    float T1 = pOffGrd->T1;
    float T4 = pOffGrd->T4;
    pOffGrd->F_Leg  = J_Matrix_Inverse[0][0] * T1 + J_Matrix_Inverse[0][1] * T4;
    pOffGrd->Tp_Leg = J_Matrix_Inverse[1][0] * T1 + J_Matrix_Inverse[1][1] * T4;
}

/**
  * @brief  离地检测中，获取腿对轮子的竖直作用力P
  * @note   函数内部直接把计算出来的值赋给OffGround结构体中的F_P变量
  * @param  pOffGrd：OffGround_StructTypeDef类型的指针，离地检测结构体指针
  * @retval 腿对轮子的竖直作用力P，单位：N
*/
float OffGround_GetLegToWheelForce(OffGround_StructTypeDef *pOffGrd)
{
    float G_l = pOffGrd->M_l * pOffGrd->g;              //单个腿部重力 = 腿部重量 * 重力加速度

    float Phi = pOffGrd->Phi;     //地面坐标系下，摆杆与竖直方向夹角（注意需要减去机体俯仰角）
    float F  = pOffGrd->F_Leg;    //腿部沿杆方向的力
    float Tp = pOffGrd->Tp_Leg;   //腿部力矩
    float L0 = pOffGrd->L0;        //摆杆长度

    float F_P = G_l + F * MyCos(Phi) + Tp * MySin(Phi) / L0;
    return F_P;
}

/**
  * @brief  获取离地检测的地面支持力
  * @note   根据OffGround结构体，获取地面支持力F_N的值
  * @param  pOffGrd：OffGround_StructTypeDef类型的指针，离地检测结构体指针
  * @retval 地面地面支持力F_N，单位：N
*/
float OffGround_GetSupportForce(OffGround_StructTypeDef *pOffGrd)
{
    float F_P       = OffGround_GetLegToWheelForce(pOffGrd); //腿对轮子的竖直作用力
    float G_w       = pOffGrd->M_w * pOffGrd->g;    //单个轮子重力 = 轮子重量 * 重力加速度

    float M_w       = pOffGrd->M_w;             //轮子质量
    float ZAcc_Body = pOffGrd->ZAcc_Body;       //机体Z轴加速度
    float Phi       = pOffGrd->Phi;             //地面坐标系下，摆杆与竖直方向夹角（注意需要减去机体俯仰角）
    float Phi_dot   = pOffGrd->Phi_dot * A2R;   //摆杆角速度（注意需要减去机体俯仰角速度）
    float Phi_ddot  = pOffGrd->Phi_ddot * A2R;  //摆杆角加速度
    float L0        = pOffGrd->L0;              //摆杆长度
    float L0_dot    = pOffGrd->L0_dot;          //摆杆长度变化率（摆杆长度速度）
    float L0_ddot   = pOffGrd->L0_ddot;         //摆杆长度加速度

    float ZAcc_Wheel =  ZAcc_Body - L0_ddot * MyCos(Phi) + 2 * L0_dot * Phi_dot * MySin(Phi) //轮子Z轴加速度
                      + L0 * Phi_ddot * MySin(Phi) + L0 * Phi_dot * Phi_dot * MyCos(Phi); 

    pOffGrd->F_N    = F_P + G_w + M_w * ZAcc_Wheel; //地面支持力F_N = 腿对轮子的竖直作用力P + 轮子重力G_w + 轮子质量 * 轮子Z轴加速度

    //注意正方向问题，到车上这些角度都得重新试一下
    //老代码里面是右边没有负号，左边有，Theta和Theta_dot
    //Pitch都是一样的不用变
    //我的代码里面定义的都是向后为正，需要和老代码的方向对应上

//  LpFilter(&LegData->L0_dot_fliter);//OffGround里面用

// 	LpFilter(&OffGroundRight_struct.L0_ddot_fliter);
// 	LpFilter(&OffGroundRight_struct.Theta_ddot_fliter);
// 	LpFilter(&OffGroundRight_struct.F_N_fliter);

// 	LpFilter(&OffGroundLeft_struct.Theta_ddot_fliter );
// 	LpFilter(&OffGroundLeft_struct.L0_ddot_fliter );
// 	LpFilter(&OffGroundLeft_struct.F_N_fliter);

// 	if( OffGroundRight_struct.F_N_fliter.out <= 20 && OffGroundLeft_struct.F_N_fliter.out <= 20 )
// 	{
// 		OffGround_Flag = 1;
// 	}
// 	else if( (OffGroundRight_struct.F_N_fliter.out >= 30 && OffGroundLeft_struct.F_N_fliter.out >= 30)&&(OffGround_Flag == 1) )
// 	{
// 		OffGround_Flag = 0;
// 	}
// 	if(OffGroundRight_struct.F_N_fliter.out <= 30)
// 	{
// 		Right_OffGround_Flag = 1;
// 	}
// 	else if(OffGroundRight_struct.F_N_fliter.out >=60)
// 	{
// 		Right_OffGround_Flag = 0;
// 	}
// 	if(OffGroundLeft_struct.F_N_fliter.out <= 30)
// 	{
// 		Left_OffGround_Flag = 1;
// 	}
// 	else if(OffGroundLeft_struct.F_N_fliter.out >=60)
// 	{
// 		Left_OffGround_Flag = 0;
// 	}
// 	if(g_chassis_jump_mode == Jump_Compress_Mode)
// 	{
// 		OffGround_Flag = 0;
// 	}

    return pOffGrd->F_N;
}


//#endregion

/**
  * @brief  龙伯格观测器计算函数（匀速模型）
  * @note   用来估计底盘速度(老代码拿过来的神奇的东西，我没太搞懂)
  * @param  observer：LuenbergerObserver_StructTypeDef类型的指针，龙伯格观测器结构体指针
  * @retval 无
*/
void LuenbergerObserver_UniformVelocityModel(LuenbergerObserver_StructTypeDef* observer)
{
    //求先验状态向量
    observer->x_PriorHatMatrix[0] = observer->x_HatMatrix[0]+GCH_TaskTime*observer->x_HatMatrix[1];
    observer->x_PriorHatMatrix[1] = observer->x_HatMatrix[1];

    /*增益矩阵赋值*/
    // if(OffGround_Flag)   //待优化：老代码里的离地标志位，等我写到离地检测了再加
    // {L11=0.002f;}
    float L11 = 0.004f; //这个值是怎么来的？我也不知道
    float L22 = 0.01f;  //这个值是怎么来的？我也不知道
    observer->L_Matrix[0][0] = L11;
    observer->L_Matrix[1][1] = L22;

    //求状态估计
    observer->x_HatMatrix[0] = (1-observer->L_Matrix[0][0])*observer->x_PriorHatMatrix[0] + observer->L_Matrix[0][0]*observer->y_Matrix[0];
    observer->x_HatMatrix[1] = (1-observer->L_Matrix[1][1])*observer->x_PriorHatMatrix[1] + observer->L_Matrix[1][1]*observer->y_Matrix[1];
}
