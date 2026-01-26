/**
 ******************************************************************************
 * @file    Chassis_Stratgy.c
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.11.2
 * @brief   底盘控制策略相关函数
 *          把底盘的控制策略函数放在这里，方便外部调用
 ******************************************************************************
 */

#include <arm_math.h>

#include "Algorithm_Simple.h"
#include "Chassis_APIFunction.h"
#include "General_AuxiliaryFunc.h"
#include "GlobalDeclare_Chassis.h"
#include "TIM_Config.h"
#include "Chassis_Stratgy.h"

/**
  * @brief  底盘模式选择参数结构体更新函数
  * @note   把底盘模式选择的相关参数，更新到底盘模式选择参数结构体中，以方便后面的模式选择处理
  *         在模式处理之前，应该先调用此函数，更新参数，然后再进入模式选择函数
  * @param  pModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型，底盘模式选择参数结构体指针
  * @retval 无
  */
//* 模式选择参数结构体更新函数
void ChassisStrategy_ModeChooseParaStructUpdate(Chassis_ModeChooseParameter_StructTypeDef* pModeChoosePara) 
{
    /*更新上次模式*/
    pModeChoosePara->MC_ModePre = GEMCH_ModePre;

    /*更新各个模式开始时间、当前时间*/
    pModeChoosePara->MC_ST_ModeStartTime = GSTCH_Data.ST_ModeStartTime; //各个模式开始时间
    pModeChoosePara->MC_TimeNow = RunTimeGet();                         //获取当前时间

    /*运动姿态变量*/
    pModeChoosePara->MC_LegLenAvgFB = (GSTCH_Data.LegLen1FB + GSTCH_Data.LegLen2FB) / 2.0f; //两腿腿长反馈值平均值
    pModeChoosePara->MC_VelFB = GSTCH_Data.VelFB;                                           //底盘速度反馈值

    /*离地检测标志*/
    if(GSTCH_Data.F_OffGround1 == true || GSTCH_Data.F_OffGround2 == true)
    {pModeChoosePara->MC_F_OffGround = true;}//离地状态标志，只要有任何一侧离地都算
    else
    {pModeChoosePara->MC_F_OffGround = false;}

    if(GSTCH_Data.F_SlipHM1 || GSTCH_Data.F_SlipHM2 || GSTCH_Data.F_BlockHM1 || GSTCH_Data.F_BlockHM2)
    {pModeChoosePara->MC_F_Struggle = true;}//脱困状态标志，只要有任何一侧异常都算
    else
    {pModeChoosePara->MC_F_Struggle = false;}

    /*AutoSafe模式专用变量*/
    pModeChoosePara->MC_HubMotor1Rx_fps = GST_SystemMonitor.HubMotor1Rx_fps;    //轮毂电机1接收帧率
    pModeChoosePara->MC_HubMotor2Rx_fps = GST_SystemMonitor.HubMotor2Rx_fps;    //轮毂电机2接收帧率
    pModeChoosePara->MC_UART4Rx_fps     = GST_SystemMonitor.UART4Rx_fps;        //串口4，即IMU2接收帧率
}

// #pragma region 检测当前条件是否满足进入某模式的函数

/**
  * @brief  遥控器模式下，检测是否需要进入手动安全模式的函数
  * @note   在函数内部会进行判断，如果达到其中任何一个条件就会进入手动安全模式
  * @param  无
  * @retval false：所有判断条件都正常，不需要进入ManualSafeMode
  *         true：任何一项判断条件生效，需要进入ManualSafeMode
*/
bool _ChIsEnter_ManualSafeMode_RCControl(void)
{
    /* 如果遥控器断开连接，不进入ManualSafeMode（应该进入AutoSafe） */
    if(IsRCConnected() == false)
    {return false;}

    /* 如果遥控器拨杆为左下右上，进入ManualSafeMode */
    if(IsLeftLevelDown() && IsRightLevelUp())
    {return true;}

    /*其他情况不进入ManualSafeMode*/
    return false;
}


/**
  * @brief  遥控器模式下，检测是否需要进入自动安全模式的函数
  * @note   在函数内部会进行一系列判断，如果达到其中任何一个条件就会进入自动安全模式
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval false：所有判断条件都正常，不需要进入AutoSafeMode
  *         true：任何一项判断条件生效，需要进入AutoSafeMode
*/
bool _ChIsEnter_AutoSafeMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    /*用一些临时变量来存储相关变量*/
    float HM1_Rx_fps = ST_ModeChoosePara.MC_HubMotor1Rx_fps; //轮毂电机1通讯帧率
    float HM2_Rx_fps = ST_ModeChoosePara.MC_HubMotor2Rx_fps; //轮毂电机2通讯帧率
    float UART4_Rx_fps = ST_ModeChoosePara.MC_UART4Rx_fps;   //串口4，即IMU2通讯帧率

    /* 如果遥控器断开连接，进入AutoSafeMode */
    if(IsRCConnected() == false)
    {return true;}

    /* 如果轮毂电机通讯异常，进入AutoSafeMode */
    if(HM1_Rx_fps < HubMotorRx_fpsMinTH || HM2_Rx_fps < HubMotorRx_fpsMinTH)
    {return true;}

    /* 如果IMU2通讯异常，进入AutoSafeMode */
    if(UART4_Rx_fps < UART4Rx_fpsMinTH)
    {return true;}

    //老代码说明：
	// TODO 考虑在保护中加入腿部摆角的判断
    // if(
	// 				((/* capacitor_msg.CAP_Vol < 13 || */capacitor_msg.CAP_Vol> 55)|| // 超电保护
    //                 ((fabs(G_ST_IMU2.Receive.pitch_angle - Med_Angle_Norm)>=20.0f || (fabs(-LegLeftPosDistribute.Theta)>28.0f && fabs(-LegRightPosDistribute.Theta)>28.0f && fabs(G_ST_IMU2.Receive.pitch_angle - Med_Angle_Norm)>=8.0f)) && Height_Choose_Cnt == 0)||
	// 				(fabs(G_ST_IMU2.Receive.pitch_angle - Med_Angle_Norm)>=18.0f && Height_Choose_Cnt == 1)||
    //                 stuck_flag || stuck_flag_still || stair_lay_flag) 
	// 				&& Speed_Flag &&  (OS_TIME() - Start_Time_Cnt )> 800000 && OffGround_Flag == 0 && Up_Stair_Flag==0)
    // {return true;}

    // if((stuck_flag_start && OS_TIME()-Start_Time_Cnt > 100000 && OS_TIME()-Start_Time_Cnt < 800000 && Speed_Flag) == true)
    // {return true;}
	
    /*如果全部判断条件正常，不进入AutoSafeMode*/
    return false;
}

/**
  * @brief  遥控器模式下，检测是否需要进入坐下模式的函数（相当于待机模式）
  * @note   在函数内部会进行一系列判断，如果达到其中任何一个条件就会进入坐下模式
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval false：不需要进入SitDownMode
  *         true： 需要进入SitDownMode
  */
bool _ChIsEnter_SittingMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    /*用一些临时变量来存储相关变量*/
    float ModePre_tmp = ST_ModeChoosePara.MC_ModePre;      //上次模式
    float LegLenAvgFB = ST_ModeChoosePara.MC_LegLenAvgFB;  //腿长平均值

    /*如果左拨杆不在中位，不进入坐下模式*/
    if(IsLeftLevelMid() == false)
    {return false;}

    /*如果上次状态是离地，不进入坐下模式*/
    if(ModePre_tmp == CHMode_RC_OffGround)
    {return false;}

    /*如果上次状态是ManualSafe、AutoSafe其中一个，允许进入坐下模式*/
    if(ModePre_tmp == CHMode_RC_ManualSafe)
    {return true;}
    else if(ModePre_tmp == CHMode_RC_AutoSafe)
    {return true;}

    /*如果上次状态是SlowSitDown，且腿长已经到达最小值附近，进入坐下模式*/
    if(ModePre_tmp == CHMode_RC_SlowSitDown && LegLenAvgFB <= LegLenMin + LegLenMinTH)
    {return true;}

    /*如果上次状态是Sitting，且拨轮没有上推，继续保持坐下模式*/
    if(ModePre_tmp == CHMode_RC_Sitting && IsRollerUp() == false)
    {return true;}

    /*否则不进入*/
    return false;
}

/**
 * @brief  遥控器模式下，检测是否需要进入起立模式的函数
 * @param
 * ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
 * @retval false：不进入StandUpMode
 *         true： 进入StandUpMode
 */
bool _ChIsEnter_StandUpMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    /*用一些临时变量来存储相关变量*/
    float ModePre = ST_ModeChoosePara.MC_ModePre;      //上次模式
    float TimeNow = ST_ModeChoosePara.MC_TimeNow;      //当前时间
    float ThisModeStartTime = ST_ModeChoosePara.MC_ST_ModeStartTime.RC_StandUp; //起立模式开始时间
    float ThisModeTotalTime = TimeNow - ThisModeStartTime;  //起立模式总时长 = 当前时间 - 起立模式开始时间

    /*如果左拨杆不在中位，不进入起立模式*/
    if(IsLeftLevelMid() == false)
    {return false;}

    /*如果上次状态是Sitting、StandUp允许进入起立模式*/
    if(ModePre == CHMode_RC_Sitting)
    {return true;}
    
    /*如果上次状态是StandUp，且StandUp模式总时间没有到，继续保持StandUp模式（否则应该进入Free）*/
    if(ModePre == CHMode_RC_StandUp && ThisModeTotalTime < CHMode_RC_StandUp_TotalTime)
    {return true;}

    /*其他情况，不进入*/
    return false;
}

/**
* @brief  遥控器模式下，检测是否需要进入底盘自由模式的函数
* @note   待补充
* @param
* ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
* @retval false：不进入FreeMode
*         true： 进入FreeMode
*/
bool _ChIsEnter_FreeMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    /*用一些临时变量来存储相关变量*/
    float ModePre_tmp = ST_ModeChoosePara.MC_ModePre;  //上次模式
    bool  F_OffGround_tmp = ST_ModeChoosePara.MC_F_OffGround; //离地状态标志，true表示离地，false表示触地

    /*如果左拨杆不在中位，不进入*/
    if(IsLeftLevelMid() == false)
    {return false;}

    /*如果拨轮上推，不进入（应该进入SlowSitDown）*/
    if(IsRollerUp() == true)
    {return false;}

    /*离地状态下，不进入Free模式*/
    if(F_OffGround_tmp == true)
    {return false;}

    /*离地状态后触地，允许进入Free模式*/
    if(ModePre_tmp == CHMode_RC_OffGround && F_OffGround_tmp == false)
    {return true;}

    /*如果上次模式是Free，允许进入*/
    if(ModePre_tmp == CHMode_RC_Free)
    {return true;}

    /*如果上次模式是StandUp，允许进入*/
    if(ModePre_tmp == CHMode_RC_StandUp)
    {return true;}

    // TODO 写到Follow模式的时候再补充
    /*如果上次状态是Follow，允许进入*/
    if(ST_ModeChoosePara.MC_ModePre == CHMode_RC_Follow)
    {return true;}

    return false;
}

/**
  * @brief  遥控器模式下，检测是否需要进入缓慢坐下模式的函数
  * @note   待补充
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval false：不进入SlowSitDownMode
  *         true： 进入SlowSitDownMode
*/
bool _ChIsEnter_SlowSitDownMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    /*用一些临时变量来存储相关变量*/
    float ModePre = ST_ModeChoosePara.MC_ModePre;  //上次模式

    /*如果上次模式是缓慢坐下，允许进入*/
    if(ModePre == CHMode_RC_SlowSitDown)
    {return true;}

    /*如果上次模式是Free，且拨轮上推，允许进入*/
    if(ModePre == CHMode_RC_Free && IsRollerUp() == true)
    {return true;}

    /*其他情况，不进入*/
    return false;
}

/**
  * @brief  遥控器模式下，检测是否需要进入底盘跟随模式的函数
  * @note   待补充
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval false：不进入FollowMode
  *         true： 进入FollowMode
  */
bool _ChIsEnter_FollowMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    float ModePre_tmp = ST_ModeChoosePara.MC_ModePre;         // 上次模式
    bool F_OffGround_tmp = ST_ModeChoosePara.MC_F_OffGround;  //离地状态标志，true表示离地，false表示触地
    // TODO （记得把StandUp模式加进去）
    /*如果左拨杆不在上位，不进入*/
    if(IsLeftLevelUp() == false)
    {return false;}

    /*离地状态下，不进入Follow模式*/
    if(F_OffGround_tmp == true)
    {return false;}

    /* 如果当前已经是 Follow 模式，继续保持 */
    if(ModePre_tmp == CHMode_RC_Follow)
    {return true;}

    /* 允许从起立模式 (StandUp) 或 自由模式 (Free) 切换进入跟随模式 */
    if(ModePre_tmp == CHMode_RC_StandUp || ModePre_tmp == CHMode_RC_Free)
    {return true;}

    return false;
}

/**
  * @brief  遥控器模式下，检测是否需要进入离地模式的函数
  * @note   待补充
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval false：不进入OffGroundMode
  *         true： 进入OffGroundMode
  */
bool _ChIsEnter_OffGroundMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    /*用一些临时变量来存储相关变量*/
    float ModePre_tmp = ST_ModeChoosePara.MC_ModePre;  //上次模式
    bool  F_OffGround_tmp = ST_ModeChoosePara.MC_F_OffGround; //离地状态标志，true表示离地，false表示触地

    /*上次模式不是OffGround、Free、Follow模式中任何一个，不进入离地模式*/
    if(ModePre_tmp != CHMode_RC_OffGround && ModePre_tmp != CHMode_RC_Free && ModePre_tmp != CHMode_RC_Follow)
    {return false;}

    /*离地标志位开启了，认为进入离地模式*/
    if(F_OffGround_tmp == true)
    {return true;}

    return false;
}

/**
 * @brief     遥控器模式下，检测是否需要进入脱困模式的函数
 * @param[in] ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
 * @retval    false：不进入OffGroundMode
 *            true： 进入OffGroundMode
 */
bool _ChIsEnter_StruggleMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara){
    // float ModePre_tmp = ST_ModeChoosePara.MC_ModePre;         // 上次模式
    // bool F_OffGround_tmp = ST_ModeChoosePara.MC_F_OffGround;  //离地状态标志，true表示离地，false表示触地
    // bool F_Struggle_tmp = ST_ModeChoosePara.MC_F_Struggle;  //脱困状态标志，true表示脱困，false表示正常

    // /*离地状态下，不进入Follow模式*/
    // if(F_OffGround_tmp == true)
    // {return false;}

    // /*排除其他不允许进入脱困模式的状态 */
    // if (ModePre_tmp == CHMode_RC_ManualSafe || 
    //     ModePre_tmp == CHMode_RC_AutoSafe   || 
    //     ModePre_tmp == CHMode_RC_Sitting    || 
    //     ModePre_tmp == CHMode_RC_SlowSitDown) 
    // {
    //     return false;
    // }

    // /*如果已经处于脱困模式，根据内部逻辑决定是否退出*/
    // if (ModePre_tmp == CHMode_RC_Struggle) {
    //     // 如果标志位还在，继续保持
    //     if (F_Struggle_tmp == true)
    //     {return true;}
    //     // 如果标志位消失，退出（由后续优先级决定去向，通常回 Free 模式）
    //     return false;
    // }

    // /*触发进入条件：不在排除列表内，且触发标志位置位*/
    // if (F_Struggle_tmp == true) {
    //     return true;
    // }

    // return false;
}

// #pragma endregion

//* 底盘模式更新函数。当检测模式条件函数返回真值时将当前模式变量变到对应变量值
/**
  * @brief  遥控器模式下，底盘模式切换函数
  * @note   根据不同的条件切换底盘的模式，要注意优先级的问题，最高优先级和次高优先级分别是手动安全模式和自动安全模式
  *         后面要添加的其他状态切换，比如跳跃、离地等等，添加时要注意优先级的顺序
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval ChassisMode_EnumTypeDef的枚举类型，底盘的工作状态
*/
ChassisMode_EnumTypeDef ChassisStrategy_ModeChoose_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) 
{
    ChassisMode_EnumTypeDef ModeTemp = CHMode_RC_AutoSafe;  // 底盘状态，默认进入RC自动安全模式，如果遥控器有指令会进行修改

    /************切换底盘的状态************/
    /*注意！！为了更加方便管理，下面的模式判断选择中，只使用ST_ModeChoosePara里的变量、遥控器相关函数来做判断*/
    /*如果想要用其他的变量来做判断，可以在Chassis_ModeChooseParameter_StructTypeDef里面添加变量*/
    /*然后在ChassisStratgy_ModeChooseParaStructUpdate中添加该变量的更新语句，最后在这里调用*/

    /*注意！！下面的一些语句可能会有优先级要求，没有必要的话不要打乱排列顺序*/
    // 优先级最高的是ManualSafe，保证了进入ManualSafe之后不会切换到其他模式
    if(_ChIsEnter_ManualSafeMode_RCControl() == true)
    {ModeTemp = CHMode_RC_ManualSafe;}     //进入RC手动安全模式

    else if(_ChIsEnter_AutoSafeMode_RCControl(ST_ModeChoosePara) == true)
    {ModeTemp = CHMode_RC_AutoSafe;}       //进入RC自动安全模式

    else if(_ChIsEnter_SittingMode_RCControl(ST_ModeChoosePara) == true)
    {ModeTemp = CHMode_RC_Sitting;}        //进入RC坐下模式

    else if(_ChIsEnter_StandUpMode_RCControl(ST_ModeChoosePara) == true)
    {ModeTemp = CHMode_RC_StandUp;}        //进入RC起立模式

    else if(_ChIsEnter_FreeMode_RCControl(ST_ModeChoosePara) == true)
    {ModeTemp = CHMode_RC_Free;}           //进入RC底盘自由模式

    else if(_ChIsEnter_SlowSitDownMode_RCControl(ST_ModeChoosePara) == true)
    {ModeTemp = CHMode_RC_SlowSitDown;}    //进入RC底盘缓慢坐下模式

    else if(_ChIsEnter_FollowMode_RCControl(ST_ModeChoosePara) == true)
    {ModeTemp = CHMode_RC_Follow;}         //进入RC底盘跟随模式

    else if (_ChIsEnter_OffGroundMode_RCControl(ST_ModeChoosePara) == true)
    {ModeTemp = CHMode_RC_OffGround;}      //进入RC底盘离地模式

    else if (_ChIsEnter_StruggleMode_RCControl(ST_ModeChoosePara) == true)
    {ModeTemp = CHMode_RC_Struggle;}      //进入RC底盘脱困模式

    return ModeTemp;
}

//* 获取底盘各模式开始时间函数。当除安全模式以外有多个模式时需要加入（只有起立模式如果没有计时会无法设置目标值）
/**
 * @brief  获取底盘各模式开始时间的函数
 * @note   在任务切换时，获取当前任务的开始时间，方便在模式控制函数中使用
 *         后续如果新写了模式，也可以在这里添加对应的获取开始时间表达式
 * @param  pCHData：CHData_StructTypeDef类型的指针，底盘数据结构体指针
 * @param  Mode：ChassisMode_EnumTypeDef类型的枚举值，底盘当前的工作状态
 * @param  ModePre：ChassisMode_EnumTypeDef类型的枚举值，底盘上次的工作状态
 * @retval 无
 */
void ChassisStrategy_ModeStartTimeUpdate(CHData_StructTypeDef* pCHData,
                                        ChassisMode_EnumTypeDef Mode,
                                        ChassisMode_EnumTypeDef ModePre) {
    /*如果模式没有切换，直接返回*/
    if (Mode == ModePre) {
        return;
    }

    /*如果模式切换，记录当前时间为该模式的开始时间*/
    switch (Mode) {
        case CHMode_RC_ManualSafe:
            pCHData->ST_ModeStartTime.RC_ManualSafe = RunTimeGet();
            break;
        case CHMode_RC_AutoSafe:
            pCHData->ST_ModeStartTime.RC_AutoSafe = RunTimeGet();
            break;
        case CHMode_RC_StandUp:
            pCHData->ST_ModeStartTime.RC_StandUp = RunTimeGet();
            break;
        case CHMode_RC_Sitting:
            pCHData->ST_ModeStartTime.RC_Sitting = RunTimeGet();
            break;
        case CHMode_RC_Free:
            pCHData->ST_ModeStartTime.RC_Free = RunTimeGet();
            break;
        case CHMode_RC_OffGround:
            pCHData->ST_ModeStartTime.RC_OffGround = RunTimeGet();
            break;
        case CHMode_RC_TouchGround:
            pCHData->ST_ModeStartTime.RC_TouchGround = RunTimeGet();
            break;
        case CHMode_RC_Struggle:
            pCHData->ST_ModeStartTime.RC_Struggle = RunTimeGet();
            break;
    }
}
//* 底盘运动模式更新函数
// 在keil里面看GEMCH_Mode和GEMCH_ModePre就能观察状态了
/**
 * @brief  底盘模式选择更新函数
 * @note   用于更新底盘模式选择相关参数
 * @param  无
 * @retval 无
 */
void CH_ChassisModeUpdate(void) {
    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara_tmp;
    ChassisStrategy_ModeChooseParaStructUpdate(&ST_ModeChoosePara_tmp);  // 更新底盘模式选择参数结构体
    //* 实现的是局部变量：ModeTemp、MC_ModePre 向全局变量
    // GEMCH_ModePre、GEMCH_Mode 赋值
    GEMCH_ModePre = GEMCH_Mode;
    //! 这个函数会在这里检测当前是否有遥控器指令指示需要切换到什么状态
    GEMCH_Mode = ChassisStrategy_ModeChoose_RCControl(ST_ModeChoosePara_tmp);  // 调用底盘模式选择函数
    ChassisStrategy_ModeStartTimeUpdate(&GSTCH_Data, GEMCH_Mode, GEMCH_ModePre);
}

// #pragma region 模式具体功能实现函数

/**
 * @brief  底盘手动腿长三档控制处理函数
 * @note   实现“摇杆上推回中-变长”、“摇杆下拨回中-变短”的逻辑
 */
void CH_LegLenAdjust_Process(void) {
    // 仅在允许的模式下运行（Free, Follow, Struggle）
    if (GEMCH_Mode != CHMode_RC_Free && GEMCH_Mode != CHMode_RC_Follow && GEMCH_Mode != CHMode_RC_Struggle) {
        GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
        GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
        GSTCH_Data.F_JoyUpLatched = false;
        GSTCH_Data.F_JoyDownLatched = false;
        return;
    }

    // 1. 检测上抬动作
    if (IsRightJoyStickUp()) {
        GSTCH_Data.F_JoyUpLatched = true;
    }
    // 2. 检测下拨动作
    if (IsRightJoyStickDown()) {
        GSTCH_Data.F_JoyDownLatched = true;
    }

    // 3. 检测回中触发（回中时判定刚才是否有推/拨动作）
    if (IsRightJoyStickUp() == false && IsRightJoyStickDown() == false) {
        // 上抬回中触发：Min -> Mid -> High
        if (GSTCH_Data.F_JoyUpLatched) {
            // 如果当前在 LegLenMin 附近，变到 LegLenMid
            if (GST_RMCtrl.STCH_Default.LegLen1ManualDes < LegLenMid - 0.001f) {
                GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;     
            } else if (GST_RMCtrl.STCH_Default.LegLen1ManualDes < LegLenHigh - 0.001f) {
                GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenHigh;
            }
            // 如果当前已经接近 LegLenMid，变到 LegLenHigh
            if (GST_RMCtrl.STCH_Default.LegLen2ManualDes < LegLenMid - 0.001f) {
                GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;       
            } else if (GST_RMCtrl.STCH_Default.LegLen2ManualDes < LegLenHigh - 0.001f) {
                GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenHigh;
            }
            GSTCH_Data.F_JoyUpLatched = false; // 清除标志
        }
        
        // 下拨回中触发：High -> Mid -> Min
        if (GSTCH_Data.F_JoyDownLatched) {
            // 如果当前在 High 附近，下调到 Mid
            if (GST_RMCtrl.STCH_Default.LegLen1ManualDes > LegLenMid + 0.001f) {
                GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
            } else if (GST_RMCtrl.STCH_Default.LegLen1ManualDes > LegLenMin + 0.001f) {
                GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMin;
            }
            // 如果当前在 Mid 附近，下调到 Min
            if (GST_RMCtrl.STCH_Default.LegLen2ManualDes > LegLenMid + 0.001f) {
                GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
            } else if (GST_RMCtrl.STCH_Default.LegLen2ManualDes > LegLenMin + 0.001f) {
                GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMin;
            }
            GSTCH_Data.F_JoyDownLatched = false; // 清除标志
        }
    }

    // 5. 如果从未触发过上述逻辑且变量未初始化，设置默认值为中腿长
    if (GST_RMCtrl.STCH_Default.LegLen1ManualDes < 0.01f) { // 简单初值保护
        GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
    }
    if (GST_RMCtrl.STCH_Default.LegLen2ManualDes < 0.01f) { // 简单初值保护
        GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
    }
}


/**
 * @brief  遥控器模式下，手动安全模式控制函数
 * @note   手动安全模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_ManualSafeMode_RCControl(void) {
    /*清空底盘相关的Des数据、位移、控制数据*/
    Chassis_AllDesDataReset();     // 清空底盘相关的Des数据
    Chassis_DisFBClear();          // 底盘位移反馈值清零
    Chassis_RobotCtrlDataReset();  // 底盘控制数据清零

    /*判断是否要手动标定关节电机零点位置*/
    GFCH_LegCalibration = IsEnterManualCalibration();
}

/**
 * @brief  遥控器模式下，自动安全模式控制函数
 * @note   自动安全模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_AutoSafeMode_RCControl(void) {
    /*清空底盘相关的Des数据、位移、控制数据*/
    Chassis_AllDesDataReset();     // 清空底盘相关的Des数据
    Chassis_DisFBClear();          // 底盘位移反馈值清零
    Chassis_RobotCtrlDataReset();  // 底盘控制数据清零
}

/**
 * @brief  遥控器模式下，坐下模式控制函数
 * @note   坐下模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_SittingMode_RCControl(void)
{
    /*清空底盘相关的Des数据、位移、控制数据*/
    Chassis_AllDesDataReset();      //清空底盘相关的Des数据
    Chassis_DisFBClear();           //底盘位移反馈值清零
    Chassis_RobotCtrlDataReset();   //底盘控制数据清零
}

/**
 * @brief  遥控器模式下，起立模式控制函数
 * @note   起立模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_StandUpMode_RCControl(void) {

    //! 数据预清零（如果只想在模式进入时执行一次的变量那么写在这里面）
    //* 下面的函数只会在模式切换的前四毫秒执行，会进行数据清除、参数设置等操作
    if (RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_StandUp < CHMode_AllMode_PreProcessTime) {
        //* 腿长PID目标值和系数设定
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpStandUp, 0.0f, PID_LegLen_KdStandUp);  // 左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpStandUp, 0.0f, PID_LegLen_KdStandUp);  // 右腿长PID系数Kp、Ki、Kd赋值

        //* 起立模式下腿长TD系数r设定
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rStandUp);  // 左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rStandUp);  // 右腿长TD系数r赋值

        /*清空底盘相关的一些数据*/
        Chassis_AllDesDataReset();     // 清空底盘相关的Des数据
        Chassis_DisFBClear();          // 底盘位移反馈值清零
        Chassis_RobotCtrlDataReset();  // 底盘控制数据清零
        return;
    }

    /********************任务开始一段时间后的调整，时间的单位是ms********************/
    /*配置默认可配置的控制量*/
    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMid;             //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMid;             //右腿目标腿长
    GST_RMCtrl.STCH_Default.DisDes          = 0.0f;                  //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;                  //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;                  //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;                  //目标偏航角速度
    GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静态前馈力）
    GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静态前馈力）

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}


/**
 * @brief  遥控器模式下，自由模式控制函数
 * @note   自由模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_FreeMode_RCControl(void) {
    /****************************该模式的前置处理****************************/
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_Free < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);   //右腿长TD系数r赋值
        
        Chassis_DisFBClear();                         //底盘位移反馈值清零

        /*配置默认可配置的控制量*/
        GST_RMCtrl.STCH_Default.LegLen1Des      = GST_RMCtrl.STCH_Default.LegLen1ManualDes; //左腿目标腿长
        GST_RMCtrl.STCH_Default.LegLen2Des      = GST_RMCtrl.STCH_Default.LegLen2ManualDes; //右腿目标腿长
        GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
        // TODO 如果是从Follow切换过来，速度是不是应该保持不变呢？
        GST_RMCtrl.STCH_Default.VelDes          = 0.0f;             //目标速度
        GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
        GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静止时的默认值）
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静止时的默认值）
    }
    //* 方便调参数
    /*腿长PID系数赋值*/
    // PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
    // PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

    /*配置默认可配置的控制量*/
    GST_RMCtrl.STCH_Default.LegLen1Des      = GST_RMCtrl.STCH_Default.LegLen1ManualDes; //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = GST_RMCtrl.STCH_Default.LegLen2ManualDes; //右腿目标腿长
    
    /*************************任务开始一段时间后*************************/
    /*检测是否进入小陀螺模式*/
    static bool F_TopMode = false;
    F_TopMode = ChModeControl_FreeMode_RCControl_IsEnterTopMode(GSTCH_Data);

    /*如果不是小陀螺模式，正常进行速度、转向的调控*/
    if(F_TopMode == false)
    {ChModeControl_FreeMode_RCControl_MoveHandler(&GSTCH_Data, &GST_RMCtrl);} //移动处理函数，包括平移、转弯的速度获取

    /*如果是小陀螺模式，进行小陀螺的相关处理*/
    else if(F_TopMode == true)
    {ChModeControl_FreeMode_RCControl_TopHandler(&GSTCH_Data, &GST_RMCtrl);} //小陀螺处理函数

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}



/**
  * @brief  遥控器模式下，缓慢坐下模式控制函数
  * @note   缓慢坐下模式下的控制策略
  * @param  无
  * @retval 无
*/
void ChModeControl_SlowSitDownMode_RCControl(void) {
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_SlowSitDown < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rSlowSitDown);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rSlowSitDown);   //右腿长TD系数r赋值
    }

    Chassis_DisFBClear();           //底盘位移反馈值清零

    /*配置默认可配置的控制量*/
    float YawAngleVelDes_Pre = GSTCH_Data.YawAngleVelDes; //目标偏航角速度上次值
    float Leg1FFForce_Pre = GSTCH_Data.Leg1ForceDes; //左腿前馈力上次值
    float Leg2FFForce_Pre = GSTCH_Data.Leg2ForceDes; //右腿前馈力上次值

    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMin;        //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMin;        //右腿目标腿长
    GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;             //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = StepChangeValue(YawAngleVelDes_Pre, 0.0f, SlowSitDown_YawAngleVelBrakeStep);  //目标偏航角速度
    GST_RMCtrl.STCH_Default.Leg1FFForce     = StepChangeValue(Leg1FFForce_Pre, LegFFForce_SlowSitDown, SlowSitDown_LegFFForceDecStep);  //左腿前馈力
    GST_RMCtrl.STCH_Default.Leg2FFForce     = StepChangeValue(Leg2FFForce_Pre, LegFFForce_SlowSitDown, SlowSitDown_LegFFForceDecStep);  //右腿前馈力

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
    //! 在模式切换函数中有个腿长判断函数位置，在那里实现的腿长小于一个值就切换到坐下模式 
}

/**
 * @brief  遥控器模式下，跟随模式控制函数
 * @note   跟随模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_FollowMode_RCControl(void) {
    /****************************该模式的前置处理****************************/
    // if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_Follow < CHMode_AllMode_PreProcessTime)
    // {
    //     /*腿长PID系数赋值*/
    //     PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
    //     PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

    //     /*腿长TD系数赋值*/
    //     TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);   //左腿长TD系数r赋值
    //     TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);   //右腿长TD系数r赋值
    // }

    // /* 使用手动控制的三档高度 */
    // GST_RMCtrl.STCH_Default.LegLen1Des = GST_RMCtrl.STCH_Default.LegLen1ManualDes;
    // GST_RMCtrl.STCH_Default.LegLen2Des = GST_RMCtrl.STCH_Default.LegLen2ManualDes;
    //待补充：Follow模式的控制策略

// 	else if( g_stDBus.stRC.SW_L == RC_SW_UP && abs(X_Offset) <= RC_CH_VALUE_CHASSIS_DEAD )
// 	{
// 		Chassis_Follow();   //底盘跟随模式
// 		LQR_Yaw_Delta = Clip(Angle_Inf_To_180(Benjamin_Position),-90,90);
// 		Turn_Flag = false;
// 	}

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    // CH_MotionUpdateAndProcess(GST_RMCtrl);
}


/**
 * @brief  遥控器模式下，离地模式控制函数
 * @note   离地模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_OffGroundMode_RCControl(void) {
    // TODO 腿部前馈力要不要变？PID、TD参数要不要变？腿长是否需要考虑一下变化？
    // FIXME 现在离地后如果底盘Pitch过大可能会腿部抽搐，考虑一下怎么解决
    
    /****************************该模式的前置处理****************************/
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_OffGround < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);   //右腿长TD系数r赋值
        
        /*配置默认可配置的控制量*/
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力
    };

    /*************************任务开始一段时间后*************************/
    GST_RMCtrl.STCH_Default.DisDes          = GSTCH_Data.DisFB; //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;             //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度

    /*左腿目标腿长*/
    if(GSTCH_Data.F_OffGround1 == true)
    {GST_RMCtrl.STCH_Default.LegLen1Des  = LegLenOffGround;}
    else
    {GST_RMCtrl.STCH_Default.LegLen1Des  = LegLenMid;}

    /*右腿目标腿长*/
    if(GSTCH_Data.F_OffGround2 == true)
    {GST_RMCtrl.STCH_Default.LegLen2Des  = LegLenOffGround;}
    else
    {GST_RMCtrl.STCH_Default.LegLen2Des  = LegLenMid;}

    GST_RMCtrl.STCH_Force.Leg1FDes = LegFFForce_OffGround;         //左腿力目标值
    GST_RMCtrl.STCH_Force.Leg1TDes = GSTCH_Data.Leg1TorqueDes;     //左腿力矩目标值
    GST_RMCtrl.STCH_Force.Leg2FDes = LegFFForce_OffGround;         //右腿力目标值
    GST_RMCtrl.STCH_Force.Leg2TDes = GSTCH_Data.Leg2TorqueDes;     //右腿力矩目标值

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}


/**
 * @brief  遥控器模式下，脱困模式控制函数
 * @note   脱困模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_StruggleMode_RCControl(void) {
    //* 利用static变量保存进入模式时的偏航角度
    static float YawAngle_Ref = 0.0f;
    /****************************该模式的前置处理****************************/
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_Struggle < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);   //右腿长TD系数r赋值
        
        Chassis_DisFBClear();                         //底盘位移反馈值清零

        /*配置默认可配置的控制量*/
        GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
        // TODO 如果是从Follow切换过来，速度是不是应该保持不变呢？
        GST_RMCtrl.STCH_Default.VelDes          = 0.0f;             //目标速度
        GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
        GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静止时的默认值）
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静止时的默认值）

        /*获取当前回转角度*/
        YawAngle_Ref = GSTCH_Data.YawAngleFB; //当前偏航角度反馈值
    };

    /*配置默认可配置的控制量*/
    // 放在外面才能实现实时控制
    GST_RMCtrl.STCH_Default.LegLen1Des      = GST_RMCtrl.STCH_Default.LegLen1ManualDes; //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = GST_RMCtrl.STCH_Default.LegLen2ManualDes; //右腿目标腿长
    
    /*************************任务开始一段时间后*************************/
    // 处于脱困模式不能进入小陀螺状态
    ChModeControl_FreeMode_RCControl_MoveHandler(&GSTCH_Data, &GST_RMCtrl);

    /*基于速度目标值计算角度误差*/
    float YawAngle_Now = GSTCH_Data.YawAngleFB;                                  //当前偏航角度反馈值
    float YawDelta_RC = GST_RMCtrl.STCH_Default.YawAngleVelDes * GCH_TaskTime;   //遥控器目标偏航角度
    float YawDelta_Hold = YawAngle_Now - YawAngle_Ref;                           //保持偏航角度变化值
    float YawDelta_Err = YawDelta_RC - YawDelta_Hold;                            //偏航角度误差值  
    
    // 计算状态强度系数
    float HM1_ErrOverDead = MyAbsf(GSTCH_HMTorqueComp.Err_HM1) - GSTCH_HMTorqueComp.Err_DZ;
    float HM1_IntensitySpan = GSTCH_HMTorqueComp.Err_Sat - GSTCH_HMTorqueComp.Err_DZ;
    float HM1_StatusIntensity = HM1_ErrOverDead / HM1_IntensitySpan; // 归一化到0~1之间的值
    GSTCH_HMTorqueComp.Lambda_HM1 = Limit(HM1_StatusIntensity, 0.0f, 1.0f); // 归一化到0~1之间的值

    float HM2_ErrOverDead = MyAbsf(GSTCH_HMTorqueComp.Err_HM2) - GSTCH_HMTorqueComp.Err_DZ;
    float HM2_IntensitySpan = GSTCH_HMTorqueComp.Err_Sat - GSTCH_HMTorqueComp.Err_DZ;
    float HM2_StatusIntensity = HM2_ErrOverDead / HM2_IntensitySpan; // 归一化到0~1之间的值
    GSTCH_HMTorqueComp.Lambda_HM2 = Limit(HM2_StatusIntensity, 0.0f, 1.0f); // 归一化到0~1之间的值

    float HM1_WeightSlipDec = GSTCH_HMTorqueComp.Max_HM_Comp_Ratio * GSTCH_Data.F_SlipHM1 * GSTCH_HMTorqueComp.Lambda_HM1;
    float HM1_WeightBlockDec = GSTCH_HMTorqueComp.Max_HM_Comp_Ratio * GSTCH_Data.F_BlockHM1 * GSTCH_HMTorqueComp.Lambda_HM1;

    float HM2_WeightSlipDec = GSTCH_HMTorqueComp.Max_HM_Comp_Ratio * GSTCH_Data.F_SlipHM2 * GSTCH_HMTorqueComp.Lambda_HM2;
    float HM2_WeightBlockDec = GSTCH_HMTorqueComp.Max_HM_Comp_Ratio * GSTCH_Data.F_BlockHM2 * GSTCH_HMTorqueComp.Lambda_HM2;
    
    GSTCH_HMTorqueComp.Weight_HM1 = 1.0f - GSTCH_HMTorqueComp.Lambda_HM1 


    // 先都在这个地方写吧，目前没想好写在哪里

    // GSTCH_HMTorqueComp.Lambda_HM1 = 


    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

// #pragma endregion

//* 模式控制最终执行函数。根据当前模式变量的变量值执行对应的模式具体功能实现函数
/**
 * @brief  遥控器模式下，底盘模式控制函数
 * @note   根据当前底盘的工作状态，调用相应的控制策略函数
 * @param  ModeNow：ChassisMode_EnumTypeDef类型的枚举值，当前底盘的工作状态
 * @retval 无
 */
void ChassisModeControl_RCControl(ChassisMode_EnumTypeDef ModeNow) {
    switch (ModeNow) {
        /*RC手动安全模式*/
        case CHMode_RC_ManualSafe:
            ChModeControl_ManualSafeMode_RCControl();
            break;

        /*RC自动安全模式*/
        case CHMode_RC_AutoSafe:
            ChModeControl_AutoSafeMode_RCControl();
            break;

        /*RC坐下模式*/
        case CHMode_RC_Sitting:
            ChModeControl_SittingMode_RCControl();
            break;

        /*RC起立模式*/
        case CHMode_RC_StandUp:
            ChModeControl_StandUpMode_RCControl();
            break;

        /*RC底盘自由模式*/
        case CHMode_RC_Free:
            ChModeControl_FreeMode_RCControl();
            break;

        /*RC底盘缓慢坐下模式*/
        case CHMode_RC_SlowSitDown:
            ChModeControl_SlowSitDownMode_RCControl();
            break;

        /*RC底盘跟随模式*/
        case CHMode_RC_Follow:
            ChModeControl_FollowMode_RCControl();
            break;

        /*RC离地模式*/
        case CHMode_RC_OffGround:
            ChModeControl_OffGroundMode_RCControl();
            break;

        /*RC脱困模式*/
        case CHMode_RC_Struggle:
            ChModeControl_StruggleMode_RCControl();
            break;
    }
}
