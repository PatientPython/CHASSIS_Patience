/**
  ******************************************************************************
  * @file    Chassis_Stratgy.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.11.2
  * @brief   底盘控制策略相关函数
  *          把底盘的控制策略函数放在这里，方便外部调用
  ******************************************************************************
*/

#include "General_AuxiliaryFunc.h"
#include "GlobalDeclare_Chassis.h"
#include "Chassis_APIFunction.h"
#include "TIM_Config.h"
#include <arm_math.h>
#include "Algorithm_Simple.h"

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
	//待优化：考虑在保护中加入腿部摆角的判断
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
  * @brief  遥控器模式下，检测是否需要进入坐下模式的函数
  * @note   在函数内部会进行一系列判断，如果达到其中任何一个条件就会进入坐下模式
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval false：不需要进入SitDownMode
  *         true： 需要进入SitDownMode
*/
bool _ChIsEnter_SittingMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    /*用一些临时变量来存储相关变量*/
    float ModePre     = ST_ModeChoosePara.MC_ModePre;      //上次模式
    float LegLenAvgFB = ST_ModeChoosePara.MC_LegLenAvgFB;  //腿长平均值

    /*如果左拨杆不在中位，不进入坐下模式*/
    if(IsLeftLevelMid() == false)
    {return false;}

    /*如果上次状态是ManualSafe、AutoSafe其中一个，允许进入坐下模式*/
    if(ModePre == CHMode_RC_ManualSafe)
    {return true;}
    else if(ModePre == CHMode_RC_AutoSafe)
    {return true;}

    /*如果上次状态是SlowSitDown，且腿长已经到达最小值附近，进入坐下模式*/
    if(ModePre == CHMode_RC_SlowSitDown && LegLenAvgFB <= LegLenMin + LegLenMinTH)
    {return true;}

    /*如果上次状态是Sitting，且拨轮没有上推，继续保持坐下模式*/
    if(ModePre == CHMode_RC_Sitting && IsRollerUp() == false)
    {return true;}

    /*否则不进入*/
    return false;
}

/**
  * @brief  遥控器模式下，检测是否需要进入起立模式的函数
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval false：不进入StandUpMode
  *         true： 进入StandUpMode
*/
bool _ChIsEnter_StandUpMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
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
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval false：不进入FreeMode
  *         true： 进入FreeMode
*/
bool _ChIsEnter_FreeMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    /*用一些临时变量来存储相关变量*/
    float ModePre = ST_ModeChoosePara.MC_ModePre;  //上次模式

    /*如果左拨杆不在中位，不进入*/
    if(IsLeftLevelMid() == false)
    {return false;}

    /*如果拨轮上推，不进入（应该进入SlowSitDown）*/
    if(IsRollerUp() == true)
    {return false;}

    /*如果上次模式是Free，允许进入*/
    if(ModePre == CHMode_RC_Free)
    {return true;}

    /*如果上次模式是StandUp，允许进入*/
    if(ModePre == CHMode_RC_StandUp)
    {return true;}

    //待修改：写到Follow模式的时候再补充
    // /*如果上次状态是Follow，允许进入*/
    // if(ST_ModeChoosePara.ModePre == CHMode_RC_Follow)
    // {return true;}

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
bool _ChIsEnter_FollowMode_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    //待补充（记得把StandUp模式加进去）

    return false;
}

/**
  * @brief  遥控器模式下，底盘模式切换函数
  * @note   根据不同的条件切换底盘的模式，要注意优先级的问题，最高优先级和次高优先级分别是手动安全模式和自动安全模式
  *         后面要添加的其他状态切换，比如跳跃、离地等等，添加时要注意优先级的顺序
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval ChassisMode_EnumTypeDef的枚举类型，底盘的工作状态
*/
ChassisMode_EnumTypeDef ChassisModeChoose_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    ChassisMode_EnumTypeDef ModeTemp = CHMode_RC_AutoSafe; //底盘状态，默认进入RC自动安全模式

    /************切换底盘的状态************/
    /*注意！！为了更加方便管理，下面的模式判断选择中，只使用ST_ModeChoosePara里的变量、遥控器相关函数来做判断*/
    /*如果想要用其他的变量来做判断，可以在Chassis_ModeChooseParameter_StructTypeDef里面添加变量*/
    /*然后在CH_ModeChooseParaStructUpdate中添加该变量的更新语句，最后在这里调用*/
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

    return ModeTemp;
}

/**
  * @brief  获取底盘各模式开始时间的函数
  * @note   在任务切换时，获取当前任务的开始时间，方便在模式控制函数中使用
  *         后续如果新写了模式，也可以在这里添加对应的获取开始时间表达式
  * @param  pCHData：CHData_StructTypeDef类型的指针，底盘数据结构体指针
  * @param  Mode：ChassisMode_EnumTypeDef类型的枚举值，底盘当前的工作状态
  * @param  ModePre：ChassisMode_EnumTypeDef类型的枚举值，底盘上次的工作状态
  * @retval 无
*/
void ChassisMode_GetModeStartTime(CHData_StructTypeDef* pCHData, ChassisMode_EnumTypeDef Mode, ChassisMode_EnumTypeDef ModePre)
{
    /*如果模式没有切换，直接返回*/
    if(Mode == ModePre)
    {return;}

    /*如果模式切换，记录当前时间为该模式的开始时间*/
    switch (Mode)
    {
        case CHMode_RC_ManualSafe:
            pCHData->ST_ModeStartTime.RC_ManualSafe = RunTimeGet();
            break;

        case CHMode_RC_AutoSafe:
            pCHData->ST_ModeStartTime.RC_AutoSafe = RunTimeGet();
            break;

        case CHMode_RC_Sitting:
            pCHData->ST_ModeStartTime.RC_Sitting = RunTimeGet();
            break;

        case CHMode_RC_StandUp:
            pCHData->ST_ModeStartTime.RC_StandUp = RunTimeGet();
            break;
        
        case CHMode_RC_SlowSitDown:
            pCHData->ST_ModeStartTime.RC_SlowSitDown = RunTimeGet();
            break;

        case CHMode_RC_Free:
            pCHData->ST_ModeStartTime.RC_Free = RunTimeGet();
            break;
        
        case CHMode_RC_Follow:
            pCHData->ST_ModeStartTime.RC_Follow = RunTimeGet();
            break;
    }
}

/**
  * @brief  遥控器模式下，手动安全模式控制函数
  * @note   手动安全模式下的控制策略
  * @param  无
  * @retval 无
*/
void ChModeControl_ManualSafeMode_RCControl(void)
{
    /*清空底盘相关的Des数据、位移、控制数据*/
    Chassis_AllDesDataReset();      //清空底盘相关的Des数据
    Chassis_DisFBClear();           //底盘位移反馈值清零
    Chassis_RobotCtrlDataReset();   //底盘控制数据清零

    /*判断是否要手动标定关节电机零点位置*/
    GFCH_LegCalibration = IsEnterManualCalibration();
}

/**
  * @brief  遥控器模式下，自动安全模式控制函数
  * @note   自动安全模式下的控制策略
  * @param  无
  * @retval 无
*/
void ChModeControl_AutoSafeMode_RCControl(void)
{
    /*清空底盘相关的Des数据、位移、控制数据*/
    Chassis_AllDesDataReset();      //清空底盘相关的Des数据
    Chassis_DisFBClear();           //底盘位移反馈值清零
    Chassis_RobotCtrlDataReset();   //底盘控制数据清零
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
void ChModeControl_StandUpMode_RCControl(void)
{
    /****************************该模式的前置处理****************************/
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_StandUp < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpStandUp, 0.0f, PID_LegLen_KdStandUp); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpStandUp, 0.0f, PID_LegLen_KdStandUp); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rStandUp);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rStandUp);   //右腿长TD系数r赋值

        /*清空底盘相关的一些数据*/
        Chassis_AllDesDataReset();      //清空底盘相关的Des数据
        Chassis_DisFBClear();           //底盘位移反馈值清零
        Chassis_RobotCtrlDataReset();   //底盘控制数据清零
        return;
    }

    /********************任务开始一段时间后的调整，时间的单位是ms********************/
    /*配置默认可配置的控制量*/
    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMid;        //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMid;        //右腿目标腿长
    GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;             //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
    GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Norm;  //左腿前馈力
    GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Norm;  //右腿前馈力

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
  * @brief  遥控器模式下，自由模式控制函数
  * @note   自由模式下的控制策略
  * @param  无
  * @retval 无
*/
void ChModeControl_FreeMode_RCControl(void)
{
    /****************************该模式的前置处理****************************/
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_Free < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);   //右腿长TD系数r赋值
        
        Chassis_DisFBClear();           //底盘位移反馈值清零

        /*配置默认可配置的控制量*/
        GST_RMCtrl.STCH_Default.LegLen1Des      = GST_RMCtrl.STCH_Default.LegLen1Des; //左腿目标腿长不变
        GST_RMCtrl.STCH_Default.LegLen2Des      = GST_RMCtrl.STCH_Default.LegLen2Des; //右腿目标腿长不变
        GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
        GST_RMCtrl.STCH_Default.VelDes          = 0.0f;             //目标速度//待考虑：如果是从Follow切换过来，速度是不是应该保持不变呢？
        GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
        GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Norm;  //左腿前馈力
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Norm;  //右腿前馈力
    }

    /*************************任务开始一段时间后*************************/


    /*********************************测试test**********************************************/
    ChModeControl_FreeMode_RCControl_MoveHandler(&GSTCH_Data, &GST_RMCtrl);//移动处理函数，包括平移、转弯的速度获取

    /*********************************测试test*******************************************/

    //#region 待复刻：小陀螺模式
//     void RC_TOPHandler(void)
// {
//     	//小陀螺模式及其后续处理
// 	if(g_stDBus.stRC.SW_L == RC_SW_MID && abs(X_Offset) > RC_CH_VALUE_CHASSIS_DEAD)
// 	{
// 		 Chassis_RC_TOP();
// 		 LQR_Yaw_Delta = 0;
// 		 Turn_Flag = true;
// 	}
//     //这一段是小陀螺慢速回正的代码，RampSignal的第三个参数越大回正越快
//     //暂时未移植到键鼠模式
//     else if(Turn_Flag == true && g_stDBus.stRC.SW_L == RC_SW_MID && abs(X_Offset) <= RC_CH_VALUE_CHASSIS_DEAD)
//     {
//         if(st_velt.fpW >= 0)
//         {
//             RampSignal( &st_velt.fpW , 0.0f , 1.0f );
//         }
//         else if(st_velt.fpW < 0)
//         {
//             RampSignal( &st_velt.fpW , 0.0f , 1.0f );
//         }
        
//         if(fabs(st_velt.fpW) < 1)
//         {
//             Turn_Flag = false;
//         }
//     }
    
// 	else if( g_stDBus.stRC.SW_L == RC_SW_UP && abs(X_Offset) <= RC_CH_VALUE_CHASSIS_DEAD )
// 	{
// 		Chassis_Follow();   //底盘跟随模式
// 		LQR_Yaw_Delta = Clip(Angle_Inf_To_180(Benjamin_Position),-90,90);
// 		Turn_Flag = false;
// 	}
    
// 	else
// 	{
// 		st_velt.fpW = 0;
// 		LQR_Yaw_Delta = 0;
// 		Turn_Flag = false;
// 	}
// }
// #endregion

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
  * @brief  遥控器模式下，缓慢坐下模式控制函数
  * @note   缓慢坐下模式下的控制策略
  * @param  无
  * @retval 无
*/
void ChModeControl_SlowSitDownMode_RCControl(void)
{
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
    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMin;        //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMin;        //右腿目标腿长
    GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;             //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
    GST_RMCtrl.STCH_Default.Leg1FFForce     = StepChangeValue(GST_RMCtrl.STCH_Default.Leg1FFForce, LegFFForce_SlowSitDown, LegFFForce_SlowSitDownStep);  //左腿前馈力
    GST_RMCtrl.STCH_Default.Leg2FFForce     = StepChangeValue(GST_RMCtrl.STCH_Default.Leg2FFForce, LegFFForce_SlowSitDown, LegFFForce_SlowSitDownStep);  //右腿前馈力

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
  * @brief  遥控器模式下，跟随模式控制函数
  * @note   跟随模式下的控制策略
  * @param  无
  * @retval 无
*/
void ChModeControl_FollowMode_RCControl(void)
{
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

    //待补充：Follow模式的控制策略


    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
  * @brief  遥控器模式下，底盘模式控制函数
  * @note   根据当前底盘的工作状态，调用相应的控制策略函数
  * @param  ModeNow：ChassisMode_EnumTypeDef类型的枚举值，当前底盘的工作状态
  * @retval 无
*/
void ChassisModeControl_RCControl(ChassisMode_EnumTypeDef ModeNow)
{
    switch(ModeNow)
    {
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
    }
}

