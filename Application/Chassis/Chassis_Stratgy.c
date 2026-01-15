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

float PID_RollComp_Kp_tmp = 0.0f;  // Roll补偿PID临时Kp变量
float PID_RollComp_Ki_tmp = 0.0f;  // Roll补偿PID临时Ki变量
float PID_RollComp_Kd_tmp = 0.0f;  // Roll补偿PID临时Kd变量

float TD_YawAngle_r_tmp = 0.0f;      // Yaw角TD临时速度因子变量
float TD_YawAngle_h0_tmp = 0.0f;     // Yaw角TD临时滤波因子变量

// // #pragma region 检测当前条件是否满足进入某模式的函数
//* 目前包括七个模式的判断函数：
//* 1. 手动安全模式：_ChIsEnter_ManualSafeMode_RCControl
//* 2. 自动安全模式：_ChIsEnter_AutoSafeMode_RCControl
//* 3. 起立模式：_ChIsEnter_StandUpMode_RCControl
// ==================================================
//* 4. 坐下模式：_ChIsEnter_SittingMode_RCControl
//* 5. 缓慢坐下模式：_ChIsEnter_SlowSitDownMode_RCControl
//* 6. 自由模式：_ChIsEnter_FreeMode_RCControl
//* 7. 跟随模式：_ChIsEnter_FollowMode_RCControl
//* 8. 离地模式：_ChIsEnter_OffGroundMode_RCControl

/**
 * @brief  遥控器模式下，检测是否需要进入手动安全模式的函数
 * @note   在函数内部会进行判断，如果达到其中任何一个条件就会进入手动安全模式
 * @param  无
 * @retval false：所有判断条件都正常，不需要进入ManualSafeMode
 *         true：任何一项判断条件生效，需要进入ManualSafeMode
 */
bool _ChIsEnter_ManualSafeMode_RCControl(void) {
    /* 如果遥控器断开连接，不进入ManualSafeMode（应该进入AutoSafe） */
    if (IsRCConnected() == false) {
        return false;
    }

    /* 如果遥控器拨杆为左下右上，进入ManualSafeMode */
    if (IsLeftLevelDown() && IsRightLevelUp()) {
        return true;
    }

    /*其他情况不进入ManualSafeMode*/
    return false;
}

/**
 * @brief  遥控器模式下，检测是否需要进入自动安全模式的函数
 * @note
 * 在函数内部会进行一系列判断，如果达到其中任何一个条件就会进入自动安全模式
 * @param
 * ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
 * @retval false：所有判断条件都正常，不需要进入AutoSafeMode
 *         true：任何一项判断条件生效，需要进入AutoSafeMode
 */
bool _ChIsEnter_AutoSafeMode_RCControl(
    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    /*用一些临时变量来存储相关变量*/
    float HM1_Rx_fps =
        ST_ModeChoosePara.MC_HubMotor1Rx_fps;  // 轮毂电机1通讯帧率
    float HM2_Rx_fps =
        ST_ModeChoosePara.MC_HubMotor2Rx_fps;  // 轮毂电机2通讯帧率
    float UART4_Rx_fps =
        ST_ModeChoosePara.MC_UART4Rx_fps;  // 串口4，即IMU2通讯帧率

    /* 如果遥控器断开连接，进入AutoSafeMode */
    if (IsRCConnected() == false) {
        return true;
    }

    /* 如果轮毂电机通讯异常，进入AutoSafeMode */
    if (HM1_Rx_fps < HubMotorRx_fpsMinTH || HM2_Rx_fps < HubMotorRx_fpsMinTH) {
        return true;
    }

    /* 如果IMU2通讯异常，进入AutoSafeMode */
    if (UART4_Rx_fps < UART4Rx_fpsMinTH) {
        return true;
    }

    // 老代码说明：
    // 待优化：考虑在保护中加入腿部摆角的判断
    //  if(
    //  				((/* capacitor_msg.CAP_Vol < 13 ||
    //  */capacitor_msg.CAP_Vol> 55)|| // 超电保护
    //                  ((fabs(G_ST_IMU2.Receive.pitch_angle -
    //                  Med_Angle_Norm)>=20.0f ||
    //                  (fabs(-LegLeftPosDistribute.Theta)>28.0f &&
    //                  fabs(-LegRightPosDistribute.Theta)>28.0f &&
    //                  fabs(G_ST_IMU2.Receive.pitch_angle -
    //                  Med_Angle_Norm)>=8.0f)) && Height_Choose_Cnt == 0)||
    //  				(fabs(G_ST_IMU2.Receive.pitch_angle -
    //  Med_Angle_Norm)>=18.0f && Height_Choose_Cnt == 1)||
    //                  stuck_flag || stuck_flag_still || stair_lay_flag)
    //  				&& Speed_Flag &&  (OS_TIME() - Start_Time_Cnt )> 800000
    //  && OffGround_Flag == 0 && Up_Stair_Flag==0) {return true;}

    // if((stuck_flag_start && OS_TIME()-Start_Time_Cnt > 100000 &&
    // OS_TIME()-Start_Time_Cnt < 800000 && Speed_Flag) == true) {return true;}

    /*如果全部判断条件正常，不进入AutoSafeMode*/
    return false;
}

///**
// * @brief  遥控器模式下，检测是否需要进入坐下模式的函数
// * @note 在函数内部会进行一系列判断，如果达到其中任何一个条件就会进入坐下模式
// * @param
// * ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
// * @retval false：不需要进入SitDownMode
// *         true： 需要进入SitDownMode
// */
// bool _ChIsEnter_SittingMode_RCControl(
//    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {}

/**
 * @brief  遥控器模式下，检测是否需要进入起立模式的函数
 * @param
 * ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
 * @retval false：不进入StandUpMode
 *         true： 进入StandUpMode
 */
bool _ChIsEnter_StandUpMode_RCControl(
    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    /*用一些临时变量来存储相关变量*/
    // float ModePre = ST_ModeChoosePara.MC_ModePre;  // 上次模式
    // float TimeNow = ST_ModeChoosePara.MC_TimeNow;  // 当前时间
    // float ThisModeStartTime =
    //     ST_ModeChoosePara.MC_ST_ModeStartTime.RC_StandUp;  //
    //     起立模式开始时间
    // float ThisModeTotalTime =
    //     TimeNow -
    //     ThisModeStartTime;  // 起立模式总时长 = 当前时间 - 起立模式开始时间

    /*如果左拨杆不在下，进入起立模式*/
    if (IsLeftLevelDown() == false) {
        return true;
    }
    // TODO 在加入其他模式时会涉及到不同运动模式之间的关系，起立和坐下这些关系
    /*其他情况，不进入*/
    return false;
}

///**
// * @brief  遥控器模式下，检测是否需要进入底盘自由模式的函数
// * @note   待补充
// * @param
// * ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
// * @retval false：不进入FreeMode
// *         true： 进入FreeMode
// */
// bool _ChIsEnter_FreeMode_RCControl(
//    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {}

///**
// * @brief  遥控器模式下，检测是否需要进入缓慢坐下模式的函数
// * @note   待补充
// * @param
// * ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
// * @retval false：不进入SlowSitDownMode
// *         true： 进入SlowSitDownMode
// */
// bool _ChIsEnter_SlowSitDownMode_RCControl(
//    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {}

///**
// * @brief  遥控器模式下，检测是否需要进入底盘跟随模式的函数
// * @note   待补充
// * @param
// * ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
// * @retval false：不进入FollowMode
// *         true： 进入FollowMode
// */
// bool _ChIsEnter_FollowMode_RCControl(
//    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {}

///**
// * @brief  遥控器模式下，检测是否需要进入离地模式的函数
// * @note   待补充
// * @param
// * ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
// * @retval false：不进入OffGroundMode
// *         true： 进入OffGroundMode
// */
// bool _ChIsEnter_OffGroundMode_RCControl(
//    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {}

// // #pragma endregion

//* 底盘模式更新函数。当检测模式条件函数返回真值时将当前模式变量变到对应变量值
/**
 * @brief  遥控器模式下，底盘模式切换函数
 * @note
 * 根据不同的条件切换底盘的模式，要注意优先级的问题，最高优先级和次高优先级分别是手动安全模式和自动安全模式
 * 后面要添加的其他状态切换，比如跳跃、离地等等，添加时要注意优先级的顺序
 * @param
 * ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
 * @retval ChassisMode_EnumTypeDef的枚举类型，底盘的工作状态
 */
ChassisMode_EnumTypeDef ChassisModeChoose_RCControl(
    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    ChassisMode_EnumTypeDef ModeTemp =
        CHMode_RC_AutoSafe;  // 底盘状态，默认进入RC自动安全模式，如果遥控器有指令会进行修改

    /************切换底盘的状态************/
    /*注意！！为了更加方便管理，下面的模式判断选择中，只使用ST_ModeChoosePara里的变量、遥控器相关函数来做判断*/
    /*如果想要用其他的变量来做判断，可以在Chassis_ModeChooseParameter_StructTypeDef里面添加变量*/
    /*然后在CH_ModeChooseParaStructUpdate中添加该变量的更新语句，最后在这里调用*/

    // XXX 优先级最高的是ManualSafe，保证了进入ManualSafe之后不会切换到其他模式
    if (_ChIsEnter_ManualSafeMode_RCControl() == true) {
        ModeTemp = CHMode_RC_ManualSafe;
    }  // 进入RC手动安全模式

    else if (_ChIsEnter_AutoSafeMode_RCControl(ST_ModeChoosePara) == true) {
        ModeTemp = CHMode_RC_AutoSafe;
    }  // 进入RC自动安全模式

    else if (_ChIsEnter_StandUpMode_RCControl(ST_ModeChoosePara) == true) {
        ModeTemp = CHMode_RC_StandUp;
    }  // 左拨杆不在下，进入RC站立模式

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
void ChassisStratgy_ModeStartTimeUpdate(CHData_StructTypeDef* pCHData,
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
        // case CHMode_RC_Sitting:
        //     pCHData->ST_ModeStartTime.RC_Sitting = RunTimeGet();
        //     break;
        case CHMode_RC_Free:
            pCHData->ST_ModeStartTime.RC_Free = RunTimeGet();
            break;
    }
}

// // #pragma region 模式具体功能实现函数
//* 目前包括七个模式的具体功能实现函数：
//* 1. 手动安全模式：ChModeControl_ManualSafeMode_RCControl
//* 2. 自动安全模式：ChModeControl_AutoSafeMode_RCControl
//* 3. 起立模式：ChModeControl_StandUpMode_RCControl
// =================================================
//* 4. 坐下模式：ChModeControl_SittingMode_RCControl
//* 5. 缓慢坐下模式：ChModeControl_SlowSitDownMode
//* 6. 自由模式：ChModeControl_FreeMode_RCControl
//* 7. 跟随模式：ChModeControl_FollowMode_RCControl

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

    // TODO 目前车的零点是固定的，不用进行手动标定
    // /*判断是否要手动标定关节电机零点位置*/
    // GFCH_LegCalibration = IsEnterManualCalibration();
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
 * @brief  遥控器模式下，起立模式控制函数
 * @note   起立模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_StandUpMode_RCControl(void) {
    static float YawAngleDes_tmp = 0.0f;
    static float DisDes_tmp = 0.0f;

    //! **腿长控制器部分 */ 
    //? 腿长TD部分
    //* 腿长TD系数r设定
    TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rStandUp);  // 左腿长TD系数r赋值
    TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rStandUp);  // 右腿长TD系数r赋值
    //* 腿长TD目标值设定和计算
    TD_SetInput(&GstCH_LegLen1TD, LegLen_StandUp_Practice);
    TD_SetInput(&GstCH_LegLen2TD, LegLen_StandUp_Practice);
    TD_Cal(&GstCH_LegLen1TD);
    TD_Cal(&GstCH_LegLen2TD);
    //? 腿长PID部分  
    //* 腿长PID目标值和系数设定
    PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpStandUp, 0.0f,
                  PID_LegLen_KdStandUp);  // 左腿长PID系数Kp、Ki、Kd赋值
    PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpStandUp, 0.0f,
                  PID_LegLen_KdStandUp);  // 右腿长PID系数Kp、Ki、Kd赋值
    //* 腿长PID目标值和反馈值设定和计算
    GSTCH_Data.LegLen1Des = TD_GetOutput(&GstCH_LegLen1TD);
    GSTCH_Data.LegLen2Des = TD_GetOutput(&GstCH_LegLen2TD);
    PID_SetDes(&GstCH_LegLen1PID, GSTCH_Data.LegLen1Des);
    PID_SetDes(&GstCH_LegLen2PID, GSTCH_Data.LegLen2Des);
    PID_SetFB(&GstCH_LegLen1PID, GSTCH_Data.LegLen1FB);
    PID_SetFB(&GstCH_LegLen2PID, GSTCH_Data.LegLen2FB);
    PID_Cal(&GstCH_LegLen1PID);
    PID_Cal(&GstCH_LegLen2PID);

    //! **横滚角控制器部分 */
    // 测出来的PID：KP 20 KI 0 KD 1000
    PID_SetKpKiKd(&GstCH_RollCompPID, 20.0f, 0.0f, 1000.0f);  // 横滚角补偿PID系数Kp、Ki、Kd赋值
    PID_SetDes(&GstCH_RollCompPID, 0.0f);  // Roll补偿PID目标值设定为0
    PID_SetFB(&GstCH_RollCompPID, GSTCH_Data.RollAngleFB);  // Roll补偿PID反馈值设定
    PID_Cal(&GstCH_RollCompPID);  // Roll补偿PID计算

    //! 支持力计算部分
    /*
     * 物理公式 (3.8):
     * [F_bl,l]   [ 1  1  1 -1] [F_psi         ]
     * [      ] = [           ] [F_l           ]
     * [F_bl,r]   [-1  1  1  1] [F_bl,gravity  ]
     *                          [F_bl,inertial ]
     *
     * M_eff = 0.5 * m_b + η_l * m_l
     * InertialCoeff = M_eff / (2 * R_l)
     *
     * F_bl,gravity = M_eff * g
     * F_bl,inertial = InertialCoeff * l_current * YawRate * v_forward
     *
     * F_psi：滚转角 PID 控制器的输出。
     * F_l：腿长 PID 控制器的输出。
     *
     * 注意符号：
     * 1. F_psi 对左右腿作用相反（产生力矩）
     * 2. F_bl,inertial 同样对左右腿作用相反（抵抗由前进及旋转运动产生的侧倾）
     * 3. F_l 和 F_bl,gravity 对双腿作用相同。
     */

    //* 前馈力计算部分
    GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1 - LegFFForce_Inertial_1;  // 左腿前馈力
    GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2 + LegFFForce_Inertial_2;  // 右腿前馈力

    //* 主动力计算部分 
    float LegLenPIDForce_Norm_1 =
        PID_GetOutput(&GstCH_LegLen1PID);  // 左腿腿长PID计算力 (N)
    float LegLenPIDForce_Norm_2 =
        PID_GetOutput(&GstCH_LegLen2PID);  // 右腿腿长PID计算力 (N)

    float RollCompForce_Norm_1 =
        PID_GetOutput(&GstCH_RollCompPID);  // 横滚角补偿PID计算力 (N)，左腿
    float RollCompForce_Norm_2 =
        PID_GetOutput(&GstCH_RollCompPID);  // 横滚角补偿PID计算力 (N)，右腿

    GSTCH_Data.Leg1ForceDes =
        GST_RMCtrl.STCH_Default.Leg1FFForce + LegLenPIDForce_Norm_1 + RollCompForce_Norm_1;
    GSTCH_Data.Leg2ForceDes =
        GST_RMCtrl.STCH_Default.Leg2FFForce + LegLenPIDForce_Norm_2 - RollCompForce_Norm_2;

    //! **前后移动遥控器控制部分 */ 
    //* 在这里面将传来的信号映射到了[-1, 1]
    // float StickX_Norm =
    //     RCChannelToNorm(GST_Receiver.ST_RC.JoyStickL_X);  // 左负右正
    float StickY_Norm =
        RCChannelToNorm(GST_Receiver.ST_RC.JoyStickL_Y);  // 下负上正

    //* 正常运行的速度是1.8m/s，摆杆角度越大越接近1m/s
    float DisVelDes_tmp = StickY_Norm * ChMove_VelDesMax;
    DisDes_tmp += DisVelDes_tmp * GCH_TaskTime;

    //! **左右转弯遥控器控制部分 */ 
    if (IsLeftJoyStickLeft()==true){
        float YawAngleStep_tmp = 1000;
        YawAngleDes_tmp += YawAngleStep_tmp * GCH_TaskTime;
    }

    TD_SetInput(&GstCH_YawAngleTD, YawAngleDes_tmp);
    GstCH_YawAngleTD.r = TD_YawAngle_r_tmp;
    GstCH_YawAngleTD.h0 = TD_YawAngle_h0_tmp;
    GstCH_YawAngleTD.SampleTime = GCH_TaskTime;
    TD_Cal(&GstCH_YawAngleTD);

    float YawAngleVelFollow = GstCH_YawAngleTD.x2;  // 目标偏航角速度跟随值

    //! 目标值赋值
    GSTCH_Data.DisDes = DisDes_tmp;     // 目标位移
    GSTCH_Data.VelDes = DisVelDes_tmp;  // 目标速度
    //* 我期望的是正转或者反转一定的角度，所以控制的是相对角度而不是绝对角度
    GSTCH_Data.YawDeltaDes = YawAngleDes_tmp;                 // 目标偏航角度
    GSTCH_Data.YawAngleVelDes = YawAngleVelFollow;           // 目标偏航角速度跟随值

    //* 其他目标值 
    GSTCH_Data.PitchAngleDes = 0.0f;                          // 目标俯仰角度
    GSTCH_Data.PitchAngleVelDes = 0.0f;                       // 目标俯仰角速度


    //! 数据预清零
    //* 下面的函数只会在模式切换的前四毫秒执行，会进行数据清除、参数设置等操作
    if (RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_StandUp <
        CHMode_AllMode_PreProcessTime) {
        /*保证再次进入起立模式时，目标值重新归零*/
        YawAngleDes_tmp = 0.0f;
        DisDes_tmp = 0.0f;
        
        /*清空底盘相关的一些数据*/
        Chassis_AllDesDataReset();     // 清空底盘相关的Des数据
        Chassis_DisFBClear();          // 底盘位移反馈值清零
        Chassis_RobotCtrlDataReset();  // 底盘控制数据清零
        return;
    }
    /********************任务开始一段时间后的调整，时间的单位是ms********************/

    //! LQR控制部分
    CH_LQRCal_Process();  // 调用LQR状态空间计算平衡所需的力矩
    //* u已经计算出来了（包括轮子转动的扭矩）

    /* 1. 将 LQR 计算结果分发到轮毂电机和 VMC 扭矩目标值 */
    GSTCH_HM1.TorqueDes = -LQR_Get_uVector(&GstCH_LQRCal, 0);  // 轮毂电机1扭矩 (L)
    GSTCH_HM2.TorqueDes =  LQR_Get_uVector(&GstCH_LQRCal, 1);  // 轮毂电机2扭矩 (R) （右轮镜像安装加负号）
    GSTCH_Data.Leg1TorqueDes = LQR_Get_uVector(&GstCH_LQRCal, 2);  // 虚拟摆杆力矩1
    GSTCH_Data.Leg2TorqueDes = LQR_Get_uVector(&GstCH_LQRCal, 3);  // 虚拟摆杆力矩2

    //! VMC控制部分 
    /* 4. 执行 VMC 分解到关节轴力矩 */
    CH_VMCCal_Process();

    //! 电机输出值设定部分 
    CH_HMTorqueToCurrent_Process(&GSTCH_HM1);
    CH_HMTorqueToCurrent_Process(&GSTCH_HM2);

    /* 映射关节电机：将 VMC 计算出的关节力矩分发到正式结构体 */
    GSTCH_JM3.TorqueDes = GstCH_Leg1VMC.T_Matrix[0];  // 左腿后关节 (phi1)
    GSTCH_JM1.TorqueDes = GstCH_Leg1VMC.T_Matrix[1];  // 左腿前关节 (phi4)
    GSTCH_JM2.TorqueDes = GstCH_Leg2VMC.T_Matrix[0];  // 右腿前关节 (phi1)
    GSTCH_JM4.TorqueDes = GstCH_Leg2VMC.T_Matrix[1];  // 右腿后关节 (phi4)

    // 数据分发完成，后续由 SendDataTask 统一通过 CAN 和 USART 发送给电机
    // SendDataTask 会自动周期执行。
}

// /**
//  * @brief  遥控器模式下，坐下模式控制函数
//  * @note   坐下模式下的控制策略
//  * @param  无
//  * @retval 无
//  */
// void ChModeControl_SittingMode_RCControl(void) {}

// /**
//  * @brief  遥控器模式下，缓慢坐下模式控制函数
//  * @note   缓慢坐下模式下的控制策略
//  * @param  无
//  * @retval 无
//  */
// void ChModeControl_SlowSitDownMode_RCControl(void) {}

// /**
//  * @brief  遥控器模式下，自由模式控制函数
//  * @note   自由模式下的控制策略
//  * @param  无
//  * @retval 无
//  */
// void ChModeControl_FreeMode_RCControl(void) {}

// /**
//  * @brief  遥控器模式下，跟随模式控制函数
//  * @note   跟随模式下的控制策略
//  * @param  无
//  * @retval 无
//  */
// void ChModeControl_FollowMode_RCControl(void) {}

// /**
//  * @brief  遥控器模式下，离地模式控制函数
//  * @note   离地模式下的控制策略
//  * @param  无
//  * @retval 无
//  */
// void ChModeControl_OffGroundMode_RCControl(void) {}
// // #pragma endregion

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

        /*RC起立模式*/
        case CHMode_RC_StandUp:
            ChModeControl_StandUpMode_RCControl();
            break;

            // /*RC坐下模式*/
            // case CHMode_RC_Sitting:
            //     ChModeControl_SittingMode_RCControl();
            //     break;

//        /*RC自由模式*/
//        case CHMode_RC_Free:
//            ChModeControl_FreeMode_RCControl();
//            break;

            // /*RC底盘缓慢坐下模式*/
            // case CHMode_RC_SlowSitDown:
            //     ChModeControl_SlowSitDownMode_RCControl();
            //     break;

            // /*RC底盘跟随模式*/
            // case CHMode_RC_Follow:
            //     ChModeControl_FollowMode_RCControl();
            //     break;

            // /*RC离地模式*/
            // case CHMode_RC_OffGround:
            //     ChModeControl_OffGroundMode_RCControl();
            //     break;
    }
}
