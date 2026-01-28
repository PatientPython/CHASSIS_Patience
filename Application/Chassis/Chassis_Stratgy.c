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
		
    /*AutoSafe模式专用变量*/
    pModeChoosePara->MC_HubMotor1Rx_fps = GST_SystemMonitor.HubMotor1Rx_fps;    //轮毂电机1接收帧率
    pModeChoosePara->MC_HubMotor2Rx_fps = GST_SystemMonitor.HubMotor2Rx_fps;    //轮毂电机2接收帧率
    pModeChoosePara->MC_UART4Rx_fps     = GST_SystemMonitor.UART4Rx_fps;        //串口4，即IMU2接收帧率
}

// #pragma region 检测当前条件是否满足进入某模式的函数

// ==========================================
// 全局中断 (Global Interrupts)
// ==========================================

// 正常状态 -> ManualSafe: 左拨杆在下且右拨杆在上
bool _IsTrans_Normal_To_ManualSafe(void) {
    /* 如果遥控器拨杆为左下右上，进入ManualSafeMode */
    if(IsLeftLevelDown() && IsRightLevelUp())
    {return true;}
    else
    {return false;} // 否则不进入
}

// 正常状态 -> AutoSafe: 通讯异常
bool _IsTrans_Normal_To_AutoSafe(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
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
    /* 否则不进入 */
    return false;
}

// ==========================================
// 状态恢复 (Recovery)
// ==========================================

// ManualSafe -> Standby: 左拨杆在中
bool _IsTrans_ManualSafe_To_Standby(void) {
    /*如果左拨杆在中位，进入StandbyMode*/
    if(IsLeftLevelMid() == true)
    {return true;}
    else
    {return false;} // 否则不进入
    
}

// AutoSafe -> Standby: 所有通讯正常
bool _IsTrans_AutoSafe_To_Standby(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    // 复用上面的检测，取反
    if(_IsTrans_Normal_To_AutoSafe(ST_ModeChoosePara) == false)
    {return true;} // 通讯正常，进入
    else
    {return false;}  // 通讯异常，不进入
}

// ==========================================
// 核心流程 (Core Flow)
// ==========================================

// Standby -> StandUp: 拨轮向上并回正
bool _IsTrans_Cmd_StandUp(void) {
    if (GSTCH_Data.F_RollerUpLatched == true){
        GSTCH_Data.F_RollerUpLatched = false;
        return true;
    }
    else
    {return false;}
}

// StandUp -> Free: 时间到了
bool _IsTrans_StandUp_To_Free(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    float TimeNow = ST_ModeChoosePara.MC_TimeNow;      //当前时间
    float ThisModeStartTime = ST_ModeChoosePara.MC_ST_ModeStartTime.RC_StandUp; //起立模式开始时间
    float ThisModeTotalTime = TimeNow - ThisModeStartTime;  //起立模式总时长 = 当前时间 - 起立模式开始时间
    // 如果StandUp模式总时间没有到，继续保持StandUp模式
    if(ThisModeTotalTime >= CHMode_RC_StandUp_TotalTime)
    {return true;}
    else
    {return false;}
}

// Free -> Sitting: 拨轮向上并回正
bool _IsTrans_Cmd_SitDown(void) {
    if (GSTCH_Data.F_RollerUpLatched == true){
        GSTCH_Data.F_RollerUpLatched = false;
        return true;
    }
    else
    {return false;}
}

// SitDown -> Standby: 腿长达到最小
bool _IsTrans_SitDown_To_Standby(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    float LegLenAvgFB = ST_ModeChoosePara.MC_LegLenAvgFB;  //腿长平均值
    if(LegLenAvgFB <= LegLenMin + LegLenMinTH)
    {return true;}
    else
    {return false;}
}

// ==========================================
// 模式切换与动作 (Mode Switch & Actions)
// ==========================================

// Free -> Follow: 左拨杆在上
bool _IsTrans_Free_To_Follow(void) {
    return IsLeftLevelUp();
}

// Follow -> Free: 左拨杆在中
bool _IsTrans_Follow_To_Free(void) {
    return IsLeftLevelMid();
}

// Free/Follow -> Jump: 拨轮向下并回正
bool _IsTrans_Cmd_Jump(void) {
    if (GSTCH_Data.F_RollerDownLatched == true){
        GSTCH_Data.F_RollerDownLatched = false;
        return true;
    }
    else
    {return false;}
}

// Free/Follow -> OffGround: 离地标志位成立
bool _IsTrans_To_OffGround(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    return ST_ModeChoosePara.MC_F_OffGround;
}

// OffGround/Jump -> Landed: 检测到落地 (离地标志位清零)
bool _IsTrans_To_Landed(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) {
    return !ST_ModeChoosePara.MC_F_OffGround;
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
    // 状态变量准备
    ChassisMode_EnumTypeDef CurrentMode = ST_ModeChoosePara.MC_ModePre;
    ChassisMode_EnumTypeDef NextMode = CurrentMode;

    // 用于记忆跳跃或者离地前是 Free 还是 Follow（默认是free）
    static ChassisMode_EnumTypeDef LastActiveMode = CHMode_RC_Free; 

    /* ========================================================== */
    /*   Layer 1: 全局中断 (Global Interrupts)                    */
    /* ========================================================== */
    
    if (_IsTrans_Normal_To_ManualSafe()) {
        return CHMode_RC_ManualSafe;
    }

    if (_IsTrans_Normal_To_AutoSafe(ST_ModeChoosePara)) {
        return CHMode_RC_AutoSafe;
    }

    /* ========================================================== */
    /*   Layer 2: 状态机流转 (Switch-Case)                        */
    /* ========================================================== */

    switch (CurrentMode) 
    {
        /* --- 故障与待机 --- */
        case CHMode_RC_ManualSafe:
            if (_IsTrans_ManualSafe_To_Standby()) NextMode = CHMode_RC_Standby;
            break;

        case CHMode_RC_AutoSafe:
            if (_IsTrans_AutoSafe_To_Standby(ST_ModeChoosePara)) NextMode = CHMode_RC_Standby;
            break;

        case CHMode_RC_Standby: 
            if (_IsTrans_Cmd_StandUp()) NextMode = CHMode_RC_StandUp;
            break;

        /* --- 启动与关闭 --- */
        case CHMode_RC_StandUp:
            if (_IsTrans_StandUp_To_Free(ST_ModeChoosePara)) {
                NextMode = CHMode_RC_Free;
                LastActiveMode = CHMode_RC_Free; // 初始化记忆
            }
            break;

        case CHMode_RC_SitDown: 
            if (_IsTrans_SitDown_To_Standby(ST_ModeChoosePara)) NextMode = CHMode_RC_Standby;
            break;

        /* --- 核心运动模式 (Free) --- */
        case CHMode_RC_Free:
            LastActiveMode = CHMode_RC_Free; // 持续更新记忆：我在Free

            if (_IsTrans_Cmd_SitDown())          NextMode = CHMode_RC_SitDown;
            else if (_IsTrans_Free_To_Follow())  NextMode = CHMode_RC_Follow;
            else if (_IsTrans_Cmd_Jump())        NextMode = CHMode_RC_Jump;
            else if (_IsTrans_To_OffGround(ST_ModeChoosePara)) NextMode = CHMode_RC_OffGround;
            break;

        /* --- 核心运动模式 (Follow) --- */
        case CHMode_RC_Follow:
            LastActiveMode = CHMode_RC_Follow; // 持续更新记忆：我在Follow

            if (_IsTrans_Follow_To_Free())       NextMode = CHMode_RC_Free;
            else if (_IsTrans_Cmd_Jump())        NextMode = CHMode_RC_Jump;
            else if (_IsTrans_To_OffGround(ST_ModeChoosePara)) NextMode = CHMode_RC_OffGround;
            break;

        /* --- 特殊动作 (Jump) --- */
        case CHMode_RC_Jump:
            // 检测到落地 -> 回到 LastActiveMode (Free 或 Follow)
            if (_IsTrans_To_Landed(ST_ModeChoosePara)) { 
                NextMode = LastActiveMode; 
            }
            break;

        /* --- 特殊状态 (OffGround) --- */
        case CHMode_RC_OffGround:
            // 检测到落地 -> 回到 LastActiveMode (Free 或 Follow)
            if (_IsTrans_To_Landed(ST_ModeChoosePara)) {
                NextMode = LastActiveMode;
            }
            break;

        default:
            NextMode = CHMode_RC_ManualSafe;
            break;
    }

    return NextMode;
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
    uint32_t TimeNow = RunTimeGet();
    switch (Mode) {
        case CHMode_RC_ManualSafe:   pCHData->ST_ModeStartTime.RC_ManualSafe = TimeNow; break;
        case CHMode_RC_AutoSafe:     pCHData->ST_ModeStartTime.RC_AutoSafe = TimeNow; break;
        case CHMode_RC_StandUp:      pCHData->ST_ModeStartTime.RC_StandUp = TimeNow; break;
        case CHMode_RC_Standby:      pCHData->ST_ModeStartTime.RC_Standby = TimeNow; break;
        case CHMode_RC_Free:         pCHData->ST_ModeStartTime.RC_Free = TimeNow; break;
        case CHMode_RC_SitDown:      pCHData->ST_ModeStartTime.RC_SitDown = TimeNow; break; // 补齐
        case CHMode_RC_Follow:       pCHData->ST_ModeStartTime.RC_Follow = TimeNow; break;      // 补齐
        case CHMode_RC_OffGround:    pCHData->ST_ModeStartTime.RC_OffGround = TimeNow; break;
        case CHMode_RC_Jump:         pCHData->ST_ModeStartTime.RC_Jump = TimeNow; break;
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
    //* 实现的是局部变量：ModeTemp、MC_ModePre 向全局变量
    // GEMCH_ModePre、GEMCH_Mode 赋值
    GEMCH_ModePre = GEMCH_Mode;
    ChassisStrategy_ModeChooseParaStructUpdate(&ST_ModeChoosePara_tmp);  // 更新底盘模式选择参数结构体
    //! 这个函数会在这里检测当前是否有遥控器指令指示需要切换到什么状态
    GEMCH_Mode = ChassisStrategy_ModeChoose_RCControl(ST_ModeChoosePara_tmp);  // 调用底盘模式选择函数
    ChassisStrategy_ModeStartTimeUpdate(&GSTCH_Data, GEMCH_Mode, GEMCH_ModePre);
}

// #pragma region 模式具体功能实现函数

/**
 * @brief  遥控器指令预处理函数
 * @note   实现对遥控器拨轮和摇杆这种不能自动锁定的输入的预处理
 */
void CH_RCInputPre_Process(void) {
    // === 静态变量：用于记忆历史状态 (上膛标志) ===
    static bool Latch_Roller_Up = false;
    static bool Latch_Roller_Down = false;
    static bool Latch_RJoy_Up = false;
    static bool Latch_RJoy_Down = false;

    // ============================================================
    // 拨轮处理 (Roller Logic) - 用于状态机切换
    // ============================================================
    
    // --- 上拨处理 ---
    if (IsRollerUp()) {
        Latch_Roller_Up = true;         // 上膛
        Latch_Roller_Down = false;      // 互斥
    } 
    else if (!IsRollerUp() && !IsRollerDown()) {
        if (Latch_Roller_Up) {
            GSTCH_Data.F_RollerUpLatched = true; // 击发
            Latch_Roller_Up = false;    // 复位
        } else {
            GSTCH_Data.F_RollerUpLatched = false;
        }
    } else { // 异常/下拨
        GSTCH_Data.F_RollerUpLatched = false;
        if(IsRollerDown()) 
        {Latch_Roller_Up = false;}
    }

    // --- 下拨处理 ---
    if (IsRollerDown()) {
        Latch_Roller_Down = true;
        Latch_Roller_Up = false;
    } else if (!IsRollerUp() && !IsRollerDown()) {
        if (Latch_Roller_Down) {
            GSTCH_Data.F_RollerDownLatched = true;
            Latch_Roller_Down = false;
        } else {
            GSTCH_Data.F_RollerDownLatched = false;
        }
    } else {
        GSTCH_Data.F_RollerDownLatched = false;
        if(IsRollerUp()) 
        {Latch_Roller_Down = false;}
    }

    // ============================================================
    // 右摇杆处理 (Right Joystick Logic) - 用于腿长控制
    // ============================================================
    
    // --- 上推处理 ---
    if (IsRightJoyStickUp()) {
        Latch_RJoy_Up = true;
        Latch_RJoy_Down = false;
    } else if (!IsRightJoyStickUp() && !IsRightJoyStickDown()) { // 摇杆回中
        if (Latch_RJoy_Up) {
            GSTCH_Data.F_RJoyUpLatched = true;
            Latch_RJoy_Up = false;
        } else {
            GSTCH_Data.F_RJoyUpLatched = false;
        }
    } else {
        GSTCH_Data.F_RJoyUpLatched = false;
        if(IsRightJoyStickDown()) 
        {Latch_RJoy_Up = false;}
    }

    // --- 下推处理 ---
    if (IsRightJoyStickDown()) {
        Latch_RJoy_Down = true;
        Latch_RJoy_Up = false;
    } else if (!IsRightJoyStickUp() && !IsRightJoyStickDown()) { // 摇杆回中
        if (Latch_RJoy_Down) {
            GSTCH_Data.F_RJoyDownLatched = true;
            Latch_RJoy_Down = false;
        } else {
            GSTCH_Data.F_RJoyDownLatched = false;
        }
    } else {
        GSTCH_Data.F_RJoyDownLatched = false;
        if(IsRightJoyStickUp()) 
        {Latch_RJoy_Down = false;}
    }
}

// TODO 腿长变化写在控制里面
//     // 仅在允许的模式下运行（Free, Follow, Struggle）
//     if (GEMCH_Mode != CHMode_RC_Free && GEMCH_Mode != CHMode_RC_Follow && GEMCH_Mode != CHMode_RC_Struggle) {
//         GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
//         GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
//         GSTCH_Data.F_JoyUpLatched = false;
//         GSTCH_Data.F_JoyDownLatched = false;
//         return;
//     }

//     // 1. 检测上抬动作
//     if (IsRightJoyStickUp()) {
//         GSTCH_Data.F_JoyUpLatched = true;
//     }
//     // 2. 检测下拨动作
//     if (IsRightJoyStickDown()) {
//         GSTCH_Data.F_JoyDownLatched = true;
//     }

//     // 3. 检测回中触发（回中时判定刚才是否有推/拨动作）
//     if (IsRightJoyStickUp() == false && IsRightJoyStickDown() == false) {
//         // 上抬回中触发：Min -> Mid -> High
//         if (GSTCH_Data.F_JoyUpLatched) {
//             // 如果当前在 LegLenMin 附近，变到 LegLenMid
//             if (GST_RMCtrl.STCH_Default.LegLen1ManualDes < LegLenMid - 0.001f) {
//                 GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;     
//             } else if (GST_RMCtrl.STCH_Default.LegLen1ManualDes < LegLenHigh - 0.001f) {
//                 GST_RMCtrl.STCH_Default.LegLen1ManualDes = 0.500f;
//             }
//             // 如果当前已经接近 LegLenMid，变到 LegLenHigh
//             if (GST_RMCtrl.STCH_Default.LegLen2ManualDes < LegLenMid - 0.001f) {
//                 GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;       
//             } else if (GST_RMCtrl.STCH_Default.LegLen2ManualDes < LegLenHigh - 0.001f) {
//                 // GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenHigh;
//                 GST_RMCtrl.STCH_Default.LegLen2ManualDes = 0.500f;
//             }
//             GSTCH_Data.F_JoyUpLatched = false; // 清除标志
//         }
        
//         // // 下拨回中触发：High -> Mid -> Min
//         // if (GSTCH_Data.F_JoyDownLatched) {
//         //     // 如果当前在 High 附近，下调到 Mid
//         //     if (GST_RMCtrl.STCH_Default.LegLen1ManualDes > LegLenMid + 0.001f) {
//         //         GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
//         //     } else if (GST_RMCtrl.STCH_Default.LegLen1ManualDes > LegLenMin + 0.001f) {
//         //         GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMin;
//         //     }
//         //     // 如果当前在 Mid 附近，下调到 Min
//         //     if (GST_RMCtrl.STCH_Default.LegLen2ManualDes > LegLenMid + 0.001f) {
//         //         GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
//         //     } else if (GST_RMCtrl.STCH_Default.LegLen2ManualDes > LegLenMin + 0.001f) {
//         //         GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMin;
//         //     }
//         //     GSTCH_Data.F_JoyDownLatched = false; // 清除标志
//         //}
//     }

//     // 5. 如果从未触发过上述逻辑且变量未初始化，设置默认值为中腿长
//     if (GST_RMCtrl.STCH_Default.LegLen1ManualDes < 0.01f) { // 简单初值保护
//         GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
//     }
//     if (GST_RMCtrl.STCH_Default.LegLen2ManualDes < 0.01f) { // 简单初值保护
//         GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
//     }
// }


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
 * @brief  遥控器模式下，待机模式控制函数
 * @note   待机模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_StandbyMode_RCControl(void)
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
  * @brief  遥控器模式下，缓慢坐下模式控制函数
  * @note   缓慢坐下模式下的控制策略
  * @param  无
  * @retval 无
*/
void ChModeControl_SitDownMode_RCControl(void) {
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_SitDown < CHMode_AllMode_PreProcessTime)
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
    float YawAngleVelDes_Pre = GSTCH_Data.YawAngleVelDes;   //目标偏航角速度上次值
    float Leg1FFForce_Pre = GSTCH_Data.Leg1ForceDes;        //左腿前馈力上次值
    float Leg2FFForce_Pre = GSTCH_Data.Leg2ForceDes;        //右腿前馈力上次值

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
        GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMid; //左腿目标腿长
        GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMid; //右腿目标腿长
        GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
        GST_RMCtrl.STCH_Default.VelDes          = GSTCH_Data.VelFB; //目标速度
        GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
        GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静止时的默认值）
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静止时的默认值）
    }
    //* 方便调参数
    /*腿长PID系数赋值*/
    // PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
    // PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

    // 只有这样才能实时修改
    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMid; //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMid; //右腿目标腿长
    
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
 * @brief  遥控器模式下，跟随模式控制函数
 * @note   跟随模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_FollowMode_RCControl(void) {
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
        GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMid; //左腿目标腿长
        GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMid; //右腿目标腿长
        GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
        GST_RMCtrl.STCH_Default.VelDes          = GSTCH_Data.VelFB;             //目标速度
        GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
        GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静止时的默认值）
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静止时的默认值）
    }
    //* 方便调参数
    /*腿长PID系数赋值*/
    // PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
    // PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

    // 只有这样才能实时修改
    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMid; //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMid; //右腿目标腿长
    
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
 * @brief  遥控器模式下，离地模式控制函数
 * @note   离地模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_OffGroundMode_RCControl(void) {
    // TODO 腿部前馈力要不要变？PID、TD参数要不要变？腿长是否需要考虑一下变化？
    
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
    GST_RMCtrl.STCH_Default.VelDes          = GSTCH_Data.VelFB; //目标速度
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

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
 * @brief  遥控器模式下，跳跃模式控制函数
 * @note   跳跃模式下的控制策略
 * @param  无
 * @retval 无
 */
// void ChModeControl_JumpMode_RCControl(void) {
//     // TODO 跳跃模式控制函数待完成
// }

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
        case CHMode_RC_Standby:
            ChModeControl_StandbyMode_RCControl();
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
        case CHMode_RC_SitDown:
            ChModeControl_SitDownMode_RCControl();
            break;

        /*RC底盘跟随模式*/
        case CHMode_RC_Follow:
            ChModeControl_FollowMode_RCControl();
            break;

        /*RC离地模式*/
        case CHMode_RC_OffGround:
            ChModeControl_OffGroundMode_RCControl();
            break;

        /*RC跳跃模式*/
        // case CHMode_RC_Jump:
        //     ChModeControl_JumpMode_RCControl();
        //     break;
    }
}
