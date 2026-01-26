/**
 ******************************************************************************
 * @file    Chassis_APIFunction.c
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.10.25
 * @brief   底盘功能函数，实现底盘的各种功能，把底盘的功能函数放在这里，方便外部调用，
 *          底盘的不同模式控制策略在Chassis_Stratgy.c文件中实现。
 *          如果后面觉得这里的函数太多了，可以考虑拆分文件
 *          包含四部分内容的函数：
 *          1. 各种FB反馈数据的修改、处理、更新相关函数
 *          2. 各种Des目标数据的修改、处理、更新相关函数
 *          3. 各种底盘数据的解算相关函数：只解算，不更改正式结构体的值
 *          4. 各种数据的重置、清零函数
 ******************************************************************************
 */
#include "Chassis_APIFunction.h"

#include <arm_math.h>

#include "Algorithm.h"
#include "Algorithm_Simple.h"
#include "GlobalDeclare_Chassis.h"
#include "GlobalDeclare_General.h"
#include "General_AuxiliaryFunc.h"
#include "Chassis_APIFunction.h"


#include "TIM_Config.h"

float PID_RollComp_Kp_tmp = 20.0f;  // Roll轴补偿PID：比例系数Kp
float PID_RollComp_Ki_tmp = 0.0f;    // Roll轴补偿PID：积分系数Ki，取0表示不使用积分
float PID_RollComp_Kd_tmp = 1000.0f;              // Roll轴补偿PID：微分系数Kd

float K_Trac_Norm_tmp = 0.0f;
// 底盘的数据修改、处理、更新相关函数：允许更改正式结构体的值
// TODO 可能需要改一下位置
//! 由于需要用到部分前面用到的函数修改和处理写在后面了（然后我改了一下函数名称）
// #pragma region 各种FB反馈数据的修改、处理、更新相关函数
/**
 * @brief  用来解析轮毂电机反馈的数据，然后分发到正式结构体、变量
 * @param  Side：RobotSide_EnumTypeDef类型的枚举值，表示左轮毂还是右轮毂
 *             @arg LeftSide：左轮毂
 *             @arg RightSide：右轮毂
 * @param  pHM：轮毂电机控制结构体指针
 * @param  pESC：C620电调反馈数据结构体指针（轮毂电机连在C620电调上）
 * @param  pLPF：轮毂电机速度一阶低通滤波器结构体指针
 * @retval 无
 */
//* 最后获得的是pHM里面的滤波后的速度（左右轮取反处理）、温度、电流反馈值
void _HM_FBData_Parse(RobotSide_EnumTypeDef Side, HMData_StructTypeDef* pHM, C620FeedBackData_StructTypeDef* pESC, LPF_StructTypeDef* pLPF) {
    /*计算原始速度值*/
    float ReductionGearboxAngleVel = pESC->AngleVelFB / pHM->ReductionRatio;  // 减速箱输出轴角速度，单位r/min
    float RawAngleVel = ReductionGearboxAngleVel * 2 * PI / 60.0f;  // 减速箱输出轴角速度，单位rad/s

    /*低通滤波处理速度值*/
    LPF_SetInput(pLPF, RawAngleVel);  // 设置低通滤波器输入值
    LPF_Cal(pLPF);                    // 调用低通滤波计算函数

    /*轮毂电机速度反馈赋值*/
    if (Side == LeftSide) {
        pHM->AngleVelFB = -LPF_GetOutput(pLPF);
    } else if (Side == RightSide) {
        pHM->AngleVelFB = LPF_GetOutput(pLPF);
    }

    /*电流值和温度值*/
    pHM->CurrentFB = pESC->CurrentFB;  // 电流值
    pHM->TempFB = pESC->TempFB;        // 温度值
}

/**
 * @brief  底盘数据低通滤波处理
 * @note   调用LPF_Cal对底盘相关数据进行低通滤波处理
 * @param  RawData：需要滤波的原始数据
 * @param  pLPF：低通滤波器结构体指针
 * @retval 滤波后的数据
 */
//* 底盘数据低通滤波处理
float _Ch_FBData_LPF(float RawData, LPF_StructTypeDef* pLPF) {
    LPF_SetInput(pLPF, RawData);
    LPF_Cal(pLPF);
    return LPF_GetOutput(pLPF);
}

/**
  * @brief  底盘离地状态标志获取函数
  * @note   根据当前腿部支持力值和上次离地状态标志，判断当前离地状态标志
  *         如果上次状态为未离地，且当前支持力小于25N，则认为离地
  *         如果上次状态为离地，则要求支持力大于70N，认为落地
  *         防止在临界值频繁切换状态
  * @param  F_OffGroundPre：上次离地状态标志，true表示离地，false表示未离地
  * @param  LegF_N：当前腿部支持力值，单位N
  * @retval 当前离地状态标志，true表示离地，false表示触地
*/
bool _Ch_OffGroundStateFlagGet(bool F_OffGroundPre, float LegF_N)
{
    bool F_OffGroundNext = F_OffGroundPre;

    // TODO 测试之后看N变化的最大值最小值来写这个判断函数
    if(F_OffGroundPre == false && LegF_N <= 25.0f)
    {F_OffGroundNext = true;}//支持力过小，认为离地
    else if(F_OffGroundPre == true && LegF_N >= 70.0f)
    {F_OffGroundNext = false;}//支持力恢复，认为落地

    return F_OffGroundNext;
}

/**
 * @brief  更新底盘相关数据的反馈值
 * @note   底盘相关，正式结构体数据的反馈值，在这里进行更新汇总处理
 *         主要的底盘相关正式结构体有：
 *             GSTCH_HM1、2：左、右轮毂电机控制结构体
 *             GSTCH_JM1~GSTCH_JM4：四个关节电机控制结构体
 *             GSTCH_Data：底盘主要数据结构体
 *         对于部分噪声较多的变量，进行观测、滤波等处理，使输出数据更平滑
 * @param  无
 * @retval 无
 */
//* 更新底盘相关数据的反馈值
void CH_FBData_Parse(void) {
    /***************** 轮毂电机相关数据更新 *****************/
    //* 更新轮毂电机结构体数据，对角速度做了低通滤波处理
    _HM_FBData_Parse(LeftSide, &GSTCH_HM1, &GstCH_HM1RxC620Data, &GstCH_HM1_AngleVelLPF);  // 处理电调反馈数据，然后分发到正式结构体、变量
    _HM_FBData_Parse(RightSide, &GSTCH_HM2, &GstCH_HM2RxC620Data, &GstCH_HM2_AngleVelLPF);  // 处理电调反馈数据，然后分发到正式结构体、变量

    /**************** IMU2-关节电机数据更新 ****************/
    GSTCH_JM1.AngleFB = GstCH_IMU2.ST_Rx.JM1_AngleFB;
    GSTCH_JM1.AngleVelFB = GstCH_IMU2.ST_Rx.JM1_AngleVelFB;
    GSTCH_JM1.TorqueFB = GstCH_IMU2.ST_Rx.JM1_TorqueFB;

    GSTCH_JM2.AngleFB = GstCH_IMU2.ST_Rx.JM2_AngleFB;
    GSTCH_JM2.AngleVelFB = GstCH_IMU2.ST_Rx.JM2_AngleVelFB;
    GSTCH_JM2.TorqueFB = GstCH_IMU2.ST_Rx.JM2_TorqueFB;

    GSTCH_JM3.AngleFB = GstCH_IMU2.ST_Rx.JM3_AngleFB;
    GSTCH_JM3.AngleVelFB = GstCH_IMU2.ST_Rx.JM3_AngleVelFB;
    GSTCH_JM3.TorqueFB = GstCH_IMU2.ST_Rx.JM3_TorqueFB;

    GSTCH_JM4.AngleFB = GstCH_IMU2.ST_Rx.JM4_AngleFB;
    GSTCH_JM4.AngleVelFB = GstCH_IMU2.ST_Rx.JM4_AngleVelFB;
    GSTCH_JM4.TorqueFB = GstCH_IMU2.ST_Rx.JM4_TorqueFB;

    /*************** IMU2-底盘运动姿态数据更新 ***************/
    GSTCH_Data.YawAngleVelFB = _Ch_FBData_LPF(GstCH_IMU2.ST_Rx.YawAngleVel, &GstCH_YawAngleVelLPF);
    GSTCH_Data.PitchAngleFB = GstCH_IMU2.ST_Rx.PitchAngle;
    GSTCH_Data.PitchAngleVelFB = _Ch_FBData_LPF(GstCH_IMU2.ST_Rx.PitchAngleVel, &GstCH_PitchAngleVelLPF);

    GSTCH_Data.RollAngleFB = GstCH_IMU2.ST_Rx.RollAngle;
    GSTCH_Data.AccXFB = - GstCH_IMU2.ST_Rx.AccX;  // 底盘云控反着安装，所以取负号 //换车时需修改
    GSTCH_Data.AccZFB =   GstCH_IMU2.ST_Rx.AccZ;  // 底盘垂直加速度反馈值，向上为正，单位m/s²
}
// #pragma endregion

// #pragma region 各种Des目标数据的修改、处理、更新相关函数
// TODO 更新目标值实现
/**
 * @brief  更新腿长目标值（TD算法）
 * @note   从机器人控制结构体中获取数据，根据TD算法，计算出腿长的目标值
 * @param  RMCtrl：RobotControl_StructTypeDef类型的机器人控制结构体变量
 * @retval 无
 */
//* 更新腿长目标值
void CH_LegLenDes_Update(RobotControl_StructTypeDef RMCtrl) {
    TD_SetInput(&GstCH_LegLen1TD, (RMCtrl.STCH_Default.LegLen1Des)); //左腿腿长TD输入值设为左腿腿长目标值
    TD_Cal(&GstCH_LegLen1TD);                                       //左腿腿长TD计算
    GSTCH_Data.LegLen1Des = TD_GetOutput(&GstCH_LegLen1TD);    //左腿腿长目标值更新为TD输出值

    TD_SetInput(&GstCH_LegLen2TD, (RMCtrl.STCH_Default.LegLen2Des)); //右腿腿长TD输入值设为右腿腿长目标值
    TD_Cal(&GstCH_LegLen2TD);                                       //右腿腿长TD计算
    GSTCH_Data.LegLen2Des = TD_GetOutput(&GstCH_LegLen2TD);    //右腿腿长目标值更新为TD输出值
}

/**
 * @brief  更新LQR相关数据的目标值
 * @note   从机器人控制结构体中，获取数据，更新LQR的计算目标值
 *         如果用户开启自定义LQR控制，则使用用户设置的所有目标值
 *         否则除了前四项外，目标值全部采用默认值0
 * @param  RMCtrl：RobotControl_StructTypeDef类型的机器人控制结构体变量
 * @retval 无
 */
//* 更新LQR相关数据的目标值
void CH_LQR_DesDataUpdate(RobotControl_StructTypeDef RMCtrl) {
    GSTCH_Data.DisDes         = RMCtrl.STCH_Default.DisDes;         //位移目标值
    GSTCH_Data.VelDes         = RMCtrl.STCH_Default.VelDes;         //速度目标值
    GSTCH_Data.YawDeltaDes    = RMCtrl.STCH_Default.YawDeltaDes;    //偏转角度目标值
    GSTCH_Data.YawAngleVelDes = RMCtrl.STCH_Default.YawAngleVelDes; //偏转角速度目标值

    /*如果用户开启自定义LQR控制，则使用用户设置的目标值*/
    if(RMCtrl.STCH_Force.F_LQR_UserSetEnable == true)
    {
        GSTCH_Data.Theta1Des         =    RMCtrl.STCH_Force.Theta1Des        ; //左腿摆角目标值
        GSTCH_Data.Theta1AngleVelDes =    RMCtrl.STCH_Force.Theta1AngleVelDes; //左腿摆角速度目标值
        GSTCH_Data.Theta2Des         =    RMCtrl.STCH_Force.Theta2Des        ; //右腿摆角目标值
        GSTCH_Data.Theta2AngleVelDes =    RMCtrl.STCH_Force.Theta2AngleVelDes; //右腿摆角速度目标值
        GSTCH_Data.PitchAngleDes     =    RMCtrl.STCH_Force.PitchAngleDes    ; //俯仰角目标值
        GSTCH_Data.PitchAngleVelDes  =    RMCtrl.STCH_Force.PitchAngleVelDes ; //俯仰角速度目标值
    }
    /*否则除前四项外，目标值全部采用默认值*/
    else
    {
        GSTCH_Data.Theta1Des         = 0.0f;
        GSTCH_Data.Theta1AngleVelDes = 0.0f;
        GSTCH_Data.Theta2Des         = 0.0f;
        GSTCH_Data.Theta2AngleVelDes = 0.0f;
        GSTCH_Data.PitchAngleDes     = 0.0f;
        GSTCH_Data.PitchAngleVelDes  = 0.0f;
    }
}

/**
 * @brief  更新VMC相关数据的目标值
 * @note   从机器人控制结构体中，获取数据，更新VMC的计算目标值
 *         如果用户开启自定义VMC控制，则使用用户设置的目标值
 *         否则采取默认的VMC计算控制
 * @param  RMCtrl：机器人控制结构体变量
 * @retval 无
 */
//* 更新VMC相关数据的目标值
void CH_VMC_DesDataUpdate(RobotControl_StructTypeDef RMCtrl) {
    /****如果用户开启自定义VMC控制，则使用用户设置的目标值****/
    if(RMCtrl.STCH_Force.F_VMC_UserSetEnable == true)
    {
        GSTCH_Data.Leg1ForceDes  = RMCtrl.STCH_Force.Leg1FDes;     //左腿力目标值
        GSTCH_Data.Leg1TorqueDes = RMCtrl.STCH_Force.Leg1TDes;     //左腿力矩目标值
        GSTCH_Data.Leg2ForceDes  = RMCtrl.STCH_Force.Leg2FDes;     //右腿力目标值
        GSTCH_Data.Leg2TorqueDes = RMCtrl.STCH_Force.Leg2TDes;     //右腿力矩目标值
        return;
    }

    /***************否则采取默认的VMC计算控制值***************/
    /****腿部前馈力获取****/
    /*
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
    VMC_FFForceUpdate(&RMCtrl);                          // 更新底盘前馈力（动态前馈力加上静态前馈力）
    float Leg1FFForce = RMCtrl.STCH_Default.Leg1FFForce; // 获取左腿前馈力
    float Leg2FFForce = RMCtrl.STCH_Default.Leg2FFForce; // 获取右腿前馈力

    /****Roll轴补偿力PID计算****/
    //! ROLL的PID参数已经在结构体初始化里面赋值完了 
    PID_SetKpKiKd(&GstCH_RollCompPID, PID_RollComp_Kp_tmp, PID_RollComp_Ki_tmp, PID_RollComp_Kd_tmp); //更新Roll轴补偿PID参数
    PID_SetDes(&GstCH_RollCompPID, 0.0f * A2R); //底盘Roll轴补偿PID目标值，默认设为0rad
    PID_SetFB(&GstCH_RollCompPID, (GSTCH_Data.RollAngleFB - ChassisRollAngleZP) * A2R); //底盘Roll轴补偿PID反馈值，单位转为弧度
    PID_Cal(&GstCH_RollCompPID);
    float RollCompForce = PID_GetOutput(&GstCH_RollCompPID); // 获取底盘Roll轴补偿力
    // float RollCompForce = 0.0f;                              // 取消Roll轴补偿力

    /****获取腿长PID的输出力****/
    /*左腿*/
    PID_SetDes(&GstCH_LegLen1PID, GSTCH_Data.LegLen1Des);  //腿长PID目标值设为目标腿长
    PID_SetFB (&GstCH_LegLen1PID, GSTCH_Data.LegLen1FB);   //腿长PID反馈值设置为实际腿长
    PID_Cal  (&GstCH_LegLen1PID);                               //腿长PID计算
    float Leg1PIDForce = PID_GetOutput(&GstCH_LegLen1PID);      //获取左腿腿长PID作用力

    /*右腿*/
    PID_SetDes(&GstCH_LegLen2PID, GSTCH_Data.LegLen2Des);  //腿长PID目标值设为目标腿长
    PID_SetFB (&GstCH_LegLen2PID, GSTCH_Data.LegLen2FB);   //腿长PID反馈值设置为实际腿长
    PID_Cal  (&GstCH_LegLen2PID);                               //腿长PID计算
    float Leg2PIDForce = PID_GetOutput(&GstCH_LegLen2PID);      //获取右腿腿长PID作用力

    /****最终等效摆杆力、力矩目标值赋值****/
    GSTCH_Data.Leg1ForceDes  = Leg1PIDForce + RollCompForce + Leg1FFForce;  //左腿沿杆F = 腿长PID输出值 + Roll轴补偿力 + 腿部前馈力
    GSTCH_Data.Leg1TorqueDes = LQR_Get_uVector(&GstCH_LQRCal, 3-1);         //左腿力矩T = LQR输出矩阵的第3个元素
    GSTCH_Data.Leg2ForceDes  = Leg2PIDForce - RollCompForce + Leg2FFForce;  //右腿沿杆F = 腿长PID输出值 - Roll轴补偿力 + 腿部前馈力
    GSTCH_Data.Leg2TorqueDes = LQR_Get_uVector(&GstCH_LQRCal, 4-1);         //右腿力矩T = LQR输出矩阵的第4个元素
}

/**
  * @brief  轮毂电机力矩转电流的函数
  * @note   把轮毂电机力矩目标值转换成电调电流目标值
  *         注意这个函数没有考虑方向性，需要自行在调用时考虑正负号
  * @param  Torque：轮毂电机力矩目标值，单位Nm
  * @retval 轮毂电机电流目标值
*/
int16_t _HM_DesData_TorqueToCurrent(float Torque)
{
    float HM_Ampere = Torque / HM_Kt;                       //计算实际需要的电流值，单位A
    float HM_CurrentDes = (HM_Ampere * HM_AmpereToCurrent); //转换成电调需要的电流值
    return (int16_t)Limit(HM_CurrentDes, HM_MinCurrent, HM_MaxCurrent);//限制在电机允许电流范围内
}

/**
  * @brief  轮毂电机打滑检测函数
  * @note   把电调电流目标值转换成轮毂电机力矩目标值
  * @param  无
  * @retval 轮毂电机力矩目标值
*/
void _HM_StruggleStateDetect(void)
{
    // 计算轮毂电机速度误差
    float HM1_VelFB = GSTCH_HM1.AngleVelFB * R_w; // 左轮轮毂电机线速度，单位m/s
    float HM2_VelFB = GSTCH_HM2.AngleVelFB * R_w; // 右轮轮毂电机线速度，单位m/s
    float HM1_VelRef = GSTCH_Data.VelFB + GSTCH_Data.xC1_dot - R_l * GSTCH_Data.YawAngleVelFB; // 左轮轮毂电机参考线速度，单位m/s
    float HM2_VelRef = GSTCH_Data.VelFB + GSTCH_Data.xC2_dot + R_l * GSTCH_Data.YawAngleVelFB; // 右轮轮毂电机参考线速度，单位m/s
    float HM1_VelErr = HM1_VelRef - HM1_VelFB; // 左轮轮毂电机速度误差，单位m/s
    float HM2_VelErr = HM2_VelRef - HM2_VelFB; // 右轮轮毂电机速度误差，单位m/s

    // 左轮轮毂电机打滑/受阻检测
    if(HM1_VelErr < -GSTCH_HMTorqueComp.Err_DZ) {
        GSTCH_Data.F_SlipHM1 = true;
    }
    else if(HM1_VelErr > GSTCH_HMTorqueComp.Err_DZ) {
        GSTCH_Data.F_BlockHM1 = true;
    }
    else {
        GSTCH_Data.F_SlipHM1 = false;
        GSTCH_Data.F_BlockHM1 = false;
    }

    // 右轮轮毂电机打滑/受阻检测
    if(HM2_VelErr < -GSTCH_HMTorqueComp.Err_DZ) {
        GSTCH_Data.F_SlipHM2 = true;
    }
    else if(HM2_VelErr > GSTCH_HMTorqueComp.Err_DZ) {
        GSTCH_Data.F_BlockHM2 = true;
    }
    else {
        GSTCH_Data.F_SlipHM2 = false;
        GSTCH_Data.F_BlockHM2 = false;
    }

    GSTCH_HMTorqueComp.Err_HM1 = HM1_VelErr;
    GSTCH_HMTorqueComp.Err_HM2 = HM2_VelErr;
}

/**
  * @brief  更新轮毂电机相关数据的目标值
  * @note   获取轮毂电机的力矩目标值，转化为电流目标值
  * @param  RMCtrl：机器人控制结构体变量
  * @retval 无
*/
void HM_DesDataUpdate(RobotControl_StructTypeDef RMCtrl)
{
    /*如果用户开启自定义轮毂电机扭矩控制，则使用用户设置的目标值*/
    if(RMCtrl.STCH_Force.F_HMTorque_UserSetEnable == true)
    {
        GSTCH_HM1.TorqueDes  = RMCtrl.STCH_Force.HM1TDes; //左轮毂电机力矩目标值
        GSTCH_HM2.TorqueDes  = RMCtrl.STCH_Force.HM2TDes; //右轮毂电机力矩目标值
    }
    /*否则采取默认的LQR计算控制*/
    /*离地检测部分*/
    else if(GSTCH_Data.F_OffGround1 == true) 
    {
        GSTCH_HM1.TorqueDes  = 0.0f;                                                                   // 左腿离地左轮目标力矩为零
        GSTCH_HM2.TorqueDes  = + LQR_Get_uVector(&GstCH_LQRCal, 2-1) + GSTCH_HMTorqueComp.T_Comp_HM2; // 右轮毂电机力矩目标值
    }
    else if(GSTCH_Data.F_OffGround2 == true) 
    {
        GSTCH_HM1.TorqueDes  = - LQR_Get_uVector(&GstCH_LQRCal, 1-1) + GSTCH_HMTorqueComp.T_Comp_HM1; // 左轮毂电机力矩目标值
        GSTCH_HM2.TorqueDes  = 0.0f;                                                                   // 右腿离地右轮目标力矩为零
    }
    else
    {
        GSTCH_HM1.TorqueDes  = - LQR_Get_uVector(&GstCH_LQRCal, 1-1) + GSTCH_HMTorqueComp.T_Comp_HM1; //左轮毂电机力矩目标值
        GSTCH_HM2.TorqueDes  = + LQR_Get_uVector(&GstCH_LQRCal, 2-1) + GSTCH_HMTorqueComp.T_Comp_HM2; //右轮毂电机力矩目标值
    }
    // 理论上讲轮毂电机应该是左轮为正、右轮为负，但是行星减速箱会反转方向，所以这里取反
    GSTCH_HM1.CurrentDes = _HM_DesData_TorqueToCurrent(GSTCH_HM1.TorqueDes); //左轮毂电机电流目标值
    GSTCH_HM2.CurrentDes = _HM_DesData_TorqueToCurrent(GSTCH_HM2.TorqueDes); //右轮毂电机电流目标值
}

/**
  * @brief  更新关节电机相关数据的目标值
  * @note   更新关节电机的力矩目标值
  * @param  RMCtrl：机器人控制结构体变量
  * @retval 无
*/
void JM_DesDataUpdate(RobotControl_StructTypeDef RMCtrl)
{
    /*如果用户开启自定义关节电机扭矩控制，则使用用户设置的目标值*/
    if(RMCtrl.STCH_Force.F_JMTorque_UserSetEnable == true)
    {
        GSTCH_JM1.TorqueDes = RMCtrl.STCH_Force.JM1TDes; //关节电机1力矩目标值
        GSTCH_JM2.TorqueDes = RMCtrl.STCH_Force.JM2TDes; //关节电机2力矩目标值
        GSTCH_JM3.TorqueDes = RMCtrl.STCH_Force.JM3TDes; //关节电机3力矩目标值
        GSTCH_JM4.TorqueDes = RMCtrl.STCH_Force.JM4TDes; //关节电机4力矩目标值
        return;
    }

    /*否则采取默认的VMC计算控制*/
    // TODO 这块好像传入的力矩不是负值，应该都是正值，上车看看对不对
    GSTCH_JM1.TorqueDes = Limit(GstCH_Leg1VMC.T_Matrix[1], -JointMotorMAXTorque, JointMotorMAXTorque); //关节电机1力矩目标值
    GSTCH_JM3.TorqueDes = Limit(GstCH_Leg1VMC.T_Matrix[0], -JointMotorMAXTorque, JointMotorMAXTorque); //关节电机3力矩目标值

    GSTCH_JM2.TorqueDes = Limit(GstCH_Leg2VMC.T_Matrix[0], -JointMotorMAXTorque, JointMotorMAXTorque); //关节电机2力矩目标值
    GSTCH_JM4.TorqueDes = Limit(GstCH_Leg2VMC.T_Matrix[1], -JointMotorMAXTorque, JointMotorMAXTorque); //关节电机4力矩目标值
}

// #pragma endregion

// #pragma region 各种底盘数据的解算相关函数：只解算，不更改正式结构体的值


/**
 * @brief  底盘腿部五连杆解算处理函数
 * @note   第一步：更新五连杆解算的数据，主要是phi1、phi4
 *         第二步：进行五连杆正运动学解算，由关节角度求解末端位置
 *         第三步：把解算后的数据分发到正式变量/正式结构体中
 * @param  无
 * @retval 无
 */
//* 底盘五连杆解算处理函数
void CH_LegKinematics_Process(void) {
    /*更新五连杆解算的phi1、phi4数据*/
    LegLinkage_AngleDataUpdate(&GstCH_LegLinkCal1, GSTCH_JM3.AngleFB, GSTCH_JM1.AngleFB);  // 更新左侧数据
    LegLinkage_AngleDataUpdate(&GstCH_LegLinkCal2, GSTCH_JM2.AngleFB, GSTCH_JM4.AngleFB);  // 更新右侧数据

    /*更新五连杆解算的phi1_dot、phi4_dot数据*/
    LegLinkage_AngleVelDataUpdate(&GstCH_LegLinkCal1, GSTCH_JM3.AngleVelFB, GSTCH_JM1.AngleVelFB);  // 更新左侧数据
    LegLinkage_AngleVelDataUpdate(&GstCH_LegLinkCal2, GSTCH_JM2.AngleVelFB, GSTCH_JM4.AngleVelFB);  // 更新右侧数据

    /*五连杆正运动学解算*/
    //* 虚拟摆杆长和角度数据存到了LegLinkageCal_StructTypeDef里面
    LegLinkage_ForwardKinematicsCal(&GstCH_LegLinkCal1);  // 左腿五连杆运动学正解
    LegLinkage_ForwardKinematicsCal(&GstCH_LegLinkCal2);  // 右腿五连杆运动学正解

    /*************** 底盘五连杆解算相关数据更新 ***************/
    GSTCH_Data.LegLen1FB = LegLinkage_GetLegLength(&GstCH_LegLinkCal1) * MM2M;  // 获取左腿实际长度
    GSTCH_Data.LegLen2FB = LegLinkage_GetLegLength(&GstCH_LegLinkCal2) * MM2M;  // 获取右腿实际长度

    GSTCH_Data.Theta1FB = LegLinkage_GetTheta(&GstCH_LegLinkCal1, LeftSide);  // 获取左腿与垂直方向夹角
    GSTCH_Data.Theta2FB = LegLinkage_GetTheta(&GstCH_LegLinkCal2, RightSide);  // 获取右腿与垂直方向夹角

    float Theta1_dotRawValue = LegLinkage_GetThetadot(&GstCH_LegLinkCal1, LeftSide);  // 获取左腿Theta角速度，原始值
    float Theta2_dotRawValue = LegLinkage_GetThetadot(&GstCH_LegLinkCal2, RightSide);  // 获取右腿Theta角速度，原始值
    GSTCH_Data.Theta1AngleVelFB = _Ch_FBData_LPF(Theta1_dotRawValue, &GstCH_Theta1dotLPF);  // 低通滤波处理
    GSTCH_Data.Theta2AngleVelFB = _Ch_FBData_LPF(Theta2_dotRawValue, &GstCH_Theta2dotLPF);  // 低通滤波处理

    float xC1_dotRawValue = LegLinkage_GetxCdot(&GstCH_LegLinkCal1, LeftSide);  // 获取左边五连杆解算C点x坐标的微分，原始值
    float xC2_dotRawValue = LegLinkage_GetxCdot(&GstCH_LegLinkCal2, RightSide);  // 获取右边五连杆解算C点x坐标的微分，原始值
    GSTCH_Data.xC1_dot = _Ch_FBData_LPF(xC1_dotRawValue, &GstCH_xC1dotLPF);  // 低通滤波处理
    GSTCH_Data.xC2_dot = _Ch_FBData_LPF(xC2_dotRawValue, &GstCH_xC2dotLPF);  // 低通滤波处理
}

/**
 * @brief  底盘离地检测计算处理函数
 * @note   更新离地检测相关数据，然后调用离地检测计算函数
 * @param  无
 * @retval 无
 */
//* 底盘离地检测计算处理函数
void CH_OffGround_Process(void) {
    /*左腿离地检测相关变量的计算*/
    OffGround_BodyZAccUpdate(&GstCH_OffGround1, GSTCH_Data.AccZFB);
    OffGround_PitchAngleUpdate(&GstCH_OffGround1, GSTCH_Data.PitchAngleFB);
    OffGround_PitchAngleVelUpdate(&GstCH_OffGround1, GSTCH_Data.PitchAngleVelFB);
    OffGround_LegLinkRelateDataUpdate(GstCH_LegLinkCal1, &GstCH_OffGround1, LeftSide);
    OffGround_TorqueDataUpdate(&GstCH_OffGround1, GSTCH_JM3.TorqueFB, GSTCH_JM1.TorqueFB);
    OffGround_GetRealFAndTp(&GstCH_OffGround1);

    /*右腿离地检测相关变量的计算*/
    OffGround_BodyZAccUpdate(&GstCH_OffGround2, GSTCH_Data.AccZFB);
    OffGround_PitchAngleUpdate(&GstCH_OffGround2, GSTCH_Data.PitchAngleFB);
    OffGround_PitchAngleVelUpdate(&GstCH_OffGround2, GSTCH_Data.PitchAngleVelFB);
    OffGround_LegLinkRelateDataUpdate(GstCH_LegLinkCal2, &GstCH_OffGround2, RightSide);
    OffGround_TorqueDataUpdate(&GstCH_OffGround2, GSTCH_JM2.TorqueFB, GSTCH_JM4.TorqueFB);
    OffGround_GetRealFAndTp(&GstCH_OffGround2);
    
    /***************** 离地检测相关数据更新 *****************/
    //* XXX 由于是循环执行，所以虽然没有更新Z轴加速度等但是可以在这个里面更新离地状态 
    float Leg1F_N_RawValue = OffGround_GetSupportForce(&GstCH_OffGround1);  // 左腿腿部支持力反馈原始值，单位N
    float Leg2F_N_RawValue = OffGround_GetSupportForce(&GstCH_OffGround2);  // 右腿腿部支持力反馈原始值，单位N
    GSTCH_Data.Leg1F_N = _Ch_FBData_LPF(Leg1F_N_RawValue, &GstCH_Leg1F_N_LPF);  // 低通滤波处理
    GSTCH_Data.Leg2F_N = _Ch_FBData_LPF(Leg2F_N_RawValue, &GstCH_Leg2F_N_LPF);  // 低通滤波处理
    // TODO 测一下离地状态下的支持力变化的最大值和最小值
    GSTCH_Data.F_OffGround1 = _Ch_OffGroundStateFlagGet(GSTCH_Data.F_OffGround1, GSTCH_Data.Leg1F_N); //左腿离地状态更新
    GSTCH_Data.F_OffGround2 = _Ch_OffGroundStateFlagGet(GSTCH_Data.F_OffGround2, GSTCH_Data.Leg2F_N); //右腿离地状态更新
}

/**
 * @brief  底盘卡尔曼滤波速度融合处理函数
 * @note   对底盘速度进行卡尔曼滤波融合处理，观测当前速度并计算轮毂电机速度误差补偿力矩
 * @param  无
 * @retval 无
 */
void CH_VelKF_Process(void) {
    // 1. 获取测量数据
    float v_wheel_theory = (GSTCH_HM1.AngleVelFB + GSTCH_HM2.AngleVelFB) / 2.0f * R_w; // 轮子测量的线速度（角速度是低通后的不用再低通了）
    float a_imu = GSTCH_Data.AccXFB; // IMU测量的机体加速度
    float v_rel_leg = (GSTCH_Data.xC1_dot + GSTCH_Data.xC2_dot) / 2.0f; // 运动学补偿 (假设车身静止时轮子的速度)

    // 重要：观测到的底盘速度 = 轮速 + 腿部摆动速度
    // 这样观测值的物理意义才与 KF 预测的底盘速度(由IMU积分而来)保持一致
    float v_body_obs = v_wheel_theory - v_rel_leg;

    // 2. 离地与打滑处理 (动态调节 R 矩阵)
    if(GSTCH_Data.F_OffGround1 || GSTCH_Data.F_OffGround2) {
        // 离地了，轮速观测值不可信，大幅增加 R[0][0]，使 KF 信任 IMU 积分
        GstCH_VelKF.R[0][0] = 1000.0f; 
    } else {
        // 正常状态
        GstCH_VelKF.R[0][0] = 0.05f; 
    }

    // 3. 执行速度预测
    KF_ChassisVel_Predict(&GstCH_VelKF);
    GstCH_VelKF.VelNxt = GstCH_VelKF.x[0];

    // 4. 执行速度更新
    KF_ChassisVel_Update(&GstCH_VelKF, v_body_obs, a_imu);
    GstCH_VelKF.VelFB = GstCH_VelKF.x[0];

    GSTCH_Data.VelFB = GstCH_VelKF.VelFB;   // 底盘速度滤波、观测处理
    GSTCH_Data.DisFB += GSTCH_Data.VelFB * GCH_TaskTime;  // 位移 = 上次位移 + 速度*时间

    GSTCH_Data.VelBody_HM_Obs = v_body_obs; // 底盘速度观测值存储
}



// TODO 细化这个注释
/**
 * @brief  左右轮轮毂电机打滑和受阻检测
 * @note   实现轮毂电机打滑和受阻检测功能，计算轮毂电机速度误差，并根据误差判断是否打滑或受阻
 * @param  无
 * @retval 无
 */
void CH_HMTorqueComp_Process(void) {
    _HM_StruggleStateDetect();
    // 计算正常牵引力矩补偿
    GSTCH_HMTorqueComp.T_Trac_HM1 = K_Trac_Norm_tmp * GSTCH_HMTorqueComp.Err_HM1;
    GSTCH_HMTorqueComp.T_Trac_HM2 = K_Trac_Norm_tmp * GSTCH_HMTorqueComp.Err_HM2;
    // 进入打滑模式后会改这个值
    GSTCH_HMTorqueComp.T_Comp_HM1 = GSTCH_HMTorqueComp.T_Trac_HM1;
    GSTCH_HMTorqueComp.T_Comp_HM2 = GSTCH_HMTorqueComp.T_Trac_HM2;
}

/**
 * @brief  底盘LQR计算处理函数
 * @note   更新LQR的x状态向量，然后调用LQR_Cal函数进行LQR计算
 * @param  无
 * @retval 无
 */
//* 底盘LQR计算处理函数
void CH_LQRCal_Process(void) {
    /*更新LQR计算的状态向量*/
    LQR_xVector_DataUpdate(&GstCH_LQRCal,
                            GSTCH_Data.DisDes            - GSTCH_Data.DisFB,                                                               //位移误差
                            GSTCH_Data.VelDes            - GSTCH_Data.VelFB,                                                               //速度误差（位移一阶导）
           
                            GSTCH_Data.YawDeltaDes * A2R,                                                                                  //偏转角增量（相当于YawAngleDes - YawAngleFB）
                           (GSTCH_Data.YawAngleVelDes    - GSTCH_Data.YawAngleVelFB)*A2R,                                                  //偏转角速度误差
           
                           (GSTCH_Data.Theta1Des         - (GSTCH_Data.Theta1FB - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP)) * A2R,   //左腿摆角误差
                           (GSTCH_Data.Theta1AngleVelDes - (GSTCH_Data.Theta1AngleVelFB - GSTCH_Data.PitchAngleVelFB)) * A2R,              //左腿摆角速度误差
           
                           (GSTCH_Data.Theta2Des         - (GSTCH_Data.Theta2FB - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP)) * A2R,   //右腿摆角误差
                           (GSTCH_Data.Theta2AngleVelDes - (GSTCH_Data.Theta2AngleVelFB - GSTCH_Data.PitchAngleVelFB)) * A2R,              //右腿摆角速度误差
           
                           (GSTCH_Data.PitchAngleDes     - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP) * A2R,                           //俯仰角误差
                           (GSTCH_Data.PitchAngleVelDes  - GSTCH_Data.PitchAngleVelFB) * A2R                                               //俯仰角速度误差
                          );

    /*更新LQR的K矩阵*/
    //* 刚开始腿长是0，然后K矩阵直接就是代入的几个默认值中的一个 LQR_DefaultK_Matrix
    LQR_K_MatrixUpdate(&GstCH_LQRCal, GST_RMCtrl.STCH_Default.LegLen1ManualDes, GST_RMCtrl.STCH_Default.LegLen2ManualDes);
    // TODO 暂时不根据腿长更新K矩阵，直接用默认值，后面需要使用拟合时，先写了这个函数里面的拟合，然后解除上面的函数注释，删除该行
    /*调用LQR计算函数*/
    //* 得到了虚拟摆杆力矩和轮毂电机力矩，存储在u_Vector[4]里面
    //* 使用 LQR_Get_uVector(LQR_StructTypeDef* LQRptr, int index)
    // 可以读取对应的index值
    LQR_Cal(&GstCH_LQRCal);
}

/**
 * @brief  VMC计算处理函数
 * @note   更新VMC的F矩阵数据，然后调用VMC_Cal函数进行VMC计算
 * @param  无
 * @retval 无
 */
//* 底盘VMC计算处理函数
void CH_VMCCal_Process(void) {
    /*VMC的F矩阵（虚拟摆杆力和力矩那个矩阵）更新*/
    VMC_FMatrixUpdate(&GstCH_Leg1VMC, GSTCH_Data.Leg1ForceDes, GSTCH_Data.Leg1TorqueDes, LeftSide);  // 左腿VMC的F矩阵更新
    VMC_FMatrixUpdate(&GstCH_Leg2VMC, GSTCH_Data.Leg2ForceDes, GSTCH_Data.Leg2TorqueDes, RightSide);  // 右腿VMC的F矩阵更新

    /*VMC计算：把等效摆杆的力、力矩转化为关节电机的力矩*/
    VMC_Cal(&GstCH_Leg1VMC, &GstCH_LegLinkCal1);  // 左腿VMC计算
    VMC_Cal(&GstCH_Leg2VMC, &GstCH_LegLinkCal2);  // 右腿VMC计算
}

// #pragma endregion

// #pragma region 各种数据的重置、清零函数

/**各项目标数据的清零、重置**/
/**
  * @brief  轮毂电机目标值数据重置函数
  * @note   把轮毂电机相关的目标值全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void HM_DesDataReset(void)
{
    GSTCH_HM1.TorqueDes  = 0.0f; //左轮毂电机力矩目标值
    GSTCH_HM2.TorqueDes  = 0.0f; //右轮毂电机力矩目标值
    GSTCH_HM1.CurrentDes = 0;    //左轮毂电机电流目标值
    GSTCH_HM2.CurrentDes = 0;    //右轮毂电机电流目标值
}

/**
  * @brief  关节电机目标值数据重置函数
  * @note   把关节电机相关的目标值全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void JM_DesDataReset(void)
{
    GSTCH_JM1.AngleDes    = GSTCH_JM1.AngleFB;  //关节电机1位置目标值
    GSTCH_JM1.AngleVelDes = 0.0f;               //关节电机1角速度目标值
    GSTCH_JM1.TorqueDes   = 0.0f;               //关节电机1力矩目标值

    GSTCH_JM2.AngleDes    = GSTCH_JM2.AngleFB;  //关节电机2位置目标值
    GSTCH_JM2.AngleVelDes = 0.0f;               //关节电机2角速度目标值
    GSTCH_JM2.TorqueDes   = 0.0f;               //关节电机2力矩目标值

    GSTCH_JM3.AngleDes    = GSTCH_JM3.AngleFB;  //关节电机3位置目标值
    GSTCH_JM3.AngleVelDes = 0.0f;               //关节电机3角速度目标值
    GSTCH_JM3.TorqueDes   = 0.0f;               //关节电机3力矩目标值

    GSTCH_JM4.AngleDes    = GSTCH_JM4.AngleFB;  //关节电机4位置目标值
    GSTCH_JM4.AngleVelDes = 0.0f;               //关节电机4角速度目标值
    GSTCH_JM4.TorqueDes   = 0.0f;               //关节电机4力矩目标值
}

/**
  * @brief  底盘TD计算相关目标值数据重置函数
  * @note   把底盘TD计算相关的目标值全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void _CH_TDCal_DataReset(void)
{
    /*腿长TD相关数据重置*/
    TD_Reset(&GstCH_LegLen1TD, GSTCH_Data.LegLen1FB*MM2M); //左腿长度TD重置
    TD_Reset(&GstCH_LegLen2TD, GSTCH_Data.LegLen2FB*MM2M); //右腿长度TD重置
}

/**
  * @brief  底盘PID计算相关目标值数据重置函数
  * @note   把底盘PID计算相关的目标值全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void _CH_PIDCal_DataReset(void)
{
    /*腿长PID相关数据清零*/
    PID_Reset(&GstCH_LegLen1PID, GSTCH_Data.LegLen1FB*MM2M);//左腿PID数据重置
    PID_Reset(&GstCH_LegLen2PID, GSTCH_Data.LegLen2FB*MM2M);//右腿PID数据重置

    /*Roll补偿PID相关数据清零*/
    PID_Reset(&GstCH_RollCompPID, GSTCH_Data.RollAngleFB); //底盘Roll补偿PID数据重置
}

/**
  * @brief  底盘标志位数据重置函数
  * @note   把底盘相关的标志位全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void _CH_Flag_Reset(void)
{
    GFCH_IMU2Restart = IMU2RestartNO;//IMU2重启标志位清零
    GFCH_LegCalibration = 0;         //腿部校准标志位清零
}

/**
  * @brief  底盘数据结构体数据重置函数
  * @note   把底盘数据结构体数据全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void _CH_GSTCH_Data_Reset(void)
{
    /*腿长目标值清零*/
    GSTCH_Data.LegLen1Des = GSTCH_Data.LegLen1FB;   //左腿目标长度清零
    GSTCH_Data.LegLen2Des = GSTCH_Data.LegLen2FB;   //右腿目标长度清零

    /*LQR计算目标值清零*/
    GSTCH_Data.DisDes            = 0.0f;    //x[0]：底盘位移目标值
    GSTCH_Data.VelDes            = 0.0f;    //x[1]：底盘速度目标值
    GSTCH_Data.YawDeltaDes       = 0.0f;    //x[2]：底盘偏航角Yaw增量目标值
    GSTCH_Data.YawAngleVelDes    = 0.0f;    //x[3]：底盘偏航角Yaw速度目标值
    GSTCH_Data.Theta1Des         = 0.0f;    //x[4]：左腿Theta角目标值
    GSTCH_Data.Theta1AngleVelDes = 0.0f;    //x[5]：左腿Theta角速度目标值
    GSTCH_Data.Theta2Des         = 0.0f;    //x[6]：右腿Theta角目标值
    GSTCH_Data.Theta2AngleVelDes = 0.0f;    //x[7]：右腿Theta角速度目标值
    GSTCH_Data.PitchAngleDes     = 0.0f;    //x[8]：底盘俯仰角Pitch目标值
    GSTCH_Data.PitchAngleVelDes  = 0.0f;    //x[9]：底盘俯仰角Pitch速度目标值

    /*VMC计算目标值清零*/
    GSTCH_Data.Leg1ForceDes  = 0.0f; //VMC：左腿力目标值清零
    GSTCH_Data.Leg2ForceDes  = 0.0f; //VMC：右腿力目标值清零
    GSTCH_Data.Leg1TorqueDes = 0.0f; //VMC：左腿力矩目标值清零
    GSTCH_Data.Leg2TorqueDes = 0.0f; //VMC：右腿力矩目标值清零

    /*底盘运动状态重置为制动状态*/
    GSTCH_Data.EM_MoveDirection = MoveDirection_Brake;
}

/**
  * @brief  底盘所有目标值数据重置函数
  * @note   把底盘所有目标值全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void Chassis_AllDesDataReset(void)
{
    /*轮毂电机相关数据重置*/
    HM_DesDataReset();      //轮毂电机目标值重置

    /*关节电机相关数据重置*/
    JM_DesDataReset();      //关节电机目标值重置

    /*底盘的通用算法相关数据重置*/
    _CH_TDCal_DataReset();   //底盘TD计算相关目标值重置
    _CH_PIDCal_DataReset();  //底盘PID计算相关目标值重置

    /*底盘相关数据重置*/
    _CH_GSTCH_Data_Reset();     //底盘数据结构体数据重置
    _CH_Flag_Reset();           //底盘标志位重置
}

/**底盘位移反馈值数据的清零、重置**/
/**
  * @brief  底盘位移反馈值清零函数
  * @note   把底盘位移反馈值清零，通常在初始化时或者进入安全模式时调用
  * @param  无
  * @retval 无
*/
void Chassis_DisFBClear(void)
{
    GSTCH_Data.DisFB = 0.0f;    //底盘位移反馈值清零
}

/**机器人整体控制中，底盘控制数据的清零、重置**/
/**
  * @brief  控制结构体中，默认可配置的底盘相关数据的重置
  * @param  无
  * @retval 无
*/
void Chassis_RobotCtrlDefaultConfigDataReset(void)
{
    /*腿长数据*/
    GST_RMCtrl.STCH_Default.LegLen1Des        = GSTCH_Data.LegLen1FB;
    GST_RMCtrl.STCH_Default.LegLen2Des        = GSTCH_Data.LegLen2FB;
    GST_RMCtrl.STCH_Default.LegLen1ManualDes  = GSTCH_Data.LegLen1FB;
    GST_RMCtrl.STCH_Default.LegLen2ManualDes  = GSTCH_Data.LegLen2FB;

    /*LQR相关数据*/
    GST_RMCtrl.STCH_Default.DisDes          = GSTCH_Data.DisFB;
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;

    /*前馈力相关数据*/
    GST_RMCtrl.STCH_Default.Leg1FFForce      = 0.0f;
    GST_RMCtrl.STCH_Default.Leg2FFForce      = 0.0f;
}

/**
  * @brief  控制结构体中，默认不可配置的底盘相关数据重置
  * @param  无
  * @retval 无
*/
void Chassis_RobotCtrlForceConfigDataReset(void)
{
    /*默认不可配置的LQR相关数据*/
    GST_RMCtrl.STCH_Force.F_LQR_UserSetEnable   = false;
    GST_RMCtrl.STCH_Force.Theta1Des             = GSTCH_Data.Theta1FB;
    GST_RMCtrl.STCH_Force.Theta1AngleVelDes     = 0.0f;
    GST_RMCtrl.STCH_Force.Theta2Des             = GSTCH_Data.Theta2FB;
    GST_RMCtrl.STCH_Force.Theta2AngleVelDes     = 0.0f;
    GST_RMCtrl.STCH_Force.PitchAngleDes         = GSTCH_Data.PitchAngleFB;
    GST_RMCtrl.STCH_Force.PitchAngleVelDes      = 0.0f;

    /*默认不可配置的VMC相关数据*/
    GST_RMCtrl.STCH_Force.F_VMC_UserSetEnable = false;
    GST_RMCtrl.STCH_Force.Leg1FDes = 0.0f;
    GST_RMCtrl.STCH_Force.Leg1TDes = 0.0f;
    GST_RMCtrl.STCH_Force.Leg2FDes = 0.0f;
    GST_RMCtrl.STCH_Force.Leg2TDes = 0.0f;
    
    /*默认不可配置的轮毂电机扭矩数据*/
    GST_RMCtrl.STCH_Force.F_HMTorque_UserSetEnable = false;
    GST_RMCtrl.STCH_Force.HM1TDes = 0.0f;
    GST_RMCtrl.STCH_Force.HM2TDes = 0.0f;
    
    /*默认不可配置的关节电机扭矩数据*/
    GST_RMCtrl.STCH_Force.F_JMTorque_UserSetEnable = false;
    GST_RMCtrl.STCH_Force.JM1TDes = 0.0f;
    GST_RMCtrl.STCH_Force.JM2TDes = 0.0f;
    GST_RMCtrl.STCH_Force.JM3TDes = 0.0f;
    GST_RMCtrl.STCH_Force.JM4TDes = 0.0f;    
}

/**
  * @brief  控制结构体中，底盘所有相关数据的重置函数
  * @param  无
  * @retval 无
*/
void Chassis_RobotCtrlDataReset(void)
{
    Chassis_RobotCtrlDefaultConfigDataReset(); //默认可配置数据重置
    Chassis_RobotCtrlForceConfigDataReset();   //默认不可配置数据重置
}

// #pragma endregion

// #pragma region 底盘其他相关函数
/**
  * @brief  检测是否需要进入手动标定状态的函数（开机后站起前进行）
  * @note   如果处于手动安全模式，并且做出内八手势，则进入手动标定状态
  *         标定关节电机的零点位置
  * @param  无
  * @retval 0：不进入手动标定状态
  *         1：进入手动标定状态
*/
uint8_t IsEnterManualCalibration(void)
{
    /*如果不处于RC手动安全模式，则不进入标定模式*/
    if(GEMCH_Mode != CHMode_RC_ManualSafe)
    {return 0;}

    /*如果做出内八手势，则进入标定状态，否则不进入*/
    if(IsInsideEightGesture() == true)
    {return 1;}

    /*默认不进入手动标定状态*/
    return 0;
}

/**
  * @brief  底盘运动控制更新与处理函数
  * @note   底盘运动控制的主要处理函数
  *         包括腿长目标值更新、LQR计算处理、VMC计算处理、关节电机与轮毂电机目标值更新等
  *         在允许运动的底盘策略Stratgy中调用
  * @param  RMCtrl：RobotControl_StructTypeDef类型，机器人控制结构体变量
  * @retval 无
*/
void CH_MotionUpdateAndProcess(RobotControl_StructTypeDef RMCtrl)
{
    /*******************腿长目标值更新*******************/
    CH_LegLenDes_Update(RMCtrl);    //采用TD算法更新腿长目标值

    /******************LQR的赋值与计算*******************/
    CH_LQR_DesDataUpdate(RMCtrl);   //更新LQR相关数据的目标值
    CH_LQRCal_Process();            //LQR计算处理

    /*******************VMC的赋值与计算******************/
    CH_VMC_DesDataUpdate(RMCtrl);   //更新VMC相关数据的目标值
    CH_VMCCal_Process();            //VMC计算处理

    /*******************更新电机目标值*******************/
    JM_DesDataUpdate(RMCtrl);       //更新关节电机目标值
    HM_DesDataUpdate(RMCtrl);       //更新轮毂电机目标值
}
// #pragma endregion

// #pragma region 底盘控制策略里面使用的相关函数
/**************************************底盘控制策略里面使用的相关函数***************************************************************************/
/*底盘的移动相关函数*/
/**
  * @brief  底盘运动状态选择函数
  * @note   根据左摇杆的前后移动，选择底盘的运动状态
  *         前进、后退、制动三种状态
  * @param  无
  * @retval Chassis_MoveDirection_EnumTypeDef类型，底盘运动状态枚举变量
*/
Chassis_MoveDirection_EnumTypeDef _CH_Move_DirectionChoose(void)
{
    Chassis_MoveDirection_EnumTypeDef MoveDirection;

    /*如果左摇杆Up，则前进*/
    if(IsLeftJoyStickBeyondDeadZoneUp() == true)
    {
        MoveDirection = MoveDirection_Forward;
    }
    /*如果左摇杆Down，则后退*/
    else if(IsLeftJoyStickBeyondDeadZoneDown() == true)
    {
        MoveDirection = MoveDirection_Backward;
    }
    /*否则都是Brake*/
    else
    {
        MoveDirection = MoveDirection_Brake;
    }

    return MoveDirection;
}

/**
  * @brief  底盘速度目标值获取函数
  * @note   根据当前底盘运动状态，计算并获取底盘速度目标值
  * @param  CHData：CHData_StructTypeDef类型，底盘数据结构体指针
  * @retval float类型，底盘速度目标值
*/
float _CH_Move_GetDesVel(CHData_StructTypeDef* CHData)
{
    Chassis_MoveDirection_EnumTypeDef MoveDirection = CHData->EM_MoveDirection; //获取当前底盘运动状态
    bool F_DirectionInvert = CHData->F_DirectionInvert;    //获取底盘运动方向反转标志位

    float VelFBNow   = CHData->VelFB;     //获取当前底盘速度反馈值
    float VelDesPre  = CHData->VelDes;   //获取上次底盘速度目标值
    float VelDesNext = 0.0f;            //定义本次底盘速度目标值变量

    if(MoveDirection == MoveDirection_Forward && F_DirectionInvert == false || MoveDirection == MoveDirection_Backward && F_DirectionInvert == true)
    {
        if(VelFBNow <= -ChMove_StillVelTH)
        {
            VelDesNext = VelFBNow + ChMove_VelBrakingChangeRateMax;
        }
        else
        {
            VelDesNext = StepChangeValue(VelDesPre, ChMove_VelDesMax, ChMove_Acc_Moving * GCH_TaskTime);    //逐步改变目标速度
            VelDesNext = Limit(VelDesNext, VelFBNow + ChMove_VelMovingChangeRateMin, VelFBNow + ChMove_VelMovingChangeRateMax); //限制目标速度变化范围
            VelDesNext = Limit(VelDesNext, ChMove_VelDesMin, ChMove_VelDesMax);                                //限制目标速度最小最大值
        }
    }

    else if(MoveDirection == MoveDirection_Backward && F_DirectionInvert == false || MoveDirection == MoveDirection_Forward && F_DirectionInvert == true)
    {
        if(VelFBNow >= ChMove_StillVelTH)
        {
            VelDesNext = VelFBNow - ChMove_VelBrakingChangeRateMax;
        }
        else
        {
            VelDesNext = StepChangeValue(VelDesPre, -ChMove_VelDesMax, ChMove_Acc_Moving * GCH_TaskTime); //逐步改变目标速度
            VelDesNext = Limit(VelDesNext, VelFBNow - ChMove_VelMovingChangeRateMax, VelFBNow - ChMove_VelMovingChangeRateMin);//限制目标速度变化范围
            VelDesNext = Limit(VelDesNext, -ChMove_VelDesMax, -ChMove_VelDesMin);                                 //限制目标速度最小最大值
        }
    }

    else if (MoveDirection == MoveDirection_Brake)
    {
        VelDesNext = StepChangeValue(VelDesPre, 0.0f, ChMove_Acc_Brake * GCH_TaskTime);
        if(VelFBNow > ChMove_BrakeVelLimitTH)//如果当前速度是正的
        {
            VelDesNext = Limit(VelDesNext, VelFBNow - ChMove_VelBrakingChangeRateMax, VelFBNow);   //限制目标速度变化范围
            VelDesNext = Limit(VelDesNext, 0.0f, 0.8f);  //限制目标速度最小最大值
        }
        else if(VelFBNow < -ChMove_BrakeVelLimitTH)//如果当前速度是负的
        {
            VelDesNext = Limit(VelDesNext, VelFBNow, VelFBNow + ChMove_VelBrakingChangeRateMax);   //限制目标速度变化范围
            VelDesNext = Limit(VelDesNext, -0.8f, 0.0f); //限制目标速度最小最大值
        }
    }

    return VelDesNext;
}

/**
  * @brief  底盘位移目标值处理函数
  * @note   如果底盘没有停下，则把位移目标值设为当前的位移反馈值。即运动过程中位移控制不介入
  * @param  CHData：CHData_StructTypeDef类型，底盘数据结构体指针
  * @param  RMCtrl：RobotControl_StructTypeDef类型，机器人控制结构体指针
  * @retval 无
*/
void _CH_Move_DisHandler(CHData_StructTypeDef* CHData, RobotControl_StructTypeDef *RMCtrl)
{
    float VelFBNow   = CHData->VelFB;     //获取当前底盘速度反馈值

    /*没停下，目标位移等于实际位移*/
    if(MyAbsf(VelFBNow) > ChMove_StillVelTH)
    {
        RMCtrl->STCH_Default.DisDes = CHData->DisFB;
    }
}

/**
  * @brief  底盘转向偏航角速度获取函数
  * @note   根据左摇杆的左右移动，获取底盘转向偏航角速度目标值
  * @param  无
  * @retval float类型，底盘偏航角速度目标值
*/
float _CH_Move_GetTurnVel(CHData_StructTypeDef* CHData)
{
    float YawAngleVelDes_Pre = CHData->YawAngleVelDes; //获取上次偏航角速度目标值
    float YawAngleVelDes_Next = 0.0f;                //定义本次偏航角速度目标值变量

    if(IsLeftJoyStickLeft() == true)
    {YawAngleVelDes_Next = StepChangeValue(YawAngleVelDes_Pre, ChMove_TurnYawVel_Normal, ChMove_YawAngleVelAddStep);}
    else if(IsLeftJoyStickRight() == true)
    {YawAngleVelDes_Next = StepChangeValue(YawAngleVelDes_Pre, -ChMove_TurnYawVel_Normal, ChMove_YawAngleVelAddStep);}
    else
    {YawAngleVelDes_Next = 0.0f;}

    return YawAngleVelDes_Next;
}

/**
  * @brief  底盘移动处理函数
  * @note   底盘移动的主要处理函数
  *         包括底盘运动状态选择、底盘速度目标值获取、底盘位移目标值处理等
  *         在允许运动的底盘策略Stratgy中调用
  * @param  CHData：CHData_StructTypeDef类型，底盘数据结构体指针
  * @param  RMCtrl：RobotControl_StructTypeDef类型，机器人控制结构体指针
  * @retval 无
*/
void ChModeControl_FreeMode_RCControl_MoveHandler(CHData_StructTypeDef* CHData, RobotControl_StructTypeDef *RMCtrl)
{
    /*选择底盘运动状态、获取底盘速度目标值*/
    CHData->EM_MoveDirection = _CH_Move_DirectionChoose();
    RMCtrl->STCH_Default.VelDes = _CH_Move_GetDesVel(CHData);

    /*转向的偏航角速度获取*/
    RMCtrl->STCH_Default.YawAngleVelDes = _CH_Move_GetTurnVel(CHData);

    /*底盘位移目标值处理*/
    _CH_Move_DisHandler(CHData, RMCtrl);
}

/**
  * @brief  RC控制Free模式下，判断陀螺模式进入函数
  * @note   在Free模式下，根据摇杆输入判断是否需要进入陀螺模式
  * @param  CHData：CHData_StructTypeDef类型，底盘数据结构体变量
  * @retval true：进入陀螺模式
  *         false：不进入陀螺模式
*/
bool ChModeControl_FreeMode_RCControl_IsEnterTopMode(CHData_StructTypeDef CHData)
{
    /*提取需要用到的变量*/
    float VelocityFBNow = CHData.VelFB;
    float YawAngleVelFBNow = CHData.YawAngleVelFB;

    /*定义需要用到的变量*/
    static uint32_t RC_TopModeDetectTime = 0;   //用来检测进入陀螺模式的时间变量

    /*************************进入陀螺模式与否的判断****************************/
    /*水平速度过大，不进入陀螺模式（防止运动时进入陀螺导致机器人甩飞出去）*/
    if(VelocityFBNow >= RCTopMode_EnterVelMinTH)
    {
        RC_TopModeDetectTime = 0;
        return false;
    }

    /*Yaw角速度较大，认为处于小陀螺模式，不退出*/
    if(MyAbsf(YawAngleVelFBNow) >= RCTopMode_ExitAngleVelTH)
    {
        return true;
    }

    /*摇杆回中，不进入陀螺模式*/
    if(IsLeftJoyStickLeft() == false && IsLeftJoyStickRight() == false)
    {
        RC_TopModeDetectTime = 0;
        return false;
    }

    /*车子水平速度较小时，检测摇杆左/右偏转时长*/
    /*摇杆持续偏转超过RCTopMode_EnterDelayTime，进入陀螺模式*/
    else if(RC_TopModeDetectTime == 0)
    {
        RC_TopModeDetectTime = RunTimeGet();
    }
    else if(RunTimeGet() - RC_TopModeDetectTime > RCTopMode_EnterDelayTime)
    {
        return true;
    }

    return false;
}

/**
  * @brief  RC控制Free模式下，小陀螺陀螺模式处理函数
  * @note   在Free模式下，陀螺模式的主要处理函数
  *         包括关闭位移控制和速度控制、获取Yaw角速度目标值等
  * @param  CHData：CHData_StructTypeDef类型，底盘数据结构体指针
  * @param  RMCtrl：RobotControl_StructTypeDef类型，机器人控制结构体指针
  * @retval 无
*/
void ChModeControl_FreeMode_RCControl_TopHandler(CHData_StructTypeDef* CHData, RobotControl_StructTypeDef *RMCtrl)
{
    /*关闭位移控制和速度控制*/
    RMCtrl->STCH_Default.DisDes = GSTCH_Data.DisFB; //陀螺模式下不进行前后移动，位移目标值设为当前位移反馈值
    RMCtrl->STCH_Default.VelDes = 0.0f;             //陀螺模式下不进行前后移动，速度目标值设为0

    /*************获取下一个偏航角速度目标值*************/
    float TopAngleVelDes_Pre = RMCtrl->STCH_Default.YawAngleVelDes; //获取上次陀螺模式偏航角速度目标值
    float TopAngleVelDes_Next = 0;

    /*根据摇杆左/右偏转，设定陀螺旋转方向*/
    if(IsLeftJoyStickLeft() == true)
    {TopAngleVelDes_Next = StepChangeValue(GST_RMCtrl.STCH_Default.YawAngleVelDes , RCTopMode_TopAngleVelDesMax , RCTopMode_TopAngleVelAddStep);}
    else if(IsLeftJoyStickRight() == true)
    {TopAngleVelDes_Next = StepChangeValue(GST_RMCtrl.STCH_Default.YawAngleVelDes , -RCTopMode_TopAngleVelDesMax , RCTopMode_TopAngleVelAddStep);}

    /*否则缓慢退出小陀螺*/
    else
    {TopAngleVelDes_Next = StepChangeValue(TopAngleVelDes_Pre , 0.0f , RCTopMode_TopAngleVelBrakeStep);}

    /*赋值给实际控制的结构体成员*/
    RMCtrl->STCH_Default.YawAngleVelDes = TopAngleVelDes_Next;
}
// #pragma endregion
