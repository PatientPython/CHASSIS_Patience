/**
 ******************************************************************************
 * @file    Chassis_APIFunction.c
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.10.25
 * @brief
 * 底盘功能函数，实现底盘的各种功能，把底盘的功能函数放在这里，方便外部调用，
 *          底盘的不同模式控制策略在Chassis_Stratgy.c文件中实现。
 *          如果后面觉得这里的函数太多了，可以考虑拆分文件
 ******************************************************************************
 */
#include "Chassis_APIFunction.h"

#include <arm_math.h>

#include "Algorithm.h"
#include "Algorithm_Simple.h"
#include "Chassis_Stratgy.h"  // ChassisModeChoose_RCControl
#include "General_AuxiliaryFunc.h"
#include "GlobalDeclare_Chassis.h"
#include "GlobalDeclare_General.h"
#include "TIM_Config.h"

// #pragma region 底盘的解算相关函数：只解算，不更改正式结构体的值
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
 * @brief  底盘LQR计算处理函数
 * @note   更新LQR的x状态向量，然后调用LQR_Cal函数进行LQR计算
 * @param  无
 * @retval 无
 */
//* 底盘LQR计算处理函数
void CH_LQRCal_Process(void) {
    /*更新LQR计算的状态向量*/
    LQR_xVector_DataUpdate(
        &GstCH_LQRCal,
        GSTCH_Data.DisDes - GSTCH_Data.DisFB,  // 位移误差
        GSTCH_Data.VelDes - GSTCH_Data.VelFB,  // 速度误差（位移一阶导）
        GSTCH_Data.YawDeltaDes * A2R,  // 偏转角增量（相当于YawAngleDes - YawAngleFB）
        (GSTCH_Data.YawAngleVelDes - GSTCH_Data.YawAngleVelFB) * A2R,  // 偏转角速度误差
        //* 将相对于车身的虚拟摆杆角度转化成了相对于地面的虚拟摆杆角度
        (GSTCH_Data.Theta1Des - (GSTCH_Data.Theta1FB - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP)) * A2R,  // 左腿摆角误差
        (GSTCH_Data.Theta1AngleVelDes - (GSTCH_Data.Theta1AngleVelFB - GSTCH_Data.PitchAngleVelFB)) * A2R,  // 左腿摆角速度误差
        (GSTCH_Data.Theta2Des - (GSTCH_Data.Theta2FB - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP)) * A2R,  // 右腿摆角误差
        (GSTCH_Data.Theta2AngleVelDes - (GSTCH_Data.Theta2AngleVelFB - GSTCH_Data.PitchAngleVelFB)) * A2R,  // 右腿摆角速度误差
        //* 将相对于车身的虚拟摆杆角度转化成了相对于地面的虚拟摆杆角度
        //* 因为底盘质心不在几何中心，ChassisPitchAngleZP用于修正正常平衡时产生的角度偏置
        (GSTCH_Data.PitchAngleDes - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP) * A2R,  // 俯仰角误差
        (GSTCH_Data.PitchAngleVelDes - GSTCH_Data.PitchAngleVelFB) * A2R  // 俯仰角速度误差
    );

    /*更新LQR的K矩阵*/
    // LQR_K_MatrixUpdate(&GstCH_LQRCal, GSTCH_Data.LegLen1FB,
    // GSTCH_Data.LegLen2FB); //根据腿长更新K矩阵
    //* 刚开始腿长是0，然后K矩阵直接就是代入的几个默认值中的一个 LQR_DefaultK_Matrix
    LQR_K_MatrixUpdate(&GstCH_LQRCal, 0.0f, 0.0f);
    // TODO 待修改：暂时不根据腿长更新K矩阵，直接用默认值，后面需要使用拟合时，先写了这个函数里面的拟合，然后解除上面的函数注释，删除该行
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

/**
 * @brief  底盘离地检测计算处理函数
 * @note   更新离地检测相关数据，然后调用离地检测计算函数
 * @param  无
 * @retval 无
 */
//* 底盘离地检测计算处理函数
void CH_SupportForce_Process(void) {
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
 * @brief  底盘侧向惯性前馈力计算处理函数
 * @note   基于物理公式实时更新 LegFFForce_Norm
 * @param  无
 * @retval 无
 */
//* 底盘侧向惯性前馈力计算处理函数
void CH_InertialFF_Process(void) {
    // 当前腿长，单位m
    float l_current = (GSTCH_Data.LegLen1FB + GSTCH_Data.LegLen2FB) / 2.0f * MM2M;
    float YawRate = GSTCH_Data.YawAngleVelFB * A2R;  // 偏航角速度，单位rad/s
    float v_forward = GSTCH_Data.VelFB;              // 前进速度，单位m/s
    //* F_bl,inertial = InertialCoeff * l_current * YawRate * v_forward
    LegFFForce_Inertial_1 = (CH_Phys_InertialCoeff * l_current * YawRate * v_forward);
    LegFFForce_Inertial_2 = (CH_Phys_InertialCoeff * l_current * YawRate * v_forward);
}

/**
 * @brief  底盘卡尔曼滤波速度融合处理函数
 * @note   调用_Ch_VelKF_Process对底盘速度进行卡尔曼滤波融合处理
 * @param  无
 * @retval 无
 */
void CH_VelKF_Process(void) {
    float YawAngleVel = GSTCH_Data.YawAngleVelFB;
    float AngleVel_Wheel1 = GSTCH_HM1.AngleVelFB;  // 左轮轮毂电机角速度，单位rad/s
    float AngleVel_Wheel2 = GSTCH_HM2.AngleVelFB;  // 右轮轮毂电机角速度，单位rad/s
    _Ch_VelKF_Process(AngleVel_Wheel1, AngleVel_Wheel2);
    GSTCH_Data.VelFB = GstCH_VelKF.x_cur;   // 底盘速度滤波、观测处理
    GSTCH_Data.DisFB += GSTCH_Data.VelFB * GCH_TaskTime;  // 位移 = 上次位移 + 速度*时间
    float AngleVel_Wheel1_nxt = (GstCH_VelKF.x_nxt - R_l * YawAngleVel) / R_w;  // 预测的左轮轮毂电机角速度，单位rad/s
    float AngleVel_Wheel2_nxt = (GstCH_VelKF.x_nxt + R_l * YawAngleVel) / R_w;  // 预测的右轮轮毂电机角速度，单位rad/s
    GstCH_VelKF.TorqueComp_L = GstCH_VelKF.K_adapt * (AngleVel_Wheel1_nxt - AngleVel_Wheel1);  // 左轮轮毂电机速度误差补偿力矩
    GstCH_VelKF.TorqueComp_R = GstCH_VelKF.K_adapt * (AngleVel_Wheel2_nxt - AngleVel_Wheel2);  // 右轮轮毂电机速度误差补偿力矩
}

/**
 * @brief  轮毂电机扭矩转电流计算处理函数
 * @note   基于Kt常数和电调映射比例，将 TorqueDes 转换为 CurrentDes
 * @param  pHM：HMData_StructTypeDef类型的电机数据结构体指针
 * @retval 无
 */
//* 轮毂电机扭矩转电流计算处理函数
void CH_HMTorqueToCurrent_Process(HMData_StructTypeDef* pHM) {
    // 1. 扭矩转电流 (I = T / Kt)
    float target_Current = pHM->TorqueDes / HM_Kt;

    // 2. 映射为 C620 的原始控制数值
    float send_Value = target_Current * HM_AmpereToCurrent;

    // 3. 数值限幅 (C620 的范围是 -16384 到 16384)
    Limit(send_Value, HM_MinCurrent, HM_MaxCurrent);

    pHM->CurrentDes = (int16_t)send_Value;
}

// #pragma endregion

// 底盘的数据修改、处理、更新相关函数：允许更改正式结构体的值

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
 * @brief  底盘速度卡尔曼滤波(KF)融合处理函数
 * @note   1. 融合轮毂电机测速（含腿部运动学相对速度）与IMU角速度积分。
 *         2. 动态检测离地或打滑状态，实时调节观测噪声矩阵 R 以维持稳态。
 *         3. 同时返回预测步(Predict)和更新步(Update)的速度值。
 * @param  AngleVel_Wheel1：左轮轮毂电机角速度，单位rad/s
 * @param  AngleVel_Wheel2：右轮轮毂电机角速度，单位rad/s
 * @retval KF_Result_t 包含两个成员的结构体：
 *             - x_nxt: 预测速度（IMU积分预估，x_k|k-1）
 *             - x_cur: 最优估计速度（传感器融合更新后，x_k|k）
 */
//* 底盘速度处理函数
void _Ch_VelKF_Process(float AngleVel_Wheel1, float AngleVel_Wheel2) {
    // 1. 获取测量数据
    float v_wheel = (AngleVel_Wheel1 + AngleVel_Wheel2) / 2.0f * WheelRadius; // 轮子测量的线速度（角速度是低通后的不用再低通了）
    float a_imu = GSTCH_Data.AccXFB; // IMU测量的机体加速度
    float v_rel_leg = (GSTCH_Data.xC1_dot + GSTCH_Data.xC2_dot) / 2.0f; // 运动学补偿 (腿对轮子的相对速度)

    // 重要：观测到的底盘速度 = 轮速 + 腿部摆动速度
    // 这样观测值的物理意义才与 KF 预测的底盘速度(由IMU积分而来)保持一致
    float v_body_obs = v_wheel + v_rel_leg;

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
    GstCH_VelKF.x_nxt = GstCH_VelKF.x[0];

    // 4. 执行速度更新
    KF_ChassisVel_Update(&GstCH_VelKF, v_body_obs, a_imu);
    GstCH_VelKF.x_cur = GstCH_VelKF.x[0];
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
// #pragma region(未实现)各种Des目标数据的修改、处理、更新相关函数
// TODO 更新目标值实现
/**
 * @brief  更新腿长目标值（TD算法）
 * @note   从机器人控制结构体中获取数据，根据TD算法，计算出腿长的目标值
 * @param  RMCtrl：RobotControl_StructTypeDef类型的机器人控制结构体变量
 * @retval 无
 */
//* 更新腿长目标值
// void CH_LegLenDes_Update(RobotControl_StructTypeDef RMCtrl) {

// }
// /**
//  * @brief  更新LQR相关数据的目标值
//  * @note   从机器人控制结构体中，获取数据，更新LQR的计算目标值
//  *         如果用户开启自定义LQR控制，则使用用户设置的所有目标值
//  *         否则除了前四项外，目标值全部采用默认值0
//  * @param  RMCtrl：RobotControl_StructTypeDef类型的机器人控制结构体变量
//  * @retval 无
//  */
// //* 更新LQR相关数据的目标值
// void CH_LQR_DesDataUpdate(RobotControl_StructTypeDef RMCtrl) {}
/**
 * @brief  更新VMC相关数据的目标值
 * @note   从机器人控制结构体中，获取数据，更新VMC的计算目标值
 *         如果用户开启自定义VMC控制，则使用用户设置的目标值
 *         否则采取默认的VMC计算控制
 * @param  RMCtrl：机器人控制结构体变量
 * @retval 无
 */
//* 更新VMC相关数据的目标值
void CH_VMC_DesDataUpdate(RobotControl_StructTypeDef RMCtrl) {}
// #pragma endregion
// #pragma region 各种数据的重置、清零函数

/**各项目标数据的清零、重置**/
/**
 * @brief  轮毂电机目标值数据重置函数
 * @note   把轮毂电机相关的目标值全部重置，机器人未运动时调用
 * @param  无
 * @retval 无
 */
void HM_DesDataReset(void) {
    GSTCH_HM1.TorqueDes = 0.0f;  // 左轮毂电机力矩目标值
    GSTCH_HM2.TorqueDes = 0.0f;  // 右轮毂电机力矩目标值
    GSTCH_HM1.CurrentDes = 0;    // 左轮毂电机电流目标值
    GSTCH_HM2.CurrentDes = 0;    // 右轮毂电机电流目标值
}

/**
 * @brief  关节电机目标值数据重置函数
 * @note   把关节电机相关的目标值全部重置，机器人未运动时调用
 * @param  无
 * @retval 无
 */
void JM_DesDataReset(void) {
    GSTCH_JM1.AngleDes = GSTCH_JM1.AngleFB;  // 关节电机1位置目标值
    GSTCH_JM1.AngleVelDes = 0.0f;            // 关节电机1角速度目标值
    GSTCH_JM1.TorqueDes = 0.0f;              // 关节电机1力矩目标值

    GSTCH_JM2.AngleDes = GSTCH_JM1.AngleFB;  // 关节电机2位置目标值
    GSTCH_JM2.AngleVelDes = 0.0f;            // 关节电机2角速度目标值
    GSTCH_JM2.TorqueDes = 0.0f;              // 关节电机2力矩目标值

    GSTCH_JM3.AngleDes = GSTCH_JM1.AngleFB;  // 关节电机3位置目标值
    GSTCH_JM3.AngleVelDes = 0.0f;            // 关节电机3角速度目标值
    GSTCH_JM3.TorqueDes = 0.0f;              // 关节电机3力矩目标值

    GSTCH_JM4.AngleDes = GSTCH_JM1.AngleFB;  // 关节电机4位置目标值
    GSTCH_JM4.AngleVelDes = 0.0f;            // 关节电机4角速度目标值
    GSTCH_JM4.TorqueDes = 0.0f;              // 关节电机4力矩目标值
}

/**
 * @brief  底盘TD计算相关目标值数据重置函数
 * @note   把底盘TD计算相关的目标值全部重置，机器人未运动时调用
 * @param  无
 * @retval 无
 */
void _CH_TDCal_DataReset(void) {
    /*腿长TD相关数据重置*/
    TD_Reset(&GstCH_LegLen1TD, GSTCH_Data.LegLen1FB * MM2M);  // 左腿长度TD重置
    TD_Reset(&GstCH_LegLen2TD, GSTCH_Data.LegLen2FB * MM2M);  // 右腿长度TD重置
}

/**
 * @brief  底盘PID计算相关目标值数据重置函数
 * @note   把底盘PID计算相关的目标值全部重置，机器人未运动时调用
 * @param  无
 * @retval 无
 */
void _CH_PIDCal_DataReset(void) {
    /*腿长PID相关数据清零*/
    PID_Reset(&GstCH_LegLen1PID,
              GSTCH_Data.LegLen1FB * MM2M);  // 左腿PID数据重置
    PID_Reset(&GstCH_LegLen2PID,
              GSTCH_Data.LegLen2FB * MM2M);  // 右腿PID数据重置

    /*Roll补偿PID相关数据清零*/
    PID_Reset(&GstCH_RollCompPID,
              GSTCH_Data.RollAngleFB);  // 底盘Roll补偿PID数据重置
}

/**
 * @brief  底盘标志位数据重置函数
 * @note   把底盘相关的标志位全部重置，机器人未运动时调用
 * @param  无
 * @retval 无
 */
void _CH_Flag_Reset(void) {
    GFCH_IMU2Restart = IMU2RestartNO;  // IMU2重启标志位清零
    GFCH_LegCalibration = 0;           // 腿部校准标志位清零
}

/**
 * @brief  底盘数据结构体数据重置函数
 * @note   把底盘数据结构体数据全部重置，机器人未运动时调用
 * @param  无
 * @retval 无
 */
void _CH_GSTCH_Data_Reset(void) {
    /*腿长目标值清零*/
    GSTCH_Data.LegLen1Des = GSTCH_Data.LegLen1FB;  // 左腿目标长度清零
    GSTCH_Data.LegLen2Des = GSTCH_Data.LegLen2FB;  // 右腿目标长度清零

    /*LQR计算目标值清零*/
    GSTCH_Data.DisDes = 0.0f;             // x[0]：底盘位移目标值
    GSTCH_Data.VelDes = 0.0f;             // x[1]：底盘速度目标值
    GSTCH_Data.YawDeltaDes = 0.0f;        // x[2]：底盘偏航角Yaw增量目标值
    GSTCH_Data.YawAngleVelDes = 0.0f;     // x[3]：底盘偏航角Yaw速度目标值
    GSTCH_Data.Theta1Des = 0.0f;          // x[4]：左腿Theta角目标值
    GSTCH_Data.Theta1AngleVelDes = 0.0f;  // x[5]：左腿Theta角速度目标值
    GSTCH_Data.Theta2Des = 0.0f;          // x[6]：右腿Theta角目标值
    GSTCH_Data.Theta2AngleVelDes = 0.0f;  // x[7]：右腿Theta角速度目标值
    GSTCH_Data.PitchAngleDes = 0.0f;      // x[8]：底盘俯仰角Pitch目标值
    GSTCH_Data.PitchAngleVelDes = 0.0f;   // x[9]：底盘俯仰角Pitch速度目标值

    /*VMC计算目标值清零*/
    GSTCH_Data.Leg1ForceDes = 0.0f;   // VMC：左腿力目标值清零
    GSTCH_Data.Leg2ForceDes = 0.0f;   // VMC：右腿力目标值清零
    GSTCH_Data.Leg1TorqueDes = 0.0f;  // VMC：左腿力矩目标值清零
    GSTCH_Data.Leg2TorqueDes = 0.0f;  // VMC：右腿力矩目标值清零

    /*底盘运动状态重置为制动状态*/
    GSTCH_Data.EM_MoveDirection = MoveDirection_Brake;
}

/**
 * @brief  底盘所有目标值数据重置函数
 * @note   把底盘所有目标值全部重置，机器人未运动时调用
 * @param  无
 * @retval 无
 */
void Chassis_AllDesDataReset(void) {
    /*轮毂电机相关数据重置*/
    HM_DesDataReset();  // 轮毂电机目标值重置

    /*关节电机相关数据重置*/
    JM_DesDataReset();  // 关节电机目标值重置

    /*底盘的通用算法相关数据重置*/
    _CH_TDCal_DataReset();   // 底盘TD计算相关目标值重置
    _CH_PIDCal_DataReset();  // 底盘PID计算相关目标值重置

    /*底盘相关数据重置*/
    _CH_GSTCH_Data_Reset();  // 底盘数据结构体数据重置
    _CH_Flag_Reset();        // 底盘标志位重置
}

/**底盘位移反馈值数据的清零、重置**/
/**
 * @brief  底盘位移反馈值清零函数
 * @note   把底盘位移反馈值清零，通常在初始化时或者进入安全模式时调用
 * @param  无
 * @retval 无
 */
//* 底盘位移反馈值清零函数
void Chassis_DisFBClear(void) {
    GSTCH_Data.DisFB = 0.0f;  // 底盘位移反馈值清零
}

//! 与安全模式相关
/**
 * @brief  控制结构体中，默认可配置的底盘相关数据的重置
 * @param  无
 * @retval 无
 */
//* 默认可配置的底盘相关数据的重置
void Chassis_RobotCtrlDefaultConfigDataReset(void) {
    /*腿长数据*/
    GST_RMCtrl.STCH_Default.LegLen1Des = GSTCH_Data.LegLen1FB;
    GST_RMCtrl.STCH_Default.LegLen2Des = GSTCH_Data.LegLen2FB;

    /*LQR相关数据*/
    GST_RMCtrl.STCH_Default.DisDes = GSTCH_Data.DisFB;
    GST_RMCtrl.STCH_Default.VelDes = 0.0f;
    GST_RMCtrl.STCH_Default.YawDeltaDes = 0.0f;
    GST_RMCtrl.STCH_Default.YawAngleVelDes = 0.0f;

    /*前馈力相关数据*/
    GST_RMCtrl.STCH_Default.Leg1FFForce = 0.0f;
    GST_RMCtrl.STCH_Default.Leg2FFForce = 0.0f;
}

/**
 * @brief  控制结构体中，默认不可配置的底盘相关数据重置
 * @param  无
 * @retval 无
 */
//* 默认不可配置的底盘相关数据重置
void Chassis_RobotCtrlForceConfigDataReset(void) {
    /*默认不可配置的LQR相关数据*/
    GST_RMCtrl.STCH_Force.F_LQR_UserSetEnable = false;
    GST_RMCtrl.STCH_Force.Theta1Des = GSTCH_Data.Theta1FB;
    GST_RMCtrl.STCH_Force.Theta1AngleVelDes = 0.0f;
    GST_RMCtrl.STCH_Force.Theta2Des = GSTCH_Data.Theta2FB;
    GST_RMCtrl.STCH_Force.Theta2AngleVelDes = 0.0f;
    GST_RMCtrl.STCH_Force.PitchAngleDes = GSTCH_Data.PitchAngleFB;
    GST_RMCtrl.STCH_Force.PitchAngleVelDes = 0.0f;

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
//* 调用前两个函数，重置底盘所有相关数据
void Chassis_RobotCtrlDataReset(void) {
    Chassis_RobotCtrlDefaultConfigDataReset();  // 默认可配置数据重置
    Chassis_RobotCtrlForceConfigDataReset();    // 默认不可配置数据重置
    //! 与安全模式相关
}
//! 与安全模式相关

// #pragma endregion
// #pragma region 底盘其他相关函数
/**
 * @brief  手动校准状态检测函数
 * @note   检测当前是否处于手动校准状态
 * @param  无
 * @retval 返回1表示处于手动校准状态，返回0表示不处于手动校准状态
 */
//* 手动校准状态检测函数
// uint8_t IsEnterManualCalibration(void) {
// if (GFCH_LegCalibration == 1) {
//     return 1;
// } else {
//     return 0;
// }
//}

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
    ChassisStratgy_ModeChooseParaStructUpdate(&ST_ModeChoosePara_tmp);  // 更新底盘模式选择参数结构体
    //* 实现的是局部变量：ModeTemp、MC_ModePre 向全局变量
    // GEMCH_ModePre、GEMCH_Mode 赋值
    GEMCH_ModePre = GEMCH_Mode;
    //! 这个函数会在这里检测当前是否有遥控器指令指示需要切换到什么状态
    GEMCH_Mode = ChassisStratgy_ModeChoose_RCControl(ST_ModeChoosePara_tmp);  // 调用底盘模式选择函数
    ChassisStratgy_ModeStartTimeUpdate(&GSTCH_Data, GEMCH_Mode, GEMCH_ModePre);
}
// #pragma endregion
