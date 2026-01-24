/**
 ******************************************************************************
 * @file    Algorithm.h
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.9.19
 * @brief   写各种偏复杂的算法，例如PID、LQR等等，以此方便外部调用
 ******************************************************************************
 */
#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "GlobalDeclare_General.h"

// #pragma region
// /****结构体声明*******************************************************************************/
/*PID算法结构体*/
typedef struct {
    /*需要初始化时赋值的成员*/
    float Kp;  // 比例系数Kp
    float Ki;  // 积分系数Ki
    float Kd;  // 微分系数Kd

    float UMax;   // 总输出最大值
    float UpMax;  // Kp项输出最大值
    float UiMax;  // Ki项输出最大值
    float UdMax;  // Kd项输出最大值

    float AddMax;  // SumE单次累加的最大值

    /*不需要初始化时赋值的成员*/
    float Des;  // 控制变量目标值
    float FB;   // 控制变量反馈值

    float Err;     // 本次偏差
    float PreErr;  // 上次偏差
    float SumErr;  // 总偏差

    float Up;  // Kp输出
    float Ui;  // Ki输出
    float Ud;  // Kd输出
    float U;   // PID计算结果值/总输出
} PID_StructTypeDef;

/*TD算法结构体*/
typedef struct {
    /*需要初始化时赋值的成员*/
    float
        r;  // 速度因子，决定TD跟踪的速度。越大x₁跟踪v越快，但微分信号x₂的噪声也会越大。
    float h0;          // 滤波因子，通常取Ts的整数倍。越大滤波效果越强
    float SampleTime;  // 采样时间，单位：s

    /*不需要初始化时赋值的成员*/
    float v;   // 原始输入
    float x1;  // 输出信号，v的平滑跟踪值
    float x2;  // 输出的导数，即x1的导数
} TD_StructTypeDef;

/*机器人底盘LQR计算结构体*/
typedef struct {
    float K_Matrix[4][10];  // LQR控制器的K矩阵，u = K*(x_Des - x_FB)
    float x_Vector[10];  // 状态向量x（目标值-实际值）x_Vector = [s, s_dot, yaw,
                         // yaw_dot, theta_ll, theta_dot_ll, theta_lr,
                         // theta_dot_lr, pitch_b, pitch_dot_b];
    float u_Vector[4];   // 控制向量u               u_Vector =
                         // [T_wl,T_wr,T_bl,T_br];
} LQR_StructTypeDef;

/*腿部五连杆解算结构体，具体变量定义见ReadMe中算法说明*/
typedef struct {
    /*需要初始化时赋值的变量*/
    float l1, l2, l3, l4,
        l5;  // 五连杆各杆长度，1、4为大腿，2、3为小腿，5为关节电机间距，一般取单位mm
    float phi1ZP,
        phi4ZP;  // phi1、phi4对应的关节电机编码器零点（相对于五连杆解算结构体的坐标系），单位：度
    float SampleTime;  // 采样时间，单位：秒

    /*不需要初始化时赋值的变量*/
    float xA, xB, xC, xD, xE;      // 五连杆各连接点坐标
    float yA, yB, yC, yD, yE;      // 五连杆各连接点坐标
    float phi1, phi4, phi2, phi3;  // 大腿夹角、小腿夹角
    float L0,
        phi0;  // 等效连杆的长度及夹角，此处phi0为等效连杆与水平线的夹角（也就是在中间的时候是90度）

    float phi1_dot, phi4_dot;  // phi1、phi4对应的关节电机角速度，单位：度/s
    float xC_Pre, xC_dot;      // C点x坐标上次值、C点x坐标的微分，单位：mm/s
} LegLinkageCal_StructTypeDef;

/*VMC算法结构体*/
typedef struct {
    float J_Matrix[2][2];  // 转化矩阵，T = J * F
    float F_Matrix[2];     // 等效摆杆的力和力矩，F = [F,
                           // Tp]，沿杆方向的力、绕杆的力矩
    float T_Matrix[2];     // 解算到关节电机上的力矩，T = [T1,
                           // T2]，对应A点、E点的电机
} VMC_StructTypeDef;

/*一阶低通滤波器结构体*/
typedef struct {
    /*需要初始化赋值的成员*/
    float Alpha;  // 滤波系数，越小滤波越强（如果滤波系数赋值了，则CutOffFreq和SampleTime无效）

    float CutOffFreq;  // 滤波器截止频率
    float SampleTime;  // 采样时间，单位：秒

    /*不需要初始化赋值的成员*/
    float Input;  // 滤波器输入值
    float Out;    // 滤波器输出值
} LPF_StructTypeDef;

/*卡尔曼滤波结构体*/
typedef struct {
    // 状态向量 x [0]:vel, [1]:acc
    float x[2];      
    // 协方差矩阵 P (2x2)
    float P[2][2];   
    // 过程噪声协方差 Q (2x2) - 相信模型的程度
    float Q[2][2];   
    // 测量噪声协方差 R (2x2) - 相信传感器的程度
    float R[2][2];   
    // 卡尔曼增益 K (2x2)
    float K[2][2];   
    // 采样时间
    float dt; 
    // 当前底盘速度测量值
    float VelFB;
    // 下一时刻底盘速度预测值
    float VelNxt;
} KF_StructTypeDef;

/*离地检测算法结构体*/
typedef struct {
    /*需要初始化赋值的成员*/
    float M_w;      //单个轮子的质量，单位千克
    float M_l;      //单个腿部五连杆的质量，单位千克（注意原解算是忽略了腿部质量的，这里加上了，如果可以忽略的话直接给0就行）
    float g;        //当地重力加速度，单位：m/s²
    float SampleTime;   //采样时间，单位：秒

    /*不需要初始化赋值的成员*/ 
    float L1, L4;                   //五连杆中大腿杆的长度，单位：米
    float Phi1, Phi2, Phi3, Phi4;   //五连杆各杆夹角，单位：度
    float Phi0;                     //五连杆坐标系下，等效摆杆与水平线夹角，单位：度
    float L0;                       //等效摆杆长度，单位：米
    float PitchAngle;               //底盘俯仰角，单位：度
    float Theta;                    //五连杆坐标系下，摆杆与竖直方向的角度（注意不是地面坐标系），单位：度
    float PitchAngleVel;            //底盘俯仰角速度，单位：度/s

    float L0_dot;                   //等效摆杆长度的速度，单位：m/s
    float L0_dot_pre;               //等效摆杆长度的速度上次值，单位：m/s
    float L0_ddot;                  //等效摆杆长度的加速度，单位：m/s²

    float Theta_dot;                //五连杆坐标系下，摆杆与竖直方向的角速度，单位：度/s
    float Theta_dot_pre;            //五连杆坐标系下，摆杆与竖直方向的角速度上次值，单位：度/s
    float Theta_ddot;               //五连杆坐标系下，摆杆与竖直方向的角加速度，单位：度/s²

    float Phi;          //地面坐标系下，摆杆与竖直方向夹角，单位：度
    float Phi_dot;      //摆杆角速度，单位：度/s
    float Phi_dot_pre;  //摆杆角速度上次值，单位：度/s
    float Phi_ddot;     //摆杆角加速度，单位：度/s²

    float T1, T4;       //五连杆中phi1、phi4关节电机力矩，单位：N·m

    float F_Leg;        //等效摆杆沿杆的力，单位：N
    float Tp_Leg;       //等效摆杆的扭矩，单位：N·m

    float ZAcc_Wheel;   //轮子Z轴加速度，单位：m/s²
    float ZAcc_Body;    //底盘Z轴加速度，单位：m/s²

    float F_N;          //地面给轮子的支持力，单位：N
} OffGround_StructTypeDef;

/* 轮毂电机模型自适应补偿结构体 */
typedef struct {
    /*需要初始化赋值的成员*/
    // 比例系数
    float K_Trac_Norm;     // 处于正常模式的牵引补偿系数
    float K_Trac_Strg;     // 处于脱困模式的牵引补偿系数
    float K_Stab;          // 稳定补偿系数

    // 补偿相关变量
    float Max_HM_Comp_Ratio;     // 轮毂电机补偿力矩最大比例值
    float Weight_HM1;            // 左轮稳定补偿力矩权重
    float Weight_HM2;            // 右轮稳定补偿力矩权重

    // 轮速误差相关变量
    float Err_Sat;         // error_saturation 误差饱和限幅值
    float Err_DZ;          // error_deadzone 误差死区限幅值

    /*不需要初始化赋值的成员*/
    // 补偿力矩
    float T_Trac_HM1;      // 左轮牵引补偿力矩, 单位: Nm
    float T_Trac_HM2;      // 右轮牵引补偿力矩, 单位: Nm
    float T_Stab_HM1;      // 左轮稳定补偿力矩, 单位: Nm
    float T_Stab_HM2;      // 右轮稳定补偿力矩, 单位: Nm
    float T_Comp_HM1;      // 左轮总补偿力矩, 单位: Nm
    float T_Comp_HM2;      // 右轮总补偿力矩, 单位: Nm

    float Err_HM1;           // 左轮轮速误差 E_l = \dot\theta_{w,l}R_w - (\hat{\dot{s}} - R_l\hat{\dot{\phi}}) * R_w
    float Err_HM2;           // 右轮轮速误差 E_r = \dot\theta_{w,r}R_w - (\hat{\dot{s}} + R_l\hat{\dot{\phi}}) * R_w
    float Lambda_HM1;        // 左轮速误差归一化值（状态强度系数）
    float Lambda_HM2;        // 右轮速误差归一化值（状态强度系数）
    // 打滑和受阻的标志位写在了底盘数据总结构体里面

} HM_TorqueComp_StructTypeDef;
// #pragma endregion

/*************************************函数声明**************************************/

// PID相关函数全家桶
void PID_StructInit(PID_StructTypeDef* PIDptr, float Kp, float Ki, float Kd, float UMax, float UpMax, float UiMax, float UdMax, float AddMax);
void PID_SetDes(PID_StructTypeDef* PIDptr, float NewDes);
void PID_SetFB(PID_StructTypeDef* PIDptr, float NewFB);
void PID_SetKpKiKd(PID_StructTypeDef* PIDptr, float NewKp, float NewKi, float NewKd);
void PID_Cal(PID_StructTypeDef* PIDptr);
float PID_GetOutput(PID_StructTypeDef* PIDptr);
void PID_Reset(PID_StructTypeDef* PIDptr, float FBValue);

// TD相关函数全家桶
void TD_StructInit(TD_StructTypeDef* TDptr, float r, float h0, float SampleTime);
void TD_SetInput(TD_StructTypeDef* TDptr, float Input);
void TD_Setr(TD_StructTypeDef* TDptr, float New_r);
void TD_Cal(TD_StructTypeDef* TDptr);
float TD_GetOutput(TD_StructTypeDef* TDptr);
void TD_Reset(TD_StructTypeDef* TDptr, float FBValue);

// LPF相关函数全家桶
void LPF_StructInit(LPF_StructTypeDef* LPFptr, float Alpha, float CutOffFreq, float SampleTime);
void LPF_SetInput(LPF_StructTypeDef* LPFptr, float NewInput);
void LPF_Cal(LPF_StructTypeDef* LPFptr);
float LPF_GetOutput(LPF_StructTypeDef* LPFptr);

// 五连杆解算相关函数全家桶
void LegLinkage_StructInit(LegLinkageCal_StructTypeDef* LegPtr, float l1, float l2, float l3, float l4, float l5, float phi1ZP, float phi4ZP, float SampleTime);
void LegLinkage_AngleDataUpdate(LegLinkageCal_StructTypeDef* LegPtr, float JMPosValue1, float JMPosValue2);
void LegLinkage_AngleVelDataUpdate(LegLinkageCal_StructTypeDef* LegPtr, float JMAngleVel1, float JMVelValue2);
void LegLinkage_ForwardKinematicsCal(LegLinkageCal_StructTypeDef* LegPtr);
float LegLinkage_GetLegLength(LegLinkageCal_StructTypeDef* LegPtr);
float LegLinkage_GetL0Length(LegLinkageCal_StructTypeDef* LegPtr);
float LegLinkage_GetTheta(LegLinkageCal_StructTypeDef* LegPtr, RobotSide_EnumTypeDef LegSide);
float LegLinkage_GetxCdot(LegLinkageCal_StructTypeDef* LegPtr, RobotSide_EnumTypeDef LegSide);
float LegLinkage_GetL0dot(LegLinkageCal_StructTypeDef* LegPtr);
float LegLinkage_GetThetadot(LegLinkageCal_StructTypeDef* LegPtr, RobotSide_EnumTypeDef LegSide);

// LQR相关函数全家桶
// LQR不用单独的初始化函数，LQR_xVector_DataUpdate代劳了
void LQR_xVector_DataUpdate(LQR_StructTypeDef* LQRptr, float DisErr, float VelErr, float YawAngleErr, float YawAngleVelErr, float Theta1AngleErr, float Theta1AngleVelErr, float Theta2AngleErr, float Theta2AngleVelErr, float PitchAngleErr, float PitchAngleVelErr);
void LQR_K_MatrixUpdate(LQR_StructTypeDef* LQRptr, float LegLen1, float LegLen2);
void LQR_Cal(LQR_StructTypeDef* LQRptr);
float LQR_Get_uVector(LQR_StructTypeDef* LQRptr, int index);

// VMC相关函数全家桶    
void VMC_FFForceUpdate(RobotControl_StructTypeDef* CHData);
void VMC_FMatrixUpdate(VMC_StructTypeDef* VMCptr, float Force, float Torque, RobotSide_EnumTypeDef LegSide);
void VMC_Cal(VMC_StructTypeDef* VMCptr, LegLinkageCal_StructTypeDef* LegPtr);
float VMC_Get_TMatrix(VMC_StructTypeDef* VMCptr, int index);

// 离地检测相关函数全家桶
void OffGround_StructInit(OffGround_StructTypeDef *pOffGrd, float Mass_Wheel, float Mass_LegLinkage, float g, float SampleTime);
void OffGround_BodyZAccUpdate(OffGround_StructTypeDef *pOffGrd, float AccZ_Body);
void OffGround_PitchAngleUpdate(OffGround_StructTypeDef *pOffGrd, float PitchAngle);
void OffGround_PitchAngleVelUpdate(OffGround_StructTypeDef *pOffGrd, float PitchAngleVel);
void OffGround_LegLinkRelateDataUpdate(LegLinkageCal_StructTypeDef LegPtr, OffGround_StructTypeDef *pOffGrd, RobotSide_EnumTypeDef LegSide);
void OffGround_TorqueDataUpdate(OffGround_StructTypeDef *pOffGrd, float T1, float T4);
void OffGround_GetRealFAndTp(OffGround_StructTypeDef *pOffGrd);
float OffGround_GetLegToWheelForce(OffGround_StructTypeDef *pOffGrd);
float OffGround_GetSupportForce(OffGround_StructTypeDef *pOffGrd);

// 卡尔曼滤波相关函数全家桶
void KF_ChassisVel_StructInit(KF_StructTypeDef *KFptr, float dt);
void KF_ChassisVel_Predict(KF_StructTypeDef* KFptr);
void KF_ChassisVel_Update(KF_StructTypeDef* KFptr, float v_body_obs, float a_imu);

// 轮毂电机模型自适应补偿相关函数全家桶
// TODO 这个位置考虑怎么还得改一下
void HM_TorqueComp_StructInit(HM_TorqueComp_StructTypeDef* pHMComp, float K_Trac_Norm, float K_Trac_Strg, float K_Stab, float Max_HM_Comp_Ratio, float Weight_HM1, float Weight_HM2, float Err_Sat, float Err_DZ);
#endif
