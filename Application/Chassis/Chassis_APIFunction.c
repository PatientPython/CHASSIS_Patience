/**
  ******************************************************************************
  * @file    Chassis_APIFunction.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.25
  * @brief   底盘功能函数，实现底盘的各种功能，把底盘的功能函数放在这里，方便外部调用，
  *          底盘的不同模式控制策略在Chassis_Stratgy.c文件中实现。
  *          如果后面觉得这里的函数太多了，可以考虑拆分文件
  ******************************************************************************
*/
#include <arm_math.h>
#include "Algorithm.h"
#include "Algorithm_Simple.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Chassis.h"
#include "General_AuxiliaryFunc.h"
#include "Chassis_APIFunction.h"
#include "TIM_Config.h"

//#region /*******************************底盘的解算相关函数：只解算，不更改正式结构体的值********************************************/
/**
  * @brief  底盘腿部五连杆解算处理函数
  * @note   第一步：更新五连杆解算的数据，主要是phi1、phi4
  *         第二步：进行五连杆正运动学解算，由关节角度求解末端位置
  *         第三步：把解算后的数据分发到正式变量/正式结构体中
  * @param  无
  * @retval 无
*/
void CH_LegLinkageCal_Process(void)
{
    /*更新五连杆解算的phi1、phi4数据*/
    LegLinkage_AngleDataUpdate(&GstCH_LegLinkCal1, GSTCH_JM3.AngleFB, GSTCH_JM1.AngleFB); //更新左侧数据
    LegLinkage_AngleDataUpdate(&GstCH_LegLinkCal2, GSTCH_JM2.AngleFB, GSTCH_JM4.AngleFB); //更新右侧数据

    /*更新五连杆解算的phi1_dot、phi4_dot数据*/
    LegLinkage_AngleVelDataUpdate(&GstCH_LegLinkCal1, GSTCH_JM3.AngleVelFB, GSTCH_JM1.AngleVelFB); //更新左侧数据
    LegLinkage_AngleVelDataUpdate(&GstCH_LegLinkCal2, GSTCH_JM2.AngleVelFB, GSTCH_JM4.AngleVelFB); //更新右侧数据

    /*五连杆正运动学解算*/
    LegLinkage_ForwardKinematicsCal(&GstCH_LegLinkCal1); // 左腿五连杆运动学正解
    LegLinkage_ForwardKinematicsCal(&GstCH_LegLinkCal2); // 右腿五连杆运动学正解
}

/**
  * @brief  底盘LQR计算处理函数
  * @note   更新LQR的x状态向量，然后调用LQR_Cal函数进行LQR计算
  * @param  无
  * @retval 无
*/
void CH_LQRCal_Process(void)
{
    /*更新LQR计算的状态向量*/
    LQR_xVector_DataUpdate(&GstCH_LQRCal,
                            GSTCH_Data.DisDes            - GSTCH_Data.DisFB,               //位移误差
                            GSTCH_Data.VelDes            - GSTCH_Data.VelFB,               //速度误差（位移一阶导）
           
                            GSTCH_Data.YawDeltaDes * A2R,                                   //偏转角增量（相当于YawAngleDes - YawAngleFB）
                           (GSTCH_Data.YawAngleVelDes    - GSTCH_Data.YawAngleVelFB)*A2R,  //偏转角速度误差
           
                           (GSTCH_Data.Theta1Des         - (GSTCH_Data.Theta1FB - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP)) * A2R,   //左腿摆角误差
                           (GSTCH_Data.Theta1AngleVelDes - (GSTCH_Data.Theta1AngleVelFB - GSTCH_Data.PitchAngleVelFB)) * A2R,              //左腿摆角速度误差
           
                           (GSTCH_Data.Theta2Des         - (GSTCH_Data.Theta2FB - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP)) * A2R,   //右腿摆角误差
                           (GSTCH_Data.Theta2AngleVelDes - (GSTCH_Data.Theta2AngleVelFB - GSTCH_Data.PitchAngleVelFB)) * A2R,              //右腿摆角速度误差
           
                           (GSTCH_Data.PitchAngleDes     - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP) * A2R, //俯仰角误差
                           (GSTCH_Data.PitchAngleVelDes  - GSTCH_Data.PitchAngleVelFB) * A2R                     //俯仰角速度误差
                          );
    
    /*更新LQR的K矩阵*/
    // LQR_K_MatrixUpdate(&GstCH_LQRCal, GSTCH_Data.LegLen1FB, GSTCH_Data.LegLen2FB); //根据腿长更新K矩阵
    LQR_K_MatrixUpdate(&GstCH_LQRCal, 0.0f, 0.0f); //待修改：暂时不根据腿长更新K矩阵，直接用默认值，后面需要使用拟合时，先写了这个函数里面的拟合，然后解除上面的函数注释，删除该行
    
    /*调用LQR计算函数*/
    LQR_Cal(&GstCH_LQRCal);
}

/**
  * @brief  VMC计算处理函数
  * @note   更新VMC的F矩阵数据，然后调用VMC_Cal函数进行VMC计算
  * @param  无
  * @retval 无
*/
void CH_VMCCal_Process(void)
{
    /*VMC的F矩阵更新*/
    VMC_FMatrixUpdate(&GstCH_Leg1VMC, GSTCH_Data.Leg1ForceDes, GSTCH_Data.Leg1TorqueDes, LeftSide);  //左腿VMC的F矩阵更新
    VMC_FMatrixUpdate(&GstCH_Leg2VMC, GSTCH_Data.Leg2ForceDes, GSTCH_Data.Leg2TorqueDes, RightSide); //右腿VMC的F矩阵更新

    /*VMC计算：把等效摆杆的力、力矩转化为关节电机的力矩*/
    VMC_Cal(&GstCH_Leg1VMC, &GstCH_LegLinkCal1); //左腿VMC计算
    VMC_Cal(&GstCH_Leg2VMC, &GstCH_LegLinkCal2); //右腿VMC计算    
}
//#endregion

/*****************************************底盘的数据修改、处理、更新相关函数：允许更改正式结构体的值***********************************/
//#region /********各种FB反馈数据的修改、处理、更新相关函数*************/
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
void _HM_FBData_Parse(RobotSide_EnumTypeDef Side, HMData_StructTypeDef* pHM, C620FeedBackData_StructTypeDef* pESC, LPF_StructTypeDef *pLPF)
{
    /*计算原始速度值*/
    float ReductionGearboxAngleVel = pESC->AngleVelFB / pHM->ReductionRatio;  //减速箱输出轴角速度，单位r/min
    float RawAngleVel = ReductionGearboxAngleVel * 2 * PI / 60.0f;            //减速箱输出轴角速度，单位rad/s

    /*低通滤波处理速度值*/
    LPF_SetInput(pLPF, RawAngleVel); //设置低通滤波器输入值
    LPF_Cal(pLPF);                   //调用低通滤波计算函数

    /*轮毂电机速度反馈赋值*/
    if(Side == LeftSide)
    {pHM->AngleVelFB = -LPF_GetOutput(pLPF);}
    else if(Side == RightSide)
    {pHM->AngleVelFB = LPF_GetOutput(pLPF);}

    /*电流值和温度值*/
    pHM->CurrentFB = pESC->CurrentFB;   //电流值
    pHM->TempFB    = pESC->TempFB;      //温度值
}

/**
  * @brief  底盘数据低通滤波处理
  * @note   调用LPF_Cal对底盘相关数据进行低通滤波处理
  * @param  RawData：需要滤波的原始数据
  * @param  pLPF：低通滤波器结构体指针
  * @retval 滤波后的数据
*/
float _Ch_FBData_LPF(float RawData, LPF_StructTypeDef* pLPF)
{
    LPF_SetInput(pLPF, RawData);
    LPF_Cal(pLPF);
    return LPF_GetOutput(pLPF);
}

/**
  * @brief  底盘速度处理函数
  * @note   使用龙伯格观测器观测底盘速度，并进行速度补偿
  *         最终得到平滑的底盘速度反馈值
  * @param  无
  * @retval 处理后的平滑底盘速度值，单位m/s
*/
float _Ch_VelFB_Process(void)
{
    /*底盘速度补偿值的获取*/
    float ChVelComp = (GSTCH_Data.xC1_dot + GSTCH_Data.xC2_dot)/2.0f; //底盘补偿速度原始值，单位m/s
    ChVelComp = _Ch_FBData_LPF(ChVelComp, &GstCH_VelCompLPF);

    /*底盘速度观测值的处理*/
    float HMAngleVelAvg        = (GSTCH_HM1.AngleVelFB + GSTCH_HM2.AngleVelFB) / 2.0f; //轮毂电机角速度平均值，单位rad/s
    float ChassisVel_TheoryRaw = HMAngleVelAvg * WheelRadius;                          //底盘理论质心水平速度原始值，单位m/s
    float ChVel_Theory         = _Ch_FBData_LPF(ChassisVel_TheoryRaw, &GstCH_TheoryVelLPF);  //底盘理论质心水平速度滤波处理

    GstCH_VelObserver.y_Matrix[0] = ChVel_Theory;                 //观测器输入值赋值，底盘速度理论原始值，单位m/s
    GstCH_VelObserver.y_Matrix[1] = GSTCH_Data.AccXFB;            //观测器输入值赋值，底盘加速度原始值，单位m/s²
    LuenbergerObserver_UniformVelocityModel(&GstCH_VelObserver);  //调用龙伯格观测器计算，神奇的老代码的东西

    /*计算最终底盘速度反馈值*/
    return GstCH_VelObserver.x_HatMatrix[0] + ChVelComp; //底盘速度 = 观测器估计速度 + 补偿速度
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
void Chassis_AllFBDataUpdate(void)
{
    /***************** 轮毂电机相关数据更新 *****************/
    _HM_FBData_Parse(LeftSide,  &GSTCH_HM1, &GstCH_HM1RxC620Data, &GstCH_HM1_AngleVelLPF); //处理电调反馈数据，然后分发到正式结构体、变量
    _HM_FBData_Parse(RightSide, &GSTCH_HM2, &GstCH_HM2RxC620Data, &GstCH_HM2_AngleVelLPF); //处理电调反馈数据，然后分发到正式结构体、变量

    /**************** IMU2-关节电机数据更新 ****************/
    GSTCH_JM1.AngleFB    = GstCH_IMU2.ST_Rx.JM1_AngleFB;
    GSTCH_JM1.AngleVelFB = GstCH_IMU2.ST_Rx.JM1_AngleVelFB;
    GSTCH_JM1.TorqueFB   = GstCH_IMU2.ST_Rx.JM1_TorqueFB;

    GSTCH_JM2.AngleFB    = GstCH_IMU2.ST_Rx.JM2_AngleFB;
    GSTCH_JM2.AngleVelFB = GstCH_IMU2.ST_Rx.JM2_AngleVelFB;
    GSTCH_JM2.TorqueFB   = GstCH_IMU2.ST_Rx.JM2_TorqueFB;

    GSTCH_JM3.AngleFB    = GstCH_IMU2.ST_Rx.JM3_AngleFB;
    GSTCH_JM3.AngleVelFB = GstCH_IMU2.ST_Rx.JM3_AngleVelFB;
    GSTCH_JM3.TorqueFB   = GstCH_IMU2.ST_Rx.JM3_TorqueFB;

    GSTCH_JM4.AngleFB    = GstCH_IMU2.ST_Rx.JM4_AngleFB;
    GSTCH_JM4.AngleVelFB = GstCH_IMU2.ST_Rx.JM4_AngleVelFB;
    GSTCH_JM4.TorqueFB   = GstCH_IMU2.ST_Rx.JM4_TorqueFB;

    /*************** IMU2-底盘运动姿态数据更新 ***************/
    GSTCH_Data.YawAngleVelFB   = _Ch_FBData_LPF(GstCH_IMU2.ST_Rx.YawAngleVel, &GstCH_YawAngleVelLPF);
    GSTCH_Data.PitchAngleFB    = GstCH_IMU2.ST_Rx.PitchAngle;
    GSTCH_Data.PitchAngleVelFB = _Ch_FBData_LPF(GstCH_IMU2.ST_Rx.PitchAngleVel, &GstCH_PitchAngleVelLPF);

    GSTCH_Data.RollAngleFB     = GstCH_IMU2.ST_Rx.RollAngle;
    // GSTCH_Data.AccXFB          =_Ch_FBData_LPF(-GstCH_IMU2.ST_Rx.AccX, &GstCH_AccX_LPF);    //底盘云控反着安装，所以取负号 //换车时需修改
    //后面看到这里记得把GstCH_AccX_LPF给删了
    GSTCH_Data.AccXFB          = -GstCH_IMU2.ST_Rx.AccX;         //不滤波了，待测试25.12.2
    GSTCH_Data.AccZFB          = GstCH_IMU2.ST_Rx.AccZ;          //待测试25.12.2

    /*************** 底盘五连杆解算相关数据更新 ***************/
    GSTCH_Data.LegLen1FB  = LegLinkage_GetLegLength(&GstCH_LegLinkCal1);  //获取左腿实际长度
    GSTCH_Data.LegLen2FB  = LegLinkage_GetLegLength(&GstCH_LegLinkCal2);  //获取右腿实际长度

    GSTCH_Data.Theta1FB   = LegLinkage_GetTheta(&GstCH_LegLinkCal1, LeftSide);  //获取左腿与垂直方向夹角
    GSTCH_Data.Theta2FB   = LegLinkage_GetTheta(&GstCH_LegLinkCal2, RightSide); //获取右腿与垂直方向夹角

    float Theta1_dotRawValue    = LegLinkage_GetThetadot(&GstCH_LegLinkCal1, LeftSide);     //获取左腿Theta角速度，原始值
    float Theta2_dotRawValue    = LegLinkage_GetThetadot(&GstCH_LegLinkCal2, RightSide);    //获取右腿Theta角速度，原始值
    GSTCH_Data.Theta1AngleVelFB = _Ch_FBData_LPF(Theta1_dotRawValue, &GstCH_Theta1dotLPF);  //低通滤波处理
    GSTCH_Data.Theta2AngleVelFB = _Ch_FBData_LPF(Theta2_dotRawValue, &GstCH_Theta2dotLPF);  //低通滤波处理

    float xC1_dotRawValue = LegLinkage_GetxCdot(&GstCH_LegLinkCal1, LeftSide);  //获取左边五连杆解算C点x坐标的微分，原始值
    float xC2_dotRawValue = LegLinkage_GetxCdot(&GstCH_LegLinkCal2, RightSide); //获取右边五连杆解算C点x坐标的微分，原始值
    GSTCH_Data.xC1_dot    = _Ch_FBData_LPF(xC1_dotRawValue, &GstCH_xC1dotLPF);  //低通滤波处理
    GSTCH_Data.xC2_dot    = _Ch_FBData_LPF(xC2_dotRawValue, &GstCH_xC2dotLPF);  //低通滤波处理

    /***************** 其他底盘相关数据更新 *****************/
    GSTCH_Data.VelFB  = _Ch_VelFB_Process();               //底盘速度滤波、观测处理
    GSTCH_Data.DisFB += GSTCH_Data.VelFB * GCH_TaskTime;  //位移 = 上次位移 + 速度*时间
}
//#endregion

//#region /************各种Des目标数据的修改、处理、更新相关函数********/
/**
  * @brief  更新腿长目标值（TD算法）
  * @note   从机器人控制结构体中获取数据，根据TD算法，计算出腿长的目标值
  * @param  RMCtrl：RobotControl_StructTypeDef类型的机器人控制结构体变量
  * @retval 无
*/
void CH_LegLenDes_Update(RobotControl_StructTypeDef RMCtrl)
{
    TD_SetInput(&GstCH_LegLen1TD, (RMCtrl.STCH_Default.LegLen1Des*MM2M)); //左腿腿长TD输入值设为左腿腿长目标值
    TD_Cal(&GstCH_LegLen1TD);                                       //左腿腿长TD计算
    GSTCH_Data.LegLen1Des = TD_GetOutput(&GstCH_LegLen1TD)*M2MM;    //左腿腿长目标值更新为TD输出值

    TD_SetInput(&GstCH_LegLen2TD, (RMCtrl.STCH_Default.LegLen2Des*MM2M)); //右腿腿长TD输入值设为右腿腿长目标值
    TD_Cal(&GstCH_LegLen2TD);                                       //右腿腿长TD计算
    GSTCH_Data.LegLen2Des = TD_GetOutput(&GstCH_LegLen2TD)*M2MM;    //右腿腿长目标值更新为TD输出值
}

/**
  * @brief  更新LQR相关数据的目标值
  * @note   从机器人控制结构体中，获取数据，更新LQR的计算目标值
  *         如果用户开启自定义LQR控制，则使用用户设置的所有目标值
  *         否则除了前四项外，目标值全部采用默认值0
  * @param  RMCtrl：RobotControl_StructTypeDef类型的机器人控制结构体变量
  * @retval 无
*/
void CH_LQR_DesDataUpdate(RobotControl_StructTypeDef RMCtrl)
{
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
void CH_VMC_DesDataUpdate(RobotControl_StructTypeDef RMCtrl)
{
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
    float Leg1FFForce = RMCtrl.STCH_Default.Leg1FFForce; //获取左腿前馈力
    float Leg2FFForce = RMCtrl.STCH_Default.Leg2FFForce; //获取右腿前馈力

    /****Roll轴补偿力PID计算****/
    PID_SetDes(&GstCH_RollCompPID, 0.0f * A2R); //底盘Roll轴补偿PID目标值，默认设为0rad
    PID_SetFB(&GstCH_RollCompPID, (GSTCH_Data.RollAngleFB - ChassisRollAngleZP) * A2R); //底盘Roll轴补偿PID反馈值，单位转为弧度
    PID_Cal(&GstCH_RollCompPID);
    float RollCompForce = PID_GetOutput(&GstCH_RollCompPID); //获取底盘Roll轴补偿力

    /****获取腿长PID的输出力****/
    /*左腿*/
    PID_SetDes(&GstCH_LegLen1PID, GSTCH_Data.LegLen1Des*MM2M);  //腿长PID目标值设为目标腿长
    PID_SetFB (&GstCH_LegLen1PID, GSTCH_Data.LegLen1FB*MM2M);   //腿长PID反馈值设置为实际腿长
    PID_Cal  (&GstCH_LegLen1PID);                               //腿长PID计算
    float Leg1PIDForce = PID_GetOutput(&GstCH_LegLen1PID);      //获取左腿腿长PID作用力

    /*右腿*/
    PID_SetDes(&GstCH_LegLen2PID, GSTCH_Data.LegLen2Des*MM2M);  //腿长PID目标值设为目标腿长
    PID_SetFB (&GstCH_LegLen2PID, GSTCH_Data.LegLen2FB*MM2M);   //腿长PID反馈值设置为实际腿长
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
    else
    {
        GSTCH_HM1.TorqueDes  = -LQR_Get_uVector(&GstCH_LQRCal, 1-1); //左轮毂电机力矩目标值
        GSTCH_HM2.TorqueDes  = -LQR_Get_uVector(&GstCH_LQRCal, 2-1); //右轮毂电机力矩目标值
    }
    GSTCH_HM1.CurrentDes = +_HM_DesData_TorqueToCurrent(GSTCH_HM1.TorqueDes); //左轮毂电机电流目标值
    GSTCH_HM2.CurrentDes = -_HM_DesData_TorqueToCurrent(GSTCH_HM2.TorqueDes); //右轮毂电机电流目标值
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
    GSTCH_JM1.TorqueDes = Limit(GstCH_Leg1VMC.T_Matrix[1], -JointMotorMAXTorque, JointMotorMAXTorque); //关节电机1力矩目标值
    GSTCH_JM3.TorqueDes = Limit(GstCH_Leg1VMC.T_Matrix[0], -JointMotorMAXTorque, JointMotorMAXTorque); //关节电机3力矩目标值

    GSTCH_JM2.TorqueDes = Limit(GstCH_Leg2VMC.T_Matrix[0], -JointMotorMAXTorque, JointMotorMAXTorque); //关节电机2力矩目标值
    GSTCH_JM4.TorqueDes = Limit(GstCH_Leg2VMC.T_Matrix[1], -JointMotorMAXTorque, JointMotorMAXTorque); //关节电机4力矩目标值
}
//#endregion

//#region /************各种数据的重置、清零函数************************/
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

    GSTCH_JM2.AngleDes    = GSTCH_JM1.AngleFB;  //关节电机2位置目标值
    GSTCH_JM2.AngleVelDes = 0.0f;               //关节电机2角速度目标值
    GSTCH_JM2.TorqueDes   = 0.0f;               //关节电机2力矩目标值

    GSTCH_JM3.AngleDes    = GSTCH_JM1.AngleFB;  //关节电机3位置目标值
    GSTCH_JM3.AngleVelDes = 0.0f;               //关节电机3角速度目标值
    GSTCH_JM3.TorqueDes   = 0.0f;               //关节电机3力矩目标值

    GSTCH_JM4.AngleDes    = GSTCH_JM1.AngleFB;  //关节电机4位置目标值
    GSTCH_JM4.AngleVelDes = 0.0f;               //关节电机4角速度目标值
    GSTCH_JM4.TorqueDes   = 0.0f;               //关节电机4力矩目标值
}

/**
  * @brief  腿部长度目标值数据重置函数
  * @note   把腿部长度相关的目标值全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void _CH_LegLen_DesDataReset(void)
{
    GSTCH_Data.LegLen1Des = GSTCH_Data.LegLen1FB;   //左腿目标长度清零
    GSTCH_Data.LegLen2Des = GSTCH_Data.LegLen2FB;   //右腿目标长度清零
}

/**
  * @brief  底盘LQR目标值数据重置函数
  * @note   把底盘LQR计算相关的目标值全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void _CH_LQRCal_DesDataReset(void)
{
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
}

/**
  * @brief  虚拟模型控制VMC计算相关目标值数据重置函数
  * @note   把虚拟模型控制计算相关的目标值全部重置，机器人未运动时调用
  * @param  无
  * @retval 无
*/
void _CH_VMCCal_DesDataReset(void)
{
    GSTCH_Data.Leg1ForceDes  = 0.0f; //VMC：左腿力目标值清零
    GSTCH_Data.Leg2ForceDes  = 0.0f; //VMC：右腿力目标值清零
    GSTCH_Data.Leg1TorqueDes = 0.0f; //VMC：左腿力矩目标值清零
    GSTCH_Data.Leg2TorqueDes = 0.0f; //VMC：右腿力矩目标值清零
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

    /*底盘特殊算法相关数据重置*/
    _CH_LQRCal_DesDataReset();  //底盘LQR计算相关目标值重置
    _CH_VMCCal_DesDataReset();  //底盘VMC计算相关目标值重置
    
    /*底盘的通用算法相关数据重置*/
    _CH_TDCal_DataReset();   //底盘TD计算相关目标值重置
    _CH_PIDCal_DataReset();  //底盘PID计算相关目标值重置

    /*其他底盘相关数据重置*/
    _CH_LegLen_DesDataReset();  //腿部长度目标值重置
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
    GST_RMCtrl.STCH_Default.LegLen1Des      = GSTCH_Data.LegLen1FB;
    GST_RMCtrl.STCH_Default.LegLen2Des      = GSTCH_Data.LegLen2FB;

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
//#endregion


/**************************************底盘其他相关函数***************************************************************************/
/**
  * @brief  检测是否需要进入手动标定状态的函数
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

/**
  * @brief  底盘模式选择参数结构体更新函数
  * @note   把底盘模式选择的相关参数，更新到底盘模式选择参数结构体中，以方便后面的模式选择处理
  *         在模式处理之前，应该先调用此函数，更新参数，然后再进入模式选择函数
  * @param  pModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型，底盘模式选择参数结构体指针
  * @retval 无
*/
void CH_ModeChooseParaStructUpdate(Chassis_ModeChooseParameter_StructTypeDef *pModeChoosePara)
{
    /*更新上次模式*/
    pModeChoosePara->MC_ModePre = GEMCH_ModePre;

    /*更新各个模式开始时间、当前时间*/
    pModeChoosePara->MC_ST_ModeStartTime = GSTCH_Data.ST_ModeStartTime; //各个模式开始时间
    pModeChoosePara->MC_TimeNow = RunTimeGet();                         //获取当前时间

    /*其他变量*/
    pModeChoosePara->MC_LegLenAvgFB = (GSTCH_Data.LegLen1FB + GSTCH_Data.LegLen2FB) / 2.0f; //两腿腿长反馈值平均值
    pModeChoosePara->MC_VelFB = GSTCH_Data.VelFB;                                           //底盘速度反馈值

    /*AutoSafe模式专用变量*/
    pModeChoosePara->MC_HubMotor1Rx_fps = GST_SystemMonitor.HubMotor1Rx_fps;    //轮毂电机1接收帧率
    pModeChoosePara->MC_HubMotor2Rx_fps = GST_SystemMonitor.HubMotor2Rx_fps;    //轮毂电机2接收帧率
    pModeChoosePara->MC_UART4Rx_fps     = GST_SystemMonitor.UART4Rx_fps;        //串口4，即IMU2接收帧率
}


