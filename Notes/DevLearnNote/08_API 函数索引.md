# 第八章 API 函数索引

## 8.1 概述

本章提供平衡步兵底盘控制系统所有 API 函数的完整索引，按功能分类排列。每个函数都包含简要说明和使用示例。

## 8.2 反馈数据处理 API

### 8.2.1 底盘反馈数据解析

**函数**: `CH_FBData_Parse()`
- **功能**: 解析底盘所有传感器反馈数据（轮毂电机、关节电机、IMU）
- **调用位置**: `ChassisTask` 主循环第一步
- **相关数据**: `GSTCH_HM1/2`, `GSTCH_JM1~4`, `GSTCH_Data`

```c
// 使用示例
void ChassisTask(void const * argument) {
    while (1) {
        // 1. 解析反馈数据
        CH_FBData_Parse();

        // 2. 后续处理...
    }
}
```

### 8.2.2 轮毂电机反馈解析

**函数**: `_HM_FBData_Parse(Side, pHM, pESC, pLPF)`
- **参数**:
  - `Side`: 左/右轮毂 (`LeftSide`/`RightSide`)
  - `pHM`: 轮毂电机数据结构体指针
  - `pESC`: C620 电调反馈结构体指针
  - `pLPF`: 低通滤波器结构体指针
- **输出**: 滤波后的角速度、电流、温度

### 8.2.3 低通滤波处理

**函数**: `_Ch_FBData_LPF(RawData, pLPF)`
- **功能**: 对底盘数据进行一阶低通滤波
- **参数**:
  - `RawData`: 原始数据
  - `pLPF`: 低通滤波器结构体指针
- **返回**: 滤波后的数据

## 8.3 运动学解算 API

### 8.3.1 腿部运动学主处理

**函数**: `CH_LegKinematics_Process()`
- **功能**: 执行双腿五连杆正运动学解算
- **调用位置**: `ChassisTask` 主循环
- **相关数据**: `GstCH_LegLinkCal1/2`

```c
// 使用示例
CH_LegKinematics_Process();

// 获取结果
float legLen1 = GSTCH_Data.LegLen1FB;  // 左腿长
float legLen2 = GSTCH_Data.LegLen2FB;  // 右腿长
float theta1 = GSTCH_Data.Theta1FB;    // 左腿摆角
float theta2 = GSTCH_Data.Theta2FB;    // 右腿摆角
```

### 8.3.2 五连杆角度更新

**函数**: `LegLinkage_AngleDataUpdate(LegPtr, JMAngle1, JMAngle2)`
- **功能**: 更新五连杆的 phi1、phi4 角度（关节电机角度 + 零点）
- **参数**:
  - `LegPtr`: 五连杆结构体指针
  - `JMAngle1/2`: 关节电机原始角度

### 8.3.3 五连杆角速度更新

**函数**: `LegLinkage_AngleVelDataUpdate(LegPtr, JMAngleVel1, JMVelValue2)`
- **功能**: 更新 phi1_dot、phi4_dot 角速度

### 8.3.4 正运动学计算

**函数**: `LegLinkage_ForwardKinematicsCal(LegPtr)`
- **功能**: 执行正运动学解算，计算各点坐标、phi2/phi3、L0/phi0
- **输出**:
  - 各连接点坐标 `(xA~E, yA~E)`
  - 小腿角度 `phi2, phi3`
  - 等效杆长度 `L0` 和角度 `phi0`

### 8.3.5 获取腿长

**函数**: `LegLinkage_GetLegLength(LegPtr)`
- **返回**: 腿长（竖直高度 yC，单位 mm）

### 8.3.6 获取等效杆长度

**函数**: `LegLinkage_GetL0Length(LegPtr)`
- **返回**: 等效连杆 L0 长度（单位 mm）

### 8.3.7 获取 Theta 角

**函数**: `LegLinkage_GetTheta(LegPtr, LegSide)`
- **参数**: `LegSide` - 左腿/右腿
- **返回**: 等效连杆与竖直方向夹角（单位 度，向后为正）

### 8.3.8 获取 Theta 角速度

**函数**: `LegLinkage_GetThetadot(LegPtr, LegSide)`
- **返回**: Theta 角速度（单位 度/s，使用雅可比矩阵计算）

### 8.3.9 获取 C 点 X 方向速度

**函数**: `LegLinkage_GetxCdot(LegPtr, LegSide)`
- **返回**: C 点 X 坐标微分（单位 m/s，向前为正）

### 8.3.10 获取 L0 长度变化率

**函数**: `LegLinkage_GetL0dot(LegPtr)`
- **返回**: L0 长度变化率（单位 m/s，变长为正）

## 8.4 LQR 控制 API

### 8.4.1 LQR 主处理

**函数**: `CH_LQRCal_Process()`
- **功能**: 执行 LQR 控制计算（状态向量更新、K 矩阵更新、控制量计算）
- **调用位置**: `ChassisTask` 主循环

```c
// 使用示例
CH_LQRCal_Process();

// 获取 LQR 输出
float T_wl = LQR_Get_uVector(&GstCH_LQRCal, 0);  // 左轮扭矩
float T_wr = LQR_Get_uVector(&GstCH_LQRCal, 1);  // 右轮扭矩
float T_bl = LQR_Get_uVector(&GstCH_LQRCal, 2);  // 左摆杆扭矩
float T_br = LQR_Get_uVector(&GstCH_LQRCal, 3);  // 右摆杆扭矩
```

### 8.4.2 状态向量更新

**函数**: `LQR_xVector_DataUpdate(LQRptr, DisErr, VelErr, YawAngleErr, ...)`
- **功能**: 更新 10 维状态向量
- **参数**: 10 个误差值（位移、速度、yaw、theta1/2、pitch 及其导数）

### 8.4.3 K 矩阵更新

**函数**: `LQR_K_MatrixUpdate(LQRptr, LegLen1, LegLen2)`
- **功能**: 根据腿长选择合适的 K 矩阵（低/中/高三档）
- **特殊处理**: 离地时自动调整 K 矩阵

### 8.4.4 LQR 计算

**函数**: `LQR_Cal(LQRptr)`
- **功能**: 执行 `u = K * x` 计算

### 8.4.5 获取控制向量

**函数**: `LQR_Get_uVector(LQRptr, index)`
- **参数**: `index` (0-3)
  - 0: 左轮扭矩 T_wl
  - 1: 右轮扭矩 T_wr
  - 2: 左摆杆扭矩 T_bl
  - 3: 右摆杆扭矩 T_br
- **返回**: 对应的扭矩值

## 8.5 VMC 控制 API

### 8.5.1 VMC 主处理

**函数**: `CH_VMCCal_Process()`
- **功能**: 执行 VMC 计算（F 矩阵更新、力矩映射）
- **调用位置**: `ChassisTask` 主循环

```c
// 使用示例
CH_VMCCal_Process();

// VMC 结果已自动更新到 GSTCH_Data
// GSTCH_Data.Leg1ForceDes   - 左腿等效力
// GSTCH_Data.Leg1TorqueDes  - 左腿等效力矩
// GSTCH_Data.Leg2ForceDes   - 右腿等效力
// GSTCH_Data.Leg2TorqueDes  - 右腿等效力矩
```

### 8.5.2 前馈力更新

**函数**: `VMC_FFForceUpdate(CHData)`
- **功能**: 更新重力前馈和惯性力前馈
- **输出**: `Leg1FFForce`, `Leg2FFForce`

### 8.5.3 F 矩阵更新

**函数**: `VMC_FMatrixUpdate(VMCptr, Force, Torque, LegSide)`
- **参数**:
  - `Force`: 沿杆方向的力
  - `Torque`: 绕杆的力矩
  - `LegSide`: 左腿/右腿

### 8.5.4 VMC 计算

**函数**: `VMC_Cal(VMCptr, LegPtr)`
- **功能**: 将虚拟力/力矩映射到关节电机力矩（`T = J * F`）

### 8.5.5 获取关节电机力矩

**函数**: `VMC_Get_TMatrix(VMCptr, index)`
- **参数**: `index` (0-1)
  - 0: phi1 关节电机力矩
  - 1: phi4 关节电机力矩
- **返回**: 对应的力矩值

## 8.6 离地检测 API

### 8.6.1 离地检测主处理

**函数**: `CH_OffGround_Process()`
- **功能**: 执行双腿离地检测，计算地面支持力
- **输出**:
  - `GSTCH_Data.Leg1F_N`, `Leg2F_N` - 地面支持力
  - `GSTCH_Data.F_OffGround1/2` - 离地标志

```c
// 使用示例
CH_OffGround_Process();

if (GSTCH_Data.F_OffGround1) {
    // 左腿离地处理
}
```

### 8.6.2 机体 Z 轴加速度更新

**函数**: `OffGround_BodyZAccUpdate(pOffGrd, AccZ_Body)`
- **功能**: 更新机体 Z 轴加速度

### 8.6.3 俯仰角更新

**函数**: `OffGround_PitchAngleUpdate(pOffGrd, PitchAngle)`
- **功能**: 更新俯仰角

### 8.6.4 俯仰角速度更新

**函数**: `OffGround_PitchAngleVelUpdate(pOffGrd, PitchAngleVel)`
- **功能**: 更新俯仰角速度

### 8.6.5 五连杆相关数据更新

**函数**: `OffGround_LegLinkRelateDataUpdate(LegPtr, pOffGrd, LegSide)`
- **功能**: 复制五连杆数据到离地检测结构体

### 8.6.6 关节电机力矩更新

**函数**: `OffGround_TorqueDataUpdate(pOffGrd, T1, T4)`
- **功能**: 更新关节电机力矩

### 8.6.7 计算虚拟摆杆力和力矩

**函数**: `OffGround_GetRealFAndTp(pOffGrd)`
- **功能**: 通过 VMC 逆矩阵计算 F_Leg 和 Tp_Leg

### 8.6.8 获取腿对轮子作用力

**函数**: `OffGround_GetLegToWheelForce(pOffGrd)`
- **返回**: 腿对轮子的竖直作用力 P（单位 N）

### 8.6.9 获取地面支持力

**函数**: `OffGround_GetSupportForce(pOffGrd)`
- **返回**: 地面支持力 F_N（单位 N）

## 8.7 卡尔曼滤波 API

### 8.7.1 速度融合主处理

**函数**: `CH_VelKF_Process()`
- **功能**: 执行卡尔曼滤波速度融合（轮速 + IMU）
- **输出**: `GSTCH_Data.VelFB` - 融合速度

```c
// 使用示例
CH_VelKF_Process();

float vel = GSTCH_Data.VelFB;  // 融合后的车身速度
```

### 8.7.2 卡尔曼滤波器初始化

**函数**: `KF_ChassisVel_StructInit(KFptr, dt)`
- **参数**:
  - `KFptr`: 卡尔曼滤波器结构体指针
  - `dt`: 采样时间

### 8.7.3 时间更新（预测）

**函数**: `KF_ChassisVel_Predict(KFptr)`
- **功能**: 执行匀加速模型预测

### 8.7.4 测量更新（校正）

**函数**: `KF_ChassisVel_Update(KFptr, v_body_obs, a_imu)`
- **参数**:
  - `v_body_obs`: 速度观测值（轮速 - 腿部摆动补偿）
  - `a_imu`: IMU 加速度

## 8.8 PID 控制 API

### 8.8.1 PID 结构体初始化

**函数**: `PID_StructInit(PIDptr, Kp, Ki, Kd, UMax, UpMax, UiMax, UdMax, AddMax)`
- **功能**: 初始化 PID 结构体所有参数

### 8.8.2 设置目标值

**函数**: `PID_SetDes(PIDptr, NewDes)`
- **功能**: 设置 PID 目标值

### 8.8.3 设置反馈值

**函数**: `PID_SetFB(PIDptr, NewFB)`
- **功能**: 设置 PID 反馈值

### 8.8.4 设置 PID 参数

**函数**: `PID_SetKpKiKd(PIDptr, Kp, Ki, Kd)`
- **功能**: 实时更新 PID 参数

### 8.8.5 PID 计算

**函数**: `PID_Cal(PIDptr)`
- **功能**: 执行 PID 计算（含各项限幅）

### 8.8.6 获取 PID 输出

**函数**: `PID_GetOutput(PIDptr)`
- **返回**: PID 总输出 U

### 8.8.7 PID 重置

**函数**: `PID_Reset(PIDptr)`
- **功能**: 重置 PID 所有状态（误差、积分、输出）

## 8.9 目标值更新 API

### 8.9.1 腿长目标值更新

**函数**: `CH_LegLenDes_Update()`
- **功能**: 更新腿长目标值（使用 TD 跟踪器）

### 8.9.2 LQR 目标值更新

**函数**: `CH_LQR_DesDataUpdate()`
- **功能**: 更新 LQR 控制目标值（位移、速度、角度等）

### 8.9.3 VMC 目标值更新

**函数**: `CH_VMC_DesDataUpdate(RMCtrl)`
- **功能**: 更新 VMC 控制目标值（等效力和力矩）
- **包含**: 腿长 PID、Roll 轴 PID、前馈力计算

### 8.9.4 轮毂电机目标值更新

**函数**: `HM_DesDataUpdate(RMCtrl)`
- **功能**: 更新轮毂电机力矩和电流目标值
- **特殊处理**: 离地状态保护

### 8.9.5 关节电机目标值更新

**函数**: `JM_DesDataUpdate(RMCtrl)`
- **功能**: 更新关节电机角度、速度、力矩目标值

## 8.10 运动控制 API

### 8.10.1 运动控制主 API

**函数**: `CH_MotionUpdateAndProcess()`
- **功能**: 执行完整的运动控制流程
- **调用顺序**:
  1. `CH_LegLenDes_Update()`
  2. `CH_LQR_DesDataUpdate()`
  3. `CH_LQRCal_Process()`
  4. `CH_VMC_DesDataUpdate()`
  5. `CH_VMCCal_Process()`
  6. `JM_DesDataUpdate()`
  7. `HM_DesDataUpdate()`

```c
// 使用示例（在模式控制函数中）
void ChModeControl_FreeMode_RCControl(void) {
    // 设置目标值
    GST_RMCtrl.STCH_Default.DisDes = ...;
    GST_RMCtrl.STCH_Default.VelDes = ...;

    // 调用运动控制 API
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}
```

### 8.10.2 自由模式移动处理

**函数**: `ChModeControl_FreeMode_RCControl_MoveHandler()`
- **功能**: 处理自由模式下的移动控制

### 8.10.3 小陀螺模式处理

**函数**: `ChModeControl_FreeMode_RCControl_TopHandler()`
- **功能**: 处理小陀螺（原地旋转）模式

## 8.11 遥控器指令 API

### 8.11.1 遥控器连接检测

**函数**: `IsRCConnected()`
- **返回**: `true` - 已连接，`false` - 未连接

### 8.11.2 拨杆位置检测

**函数**:
- `IsLeftLevelUp()` / `IsLeftLevelMid()` / `IsLeftLevelDown()`
- `IsRightLevelUp()` / `IsRightLevelMid()` / `IsRightLevelDown()`
- **返回**: `true`/`false`

### 8.11.3 拨轮检测

**函数**:
- `IsRollerUp()` / `IsRollerMid()` / `IsRollerDown()`
- **返回**: `true`/`false`

### 8.11.4 摇杆检测

**函数**:
- `IsLeftJoyStickUp()` / `IsLeftJoyStickDown()` / `IsLeftJoyStickLeft()` / `IsLeftJoyStickRight()`
- `IsRightJoyStickUp()` / `IsRightJoyStickDown()` / `IsRightJoyStickLeft()` / `IsRightJoyStickRight()`
- **返回**: `true`/`false`

### 8.11.5 通道值归一化

**函数**: `RCChannelToNorm(ChannelValue)`
- **功能**: 将遥控器通道值归一化到 [-1, 1] 范围
- **返回**: 归一化后的值

## 8.12 重置/清零 API

### 8.12.1 底盘所有目标值重置

**函数**: `Chassis_AllDesDataReset()`
- **功能**: 重置所有底盘目标值（位移、速度、角度、腿长等）

### 8.12.2 位移反馈清零

**函数**: `Chassis_DisFBClear()`
- **功能**: 将位移反馈清零（用于相对位移控制）

### 8.12.3 机器人控制数据重置

**函数**: `Chassis_RobotCtrlDataReset()`
- **功能**: 重置机器人控制相关数据

### 8.12.4 轮毂电机目标值重置

**函数**: `HM_DesDataReset()`
- **功能**: 重置轮毂电机力矩和电流目标值

### 8.12.5 关节电机目标值重置

**函数**: `JM_DesDataReset()`
- **功能**: 重置关节电机角度、速度、力矩目标值

## 8.13 状态机 API

### 8.13.1 模式选择参数更新

**函数**: `ChassisStrategy_ModeChooseParaStructUpdate(pModeChoosePara)`
- **功能**: 更新状态机需要的参数（模式、时间、传感器状态等）

### 8.13.2 模式选择

**函数**: `ChassisStrategy_ModeChoose_RCControl(ST_ModeChoosePara)`
- **功能**: 执行状态机流转，返回下一个模式
- **返回**: `ChassisMode_EnumTypeDef` 枚举值

### 8.13.3 模式开始时间更新

**函数**: `ChassisStrategy_ModeStartTimeUpdate(pCHData, Mode, ModePre)`
- **功能**: 记录各模式的开始时间

### 8.13.4 模式更新

**函数**: `CH_ChassisModeUpdate()`
- **功能**: 执行完整的模式更新流程（参数更新→状态机→时间记录）

### 8.13.5 模式控制执行

**函数**: `ChassisModeControl_RCControl(ModeNow)`
- **功能**: 根据当前模式执行相应的控制策略

## 8.14 完整调用示例

```c
// ChassisTask 主任务完整示例
void ChassisTask(void const * argument) {
    uint32_t WakeTime = RunTimeGet();

    for (;;) {
        /* 1kHz 主循环 */
        if (RunTimeGet() - WakeTime >= 1) {
            WakeTime = RunTimeGet();

            /* === 1. 反馈数据解析 === */
            CH_FBData_Parse();

            /* === 2. 运动学解算 === */
            CH_LegKinematics_Process();

            /* === 3. 离地检测 === */
            CH_OffGround_Process();

            /* === 4. 速度融合 === */
            CH_VelKF_Process();

            /* === 5. 轮毂电机补偿 === */
            CH_HMTorqueComp_Process();

            /* === 6. 遥控器预处理 === */
            CH_RCInputPre_Process();

            /* === 7. 模式更新 === */
            CH_ChassisModeUpdate();

            /* === 8. 模式控制执行 === */
            ChassisModeControl_RCControl(GEMCH_Mode);
        }
    }
}
```

## 8.15 全局变量索引

### 8.15.1 底盘主数据结构

```c
CHData_StructTypeDef GSTCH_Data;
```
**重要成员**:
| 成员 | 含义 | 单位 |
|------|------|------|
| `DisFB` | 位移反馈 | m |
| `VelFB` | 速度反馈 | m/s |
| `PitchAngleFB` | 俯仰角反馈 | 度 |
| `RollAngleFB` | 横滚角反馈 | 度 |
| `LegLen1FB`, `LegLen2FB` | 腿长反馈 | m |
| `Theta1FB`, `Theta2FB` | 腿摆角反馈 | 度 |
| `F_OffGround1`, `F_OffGround2` | 离地标志 | - |
| `Leg1F_N`, `Leg2F_N` | 地面支持力 | N |

### 8.15.2 算法结构体

```c
LQR_StructTypeDef        GstCH_LQRCal;       // LQR 计算
LegLinkageCal_StructTypeDef GstCH_LegLinkCal1/2;  // 五连杆解算
VMC_StructTypeDef        GstCH_Leg1VMC, GstCH_Leg2VMC;  // VMC 计算
OffGround_StructTypeDef  GstCH_OffGround1/2;  // 离地检测
KF_StructTypeDef         GstCH_VelKF;        // 速度卡尔曼滤波
PID_StructTypeDef        GstCH_LegLen1PID, GstCH_LegLen2PID;  // 腿长 PID
PID_StructTypeDef        GstCH_RollCompPID;  // Roll 补偿 PID
```

### 8.15.3 模式枚举

```c
ChassisMode_EnumTypeDef  GEMCH_Mode;      // 当前模式
ChassisMode_EnumTypeDef  GEMCH_ModePre;   // 上次模式
```

---

*文档结束*

---

## 文档版本信息

| 版本 | 日期 | 作者 | 说明 |
|------|------|------|------|
| v1.0 | 2026-02-26 | AI Assistant | 初始版本，完整 8 章 |

## 修订记录

- 2026-02-26: 完成全部 8 章文档编写
  - 00_概述.md
  - 01_LQR 控制.md
  - 02_五连杆解算.md
  - 03_离地检测.md
  - 04_速度卡尔曼滤波.md
  - 05_PID 与前馈.md
  - 06_状态机与遥控.md
  - 07_跳跃控制.md
  - 08_API 函数索引.md
