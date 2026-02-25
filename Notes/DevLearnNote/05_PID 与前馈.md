# 第五章 PID 控制与前馈补偿

## 5.1 概述

在平衡步兵底盘系统中，PID 控制和前馈补偿用于实现以下功能：

1. **腿长控制**: 通过 PID 调节腿部支撑力，实现高度控制
2. **Roll 轴补偿**: 通过 PID 调节侧向力，防止侧翻
3. **前馈补偿**: 基于物理模型的重力前馈和惯性力前馈，提高响应速度

## 5.2 PID 控制数据结构

### 5.2.1 结构体定义

```c
// API/Algorithm.h
typedef struct {
    // 参数（初始化时赋值）
    float Kp, Ki, Kd;       // PID 参数
    float UMax, UpMax;      // 总输出限幅、P 项限幅
    float UiMax, UdMax;     // I 项限幅、D 项限幅
    float AddMax;           // 误差累加限幅（防积分饱和）

    // 运行时变量
    float Des;              // 目标值
    float FB;               // 反馈值
    float Err;              // 当前误差 (Des - FB)
    float PreErr;           // 上次误差
    float SumErr;           // 误差累积（用于积分项）
    float Up, Ui, Ud, U;    // P 项、I 项、D 项输出和总输出
} PID_StructTypeDef;
```

### 5.2.2 全局 PID 变量

```c
// Global/GlobalDeclare_Chassis.c
PID_StructTypeDef GstCH_LegLen1PID;   // 左腿腿长 PID
PID_StructTypeDef GstCH_LegLen2PID;   // 右腿腿长 PID
PID_StructTypeDef GstCH_RollCompPID;  // Roll 轴补偿 PID
```

### 5.2.3 PID 初始化参数

**腿长 PID 参数**:
```c
// Global/GlobalDeclare_Chassis.c L89-96
#define PID_LegLen_Kp 0.0f    // Kp 在外部根据不同模式赋值
#define PID_LegLen_Ki 0.0f    // 不使用积分
#define PID_LegLen_Kd 0.0f    // Kd 在外部根据不同模式赋值
#define PID_LegLen_UMax 400.0f    // 总输出最大值 (N)
#define PID_LegLen_UpMax 400.0f   // P 项输出最大值
#define PID_LegLen_UiMax 0.0f     // I 项输出最大值（不使用积分）
#define PID_LegLen_UdMax 400.0f   // D 项输出最大值
#define PID_LegLen_AddMax 0.01f   // 误差累加限幅
```

**Roll 轴补偿 PID 参数**:
```c
// Global/GlobalDeclare_Chassis.c L145-153
#define PID_RollComp_Kp 650.0f      // 比例系数
#define PID_RollComp_Ki 0.0f        // 不使用积分
#define PID_RollComp_Kd 15000.0f    // 微分系数
#define PID_RollComp_UMax 400.0f    // 总输出最大值 (N)
#define PID_RollComp_UpMax 400.0f   // P 项输出最大值
#define PID_RollComp_UiMax 0.0f     // I 项输出最大值
#define PID_RollComp_UdMax 50.0f    // D 项输出最大值
#define PID_RollComp_AddMax 0.01f   // 误差累加限幅
```

## 5.3 PID 控制 API 函数

### 5.3.1 基本 API 函数

**结构体初始化**:
```c
// API/Algorithm.c L60-86
void PID_StructInit(PID_StructTypeDef* PIDptr,
                    float Kp, float Ki, float Kd,
                    float UMax, float UpMax, float UiMax, float UdMax,
                    float AddMax) {
    PIDptr->Kp = Kp;  PIDptr->Ki = Ki;  PIDptr->Kd = Kd;
    PIDptr->UMax = UMax;  PIDptr->UpMax = UpMax;
    PIDptr->UiMax = UiMax;  PIDptr->UdMax = UdMax;
    PIDptr->AddMax = AddMax;
    PIDptr->Des = 0.0f;  PIDptr->FB = 0.0f;
    PIDptr->Err = 0.0f;  PIDptr->PreErr = 0.0f;
    PIDptr->SumErr = 0.0f;
    PIDptr->Up = 0.0f;   PIDptr->Ui = 0.0f;
    PIDptr->Ud = 0.0f;   PIDptr->U = 0.0f;
}
```

**设置目标值和反馈值**:
```c
// API/Algorithm.c L95-106
void PID_SetDes(PID_StructTypeDef* PIDptr, float NewDes) {
    PIDptr->Des = NewDes;
}

void PID_SetFB(PID_StructTypeDef* PIDptr, float NewFB) {
    PIDptr->FB = NewFB;
}

void PID_SetKpKiKd(PID_StructTypeDef* PIDptr, float Kp, float Ki, float Kd) {
    PIDptr->Kp = Kp;  PIDptr->Ki = Ki;  PIDptr->Kd = Kd;
}
```

### 5.3.2 PID 计算函数

```c
// API/Algorithm.c L130-162
void PID_Cal(PID_StructTypeDef* PIDptr) {
    // 1. 计算误差
    PIDptr->Err = PIDptr->Des - PIDptr->FB;

    // 2. P 项计算（比例项）
    PIDptr->Up = PIDptr->Kp * PIDptr->Err;
    LimitFloat(&PIDptr->Up, -PIDptr->UpMax, PIDptr->UpMax);

    // 3. I 项计算（积分项）
    PIDptr->SumErr += PIDptr->Err;
    LimitFloat(&PIDptr->SumErr, -PIDptr->AddMax, PIDptr->AddMax);  // 防积分饱和
    PIDptr->Ui = PIDptr->Ki * PIDptr->SumErr;
    LimitFloat(&PIDptr->Ui, -PIDptr->UiMax, PIDptr->UiMax);

    // 4. D 项计算（微分项）
    PIDptr->Ud = PIDptr->Kd * (PIDptr->Err - PIDptr->PreErr);
    LimitFloat(&PIDptr->Ud, -PIDptr->UdMax, PIDptr->UdMax);

    // 5. 总输出
    PIDptr->U = PIDptr->Up + PIDptr->Ui + PIDptr->Ud;
    LimitFloat(&PIDptr->U, -PIDptr->UMax, PIDptr->UMax);

    // 6. 保存上次误差
    PIDptr->PreErr = PIDptr->Err;
}
```

**计算流程**:
```
Des, FB → Err = Des - FB
              ↓
    ┌───────┬───────┬───────┐
    ↓       ↓       ↓       ↓
   Up      Ui      Ud      U
  (P 项)   (I 项)   (D 项)  (总输出)
    │       │       │       │
    └───────┴───────┴───────┘
```

### 5.3.3 输出读取和重置

```c
// API/Algorithm.c L170-191
float PID_GetOutput(PID_StructTypeDef* PIDptr) {
    return PIDptr->U;
}

void PID_Reset(PID_StructTypeDef* PIDptr) {
    PIDptr->Err = 0.0f;
    PIDptr->PreErr = 0.0f;
    PIDptr->SumErr = 0.0f;
    PIDptr->Up = 0.0f;
    PIDptr->Ui = 0.0f;
    PIDptr->Ud = 0.0f;
    PIDptr->U = 0.0f;
}
```

## 5.4 腿长 PID 控制

### 5.4.1 不同模式的 PID 参数

```c
// Global/GlobalDeclare_Chassis.c L116-137
// 起立模式
float PID_LegLen_KpStandUp = 800.0f;
float PID_LegLen_KdStandUp = 20000.0f;

// 正常模式
float PID_LegLen_KpNorm = 1500.0f;
float PID_LegLen_KdNorm = 120000.0f;

// 跳跃模式各阶段
float PID_LegLen_KpJump_Compress = 1000.0f;     // 压缩蓄力
float PID_LegLen_KdJump_Compress = 108000.0f;
float PID_LegLen_KpJump_Takeoff = 1800.0f;      // 起跳伸展
float PID_LegLen_KdJump_Takeoff = 0.0f;         // 零阻尼，快速响应
float PID_LegLen_KpJump_Retract = 3000.0f;      // 收腿
float PID_LegLen_KdJump_Retract = 120000.0f;
float PID_LegLen_KpJump_AirFree = 750.0f;       // 空中自由
float PID_LegLen_KdJump_AirFree = 120000.0f;
float PID_LegLen_KpJump_Landing = 1200.0f;      // 着陆缓冲
float PID_LegLen_KdJump_Landing = 108000.0f;
```

**参数设计思路**:
| 模式 | Kp | Kd | 说明 |
|------|-----|--------|------|
| 起立 | 800 | 20000 | 较低增益，平稳起立 |
| 正常 | 1500 | 120000 | 中等增益，兼顾响应和稳定性 |
| 压缩 | 1000 | 108000 | 较低增益，缓慢下蹲蓄力 |
| 起跳 | 1800 | 0 | 高增益 + 零阻尼，爆发性伸腿 |
| 收腿 | 3000 | 120000 | 最高增益，快速收缩腿部 |
| 空中 | 750 | 120000 | 低增益，保持姿态 |
| 着陆 | 1200 | 108000 | 中等增益，缓冲冲击 |

### 5.4.2 腿长 PID 计算

```c
// Application/Chassis/Chassis_APIFunction.c L272-283
// 左腿
PID_SetDes(&GstCH_LegLen1PID, GSTCH_Data.LegLen1Des);  // 目标腿长
PID_SetFB(&GstCH_LegLen1PID, GSTCH_Data.LegLen1FB);    // 实际腿长
PID_Cal(&GstCH_LegLen1PID);
float Leg1PIDForce = PID_GetOutput(&GstCH_LegLen1PID);  // 左腿 PID 作用力

// 右腿
PID_SetDes(&GstCH_LegLen2PID, GSTCH_Data.LegLen2Des);
PID_SetFB(&GstCH_LegLen2PID, GSTCH_Data.LegLen2FB);
PID_Cal(&GstCH_LegLen2PID);
float Leg2PIDForce = PID_GetOutput(&GstCH_LegLen2PID);  // 右腿 PID 作用力
```

**物理含义**:
- 输入：腿长误差 (m)
- 输出：沿虚拟摆杆方向的力 (N)
- PD 控制：`F = Kp * e + Kd * ė`

## 5.5 Roll 轴补偿 PID

### 5.5.1 Roll 轴控制原理

Roll 轴（横滚轴）倾斜会导致车身侧翻。通过检测 Roll 角，产生侧向补偿力：

```
Roll 角 → PID → 侧向补偿力
```

补偿力作用：
- 左腿：`+F_roll`（增加支撑力）
- 右腿：`-F_roll`（减小支撑力）

产生恢复力矩，使车身保持水平。

### 5.5.2 Roll 轴 PID 计算

```c
// Application/Chassis/Chassis_APIFunction.c L263-269
// 更新 PID 参数
PID_SetKpKiKd(&GstCH_RollCompPID,
              PID_RollComp_Kp_tmp,   // 650.0
              PID_RollComp_Ki_tmp,   // 0.0
              PID_RollComp_Kd_tmp);  // 35000.0

// 设置目标值和反馈值
PID_SetDes(&GstCH_RollCompPID, 0.0f * A2R);  // 目标 Roll 角 = 0 rad
PID_SetFB(&GstCH_RollCompPID,
          (GSTCH_Data.RollAngleFB - ChassisRollAngleZP) * A2R);  // 实际 Roll 角 (rad)

// 执行 PID 计算
PID_Cal(&GstCH_RollCompPID);
float RollCompForce = PID_GetOutput(&GstCH_RollCompPID);
```

**参数说明**:
- 目标值：0 rad（保持水平）
- 反馈值：Roll 角反馈 - 零点偏移（转换为弧度）
- 输出：侧向补偿力 (N)

## 5.6 前馈补偿

### 5.6.1 重力前馈

**原理**: 腿部需要持续输出力来支撑车身重力。使用前馈可以直接补偿重力，减少 PID 的稳态误差。

```c
// Global/GlobalDeclare_Chassis.c L189-192
float CH_Phys_EffMass = 0.5f * CH_Phys_Mass_Body + EFF_LEG_MASS_RATIO * CH_Phys_Mass_Leg;
float LegFForce_Gravity_1 = CH_Phys_EffMass * g;  // 左腿重力前馈
float LegFForce_Gravity_2 = CH_Phys_EffMass * g;  // 右腿重力前馈
```

**计算**:
```
F_gravity = M_eff * g
M_eff = 0.5 * m_body + η_l * m_leg
```

其中：
- `M_eff`: 单腿等效质量
- `m_body`: 车身质量
- `m_leg`: 单腿质量
- `η_l`: 腿部质量参与系数

### 5.6.2 惯性力前馈

**原理**: 当车身旋转（Yaw 转动）并前进时，会产生侧向惯性力，导致 Roll 轴倾斜。前馈补偿可以提前抵消这个力。

```c
// API/Algorithm.c L866-876
void VMC_FFForceUpdate(RobotControl_StructTypeDef* CHData) {
    // 当前平均腿长
    float l_current = (GSTCH_Data.LegLen1FB + GSTCH_Data.LegLen2FB) / 2.0f;

    // 偏航角速度 (rad/s)
    float YawRate = GSTCH_Data.YawAngleVelFB * A2R;

    // 前进速度 (m/s)
    float v_forward = GSTCH_Data.VelFB;

    // 惯性力计算
    float InertialCoeff = CH_Phys_InertialCoeff;
    LegFForce_Inertial_1 = InertialCoeff * l_current * YawRate * v_forward;  // 左腿
    LegFForce_Inertial_2 = InertialCoeff * l_current * YawRate * v_forward;  // 右腿

    // 最终前馈力（重力 - 惯性力）
    CHData->STCH_Default.Leg1FFForce = LegFForce_Gravity_1 - LegFForce_Inertial_1;
    CHData->STCH_Default.Leg2FFForce = LegFForce_Gravity_2 + LegFForce_Inertial_2;
}
```

**物理公式**:
```
F_inertial = C_inertial * l_current * ω_yaw * v_forward
```

其中：
- `C_inertial`: 惯性系数（与质量分布相关）
- `l_current`: 当前腿长
- `ω_yaw`: 偏航角速度
- `v_forward`: 前进速度

### 5.6.3 跳跃前馈

在跳跃起跳阶段，需要额外的爆发性前馈力：

```c
// Global/GlobalDeclare_Chassis.c L203-204
float LegFForce_Jump = 1000.0f;  // 跳跃前馈力 (N)
```

## 5.7 最终等效力合成

### 5.7.1 合成公式

```c
// Application/Chassis/Chassis_APIFunction.c L285-289
// 左腿
GSTCH_Data.Leg1ForceDes = Leg1PIDForce + RollCompForce + Leg1FFForce;
GSTCH_Data.Leg1TorqueDes = LQR_Get_uVector(&GstCH_LQRCal, 2);  // LQR 输出的摆杆力矩

// 右腿
GSTCH_Data.Leg2ForceDes = Leg2PIDForce - RollCompForce + Leg2FFForce;
GSTCH_Data.Leg2TorqueDes = LQR_Get_uVector(&GstCH_LQRCal, 3);
```

**合成说明**:
| 分量 | 左腿 | 右腿 | 说明 |
|------|------|------|------|
| 腿长 PID | `+F_pid1` | `+F_pid2` | 高度控制 |
| Roll 补偿 | `+F_roll` | `-F_roll` | 侧向平衡（左右相反） |
| 重力前馈 | `+F_grav` | `+F_grav` | 重力补偿（相同） |
| 惯性前馈 | `-F_inert` | `+F_inert` | 惯性补偿（左右相反） |

### 5.7.2 数据流图

```
┌─────────────────┐
│  腿长目标值     │
│  腿长反馈值     │
└────────┬────────┘
         ↓
    ┌────┴────┐
    │ 腿长 PID │
    └────┬────┘
         │ F_pid
         ↓
┌────────┼─────────┐
│  +     │         │
│        ↓         │
│   ┌────┴────┐   │
│   │Roll PID │   │
│   └────┬────┘   │
│        │ F_roll │
│        ↓        │
│   ┌────┴────┐   │
│   │前馈力   │   │
│   │F_ff     │   │
│   └────┬────┘   │
│        │        │
└────────┼────────┘
         ↓
    F_leg_des
         ↓
    ┌────┴────┐
    │  VMC    │
    │ 映射到  │
    │关节电机 │
    └─────────┘
```

## 5.8 API 调用示例

### 5.8.1 基本使用

```c
// 在 ChassisTask 1kHz 循环中

// 1. 更新 PID 参数（根据模式）
PID_SetKpKiKd(&GstCH_LegLen1PID,
              PID_LegLen_KpNorm,  // 1500.0
              0.0f,               // 不使用积分
              PID_LegLen_KdNorm); // 120000.0

// 2. 设置目标值和反馈值
PID_SetDes(&GstCH_LegLen1PID, GSTCH_Data.LegLen1Des);
PID_SetFB(&GstCH_LegLen1PID, GSTCH_Data.LegLen1FB);

// 3. 执行 PID 计算
PID_Cal(&GstCH_LegLen1PID);

// 4. 读取输出
float force = PID_GetOutput(&GstCH_LegLen1PID);
```

### 5.8.2 完整 VMC 目标值更新

```c
// 调用 VMC 目标值更新函数（包含 PID 和前馈）
CH_VMC_DesDataUpdate(RMCtrl);

// 此时 GSTCH_Data 中已包含：
// - Leg1ForceDes: 左腿等效力
// - Leg1TorqueDes: 左腿等效力矩
// - Leg2ForceDes: 右腿等效力
// - Leg2TorqueDes: 右腿等效力矩
```

## 5.9 调试建议

### 5.9.1 观测变量

| 变量 | 含义 | 正常范围 |
|------|------|----------|
| `GstCH_LegLen1PID.Err` | 左腿长误差 | < 0.01 m |
| `GstCH_LegLen1PID.U` | 左腿 PID 输出 | -200~200 N |
| `GstCH_RollCompPID.Err` | Roll 角误差 | < 0.05 rad |
| `GstCH_RollCompPID.U` | Roll 补偿力 | -50~50 N |
| `LegFForce_Gravity` | 重力前馈力 | ~100 N |
| `LegFForce_Inertial` | 惯性前馈力 | -20~20 N |

### 5.9.2 参数整定流程

1. **先调前馈**: 设置正确的前馈力，减小 PID 负担
2. **再调 P 项**: 从 0 逐渐增大，直到响应满意
3. **最后调 D 项**: 增加阻尼，抑制超调
4. **一般不用 I 项**: 腿长控制使用 PD 即可

### 5.9.3 常见问题

**问题 1**: 腿长响应慢

**解决**: 增大 `Kp`，或检查前馈力是否正确

**问题 2**: 腿长振荡

**解决**: 增大 `Kd`，或减小 `Kp`

**问题 3**: Roll 轴晃动

**解决**: 增大 `PID_RollComp_Kp`，或检查 Roll 角零点校准

---

*下一章：[第六章 状态机与遥控器逻辑](./06_状态机与遥控.md)*
