# 第一章 LQR 平衡控制

## 1.1 LQR 控制理论基础

### 1.1.1 LQR 基本原理

LQR（Linear Quadratic Regulator，线性二次型调节器）是一种现代控制理论中的最优控制方法。其核心思想是：

1. **线性系统**: 将非线性系统在平衡点附近线性化，得到状态空间模型：
   ```
   ẋ = Ax + Bu
   ```
   其中：
   - `x` 是状态向量（10 维）
   - `u` 是控制向量（4 维）
   - `A` 是系统矩阵
   - `B` 是输入矩阵

2. **二次型代价函数**: 定义代价函数：
   ```
   J = ∫(xᵀQx + uᵀRu)dt
   ```
   其中：
   - `Q` 是状态权重矩阵（半正定）
   - `R` 是控制权重矩阵（正定）

3. **最优控制律**: 通过求解代数 Riccati 方程，得到最优反馈增益矩阵 `K`：
   ```
   u = -Kx
   ```

### 1.1.2 本系统中的 LQR 实现

在本平衡步兵底盘系统中，LQR 控制器的设计如下：

**状态向量 x（10 维）**:
```
x = [s, s_dot, yaw, yaw_dot, θ_ll, θ̇_ll, θ_lr, θ̇_lr, pitch, pitch_dot]
```

| 索引 | 变量 | 物理含义 | 单位 |
|------|------|----------|------|
| 0 | s | 位移误差 | m |
| 1 | s_dot | 速度误差 | m/s |
| 2 | yaw | 偏航角误差 | rad |
| 3 | yaw_dot | 偏航角速度误差 | rad/s |
| 4 | theta_ll | 左腿摆角误差 | rad |
| 5 | theta_dot_ll | 左腿摆角速度误差 | rad/s |
| 6 | theta_lr | 右腿摆角误差 | rad |
| 7 | theta_dot_lr | 右腿摆角速度误差 | rad/s |
| 8 | pitch | 俯仰角误差 | rad |
| 9 | pitch_dot | 俯仰角速度误差 | rad/s |

**控制向量 u（4 维）**:
```
u = [T_wl, T_wr, T_bl, T_br]
```

| 索引 | 变量 | 物理含义 | 执行器 |
|------|------|----------|--------|
| 0 | T_wl | 左轮扭矩 | 左轮毂电机 |
| 1 | T_wr | 右轮扭矩 | 右轮毂电机 |
| 2 | T_bl | 左摆杆扭矩 | 左关节电机 |
| 3 | T_br | 右摆杆扭矩 | 右关节电机 |

> **注意**: 这里的控制向量是**虚拟模型量**，即基于虚拟的五连杆模型计算出的理论扭矩。

## 1.2 K 矩阵的计算与存储

### 1.2.1 K 矩阵的离线计算

K 矩阵通过 MATLAB 的 LQR 函数离线计算：

```matlab
K = lqr(A, B, Q, R);
```

其中 `A`和`B` 矩阵是通过五连杆动力学模型在平衡点线性化得到的。`Q`和`R` 矩阵通过人工调节，平衡系统响应速度和控制能量。

### 1.2.2 三档 K 矩阵存储

由于五连杆系统在不同腿长下的动力学特性不同，系统预存了三组 K 矩阵：

```c
// API/Algorithm.c
float LQR_K_Matrix_LegLenLow[4][10];    // 低腿长 K 矩阵 (约 0.12m)
float LQR_K_Matrix_LegLenMid[4][10];    // 中腿长 K 矩阵 (约 0.20m)
float LQR_K_Matrix_LegLenHigh[4][10];   // 高腿长 K 矩阵 (约 0.35m)
```

**低腿长 K 矩阵示例**:
```c
float LQR_K_Matrix_LegLenLow[4][10] = {
    {-5.0238, -7.3403, -2.7564, -1.5820, -22.5138, -2.2772, -8.2921, -1.0291, 26.9914, 1.3385},
    {-5.0238, -7.3403,  2.7564,  1.5820, -8.2921,  -1.0291, -22.5138, -2.2772, 26.9914, 1.3385},
    {4.2950,  6.2220,  -7.0016, -4.0609, 46.9070,  3.9621,  -21.6235, -1.2903, 277.4547, 7.9496},
    {4.2950,  6.2220,  7.0016,  4.0609,  -21.6235, -1.2903, 46.9070,  3.9621, 277.4547, 7.9496}
};
```

### 1.2.3 K 矩阵的在线选择

系统根据当前腿长实时选择合适的 K 矩阵：

```c
// API/Algorithm.c L694-771
void LQR_K_MatrixUpdate(LQR_StructTypeDef* LQRptr, float LegLen1, float LegLen2)
{
    // 高腿长
    if((LegLen1 == LegLenHigh) || (LegLen2 == LegLenHigh)) {
        // 拷贝高腿长 K 矩阵
    }
    // 低腿长
    else if ((LegLen1 == LegLenLow) || (LegLen2 == LegLenLow)) {
        // 拷贝低腿长 K 矩阵
    }
    // 中腿长（默认）
    else {
        // 拷贝中腿长 K 矩阵
    }

    // 离地状态特殊处理
    if(GSTCH_DataGet_F_OffGround1(GSTCH_Data)) {  // 左腿离地
        // 清零左轮轮毂电机和关节电机增益
        // 只对关节电机摆角维持相关项赋值
    }
    if(GSTCH_DataGet_F_OffGround2(GSTCH_Data)) {  // 右腿离地
        // 清零右轮轮毂电机和关节电机增益
        // 只对关节电机摆角维持相关项赋值
    }
}
```

**离地时的 K 矩阵处理逻辑**:
- 当某腿离地时，该侧的轮毂电机控制失去意义（轮子悬空）
- 仅保留关节电机对摆角的控制增益，维持腿部姿态

## 1.3 LQR 控制的 API 实现

### 1.3.1 数据结构定义

```c
// API/Algorithm.h
typedef struct {
    float K_Matrix[4][10];   // LQR 控制器的 K 矩阵
    float x_Vector[10];      // 状态向量 x（目标值 - 实际值）
    float u_Vector[4];       // 控制向量 u
} LQR_StructTypeDef;

// 全局变量
LQR_StructTypeDef GstCH_LQRCal;  // 底盘 LQR 计算结构体
```

### 1.3.2 状态向量更新函数

```c
// API/Algorithm.c L797-813
void LQR_xVector_DataUpdate(LQR_StructTypeDef* LQRptr,
                            float DisErr, float VelErr,
                            float YawAngleErr, float YawAngleVelErr,
                            float Theta1AngleErr, float Theta1AngleVelErr,
                            float Theta2AngleErr, float Theta2AngleVelErr,
                            float PitchAngleErr, float PitchAngleVelErr)
{
    LQRptr->x_Vector[0] = DisErr;
    LQRptr->x_Vector[1] = VelErr;
    LQRptr->x_Vector[2] = YawAngleErr;
    LQRptr->x_Vector[3] = YawAngleVelErr;
    LQRptr->x_Vector[4] = Theta1AngleErr;
    LQRptr->x_Vector[5] = Theta1AngleVelErr;
    LQRptr->x_Vector[6] = Theta2AngleErr;
    LQRptr->x_Vector[7] = Theta2AngleVelErr;
    LQRptr->x_Vector[8] = PitchAngleErr;
    LQRptr->x_Vector[9] = PitchAngleVelErr;
}
```

### 1.3.3 LQR 计算函数

```c
// API/Algorithm.c L823-830
void LQR_Cal(LQR_StructTypeDef* LQRptr) {
    for (int i = 0; i < 4; i++) {
        LQRptr->u_Vector[i] = 0.0f;
        for (int j = 0; j < 10; j++) {
            LQRptr->u_Vector[i] += LQRptr->K_Matrix[i][j] * LQRptr->x_Vector[j];
        }
    }
}
```

**计算公式**:
```
u[i] = Σ(K[i][j] * x[j])  for j=0 to 9
```

### 1.3.4 控制向量读取函数

```c
// API/Algorithm.c L846-852
float LQR_Get_uVector(LQR_StructTypeDef* LQRptr, int index) {
    if ((index < 0) || (index > 3)) {
        return 0.0f;  // 索引越界，返回 0
    }
    return LQRptr->u_Vector[index];
}
```

**返回值含义**:
| index | 返回值 | 物理含义 |
|-------|--------|----------|
| 0 | T_wl | 左轮扭矩 |
| 1 | T_wr | 右轮扭矩 |
| 2 | T_bl | 左摆杆扭矩（左髋关节） |
| 3 | T_br | 右摆杆扭矩（右髋关节） |

## 1.4 LQR 控制的处理流程

### 1.4.1 主处理函数

```c
// Application/Chassis/Chassis_APIFunction.c L541-569
void CH_LQRCal_Process(void) {
    /* 1. 更新 LQR 状态向量 */
    LQR_xVector_DataUpdate(&GstCH_LQRCal,
        // 位移误差
        GSTCH_Data.DisDes - GSTCH_Data.DisFB,
        // 速度误差
        GSTCH_Data.VelDes - GSTCH_Data.VelFB,
        // 偏转角增量（弧度）
        GSTCH_Data.YawDeltaDes * A2R,
        // 偏转角速度误差（弧度/s）
        (GSTCH_Data.YawAngleVelDes - GSTCH_Data.YawAngleVelFB) * A2R,
        // 左腿摆角误差（弧度）
        (GSTCH_Data.Theta1Des - (GSTCH_Data.Theta1FB - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP)) * A2R,
        // 左腿摆角速度误差（弧度/s）
        (GSTCH_Data.Theta1AngleVelDes - (GSTCH_Data.Theta1AngleVelFB - GSTCH_Data.PitchAngleVelFB)) * A2R,
        // 右腿摆角误差（弧度）
        (GSTCH_Data.Theta2Des - (GSTCH_Data.Theta2FB - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP)) * A2R,
        // 右腿摆角速度误差（弧度/s）
        (GSTCH_Data.Theta2AngleVelDes - (GSTCH_Data.Theta2AngleVelFB - GSTCH_Data.PitchAngleVelFB)) * A2R,
        // 俯仰角误差（弧度）
        (GSTCH_Data.PitchAngleDes - GSTCH_Data.PitchAngleFB + ChassisPitchAngleZP) * A2R,
        // 俯仰角速度误差（弧度/s）
        (GSTCH_Data.PitchAngleVelDes - GSTCH_Data.PitchAngleVelFB) * A2R
    );

    /* 2. 更新 LQR 的 K 矩阵 */
    LQR_K_MatrixUpdate(&GstCH_LQRCal,
                       GST_RMCtrl.STCH_Default.LegLen1ManualDes,
                       GST_RMCtrl.STCH_Default.LegLen2ManualDes);

    /* 3. 执行 LQR 计算 */
    LQR_Cal(&GstCH_LQRCal);
}
```

### 1.4.2 误差计算中的坐标系补偿

在计算腿部摆角误差时，需要补偿车身俯仰角的影响：

```
θ_leg_error = θ_leg_des - (θ_leg_FB - θ_pitch_FB + θ_pitch_ZP)
```

**物理含义**: 腿部摆角是相对于车身的角度，而 IMU 测量的俯仰角是相对于世界坐标系的。因此需要将车身俯仰角从腿部反馈中扣除，得到相对于竖直方向的摆角。

## 1.5 LQR 与 VMC 的协同

### 1.5.1 VMC 引入的必要性

LQR 计算出的控制向量 `u` 是**基于虚拟模型**的理论输出：
- `T_wl, T_wr`: 虚拟的轮子扭矩
- `T_bl, T_br`: 虚拟的摆杆扭矩

但实际系统中：
- 轮子扭矩由轮毂电机直接输出
- 摆杆扭矩需要通过五连杆机构，由关节电机产生

因此需要**VMC（Virtual Model Control，虚拟模型控制）**将虚拟的摆杆扭矩和径向力映射到实际的关节电机扭矩。

### 1.5.2 LQR 输出的使用

```c
// Application/Chassis/Chassis_APIFunction.c L286-289
// 左腿虚拟摆杆力矩 = LQR 输出矩阵的第 3 个元素（索引 2）
GSTCH_Data.Leg1TorqueDes = LQR_Get_uVector(&GstCH_LQRCal, 2);

// 右腿虚拟摆杆力矩 = LQR 输出矩阵的第 4 个元素（索引 3）
GSTCH_Data.Leg2TorqueDes = LQR_Get_uVector(&GstCH_LQRCal, 3);
```

### 1.5.3 完整的数据流

```
┌─────────────────┐
│ LQR 计算        │
│ u = K * x       │
└────────┬────────┘
         │
    ┌────┴────┐
    │         │
    v         v
┌───────┐ ┌───────┐
│T_bl   │ │T_br   │  (虚拟摆杆扭矩)
│T_wl   │ │T_wr   │  (虚拟轮子扭矩)
└───┬───┘ └───┬───┘
    │         │
    v         v
┌─────────────────┐
│ VMC 计算        │
│ T_motor = J * F │
└────────┬────────┘
         │
         v
┌─────────────────┐
│ 关节电机扭矩    │
│ T_JM1, T_JM2    │
│ T_JM3, T_JM4    │
└─────────────────┘
```

## 1.6 API 调用示例

### 1.6.1 基本使用

```c
// 在 ChassisTask 1kHz 主循环中

// 1. 更新反馈数据
CH_FBData_Parse();

// 2. 腿部运动学解算
CH_LegKinematics_Process();

// 3. LQR 计算（包含状态向量更新、K 矩阵更新、控制量计算）
CH_LQRCal_Process();

// 4. 读取 LQR 输出
float T_wl = LQR_Get_uVector(&GstCH_LQRCal, 0);  // 左轮扭矩
float T_wr = LQR_Get_uVector(&GstCH_LQRCal, 1);  // 右轮扭矩
float T_bl = LQR_Get_uVector(&GstCH_LQRCal, 2);  // 左摆杆扭矩
float T_br = LQR_Get_uVector(&GstCH_LQRCal, 3);  // 右摆杆扭矩

// 5. VMC 计算（将虚拟扭矩映射到关节电机）
CH_VMCCal_Process();
```

### 1.6.2 离地状态下的 LQR

```c
// 离地检测后，K 矩阵会自动调整
if (GSTCH_Data.F_OffGround1) {
    // 左腿离地时，LQR_K_MatrixUpdate 会：
    // 1. 清零左轮轮毂电机增益 (K_Matrix[0][j] = 0)
    // 2. 清零左腿关节电机除摆角外的增益
    // 3. 仅保留摆角维持增益 K_Matrix[2][4~7]
}
```

## 1.7 调试建议

### 1.7.1 观察变量

在 Keil 调试时，建议观察以下变量：

| 变量 | 含义 | 正常范围 |
|------|------|----------|
| `GstCH_LQRCal.x_Vector[8]` | 俯仰角误差 | < 0.1 rad |
| `GstCH_LQRCal.x_Vector[9]` | 俯仰角速度误差 | < 0.5 rad/s |
| `GstCH_LQRCal.u_Vector[2]` | 左摆杆扭矩指令 | < 10 Nm |
| `GstCH_LQRCal.u_Vector[3]` | 右摆杆扭矩指令 | < 10 Nm |

### 1.7.2 K 矩阵调整

如需调整系统响应特性，可修改 MATLAB 中的 `Q` 和`R` 矩阵后重新计算：

- **增大 Q**: 加快状态收敛，但可能引起振荡
- **增大 R**: 减小控制输出，响应变慢但更平滑

---

*下一章：[第二章 五连杆运动学与动力学解算](./02_五连杆解算.md)*
