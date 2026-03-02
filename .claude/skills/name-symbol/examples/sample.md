# 变量命名文档

> 本文件由 `/naming` 技能自动生成，请审查后确认。

## 理论来源

- **输入文件**：`VMC推导.pdf`, `五连杆模型说明.md`
- **理论概述**：五连杆腿部模型的VMC（虚拟模型控制）推导，将双关节腿部抽象为虚拟弹簧-阻尼系统
- **生成日期**：2026-02-23

---

## 1. 物理量

| # | 数学符号 | 物理含义 | 代码命名 | 类型 | 单位 | 取值范围 | 使用场景 |
|---|---------|---------|---------|------|------|---------|---------|
| 1 | θ | 摆杆角度(与竖直方向夹角) | G_F_CH_PendulumAng | float | rad | [-0.5, 0.5] | LQR状态量x[0] |
| 2 | θ̇ | 摆杆角速度 | G_F_CH_PendulumAngSpd | float | rad/s | [-10, 10] | LQR状态量x[1] |
| 3 | l₀ | 虚拟腿长 | G_F_CH_LegLen | float | m | [0.15, 0.35] | VMC输出/LQR状态量 |
| 4 | φ₁ | 左髋关节角度 | G_F_CH_HipAngL | float | rad | [0, π] | 正运动学输入 |
| 5 | φ₂ | 右髋关节角度 | G_F_CH_HipAngR | float | rad | [0, π] | 正运动学输入 |
| 6 | φ₀ | 虚拟腿摆角 | G_F_CH_LegAng | float | rad | [-π/4, π/4] | VMC输出 |
| 7 | F | 虚拟腿支撑力 | G_F_CH_LegForce | float | N | [0, 100] | VMC虚拟力 |
| 8 | Tp | 虚拟腿摆动力矩 | G_F_CH_LegTorque | float | N·m | [-20, 20] | VMC虚拟力矩 |

---

## 2. 控制变量

| # | 数学符号 | 物理含义 | 代码命名 | 类型 | 单位 | 取值范围 | 使用场景 |
|---|---------|---------|---------|------|------|---------|---------|
| 1 | v_ref | 目标前进速度 | G_F_CH_SpdRef | float | m/s | [-2, 2] | 遥控器输入→速度环 |
| 2 | v_fdb | 实际前进速度 | G_F_CH_SpdFdb | float | m/s | [-3, 3] | 编码器反馈 |
| 3 | θ_ref | 目标摆角 | G_F_CH_AngRef | float | rad | [-0.3, 0.3] | 速度环输出→LQR |
| 4 | l₀_ref | 目标腿长 | G_F_CH_LegLenRef | float | m | [0.15, 0.35] | 腿长控制环 |
| 5 | T₁ | 左关节输出力矩 | G_F_CH_JointTorqueL | float | N·m | [-10, 10] | VMC逆解→电机 |
| 6 | T₂ | 右关节输出力矩 | G_F_CH_JointTorqueR | float | N·m | [-10, 10] | VMC逆解→电机 |

---

## 3. 状态变量

| # | 状态向量位置 | 数学符号 | 物理含义 | 代码命名 | 类型 | 单位 |
|---|------------|---------|---------|---------|------|------|
| 1 | x[0] | θ | 摆杆角度 | G_F_CH_StateAng | float | rad |
| 2 | x[1] | θ̇ | 摆杆角速度 | G_F_CH_StateAngSpd | float | rad/s |
| 3 | x[2] | x | 位移 | G_F_CH_StatePos | float | m |
| 4 | x[3] | ẋ | 速度 | G_F_CH_StateSpd | float | m/s |
| 5 | x[4] | φ₀ | 腿摆角 | G_F_CH_StateLegAng | float | rad |
| 6 | x[5] | φ̇₀ | 腿摆角速度 | G_F_CH_StateLegAngSpd | float | rad/s |

---

## 4. 参数与常量

| # | 数学符号 | 含义 | 代码命名 | 类型 | 值 | 备注 |
|---|---------|------|---------|------|-----|------|
| 1 | M | 机体质量 | BODY_MASS | const float | 3.5f | kg |
| 2 | m | 腿部等效质量 | LEG_MASS | const float | 0.3f | kg |
| 3 | L₁ | 大腿杆长 | LEG_UPPER_LEN | const float | 0.15f | m |
| 4 | L₂ | 小腿杆长 | LEG_LOWER_LEN | const float | 0.15f | m |
| 5 | g | 重力加速度 | GRAVITY | const float | 9.81f | m/s² |
| 6 | Kp_spd | 速度环P | G_F_CH_PidSpdKp | float | 15.0f | 可调参数 |
| 7 | Ki_spd | 速度环I | G_F_CH_PidSpdKi | float | 0.5f | 可调参数 |
| 8 | Kd_spd | 速度环D | G_F_CH_PidSpdKd | float | 2.0f | 可调参数 |

---

## 5. 中间计算量

| # | 数学符号 | 含义 | 代码命名 | 类型 | 出现函数 |
|---|---------|------|---------|------|---------|
| 1 | J | 雅可比矩阵元素 | j11, j12, j21, j22 | float | CalcVMC() |
| 2 | α | 五连杆中间角 | alpha | float | CalcForwardKinematics() |
| 3 | β | 五连杆中间角 | beta | float | CalcForwardKinematics() |
| 4 | d | 两髋关节间距 | hip_dist | float | CalcForwardKinematics() |
| 5 | θ_err | 角度误差 | ang_err | float | CalcLQR() |
| 6 | u | LQR控制输出向量 | lqr_output | float[2] | CalcLQR() |

---

## 6. 结构体定义

| # | 结构体类型名 | 实例名 | 包含成员 | 用途 |
|---|------------|--------|---------|------|
| 1 | ST_PidParam | st_CH_SpdPid | Kp, Ki, Kd, IntegralMax, OutMax, OutMin, Integral, LastErr | 速度环PID参数与状态 |
| 2 | ST_LegState | st_CH_LegL, st_CH_LegR | HipAng, LegLen, LegAng, Force, Torque | 左/右腿状态 |
| 3 | ST_LqrGain | st_CH_LqrK | K[2][6] | LQR增益矩阵 |
| 4 | ST_ChassisState | st_CH_State | Ang, AngSpd, Pos, Spd, LegAngL, LegAngR | 整车状态向量 |

---

## 7. 枚举定义

| # | 枚举类型名 | 枚举值 | 含义 |
|---|-----------|-------|------|
| 1 | EM_ChassisMode | CHASSIS_MODE_IDLE = 0 | 待机，电机不使能 |
| | | CHASSIS_MODE_STAND = 1 | 站立，腿长控制 |
| | | CHASSIS_MODE_BALANCE = 2 | 平衡行走 |
| | | CHASSIS_MODE_FALLEN = 3 | 跌倒检测，自动关闭 |

---

## 命名规范摘要

- 全局变量：`G_[类型][模块]_描述`（如 `G_F_CH_WheelSpd`）
- 静态变量：`S_[类型][模块]_描述`
- 局部变量：直接描述（如 `torque_eq`）
- 结构体类型：`ST_Xxx`，实例：`st_XX_Xxx`
- 枚举类型：`EM_Xxx`，值：`XXX_STATE_NAME`
- 宏/常量：`MAX_MOTOR_SPEED`（全大写下划线）
- 函数：公开无前缀，内部 `_` 前缀，底层 `__` 前缀
