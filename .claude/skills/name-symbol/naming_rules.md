# 命名规范合集

> 本文件整合三个来源中**仅命名相关**的规则，供 naming skill 按需加载。

---

## 1. 项目自身命名规范（最高优先级）

来源：`User/ReadMe.txt`

### 1.1 变量三段式前缀：`[作用域][类型][模块]_描述`

| 作用域 | 类型 | 模块 |
|--------|------|------|
| `G` 全局 | `ST`/`st` 结构体(类型/实例) | `GM` 云台 |
| `S` 静态 | `EM` 枚举 | `CH` 底盘 |
| 无 局部 | `F` 浮点 | 按需扩展 |
| | 无 整型/其他 | |

**示例**：

```c
G_F_CH_WheelSpeed      // 全局-浮点-底盘-轮速
S_ST_CH_PidParam        // 静态-结构体实例-底盘-PID参数
G_EM_GM_RunState        // 全局-枚举-yun't-运行状态
```

### 1.2 函数命名前缀

| 前缀 | 含义 | 声明 |
|------|------|------|
| 无 | 公开函数 | `.h` 中声明 |
| `_` | 模块内部函数 | `static` 在 `.c` 中 |
| `__` | 底层/私有函数 | `static` 在 `.c` 中 |

### 1.3 结构体命名

- 类型定义：`typedef struct { ... } ST_PidParam;`
- 实例：`ST_PidParam st_CH_LegPid;`

### 1.4 项目缩写对照表

| 全称 | 缩写 | 全称 | 缩写 |
|------|------|------|------|
| Chassis | CH | Motor | Mtr |
| Speed | Spd | Angle | Ang |
| Position | Pos | Current | Cur |
| Target/Reference | Ref | Feedback | Fdb |
| Control | Ctrl | Parameter | Param |
| Initialize | Init | Callback | Cb |
| Communication | Comm | Receive | Rx |
| Transmit | Tx | | |

---

## 2. 华为C语言编程规范 — §3 标识符命名

### 强制规则

| 编号 | 规则 | 示例 |
|------|------|------|
| 3.1 | 项目内保持统一命名风格（本项目：unix_like+项目前缀） | `motor_speed` ✅ `motorSpeed` ❌ |
| 3.2 | 全局变量加 `g_` 前缀（项目扩展为 `G_`+类型+模块） | `G_F_CH_speed` |
| 3.3 | 静态变量加 `s_` 前缀（项目扩展为 `S_`+类型+模块） | `S_ST_CH_pid` |
| 3.4 | 禁止单字节变量，仅允许 `i/j/k` 循环变量 | `a` ❌ `angle` ✅ |
| 3.5 | 宏/枚举值全大写+下划线 | `MAX_MOTOR_SPEED` |
| 3.6 | 宏名禁止以 `_` 开头和结尾（头文件保护符除外）| |

### 建议

| 编号 | 建议 |
|------|------|
| 3.1 | 用反义词组命名互斥变量：add/remove, begin/end, create/destroy, start/stop, send/receive, lock/unlock, show/hide, min/max, next/previous, open/close |
| 3.2 | 避免名字含数字编号（除非逻辑需要）|
| 3.7 | 不使用匈牙利命名法（`dwWidth` ❌）|
| 3.8 | 变量用名词/形容词+名词 |
| 3.9 | 函数用动词/动词+名词：`SetMotorSpeed()`, `GetAngle()` |
| 3.10 | 函数指针按函数规则命名 |

### 通用缩写表

| 全称 | 缩写 | 全称 | 缩写 |
|------|------|------|------|
| argument | arg | buffer | buff |
| clock | clk | command | cmd |
| compare | cmp | configuration | cfg |
| device | dev | error | err |
| hexadecimal | hex | increment | inc |
| initialize | init | maximum | max |
| message | msg | minimum | min |
| parameter | para | previous | prev |
| register | reg | semaphore | sem |
| statistic | stat | synchronize | sync |
| temporary | tmp | | |

---

## 3. Google C++ Style Guide — 命名（适用C部分）

| 规则 | 说明 |
|------|------|
| 宏命名 | 全大写+下划线：`MY_MACRO_NAME` |
| 常量 | `k`前缀大驼峰 `kMaxSpeed` 或全大写 `MAX_SPEED`（与华为一致即可）|
| 变量 | 小写+下划线：`motor_speed` |
| 名称描述性 | 不为省空间牺牲可读性 |

---

## 4. 优先级

**项目规范** > **华为强制规则** > **华为建议** > **Google参考**

例外：第三方库、官方库、平台代码保持原风格。
