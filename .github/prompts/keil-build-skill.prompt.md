# Keil Build Skill

此 skill 提供 Keil µVision 项目的自动编译和错误修复功能。

## 编译配置

项目使用以下配置（存储在 `~/.claude/agent-memory/keil-quick-debugger/MEMORY.md`）:
- UV4_PATH: D:\Programfile\MDK\Core\UV4\UV4.exe
- KEIL_PROJECT_PATH: E:\CODE_project\BalanceSoldier\ChassisControl\CHASSIS_Patience\Project\Project.uvprojx
- KEIL_LOG_PATH: E:\CODE_project\BalanceSoldier\ChassisControl\CHASSIS_Patience\Project\.vscode\uv4.log
- KEIL_TARGET_NAME: Target

## 使用方式

当用户要求:
- "编译" / "build" / "keil-build"
- "编译项目" / "编译代码"
- "运行 keil-build"
- 任何关于编译 Keil 项目的请求

## 执行流程

### 1. 运行编译
优先使用 `keil-build` 命令（Bash 环境）:
```
keil-build
```

如果返回 127/command-not-found，使用 fallback 路径:
```
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$USERPROFILE/.claude/hooks/keil-build-runner.ps1"
```

### 2. 读取编译结果
从固定路径读取 `uv4.log`:
`E:\CODE_project\BalanceSoldier\ChassisControl\CHASSIS_Patience\Project\.vscode\uv4.log`

### 3. 解析错误
- 如果包含 `0 Error(s)` → 编译成功，输出成功信息
- 否则提取错误信息（每次只处理第一个错误）

### 4. 自动修复（最多5轮）
对于简单的编译错误进行最小修复:
- 语法错误
- 拼写错误
- 缺少分号
- 类型不匹配
- 未定义的符号

修复原则:
- 只修改错误行涉及的文件
- 不修改业务逻辑
- 每轮只修复一个主要错误
- 如果5轮后仍有问题，返回错误并给出分析

### 5. 停止条件
- 编译成功: `0 Error(s)` → 输出 `Build clean. 0 Error(s).`
- 5轮修复失败 → 输出当前错误和已尝试的修复

## 输出格式

每轮输出格式:
`[Round N] <错误摘要> → <修改的文件> → <剩余错误数>`

## 项目约束

- 遵循前缀规则: `G/S` 作用域 · `ST/st/EM/F` 类型 · `CH/GB` 组件
- 不修改业务逻辑
- 只修复编译器报告的错误
- 使用 `keil-build` + `uv4.log` 作为编译状态来源
