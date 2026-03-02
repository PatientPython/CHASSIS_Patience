---
name: commit
description: Git commit changes with intelligent module detection and grouping. Supports auto mode (hook-triggered) and manual mode with custom descriptions.
user-invocable: true
---

# Git Commit Skill

智能 Git 提交技能。自动分析代码变更，按模块分组，生成规范的 commit message。

## 使用方式

### 1️⃣ 自动模式（Hook 触发或无参数调用）
```
/commit
```
**行为**：
- 自动检测未提交的变更
- 按代码模块分类（Chassis、Gimbal、Algorithm、Communication 等）
- 逐个模块生成提交
- 自动推断功能描述
- **Message 格式**：`CPST/CPED: [修复|优化|添加] [模块] - [自动推断描述]`

**示例日志**：
```
CPST: 修改 Chassis - Chassis_Task.c, Chassis_APIFunction.c
CPST: 优化 Algorithm - Algorithm.c
CPST: 添加 Config - GlobalDeclare_Chassis.h
```

---

### 2️⃣ 手动提交 + 自定义描述
```
/commit 修改了 LQR 控制边界条件的判断逻辑
```
**行为**：
- 使用你的自定义描述替代自动推断
- 若涉及单个模块，直接提交所有变更
- 若涉及多个模块，询问是否分组提交或合并提交
- **Message 格式**（不含 Hook 前缀）：`[修复|优化|添加] [模块] - 修改了 LQR 控制边界条件的判断逻辑`

**提交结果**：
```
修改 Chassis - 修改了 LQR 控制边界条件的判断逻辑
```

---

### 3️⃣ 带选项的提交
```
/commit --auto-group 优化控制算法
```
**行为**：
- `--auto-group`: 强制按模块分组提交（即使只改了一个模块）
- 后跟的文本作为自定义描述
- **Message 格式**：`[修复|优化|添加] [模块] - 优化控制算法`

**提交结果**（假设改了多个模块）：
```
修改 Chassis - 优化控制算法
修改 Algorithm - 优化控制算法
```

---

## 工作流程

### 步骤一：检测变更
运行 `git status --porcelain` 获取所有未 commit 的变更。若无变更且非手动调用，则退出。

### 步骤二：分类模块
按文件路径自动识别所属模块：

| 路径模式 | 模块名 |
|---------|--------|
| `Application/Chassis/**` | Chassis |
| `Application/Gimbal/**` | Gimbal |
| `Application/Shooter/**` | Shooter |
| `API/Algorithm**` | Algorithm |
| `Communication/**` | Communication |
| `Global/GlobalDeclare_**` | 目标模块 + Config |
| `Project/`, `User/**` | Config |
| `*.md`, `*.txt` | Doc |

### 步骤三：决定提交策略

| 场景 | 策略 |
|------|------|
| 自动模式 + 无自定义描述 | 按模块逐个提交 |
| 自动模式 + 有自定义描述 | 仍按模块逐个提交，使用自定义描述 |
| 手动模式 + 单模块 | 合并一个提交 |
| 手动模式 + 多模块 | 询问用户：分组还是合并 |
| `--auto-group` 选项 | 强制按模块分组 |

### 步骤四：生成 Commit Message

**从代码推断操作类型**：
- 包含"修复"、"fix"、"bug" → `修复`
- 包含"优化"、"optim" → `优化`
- 包含"添加"、"add" → `添加`
- 削除占比 >50% → `删除`
- 默认 → 根据模块推断

**Message 格式规范**：
```
[前缀]: [操作类型] [模块] - [描述（最多 50 字）]
```

字符限制：不超过 72 字符（Git 约定）

**前缀规则**：
- Hook 在 `Stop` 事件触发 → `CPED`
- Hook 在 `UserPromptSubmit` 事件触发 → `CPST`
- 用户手动调用 → 无前缀（保留 `[操作类型] [模块] - [描述]`）

### 步骤五：执行提交
```bash
# 逐个模块
git add <files_for_module_1>
git commit -m "message_1"
git add <files_for_module_2>
git commit -m "message_2"
...

# 或合并提交
git add <all_files>
git commit -m "message"
```

### 步骤六：显示结果
输出提交摘要和最近的 git log（确认提交成功）。

---

## 环境变量

Hook 会自动设置以下环境变量，无需用户关注：

| 变量 | 值 | 含义 |
|------|-----|------|
| `HOOK_TRIGGERED` | `true` | 由 Hook 触发（否则为手动调用） |
| `HOOK_EVENT_TYPE` | `stop` \| `prompt_submit` | Hook 事件类型 |

---

## 高级用法

### 只提交特定文件（需要扩展）
```
/commit --files Application/Chassis/Chassis_Task.c
```
*(当前版本暂不支持，可在后续扩展)*

### 干跑一遍（预览会发生什么）
```
/commit --dry-run
```
*(当前版本暂不支持，可在后续扩展)*

---

## 故障排除

| 问题 | 解决方案 |
|------|---------|
| 没有任何变更但 skill 卡住 | 检查 git status，确保不在 rebase/merge 中 |
| Message 包含乱码 | 检查文件编码（应为 UTF-8） |
| 提交失败提示权限不足 | 检查 .git 目录权限 |
| 意外代码没有提交 | 手动运行 `git status` 确认文件状态 |

---

## Commit Message 示例

### 自动模式日志
```
CPST: 修改 Chassis - Chassis_Task.c, Chassis_APIFunction.c
CPST: 优化 Algorithm - Algorithm.c
CPED: 添加 Config - GlobalDeclare_Chassis.h
```

### 手动模式日志（改了一个模块）
```
修改 Chassis - 修改了 LQR 控制边界条件的判断逻辑
```

### 手动模式日志（改了多个模块，分组提交）
```
修改 Chassis - 优化控制算法
修改 Algorithm - 优化控制算法
更新 Config - 优化控制算法
```

---

## 内部实现

Skill 调用 `.claude/hooks/auto-commit.py` Python 脚本，该脚本负责：
1. 解析 Git 变更
2. 分类模块
3. 生成提交信息
4. 执行 `git add` 和 `git commit`

目前支持自动模式和单一自定义描述，后续可扩展更多选项。
