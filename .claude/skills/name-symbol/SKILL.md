---
name: name-symbol
description: Extract all variables from the control theory description documents and generate a standard naming document in accordance with the project's naming conventions. Accepts inputs in the form of documents, textual descriptions, images, etc.
disable-model-invocation: true
argument-hint: <theory description file path or description text>
allowed-tools: Read, Grep, Glob, Bash, Write
---

## 触发方式

```
/name-symbol docs/VMC推导.pdf
/name-symbol docs/LQR_theory.md docs/leg_model.md
/name-symbol "五连杆模型，包含腿长l0、摆角theta、髋关节角phi1/phi2..."
```

## 附加资源

- 命名规范详见 [naming_rules.md](naming_rules.md)
- 输出模板见 [template.md](template.md)，按此格式填写
- 预期输出示例见 [examples/sample.md](examples/sample.md)
- 验证脚本见 [scripts/validate.sh](scripts/validate.sh)，用于检查命名是否符合规范

## 执行流程

### 第一步：理论梳理

读取 `$ARGUMENTS` 指定的文件（支持 .md / .txt / .pdf / .c / .h），以及用户在对话中提供的文字描述和图片，全面理解控制理论内容：

- 识别物理系统模型（如五连杆、倒立摆、VMC等）
- 识别控制算法（如 PID、LQR、卡尔曼滤波等）
- 提取所有涉及的物理量、中间变量、控制变量、状态变量、参数

### 第二步：变量分类

将提取的变量按以下维度分类：

- **物理量**：角度、速度、加速度、力/力矩、长度/距离等
- **控制变量**：目标值、反馈值、误差、控制输出
- **状态变量**：状态向量分量
- **参数/常量**：PID增益、物理常数、限幅值
- **中间计算量**：推导过程的中间变量

### 第三步：按规范命名

加载 [naming_rules.md](naming_rules.md)，为每个变量生成规范命名：

1. 确定作用域 → `G_`(全局) / `S_`(静态) / 无(局部)
2. 确定类型前缀 → `F`(float) / `ST`(结构体) / `EM`(枚举) / 无(int)
3. 确定模块前缀 → `CH`(底盘) / `GB`(通用) / 按需扩展
4. 组合描述部分 → 使用项目缩写表 + 华为通用缩写表

### 第四步：填写模板输出

按 [template.md](template.md) 的格式输出命名文档。

**输出路径规则：**

1. 在项目根目录下查找是否已存在 当前工作区（workspaceFolder）下的 References/SymbolName/ 路径
2. 若存在，将报告保存到该目录下
3. 若不存在，创建完整路径 `References/SymbolName/`
4. 文件名使用涉及的**主要内容的简短描述**，格式为 `简短描述.md`
   - 示例：`VMC五连杆变量.md`、`LQR状态量命名.md`、`PID参数命名.md`

可运行验证脚本检查格式和命名规则：

```bash
bash scripts/validate.sh References/SymbolName/<文件名>.md
```

### 第五步：等待用户审批

- 输出：「命名文档已生成，请查看 `References/SymbolName/<文件名>.md`，确认或修改后告知」
- 用户可修改任何命名建议
- **不自动修改任何代码**，用户确认后才将命名应用到代码中

## 约束

- 仅负责变量命名，不负责代码逻辑实现
- 对于数学公式中的符号（如 θ、φ），保留原始符号作为注释说明
- 同一物理量在不同上下文中出现时，仅命名一次，注明所有使用场景
- **输出路径固定为 当前工作区（workspaceFolder）下的 References/SymbolName/ 路径。`**，生成前必须先检查该路径是否已存在，存在则直接写入，不存在则创建

