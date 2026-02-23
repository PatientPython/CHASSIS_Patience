# Keil Build Skill 计划

## 目标
创建一个名为 `keil-build` 的 Copilot Skill，能够：
1. 自动调用 `tasks.json` 中的 Keil 编译任务
2. 读取编译结果
3. 对常见的编译小错误进行自动修复

---

## 当前状态

### 已验证
- ✅ Keil UV4 路径: `D:\Programfile\MDK\Core\UV4\UV4.exe`
- ✅ 项目路径: `E:\CODE_project\BalanceSoldier\ChassisControl\CHASSIS_Patience\Project\Project.uvprojx`
- ✅ 编译任务可用且运行正常（0 Error(s), 0 Warning(s), 编译时间 2秒）

### tasks.json 现有任务
- **label**: "Keil Build"
- **触发方式**: 运行 `shell: build` 或通过 Copilot 调用
- **输出**: 解析日志文件 `uv4.log`，提取错误和警告

---

## 实现方案

### 1. Skill 触发方式
```
用户输入: "编译 Keil 项目" 或 "build keil"
↓ Copilot 调用 task.json 中的 "Keil Build" 任务
```

### 2. 编译结果解析
任务执行后，读取 `.vscode/uv4.log` 文件：
- 匹配 `0 Error(s)` → 编译成功
- 匹配 `error:` 或 `Error(` → 提取错误信息

### 3. 常见错误自动修复（V1）
| 错误类型 | 检测方式 | 自动修复方案 |
|---------|---------|-------------|
| 缺失头文件 | `#include "xxx.h"` 找不到 | 添加缺失的 include 路径 |
| 未定义标识符 | `undefined identifier` | 检查拼写或添加声明 |
| 语法小错误 | 缺少分号、括号不匹配 | 定位并修复 |

### 4. 修复流程
1. 运行编译任务
2. 解析错误输出
3. 判断是否为可自动修复类型
4. 如果是 → 自动修复 → 重新编译
5. 如果否 → 返回错误信息给用户

---

## 下一步
1. 完善 tasks.json 的 problemMatcher 以支持更多错误格式
2. 添加错误分类逻辑
3. 实现自动修复函数
