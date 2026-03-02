---
name: commit-push-pr
description: One-command workflow for git commit, push, and create pull request. Streamlines the development cycle with intelligent defaults and customization options.
user-invocable: true
---

# Commit-Push-PR Skill

一键完成 Git 提交、推送、创建 PR 的完整工作流程。自动化开发周期，提高效率。

## 快速开始

```bash
/commit-push-pr
```

**效果**：
1. 执行 `/commit` 提交变更（使用自动推断的描述）
2. 自动推送到远程分支
3. 创建 PR（本分支 → 检测到的主分支）
4. 输出 PR 链接

---

## 使用方式

### 1️⃣ 一键提交、推送、创建 PR（推荐）
```bash
/commit-push-pr
```

**工作流程**：
- ✅ 自动检测未提交的变更
- ✅ 按模块分类并提交（复用 `/commit` 逻辑）
- ✅ 推送到当前分支的远程跟踪分支
- ✅ 自动检测目标分支（通常为 `main` 或 `master`）
- ✅ 创建 PR：当前分支 → 目标分支
- ✅ 输出 PR 链接和状态

**输出示例**：
```
✅ Commit: CPST: 修改 Chassis - Chassis_Task.c, Chassis_APIFunction.c
✅ Push: Successfully pushed to origin/feature-branch
✅ PR Created: https://github.com/user/repo/pull/42
   Title: Update Chassis Control - Commit message
   Target: main ← feature-branch
```

---

### 2️⃣ 自定义提交信息
```bash
/commit-push-pr 修改了 LQR 控制边界条件的判断逻辑
```

**行为**：
- 使用自定义描述进行提交（代替自动推断）
- 同样自动推送和创建 PR
- PR 标题自动从提交信息推导

**输出示例**：
```
✅ Commit: 修改 Chassis - 修改了 LQR 控制边界条件的判断逻辑
✅ Push: Successfully pushed to origin/feature-branch
✅ PR Created: https://github.com/user/repo/pull/43
   Title: Modify Chassis Control - LQR boundary logic
   Target: main ← feature-branch
```

---

### 3️⃣ 指定目标分支
```bash
/commit-push-pr --target develop 优化控制算法
```

**选项**：
- `--target <branch>`: 指定 PR 的目标分支（默认自动检测）

**行为**：
- 提交使用自定义描述 "优化控制算法"
- 推送到当前分支
- 创建 PR 到指定的 `develop` 分支

**示例**：
```
✅ Commit: 修改 Chassis - 优化控制算法
✅ Push: Successfully pushed to origin/feature-lqr
✅ PR Created: https://github.com/user/repo/pull/44
   Title: Optimize Chassis Control
   Target: develop ← feature-lqr
```

---

### 4️⃣ 创建草稿 PR（Draft PR）
```bash
/commit-push-pr --draft
```

**选项**：
- `--draft`: 创建为草稿 PR（标记为 "Draft"，不会自动触发 CI/CD）

**用途**：
- 早期反馈阶段
- 还在开发中的功能
- 不想立即合并

**输出**：
```
✅ Commit: CPST: [自动检测]
✅ Push: Successfully pushed to origin/feature-draft
✅ PR Created (Draft): https://github.com/user/repo/pull/45
   Title: [Draft] Feature - [Description]
   Target: main ← feature-draft
   Status: DRAFT
```

---

### 5️⃣ 自定义 PR 标题和描述
```bash
/commit-push-pr --title "Fix LQR boundary" --body "修复了..." 优化边界条件
```

**选项**：
- `--title <text>`: 自定义 PR 标题
- `--body <text>`: 自定义 PR 描述
- 后跟的文本作为提交信息

**行为**：
- 提交信息：使用后跟的文本
- PR 标题：使用 `--title` 指定的内容
- PR 描述：使用 `--body` 指定的内容（支持 Markdown）

**示例**：
```
✅ Commit: 修改 Chassis - 优化边界条件
✅ Push: Successfully pushed to origin/feature-opt
✅ PR Created: https://github.com/user/repo/pull/46
   Title: Fix LQR boundary
   Body: 修复了...
   Target: main ← feature-opt
```

---

## 工作流程详解

### 步骤 1：验证前置条件
- ✅ 检查 `git` 环境是否可用
- ✅ 检查当前目录是否是 Git 仓库
- ✅ 检查是否有未提交的变更
- ✅ 确认当前分支不是保护分支（`main`, `master` 等）
- ✅ 确认已配置远程仓库（`origin`）

### 步骤 2：执行提交（复用 `/commit` 逻辑）
运行 `/commit` skill（带用户提供的自定义描述或自动推断）：
- 自动检测变更
- 按模块分类
- 生成规范的 commit message
- 逐个提交

**输出示例**：
```
✅ Commit: CPST: 修改 Chassis - Chassis_Task.c
✅ Commit: CPST: 优化 Algorithm - Algorithm.c
```

### 步骤 3：推送到远程
```bash
git push origin <current-branch>
```

- 推送当前分支到对应的远程跟踪分支
- 若远程分支不存在，自动创建

**输出**：
```
✅ Push: Successfully pushed to origin/feature-lqr
   Commits: 2
   Files changed: 3
```

### 步骤 4：检测目标分支
自动按优先级检测目标分支：

| 优先级 | 检测方式 | 用途 |
|--------|---------|------|
| 1 | `--target` 选项 | 用户手动指定 |
| 2 | 远程分支 `develop` | 开发分支 |
| 3 | 远程分支 `main` 或 `master` | 主分支 |
| 4 | 默认分支（仓库设置） | GitHub 默认分支 |

### 步骤 5：创建 PR
使用 GitHub CLI 或 Git 原生命令创建 PR：

```bash
# 使用 GitHub CLI（推荐）
gh pr create --title "..." --body "..." --base <target-branch>

# 或仓库特定的 Web URL
https://github.com/user/repo/compare/main...feature-branch
```

**PR 设置**：
- **标题**（Title）：
  - 若指定 `--title`：使用 `--title` 内容
  - 若自定义提交：从提交信息推导（e.g., "Modify Chassis Control - LQR boundary")
  - 若自动提交：从提交信息推导
  
- **描述**（Body）：
  - 若指定 `--body`：使用 `--body` 内容
  - 否则包含提交历史摘要和 Changelog

- **标签**（Labels）：自动添加基于变更的模块（可选）
  - `chassis`, `algorithm`, `gimbal`, `communication` 等

### 步骤 6：输出摘要
显示完整的工作流程结果：

```
═══════════════════════════════════════════════════════════
✅ All Tasks Completed Successfully

📝 Commits:
  • CPST: 修改 Chassis - Chassis_Task.c, Chassis_APIFunction.c
  • CPST: 优化 Algorithm - Algorithm.c

📤 Push:
  • Target: origin/feature-lqr
  • Status: ✅ Success
  • Remote: upstream

🔗 Pull Request Created:
  • URL: https://github.com/user/repo/pull/42
  • Title: Update Chassis Control and Algorithm
  • Base: main ← feature-lqr
  • Author: Your Name
  • Status: OPEN (Draft: NO)

⏱️  Time: 3.2s
═══════════════════════════════════════════════════════════
```

---

## 前置条件

### 必需
- ✅ Git 已安装且可用
- ✅ 当前仓库已初始化（`.git` 目录存在）
- ✅ 已配置远程仓库（`origin`）
- ✅ 已配置 Git 用户信息（`git config user.name` 和 `user.email`）

### 推荐
- ✅ GitHub CLI (`gh`) 已安装（用于创建 PR）
  - 安装：[https://cli.github.com/](https://cli.github.com/)
  - 登录：`gh auth login`
- ✅ 当前分支不是保护分支（`main`, `master`, `develop` 等）

### 未来支持
- 🔄 GitLab、Gitee 等其他平台的 PR 创建
- 🔄 Jira 集成（自动链接 PR 到 Issue）
- 🔄 自动 PR 模板（基于 `.github/pull_request_template.md`）

---

## 环境变量

| 变量 | 说明 |
|------|------|
| `GIT_EDITOR` | 若需交互式编辑，使用此编辑器 |
| `GITHUB_TOKEN` | GitHub CLI 认证令牌（自动检测） |
| `GH_HOST` | GitHub 企业版域名（可选） |

---

## 错误处理

| 场景 | 处理方式 |
|------|---------|
| 未提交的变更为空 | 提示用户，不执行后续步骤 |
| 提交失败（冲突等） | 停止流程，显示错误信息，建议手动解决 |
| 推送失败（权限等） | 停止流程，提示检查权限或网络 |
| 目标分支不存在 | 自动尝试下一优先级的分支 |
| PR 创建失败 | 显示仓库链接供手动创建 |
| 当前分支是保护分支 | 拒绝执行，提示切换分支 |

**恢复流程**：
- 若某步失败，用户可修复问题后重新运行 `/commit-push-pr`
- 智能检测已完成的步骤，跳过重复执行

---

## 集成示例

### 与代码审查工作流集成
```bash
# 1. 本地开发及提交、推送、创建 PR
/commit-push-pr 修改了 LQR 算法

# 2. GitHub 上 PR 自动触发 CI/CD 和代码审查
# (在项目的 .github/workflows 中配置)

# 3. 审查完成后，手动合并或使用 /merge-work-branch
/merge-work-branch
```

### 与持续集成集成
- PR 创建后自动触发 CI 流程（GitHub Actions 等）
- 在 PR 详情页查看构建和测试结果
- 若失败，修改代码后重新 push（自动更新 PR）

---

## 常见场景

### 场景 1：小改动快速提交
```bash
/commit-push-pr
# 使用自动推断的提交信息，快速完成工作流
```

### 场景 2：整个特性分支的 PR
```bash
/commit-push-pr --target develop 完成了用户认证功能
# 指定目标分支为 develop（开发分支）
```

### 场景 3：需要评审的草稿 PR
```bash
/commit-push-pr --draft 功能还在开发中，请先评审思路
# 创建为草稿 PR，邀请评审者早期反馈
```

### 场景 4：定制 PR 标题和描述
```bash
/commit-push-pr --title "Refactor LQR Algorithm" --body "
- Improved numerical stability
- Reduced computation time by 20%
- Added unit tests
" 重构了 LQR 算法
```

### 场景 5：修复 PR
```bash
# 原 PR：#42
# 发现问题后，再次修改并推送
git add .
/commit-push-pr 修复了边界条件检查
# 同一分支推送后，#42 PR 自动更新
```

---

## 性能指标

- **平均执行时间**：2-5 秒（取决于网络和提交数量）
- **支持的最大变更量**：无限制（受 Git/GitHub 限制）
- **并发执行**：单线程顺序执行（确保一致性）

---

## 故障排除

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| "Not in a git repository" | 当前目录不是 Git 仓库 | 切换到仓库目录或初始化 Git |
| "No changes to commit" | 没有未提交的变更 | 确认是否有修改，或查看 `git status` |
| "Permission denied for push" | 无权限推送到远程 | 检查 SSH 密钥或 HTTPS token |
| "Current branch is protected" | 当前分支是保护分支 | 切换到工作分支（如 `feature/*` 或 `develop`） |
| "gh: command not found" | GitHub CLI 未安装 | 安装 `gh`：https://cli.github.com/ |
| "Failed to create PR" | PR 创建出错 | 检查网络和权限，或手动创建 PR |
| "Upstream branch tracking issue" | 本地分支与远程分支未关联 | 手动设置：`git push -u origin <branch>` |

---

## 内部实现

### 脚本结构
Skill 由以下部分组成：

1. **SKILL.md** - 使用文档和行为规范（此文件）
2. **auto-commit-push-pr.py** - 核心实现脚本
   - 位置：`.claude/hooks/auto-commit-push-pr.py`
   - 依赖：`auto-commit.py`（提交逻辑复用）
   - 语言：Python 3.6+

### 执行流程

```
用户输入 /commit-push-pr [options]
    ↓
Claude Code 解析 SKILL.md
    ↓
调用 auto-commit-push-pr.py 脚本
    ↓
[Step 1] 校验环境
  - Git 仓库检查
  - 变更检测
  - 保护分支检查
  - 远程仓库检查
    ↓
[Step 2] 执行 Commit（调用 auto-commit.py）
  - 自动检测变更
  - 按模块分类
  - 生成规范 message
    ↓
[Step 3] 执行 Push
  - git push -u origin <branch>
  - 失败重试逻辑
    ↓
[Step 4] 检测目标分支
  - 优先级：--target > develop > main/master > 默认
    ↓
[Step 5] 创建 PR
  - 尝试 gh(GitHub CLI)
  - 失败则生成 web URL
    ↓
显示完整摘要
    ↓
返回状态码
```

### 依赖关系

```
commit-push-pr (主程序)
├── auto-commit.py (提交逻辑)
├── git (命令行)
└── gh (GitHub CLI，可选)
```

### 环境变量

| 变量 | 来源 | 用途 |
|------|------|------|
| `PATH` | 系统 | 查找 git、gh、python3 |
| `HOME` | 系统 | Git 配置和 SSH key |
| `GIT_*` | Git | 通常被清理以避免干扰 |
| `GITHUB_TOKEN` | 用户/.zsh | GitHub CLI 认证 |

### 错误处理

脚本使用分层错误处理：

1. **验证层** - 前置条件检查，早期失败
2. **执行层** - 各步骤独立错误捕获
3. **回退层** - gh 失败时使用 web URL
4. **恢复层** - 允许用户重新运行继续从失败点

### 代码库约定

脚本遵循项目的编码规范：
- Python 3 格式字符串
- UTF-8 编码处理
- 类型提示（可选，已使用）
- 详细的错误信息和日志

---

## 更新日志

### v1.0 (当前)
- ✅ 基础功能：commit → push → create PR
- ✅ 支持自定义提交信息
- ✅ 支持指定目标分支
- ✅ 支持草稿 PR
- ✅ 支持自定义 PR 标题和描述
- ✅ 自动目标分支检测
- ✅ 错误处理和恢复机制
- ✅ GitHub CLI 集成（带 web URL 回退）
- ✅ 与 auto-commit.py 无缝集成

### 计划中
- 🔄 GitLab 和 Gitee 支持
- 🔄 Jira Issue 自动链接
- 🔄 PR 模板自动填充
- 🔄 代码审查者自动分配
- 🔄 自动合并（若 CI 通过）
- 🔄 多分支并发 PR（stack PR）
- 🔄 Commit 历史的 AI 自动生成 PR 描述
