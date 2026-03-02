---
name: commit-push-pr
description: One-command workflow for git commit, push, and create pull request.
user-invocable: true
---

# Commit-Push-PR Skill

一键完成提交、推送、创建 PR 的完整工作流程。

## 使用方式

### 基本用法
```bash
/commit-push-pr
```
自动提交、推送、创建 PR（目标分支自动检测）。

### 自定义提交信息
```bash
/commit-push-pr 修改了 LQR 算法
```

### 指定目标分支
```bash
/commit-push-pr --target develop 优化控制
```

### 创建草稿 PR
```bash
/commit-push-pr --draft
```

### 自定义 PR 标题
```bash
/commit-push-pr --title "Fix LQR" 修改边界条件
```

## 工作流程

1. 验证环境（Git 仓库、变更、分支等）
2. 提交变更（复用 `/commit` 逻辑自动分类）
3. 推送代码（`git push -u origin <当前分支>`）
4. 检测目标分支（优先级：`--target` > `develop` > `main/master`）
5. 使用 GitHub CLI创建 PR
6. 显示结果

## 前置条件

- Git 已安装且配置好用户信息
- 当前在 Git 仓库中
- 当前分支不是保护分支（`main`, `master` 等）
- 已配置远程仓库（`origin`）
- 推荐：`gh auth login` 登录 GitHub CLI

## 选项

| 选项 | 说明 |
|------|------|
| `--target <branch>` | 指定 PR 目标分支 |
| `--title <text>` | 自定义 PR 标题 |
| `--body <text>` | 自定义 PR 描述 |
| `--draft` | 创建为草稿 PR |

## 输出示例

```
[>>] Commit-Push-PR Workflow Started

[OK] Environment validated

[*] Step 1: Committing changes...
[*] Step 2: Pushing to remote...
[*] Step 3: Detecting target branch...
[*] Step 4: Creating pull request...

============================================================
[OK] All Tasks Completed Successfully

[Commits]:
  + CPST: 修改 Chassis - Chassis_Task.c

[Push]:
  + Target: origin/feature-branch
  + Status: OK

[PR Created]:
  + URL: https://github.com/user/repo/pull/42
  + Status: OPEN (Draft: NO)

============================================================
```

## 常见场景

| 场景 | 命令 |
|------|------|
| 快速提交-推送-PR | `/commit-push-pr` |
| 自定义提交信息 | `/commit-push-pr 修复了 bug` |
| 指定开发分支 | `/commit-push-pr --target develop` |
| 草稿 PR | `/commit-push-pr --draft` |

## 故障排除

| 问题 | 解决方案 |
|------|---------|
| No changes to commit | 确认有本地修改：`git status` |
| Cannot push to protected branch | 切换到工作分支 |
| PR creation failed | 检查网络和 GitHub CLI 认证 |
| Push failed | 检查 SSH key 或 HTTPS token |
