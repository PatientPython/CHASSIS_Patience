# Superpowers 嵌入式开发流程定制方案 v4 (Final)

**核心定位：** AI 快速产出符合规范、自解释的代码 → 开发者上车调试 → 基于效果迭代

## 定制后的技能架构

## Proposed Changes

### 1. 复制并简化 keil-build skill

#### [NEW] skills/keil-build/

从 `C:\Users\35766\.claude\skills\keil-build` 复制。简化为**单次编译 + 输出结果**（去掉 round 1-8 修复循环，去掉"只修编译错误不改业务逻辑"等约束）。

---

### 2. TDD → compile-and-verify

#### [RENAME + REWRITE] skills/test-driven-development/ → skills/compile-and-verify/

- AI 目标：产出符合代码库风格、自解释的高质量代码
- 验证手段：keil-build `0 Error(s)` + 独立算法可做 unit test（可选）
- 合并 `verification-before-completion` 的核心内容：**AI 声称完成前必须有编译证据**
- 保留 YAGNI、DRY、最小实现
- 删除所有"先写测试""删除已写代码"铁律

---

### 3. brainstorming — 嵌入式深度访谈

#### [MODIFY] skills/brainstorming/SKILL.md

- 增加 Interview 阶段：**只问与当前任务相关的嵌入式问题**（不套模板）
- 提示引用 `References/ControlTheory/` 下的参考资料
- 接口/函数签名定义在此阶段完成，引用 `naming-reference` 附件
- 输出保存到 `References/DesignNote/YYYY-MM-DD-<topic>-design.md`

---

### 4. writing-plans → create-todolist

#### [RENAME + MODIFY] skills/writing-plans/ → skills/create-todolist/

- 重命名，贴合实际作用：将大计划拆成可执行小任务
- **使用官方 todo 工具**：
  - Claude Code: `TaskCreate` / `TaskGet` / `TaskList` / `TaskUpdate`
  - OpenCode: `todowrite`
- 计划保存到 `References/PlanPrompt/YYYY-MM-DD-<feature-name>.md`
- 任务步骤：实现 → keil-build → 可选 unit test → 提交
- 同步更新 [commands/write-plan.md](file:///e:/Patience%20Vault/Resources/superpowers%204.3.1/commands/write-plan.md) 的引用

---

### 5. 删除 systematic-debugging

#### [DELETE] skills/systematic-debugging/ （整个目录）

依赖清理：
- `writing-skills/SKILL.md` 中引用 `systematic-debugging` 作为示例 → 替换为其他 skill 名
- `systematic-debugging` 自身引用的 `test-driven-development`、`verification-before-completion` → 随删除一并消失

---

### 6. 删除 verification-before-completion

#### [DELETE] skills/verification-before-completion/ （整个目录）

核心内容已合并到 `compile-and-verify`（第 2 条）。依赖清理：
- `writing-skills/SKILL.md` 中引用作为示例 → 替换
- `systematic-debugging` 中引用 → 已删除

---

### 7. 创建 3 个编码规范参考附件

#### [NEW] 以下 3 个参考文件

从项目 ReadMe.txt + 华为C规范 + Google C++ Guide 提取：

| 文件 | 用途 | 主要来源 |
|------|------|----------|
| `skills/create-todolist/naming-reference.md` | 命名规范，brainstorm 阶段定义接口时引用 | ReadMe.txt（最高权重）+ 华为/Google 缩写规则 |
| `skills/subagent-driven-development/spec-compliance-ref.md` | 规范合规审查参考 | ReadMe.txt 编码规则 + 华为C规范精选 |
| `skills/subagent-driven-development/code-quality-ref.md` | 代码质量审查参考 | 华为C规范（函数设计/质量保证）+ Google C++ Guide 精选 |

---

### 8. subagent-driven-development 适配

#### [MODIFY] skills/subagent-driven-development/SKILL.md + 3 个 prompt 模板

- 完成标准：keil-build `0 Error(s)`
- [implementer-prompt.md](file:///e:/Patience%20Vault/Resources/superpowers%204.3.1/skills/subagent-driven-development/implementer-prompt.md) 中的 TDD 引用 → compile-and-verify
- [spec-reviewer-prompt.md](file:///e:/Patience%20Vault/Resources/superpowers%204.3.1/skills/subagent-driven-development/spec-reviewer-prompt.md) 引用 spec-compliance-ref
- [code-quality-reviewer-prompt.md](file:///e:/Patience%20Vault/Resources/superpowers%204.3.1/skills/subagent-driven-development/code-quality-reviewer-prompt.md) 引用 code-quality-ref
- Integration 部分更新所有 skill 名

---

### 9. executing-plans 适配

#### [MODIFY] skills/executing-plans/SKILL.md

- 验证步骤改为 keil-build
- writing-plans → create-todolist 引用更新

---

### 10. using-superpowers 更新

#### [MODIFY] skills/using-superpowers/SKILL.md

- Skill Types: `TDD` → `compile-and-verify`
- 删除 `debugging` 引用
- 更新 skill 名映射表

---

### 11. agents/code-reviewer + commands/ 更新

#### [MODIFY] agents/code-reviewer.md

- "test coverage" → "是否符合设计文档接口 + 编译通过"
- 引用 code-quality-ref 附件

#### [MODIFY] commands/write-plan.md

- `superpowers:writing-plans` → `superpowers:create-todolist`

---

### 12. README.md 重写

#### [MODIFY] README.md

完全重写为中文版，反映定制后的嵌入式工作流，清晰描述各 skill 职责。

---

## 不修改

| 项目 | 原因 |
|------|------|
| `using-git-worktrees` | 用户自学 |
| `requesting/receiving-code-review` | 功能保留 |
| `dispatching-parallel-agents` | 功能保留 |
| `writing-skills` | 仅做引用清理 |
| `hooks/` `.opencode/` 插件 | 逻辑正确 |

---

## Verification Plan

1. 逐文件审查修改后的每个 SKILL.md
2. 路径一致性：brainstorm → `References/DesignNote/`，plan → `References/PlanPrompt/`
3. 所有 `superpowers:` 引用无断链
4. 闭环验证：Mermaid 图中的每条连线在代码中有对应
